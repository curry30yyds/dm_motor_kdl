#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <kdl/velocityprofile_spline.hpp>
#include <sensor_msgs/JointState.h>
#include "dm_motor_controller/mit_control_frame.h"

struct TrajectoryParams {
  std::vector<double> points;
  double segment_time = 2.0;
  double frequency = 100.0;
  double dt = 0.01;
  double kp = 10.0;
  double kd = 1.0;
  double torque = 0.0;
  bool loop = false;
  std::string csv_path = "data/trajectory_latest.csv";
  std::string csv_all_path = "data/trajectory_all.csv";
};

double actual_position = 0.0;
double actual_velocity = 0.0;
double filtered_velocity = 0.0;
const double alpha = 0.1;
bool joint_state_received = false;
bool first_loop = true;

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  if (!msg->position.empty()) actual_position = msg->position[0];
  if (!msg->velocity.empty()) {
    actual_velocity = msg->velocity[0];
    filtered_velocity = alpha * actual_velocity + (1 - alpha) * filtered_velocity;
  }
  joint_state_received = true;
}

bool LoadTrajectoryParameters(const std::string& yaml_path, TrajectoryParams* params) {
  YAML::Node config = YAML::LoadFile(yaml_path);
  if (!config["points"] || config["points"].size() < 2) {
    ROS_ERROR("At least two path points are required in YAML.");
    return false;
  }
  for (const auto& p : config["points"]) {
    params->points.push_back(p.as<double>());
  }
  if (config["segment_time"]) params->segment_time = config["segment_time"].as<double>();
  if (config["frequency"]) {
    params->frequency = config["frequency"].as<double>();
    if (params->frequency <= 0.0) {
      ROS_WARN("Invalid frequency, using default 100 Hz");
      params->frequency = 100.0;
    }
  }
  params->dt = 1.0 / params->frequency;
  if (config["kp"]) params->kp = config["kp"].as<double>();
  if (config["kd"]) params->kd = config["kd"].as<double>();
  if (config["torque"]) params->torque = config["torque"].as<double>();
  if (config["loop"]) params->loop = config["loop"].as<bool>();

  ROS_INFO_STREAM("Loaded parameters: freq=" << params->frequency << ", dt=" << params->dt);
  return true;
}

void SaveCSVHeader(std::ofstream& csv_file) {
  csv_file << "time,expected_pos,expected_vel,actual_pos,actual_vel,error_pos,error_vel\n";
}

void PublishTrajectorySegment(const ros::Publisher& mit_pub,
                              std::ofstream* csv_latest,
                              std::ofstream& csv_all,
                              const TrajectoryParams& params,
                              double start_pos,
                              double end_pos,
                              double& global_time) {
  KDL::VelocityProfile_Spline spline;
  spline.SetProfileDuration(start_pos, 0.0, 0.0, end_pos, 0.0, 0.0, params.segment_time);

  for (double t = 0.0; t <= params.segment_time; t += params.dt) {
    double pos = spline.Pos(t);
    double vel = spline.Vel(t);
    double time_stamp = global_time + t;

    double err_pos = pos - actual_position;
    double err_vel = vel - filtered_velocity;

    if (csv_latest) {
      (*csv_latest) << time_stamp << "," << pos << "," << vel << ","
                    << actual_position << "," << filtered_velocity << ","
                    << err_pos << "," << err_vel << "\n";
    }

    csv_all << time_stamp << "," << pos << "," << vel << ","
            << actual_position << "," << filtered_velocity << ","
            << err_pos << "," << err_vel << "\n";

    dm_motor_controller::mit_control_frame cmd;
    cmd.p_des = pos;
    cmd.v_des = vel;
    cmd.kp = params.kp;
    cmd.kd = params.kd;
    cmd.t_ff = params.torque;
    mit_pub.publish(cmd);

    ros::spinOnce();
    ros::Duration(params.dt).sleep();
  }
  global_time += params.segment_time;
}

void ExecuteTrajectory(const ros::Publisher& mit_pub, const TrajectoryParams& params) {
  bool write_all_header = true;
  do {
    std::ofstream all_file(params.csv_all_path, std::ios::app);
    std::ofstream latest_file;
    std::ofstream* latest_file_ptr = nullptr;

    if (!all_file.is_open()) {
      ROS_ERROR("Failed to open all.csv file.");
      return;
    }

    if (first_loop) {
      latest_file.open(params.csv_path);
      if (!latest_file.is_open()) {
        ROS_ERROR("Failed to open latest.csv file.");
        return;
      }
      latest_file_ptr = &latest_file;
      SaveCSVHeader(latest_file);
    }

    if (write_all_header) {
      SaveCSVHeader(all_file);
      write_all_header = false;
    }

    double global_time = 0.0;
    for (size_t i = 0; i + 1 < params.points.size(); ++i) {
      PublishTrajectorySegment(mit_pub, latest_file_ptr, all_file, params,
                               params.points[i], params.points[i + 1], global_time);
    }

    if (first_loop) {
      latest_file.close();
      first_loop = false;
    }
    all_file.close();

    ROS_INFO("Trajectory execution complete. Looping: %s", params.loop ? "true" : "false");
  } while (ros::ok() && params.loop);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_logger_node");
  ros::NodeHandle nh("~");

  std::string yaml_path;
  ros::param::param<std::string>("~yaml_path", yaml_path, "config/trajectory_kdl.yaml");

  TrajectoryParams params;
  ros::param::param<std::string>("~csv_path", params.csv_path, "data/trajectory_latest.csv");
  ros::param::param<std::string>("~csv_all_path", params.csv_all_path, "data/trajectory_all.csv");

  if (!LoadTrajectoryParameters(yaml_path, &params)) return 1;

  ros::Publisher mit_pub = nh.advertise<dm_motor_controller::mit_control_frame>("/dm_motor/mit_command", 10);
  ros::Subscriber joint_sub = nh.subscribe("/dm_motor/state", 10, JointStateCallback);

  std::cout << "Press ENTER to start trajectory execution..." << std::endl;
  std::cin.get();

  ExecuteTrajectory(mit_pub, params);
  return 0;
}
