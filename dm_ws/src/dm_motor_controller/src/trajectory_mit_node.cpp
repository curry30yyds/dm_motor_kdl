#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <kdl/velocityprofile_spline.hpp>
#include "dm_motor_controller/MITCommand.h"

struct TrajectoryParams {
  std::vector<double> points;
  double segment_time = 2.0;
  double dt = 0.01;
  double kp = 10.0;
  double kd = 1.0;
  double torque = 0.0;
  bool loop = false;
  std::string csv_path = "data/trajectory_latest.csv";
  std::string csv_all_path = "data/trajectory_all.csv";
};

bool LoadTrajectoryParameters(const std::string& yaml_path,
                              TrajectoryParams* params) {
  YAML::Node config = YAML::LoadFile(yaml_path);
  if (!config["points"] || config["points"].size() < 2) {
    ROS_ERROR("At least two path points are required in YAML.");
    return false;
  }

  for (const auto& p : config["points"]) {
    params->points.push_back(p.as<double>());
  }

  if (config["segment_time"]) {
    params->segment_time = config["segment_time"].as<double>();
  }
  if (config["dt"]) {
    params->dt = config["dt"].as<double>();
  }
  if (config["kp"]) {
    params->kp = config["kp"].as<double>();
  }
  if (config["kd"]) {
    params->kd = config["kd"].as<double>();
  }
  if (config["torque"]) {
    params->torque = config["torque"].as<double>();
  }
  if (config["loop"]) {
    params->loop = config["loop"].as<bool>();
  }

  return true;
}

void SaveCSVHeader(std::ofstream& csv_file) {
  csv_file << "time,pos,vel\n";
}

void PublishTrajectorySegment(const ros::Publisher& mit_pub,
                              std::ofstream& csv_latest,
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

    csv_latest << time_stamp << "," << pos << "," << vel << "\n";
    csv_all << time_stamp << "," << pos << "," << vel << "\n";

    ROS_INFO_STREAM("t=" << time_stamp << " pos=" << pos << " vel=" << vel);

    dm_motor_controller::MITCommand cmd;
    cmd.position = pos;
    cmd.velocity = vel;
    cmd.kp = params.kp;
    cmd.kd = params.kd;
    cmd.torque = params.torque;
    mit_pub.publish(cmd);

    ros::Duration(params.dt).sleep();
  }

  global_time += params.segment_time;
}

void ExecuteTrajectory(const ros::Publisher& mit_pub, const TrajectoryParams& params) {
  bool write_all_header = true;

  do {
    std::ofstream latest_file(params.csv_path);  // overwrite each loop
    std::ofstream all_file(params.csv_all_path, std::ios::app);  // append

    if (!latest_file.is_open() || !all_file.is_open()) {
      ROS_ERROR("Failed to open CSV files.");
      return;
    }

    SaveCSVHeader(latest_file);
    if (write_all_header) {
      SaveCSVHeader(all_file);
      write_all_header = false;
    }

    double global_time = 0.0;
    for (size_t i = 0; i + 1 < params.points.size(); ++i) {
      PublishTrajectorySegment(mit_pub, latest_file, all_file, params,
                               params.points[i], params.points[i + 1], global_time);
    }

    latest_file.close();
    all_file.close();

    ROS_INFO("Trajectory execution complete. Looping: %s", params.loop ? "true" : "false");
  } while (ros::ok() && params.loop);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_mit_node");
  ros::NodeHandle nh("~");

  std::string yaml_path;
  ros::param::param<std::string>("~yaml_path", yaml_path, "config/trajectory_kdl.yaml");

  TrajectoryParams params;
  ros::param::param<std::string>("~csv_path", params.csv_path, "data/trajectory_latest.csv");
  ros::param::param<std::string>("~csv_all_path", params.csv_all_path, "data/trajectory_all.csv");

  if (!LoadTrajectoryParameters(yaml_path, &params)) {
    return 1;
  }

  ros::Publisher mit_pub = nh.advertise<dm_motor_controller::MITCommand>(
      "/dm_motor/mit_command", 10);

  std::cout << "Press ENTER to start trajectory execution..." << std::endl;
  std::cin.get();  // Wait for keypress

  ExecuteTrajectory(mit_pub, params);

  return 0;
}
