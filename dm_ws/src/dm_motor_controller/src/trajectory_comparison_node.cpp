#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <kdl/velocityprofile_spline.hpp>
#include <hy_hardware_interface/test_dm_4dof_state.h>
#include <hy_hardware_interface/test_dm_4dof_control.h>
#include <algorithm>

namespace dm_motor_trajectory_node
{

  enum ControlMode
  {
    POSITION,
    VELOCITY
  };

  struct TrajectoryParams
  {
    int loop_count = -1;
    std::vector<double> points;
    double segment_time = 2.0;
    double frequency = 200.0;
    double dt = 0.005;
    double kp = 10.0;
    double kd = 1.0;
    double torque = 0.0;
    double pos_gain = 1.0;
    double vel_gain = 1.0;
    double ff_gain = 1.0;
    double v_max = 5.0;
    bool loop = false;
    std::string csv_path = "data/trajectory_comparison.csv";
    ControlMode mode = VELOCITY;
  };

  bool LoadParams(const std::string &yaml_path, TrajectoryParams &params)
  {
    YAML::Node config = YAML::LoadFile(yaml_path);
    if (!config["points"] || config["points"].size() < 2)
    {
      ROS_ERROR("At least two points required.");
      return false;
    }
    for (const auto &p : config["points"])
    {
      params.points.push_back(p.as<double>());
    }
    if (config["segment_time"])
      params.segment_time = config["segment_time"].as<double>();
    if (config["frequency"])
      params.frequency = config["frequency"].as<double>();
    params.dt = 1.0 / params.frequency;
    if (config["kp"])
      params.kp = config["kp"].as<double>();
    if (config["kd"])
      params.kd = config["kd"].as<double>();
    if (config["torque"])
      params.torque = config["torque"].as<double>();
    if (config["pos_gain"])
      params.pos_gain = config["pos_gain"].as<double>();
    if (config["vel_gain"])
      params.vel_gain = config["vel_gain"].as<double>();
    if (config["ff_gain"])
      params.ff_gain = config["ff_gain"].as<double>();
    if (config["v_max"])
      params.v_max = config["v_max"].as<double>();
    if (config["loop"])
      params.loop = config["loop"].as<bool>();
    if (config["control_mode"])
    {
      std::string mode_str = config["control_mode"].as<std::string>();
      if (mode_str == "position")
        params.mode = POSITION;
      else if (mode_str == "velocity")
        params.mode = VELOCITY;
    }
    if (config["loop_count"])
      params.loop_count = config["loop_count"].as<int>();
    return true;
  }

  class PIDController
  {
  public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

    void reset()
    {
      prev_error_ = 0.0;
      integral_ = 0.0;
    }

    double compute(double error, double dt)
    {
      integral_ += error * dt;
      double derivative = (error - prev_error_) / dt;
      prev_error_ = error;
      return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

  private:
    double kp_, ki_, kd_;
    double prev_error_;
    double integral_;
  };

  class TrajectoryExecutor
  {
  public:
    TrajectoryExecutor() : actual_position_(0.0), actual_velocity_(0.0), filtered_velocity_(0.0), joint_state_received_(false) {}

    void JointStateCallback(const hy_hardware_interface::test_dm_4dof_state::ConstPtr &msg)
    {
      if (!msg->joint_states.empty())
      {
        actual_position_ = msg->joint_states[0].position;
        actual_velocity_ = msg->joint_states[0].velocity;
        filtered_velocity_ = alpha_ * actual_velocity_ + (1 - alpha_) * filtered_velocity_;
        joint_state_received_ = true;
      }
    }

    void Execute(const ros::Publisher &pub, const TrajectoryParams &params)
    {
      int executed_loops = 0;
      bool saved_csv = false;
      std::ofstream csv;
      double global_time = 0.0;
      PIDController pid(params.kp, 0.0, 0.0);

      do
      {
        if (!saved_csv)
        {
          csv.open(params.csv_path);
          csv << "time,expected_pos,expected_vel,actual_pos,actual_vel,error_pos,error_vel\n";
        }

        for (size_t i = 0; i + 1 < params.points.size(); ++i)
        {
          KDL::VelocityProfile_Spline spline;
          spline.SetProfileDuration(params.points[i], 0.0, 0.0, params.points[i + 1], 0.0, 0.0, params.segment_time);

          for (double t = 0.0; t <= params.segment_time; t += params.dt)
          {
            ros::spinOnce();

            double pos = spline.Pos(t);
            double vel = spline.Vel(t);
            double time_now = global_time + t;
            double err_pos = pos - actual_position_;
            double err_vel = vel - filtered_velocity_;

            hy_hardware_interface::test_dm_4dof_control cmd;
            cmd.joint_control_frames.resize(1);
            cmd.joint_control_frames[0].p_des = static_cast<float>(pos);
            cmd.joint_control_frames[0].kp = static_cast<float>(params.kp);
            cmd.joint_control_frames[0].kd = static_cast<float>(params.kd);
            cmd.joint_control_frames[0].t_ff = static_cast<float>(params.ff_gain * vel);

            if (params.mode == POSITION)
            {
              cmd.joint_control_frames[0].v_des = 0.0f;
              cmd.joint_control_frames[0].kd = 0.0f;
            }
            else
            {
              double v_ctrl = params.ff_gain * vel + params.pos_gain * err_pos + params.vel_gain * err_vel;
              v_ctrl = std::max(-params.v_max, std::min(v_ctrl, params.v_max));
              cmd.joint_control_frames[0].v_des = static_cast<float>(v_ctrl);
            }

            pub.publish(cmd);

            if (!saved_csv)
            {
              csv << time_now << "," << pos << "," << vel << "," << actual_position_ << "," << filtered_velocity_ << ","
                  << err_pos << "," << err_vel << "\n";
            }

            ros::Duration(params.dt).sleep();
          }
          global_time += params.segment_time;
        }

        if (!saved_csv)
        {
          csv.close();
          saved_csv = true;
        }

        ++executed_loops;
        ROS_INFO("Loop %d complete. Mode: %s", executed_loops, params.mode == POSITION ? "POSITION" : "VELOCITY");

      } while (ros::ok() && params.loop && (params.loop_count < 0 || executed_loops < params.loop_count));
    }

    bool WaitForState()
    {
      ros::Rate loop_rate(100);
      ROS_INFO("Waiting for joint state...");
      while (ros::ok() && !joint_state_received_)
      {
        ros::spinOnce();
        loop_rate.sleep();
      }
      return joint_state_received_;
    }

  private:
    double actual_position_;
    double actual_velocity_;
    double filtered_velocity_;
    bool joint_state_received_;
    const double alpha_ = 0.1;
  };

} // namespace dm_motor_trajectory_node

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_comparison_node");
  ros::NodeHandle nh("~");

  dm_motor_trajectory_node::TrajectoryParams params;
  std::string yaml_path;
  ros::param::param<std::string>("~yaml_path", yaml_path, "config/trajectory_kdl.yaml");
  ros::param::param<std::string>("~csv_path", params.csv_path, "data/trajectory_comparison.csv");

  if (!dm_motor_trajectory_node::LoadParams(yaml_path, params))
    return 1;

  ros::Publisher mit_pub = nh.advertise<hy_hardware_interface::test_dm_4dof_control>("/JointCmds", 10);
  dm_motor_trajectory_node::TrajectoryExecutor executor;
  ros::Subscriber joint_sub = nh.subscribe("/JointStates", 10, &dm_motor_trajectory_node::TrajectoryExecutor::JointStateCallback, &executor);

  std::cout << "Press ENTER to start control mode: " << (params.mode == dm_motor_trajectory_node::POSITION ? "POSITION" : "VELOCITY") << std::endl;
  std::cin.get();

  if (!executor.WaitForState())
    return 1;
  executor.Execute(mit_pub, params);
  return 0;
}
