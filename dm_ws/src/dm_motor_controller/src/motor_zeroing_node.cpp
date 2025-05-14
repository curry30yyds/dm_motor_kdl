#include <ros/ros.h>
#include <memory>
#include <string>
#include <vector>
#include <hy_hardware_interface/test_dm_4dof_state.h>
#include <hy_hardware_interface/test_dm_4dof_control.h>
#include <kdl/path_line.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/frames.hpp>
#include <kdl/rotational_interpolation_sa.hpp>

namespace dm_motor_zeroing_node
{

  class ZeroingController
  {
  public:
    ZeroingController(ros::NodeHandle &nh)
        : joint_state_received_(false), zeroing_complete_(false)
    {
      joint_sub_ = nh.subscribe("/JointStates", 1, &ZeroingController::JointStateCallback, this);
      mit_pub_ = nh.advertise<hy_hardware_interface::test_dm_4dof_control>("/JointCmds", 1);
    }

    void Run()
    {
      ros::Rate loop_rate(200);
      ROS_INFO("Waiting for joint states...");
      while (ros::ok() && !joint_state_received_)
      {
        ros::spinOnce();
        loop_rate.sleep();
      }

      if (current_positions_.empty())
      {
        ROS_ERROR("No joint states received.");
        return;
      }

      ROS_INFO("Generating zeroing trajectories...");
      trajectories_.clear();
      for (double pos : current_positions_)
      {
        KDL::Frame f_start(KDL::Vector(pos, 0.0, 0.0));
        KDL::Frame f_end(KDL::Vector(0.0, 0.0, 0.0));
        auto *orient = new KDL::RotationalInterpolation_SingleAxis();
        auto *path = new KDL::Path_Line(f_start, f_end, orient, 0.1);
        auto *profile = new KDL::VelocityProfile_Trap(0.5, 1.0);
        profile->SetProfile(0.0, path->PathLength());
        trajectories_.emplace_back(new KDL::Trajectory_Segment(path, profile));
      }

      double duration = trajectories_.front()->Duration();
      ROS_INFO_STREAM("Zeroing duration: " << duration << "s");

      double t = 0.0;
      ros::Rate rate(200);
      while (ros::ok() && t <= duration)
      {
        hy_hardware_interface::test_dm_4dof_control cmd;
        cmd.joint_control_frames.resize(trajectories_.size());

        for (size_t i = 0; i < trajectories_.size(); ++i)
        {
          KDL::Frame pose = trajectories_[i]->Pos(t);
          KDL::Twist vel = trajectories_[i]->Vel(t);

          cmd.joint_control_frames[i].p_des = pose.p.x();
          cmd.joint_control_frames[i].v_des = vel.vel.x();
          cmd.joint_control_frames[i].kp = 1.0;
          cmd.joint_control_frames[i].kd = 0.1;
          cmd.joint_control_frames[i].t_ff = 0.0;
        }

        mit_pub_.publish(cmd);
        ros::spinOnce();
        rate.sleep();
        t += 0.005;
      }

      // hy_hardware_interface::test_dm_4dof_control hold_cmd;
      // hold_cmd.joint_control_frames.resize(trajectories_.size());
      // for (auto &frame : hold_cmd.joint_control_frames)
      // {
      //   frame.p_des = 0.0;
      //   frame.v_des = 0.0;
      //   frame.kp = 1.0;
      //   frame.kd = 0.1;
      //   frame.t_ff = 0.0;
      // }
      // mit_pub_.publish(hold_cmd);
      // ROS_INFO("Zeroing complete. Motors holding at origin.");
    }

  private:
    void JointStateCallback(const hy_hardware_interface::test_dm_4dof_state::ConstPtr &msg)
    {
      if (msg->joint_states.empty())
        return;
      if (!joint_state_received_)
      {
        current_positions_.resize(msg->joint_states.size());
        trajectories_.resize(msg->joint_states.size());
      }
      for (size_t i = 0; i < msg->joint_states.size(); ++i)
      {
        current_positions_[i] = msg->joint_states[i].position;
      }
      joint_state_received_ = true;
    }

    ros::Subscriber joint_sub_;
    ros::Publisher mit_pub_;
    std::vector<double> current_positions_;
    std::vector<std::unique_ptr<KDL::Trajectory_Segment>> trajectories_;
    bool joint_state_received_;
    bool zeroing_complete_;
  };

} // namespace dm_motor_zeroing_node

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_zeroing_node");
  ros::NodeHandle nh;
  dm_motor_zeroing_node::ZeroingController zeroing(nh);
  zeroing.Run();
  return 0;
}
