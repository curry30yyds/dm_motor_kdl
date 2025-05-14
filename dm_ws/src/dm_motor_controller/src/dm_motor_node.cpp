#include <ros/ros.h>
#include <memory>
#include <string>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include "dm_motor_controller/damiao.h"
#include "dm_motor_controller/mit_control_frame.h"

namespace dm_motor_node
{

  class MotorInterface
  {
  public:
    MotorInterface(const std::string &serial_port, int baudrate)
        : motor_(damiao::DM4310, 0x01, 0x10), has_pos_command_(false), has_mit_command_(false)
    {
      try
      {
        serial_ = std::make_shared<SerialPort>(serial_port, baudrate);
        motor_control_ = std::make_shared<damiao::Motor_Control>(serial_);
        motor_control_->addMotor(&motor_);
        motor_control_->switchControlMode(motor_, damiao::POS_VEL_MODE);
        motor_control_->enable(motor_);
        control_mode_ = damiao::POS_VEL_MODE;
        ROS_INFO("Motor initialized and enabled in POS_VEL_MODE.");
      }
      catch (const std::exception &e)
      {
        ROS_ERROR("Initialization failed: %s", e.what());
        throw;
      }
    }

    void InitROS(ros::NodeHandle &nh)
    {
      joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/JointStates", 1);
      pos_sub_ = nh.subscribe("dm_motor/command", 1, &MotorInterface::PositionCommandCallback, this);
      mit_sub_ = nh.subscribe("/JointCmds", 1, &MotorInterface::MITCommandCallback, this);
    }

    void SpinOnce()
    {
      if (mit_sub_.getNumPublishers() > 0 && control_mode_ != damiao::MIT_MODE)
      {
        motor_control_->switchControlMode(motor_, damiao::MIT_MODE);
        motor_control_->enable(motor_);
        control_mode_ = damiao::MIT_MODE;
        ROS_INFO("Auto-switched to MIT_MODE.");
      }

      if (has_pos_command_ && control_mode_ == damiao::POS_VEL_MODE)
      {
        motor_control_->control_pos_vel(motor_, target_position_, 5.0);
      }

      if (has_mit_command_ && control_mode_ == damiao::MIT_MODE)
      {
        motor_control_->control_mit(motor_,
                                    mit_cmd_.kp,
                                    mit_cmd_.kd,
                                    mit_cmd_.p_des,
                                    mit_cmd_.v_des,
                                    mit_cmd_.t_ff);
      }

      PublishJointState();
    }

  private:
    void PositionCommandCallback(const std_msgs::Float64::ConstPtr &msg)
    {
      target_position_ = msg->data;
      has_pos_command_ = true;
    }

    void MITCommandCallback(const dm_motor_controller::mit_control_frame::ConstPtr &msg)
    {
      mit_cmd_ = *msg;
      has_mit_command_ = true;
    }

    void PublishJointState()
    {
      sensor_msgs::JointState state;
      state.header.stamp = ros::Time::now();
      state.name.push_back("motor_joint");
      state.position.push_back(motor_.Get_Position());
      state.velocity.push_back(motor_.Get_Velocity());
      state.effort.push_back(motor_.Get_tau());
      joint_state_pub_.publish(state);
    }

    std::shared_ptr<SerialPort> serial_;
    std::shared_ptr<damiao::Motor_Control> motor_control_;
    damiao::Motor motor_;

    ros::Publisher joint_state_pub_;
    ros::Subscriber pos_sub_;
    ros::Subscriber mit_sub_;

    int control_mode_ = -1;
    double target_position_ = 0.0;
    bool has_pos_command_ = false;
    bool has_mit_command_ = false;
    dm_motor_controller::mit_control_frame mit_cmd_;
  };

} // namespace dm_motor_node

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dm_motor_node");
  ros::NodeHandle nh;

  try
  {
    dm_motor_node::MotorInterface motor_if("/dev/ttyACM0", B921600);
    motor_if.InitROS(nh);

    ros::Rate loop_rate(200);
    while (ros::ok())
    {
      motor_if.SpinOnce();
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  catch (...)
  {
    ROS_FATAL("Failed to initialize motor interface. Shutting down.");
    return 1;
  }

  return 0;
}
