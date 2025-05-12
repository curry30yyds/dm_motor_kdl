#include <ros/ros.h>
#include <dm_motor_controller/MITCommand.h>
#include <sensor_msgs/JointState.h>
#include <cmath>

double current_position = 0.0;
bool joint_state_received = false;
bool zeroing_complete = false;

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  if (!msg->position.empty()) {
    current_position = msg->position[0];
    joint_state_received = true;
    ROS_INFO_STREAM("[DEBUG] Received pos=" << current_position);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "motor_zeroing_node");
  ros::NodeHandle nh;

  ros::Subscriber joint_state_sub =
      nh.subscribe("dm_motor/state", 10, JointStateCallback);
  ros::Publisher mit_pub =
      nh.advertise<dm_motor_controller::MITCommand>("/dm_motor/mit_command", 10);

  ros::Rate loop_rate(100);  // 100 Hz

  ROS_INFO("Waiting for joint state update...");
  while (ros::ok() && !joint_state_received) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO_STREAM("Starting zeroing from pos=" << current_position);

  while (ros::ok() && !zeroing_complete) {
    ros::spinOnce();

    dm_motor_controller::MITCommand cmd;
    cmd.kp = 1.0;
    cmd.kd = 0.1;
    cmd.torque = 0.0;
    cmd.position = 0.0;

    double error = current_position - 0.0;
    if (std::abs(error) > 0.001) {
      cmd.velocity = -0.05 * (error > 0 ? 1 : -1);
      mit_pub.publish(cmd);
      ROS_INFO_STREAM("Zeroing... pos=" << current_position
                      << " vel_cmd=" << cmd.velocity);
    } else {
      cmd.velocity = 0.0;
      mit_pub.publish(cmd);
      ROS_INFO("Zeroing complete. Holding position at 0.");
      zeroing_complete = true;
    }

    loop_rate.sleep();
  }

  ros::spin();
  return 0;
}