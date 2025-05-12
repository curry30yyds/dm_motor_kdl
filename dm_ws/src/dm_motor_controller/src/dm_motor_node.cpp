#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "dm_motor_controller/damiao.h"
#include "dm_motor_controller/MITCommand.h"

std::shared_ptr<damiao::Motor_Control> dm;
damiao::Motor M1(damiao::DM4310, 0x01, 0x00);

ros::Publisher joint_state_pub;
int control_mode = -1;  

void controlCallback(const std_msgs::Float64::ConstPtr& msg)
{
    double target_position = msg->data;
    ROS_INFO_STREAM("Received position command: " << target_position << " rad");

    if (control_mode != damiao::POS_VEL_MODE) {
        dm->switchControlMode(M1, damiao::POS_VEL_MODE);
        control_mode = damiao::POS_VEL_MODE;
        dm->enable(M1);
        ROS_INFO("Switched to POS_VEL_MODE.");
    }

    dm->control_pos_vel(M1, target_position, 5.0); 
    ROS_INFO("Position control command sent.");
}

void mitCallback(const dm_motor_controller::MITCommand::ConstPtr& msg)
{
    ROS_INFO("Received MIT control command.");

    if (control_mode != damiao::MIT_MODE) {
        dm->switchControlMode(M1, damiao::MIT_MODE);
        control_mode = damiao::MIT_MODE;
        dm->enable(M1);
        ROS_INFO("Switched to MIT_MODE.");
    }

    dm->control_mit(M1, msg->kp, msg->kd, msg->position, msg->velocity, msg->torque);
    ROS_INFO("MIT control command sent.");
}

void publishJointState()
{
    sensor_msgs::JointState state;
    state.header.stamp = ros::Time::now();
    state.name.push_back("motor_joint");

    double pos = M1.Get_Position();
    double vel = M1.Get_Velocity();
    double tau = M1.Get_tau();

    state.position.push_back(pos);
    state.velocity.push_back(vel);
    state.effort.push_back(tau);

    joint_state_pub.publish(state);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dm_motor_node");
    ros::NodeHandle nh;

    ROS_INFO("Opening serial port: /dev/ttyACM0 ...");
    auto serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    ROS_INFO("Serial port opened.");

    dm = std::make_shared<damiao::Motor_Control>(serial);
    dm->addMotor(&M1);
    ROS_INFO("Motor added.");

    dm->switchControlMode(M1, damiao::POS_VEL_MODE);
    dm->enable(M1);
    control_mode = damiao::POS_VEL_MODE;
    ROS_INFO("Motor enabled in POS_VEL_MODE.");

    ros::Subscriber pos_sub = nh.subscribe("dm_motor/command", 10, controlCallback);
    ros::Subscriber mit_sub = nh.subscribe("dm_motor/mit_command", 10, mitCallback);

    joint_state_pub = nh.advertise<sensor_msgs::JointState>("dm_motor/state", 10);

    ros::Rate loop_rate(50);  // Publish joint state at 50 Hz
    while (ros::ok())
    {
        publishJointState();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
