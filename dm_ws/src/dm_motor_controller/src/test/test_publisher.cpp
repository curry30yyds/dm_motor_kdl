#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <hy_hardware_interface/test_dm_4dof_state.h>
#include <hy_hardware_interface/joint_state.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_joint_and_position_publisher");
    ros::NodeHandle nh;

    ros::Publisher joint_pub = nh.advertise<hy_hardware_interface::test_dm_4dof_state>("/JointStates", 10);
    ros::Publisher pos_pub = nh.advertise<std_msgs::Float64MultiArray>("/position", 1);

    ros::Rate loop_rate(200); // 1Hz for simplicity

    // 构造 position 数据，每个关节 4 个点，共 16 个数据
    std_msgs::Float64MultiArray position_msg;
    position_msg.layout.dim.resize(1);
    position_msg.layout.dim[0].label = "joint";
    position_msg.layout.dim[0].size = 16;
    position_msg.layout.dim[0].stride = 16;
    position_msg.layout.data_offset = 0;

    position_msg.data = {
        0.0, 0.5, 1.0, 1.5,   // Joint 0
        -1.0, -0.5, 0.0, 0.5, // Joint 1  0.75
        1.0, 0.5, 0.0, -0.5,  // Joint 2
        0.0, -0.5, -1.0, -1.5 // Joint 3
    };

    while (ros::ok())
    {
        // 发布 JointStates 模拟关节当前状态
        hy_hardware_interface::test_dm_4dof_state state_msg;
        state_msg.header.stamp = ros::Time::now();

        for (int i = 0; i < 4; ++i)
        {
            // + i * 0.1
            hy_hardware_interface::joint_state joint;
            joint.position = 5.0;
            joint.velocity = 0.0;
            joint.torque = 0.0;
            joint.temperature = 0;
            joint.current = 0;
            joint.voltage = 0;
            joint.running_state = 1;
            joint.error_type = 0;
            state_msg.joint_states.push_back(joint);
        }

        joint_pub.publish(state_msg);
        pos_pub.publish(position_msg);

        ROS_INFO("Published test JointStates and Position.");

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
