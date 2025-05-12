#include <ros/ros.h>
#include <dm_motor_controller/MITCommand.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle nh("~");

    ros::Publisher pub = nh.advertise<dm_motor_controller::MITCommand>("/dm_motor/mit_command", 10);
    ros::Duration(1.0).sleep();

    std::string file_path, mode;
    nh.param<std::string>("trajectory_file", file_path, "config/trajectory.yaml");
    nh.param<std::string>("mode", mode, "p");

    YAML::Node config = YAML::LoadFile(file_path);


    double frequency = config["rate"] ? config["rate"].as<double>() : 1.0;
    if (frequency <= 0.0) {
        ROS_WARN("Invalid rate in YAML, defaulting to 1.0 Hz");
        frequency = 1.0;
    }

    auto params = config["params"];
    std::vector<dm_motor_controller::MITCommand> traj;
    for (const auto& point : params) {
        if (point["mode"].as<std::string>() != mode) continue;

        dm_motor_controller::MITCommand cmd;
        cmd.kp = point["kp"].as<float>();
        cmd.kd = point["kd"].as<float>();
        cmd.position = point["position"].as<float>();
        cmd.velocity = point["velocity"].as<float>();
        cmd.torque = point["torque"].as<float>();
        traj.push_back(cmd);
    }

    if (traj.empty()) {
        ROS_WARN("No valid trajectory points found for mode '%s'", mode.c_str());
        return 0;
    }

    std::string mode_str = (mode == "p") ? "Position Control" :
                           (mode == "v") ? "Velocity Control" : "Unknown";
    ROS_INFO("Running in [%s] mode with %lu trajectory points at %.2f Hz.", mode_str.c_str(), traj.size(), frequency);

    ros::Rate rate(frequency);
    for (size_t i = 0; i < traj.size() && ros::ok(); ++i) {
        ROS_INFO("Sending point %lu/%lu: kp=%.2f, kd=%.2f, pos=%.2f, vel=%.2f, tq=%.2f",
                 i+1, traj.size(), traj[i].kp, traj[i].kd, traj[i].position,
                 traj[i].velocity, traj[i].torque);
        pub.publish(traj[i]);
        rate.sleep();
    }

    return 0;
}
