#include <ros/ros.h>
#include <dm_motor_controller/mit_control_frame.h>
#include <kdl/path_line.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/frames.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/utilities/error.h>
#include <kdl/utilities/utility.h>

KDL::Trajectory_Segment* generate_trajectory(double start, double end, double v_max, double a_max) {
    ROS_INFO("Generating trajectory from %.3f to %.3f", start, end);

    KDL::Frame f_start(KDL::Vector(start, 0, 0));
    KDL::Frame f_end(KDL::Vector(end, 0, 0));
    KDL::RotationalInterpolation* rot_interp = new KDL::RotationalInterpolation_SingleAxis();
    KDL::Path_Line* path = new KDL::Path_Line(f_start, f_end, rot_interp, 1e-6, false);

    double path_length = path->PathLength();
    if (path_length <= 0) {
        ROS_ERROR("Path length is invalid: %.3f", path_length);
        delete path;
        return nullptr;
    }

    KDL::VelocityProfile_Trap* profile = new KDL::VelocityProfile_Trap(v_max, a_max);
    profile->SetProfile(0.0, path_length);

    return new KDL::Trajectory_Segment(path, profile);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "kdl_trajectory_generator");
    ros::NodeHandle nh("~");

    std::string yaml_path;
    nh.param<std::string>("config_file", yaml_path, "config/trajectory_config.yaml");
    YAML::Node config = YAML::LoadFile(yaml_path);
    auto traj = config["trajectory"];

    double theta_start = traj["theta_start"].as<double>();
    double theta_end   = traj["theta_end"].as<double>();
    double kp = traj["kp"].as<double>();
    double kd = traj["kd"].as<double>();
    double v_max = traj["v_max"].as<double>();
    double a_max = traj["a_max"].as<double>();
    double freq  = traj["frequency"].as<double>();
    bool loop = traj["loop"].as<bool>();
    bool save_csv = traj["save_csv"].as<bool>();
    std::string csv_path = traj["csv_path"].as<std::string>();

    ros::Publisher pub = nh.advertise<dm_motor_controller::mit_control_frame>("/dm_motor/mit_command", 10);
    ros::Duration(1.0).sleep();

    std::ofstream csv_file;
    if (save_csv) {
        csv_file.open(csv_path);
        csv_file << "t,position,velocity\n";
    }

    ros::Rate rate(freq);
    double dt = 1.0 / freq;

    bool forward = true;
    while (ros::ok()) {
        double start = forward ? theta_start : theta_end;
        double end   = forward ? theta_end   : theta_start;

        KDL::Trajectory_Segment* segment = generate_trajectory(start, end, v_max, a_max);
        if (segment == nullptr) {
            ROS_ERROR("Trajectory generation failed, skipping iteration.");
            break;
        }

        double total_time = segment->Duration();
        ROS_INFO("Trajectory duration: %.3f seconds", total_time);

        double t = 0.0;
        while (ros::ok() && t <= total_time) {
            KDL::Frame pos_frame = segment->Pos(t);
            KDL::Twist vel_twist = segment->Vel(t);

            double pos = pos_frame.p.x();
            double vel = vel_twist.vel.x();

            dm_motor_controller::mit_control_frame cmd;
            cmd.p_des = pos;
            cmd.v_des = vel;
            cmd.kp = kp;
            cmd.kd = kd;
            cmd.t_ff = 0.0;

            pub.publish(cmd);

            if (save_csv) {
                csv_file << t << "," << pos << "," << vel << "\n";
            }

            rate.sleep();
            t += dt;
        }

        delete segment;

        if (!loop) break;
        forward = !forward;
    }

    if (save_csv) {
        csv_file.close();
    }

    return 0;
}
