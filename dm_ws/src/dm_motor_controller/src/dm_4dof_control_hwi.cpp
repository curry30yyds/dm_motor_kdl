#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <kdl/velocityprofile_spline.hpp>
#include <hy_hardware_interface/test_dm_4dof_state.h>
#include <hy_hardware_interface/test_dm_4dof_control.h>
#include <std_msgs/Float64MultiArray.h>
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

    class TrajectoryExecutor
    {
    public:
        TrajectoryExecutor() : joint_state_received_(false) {}

        void JointStateCallback(const hy_hardware_interface::test_dm_4dof_state::ConstPtr &msg)
        {
            latest_state_ = *msg;
            joint_state_received_ = true;
            static bool first_print = true;
            if (first_print)
            {
                ROS_INFO("Received %lu joint states.", msg->joint_states.size());
                first_print = false;
            }
        }

        void PositionArrayCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
        {
            position_sequences_ = msg->data;
        }

        void Execute(const ros::Publisher &pub, const TrajectoryParams &params)
        {
            int joint_num = latest_state_.joint_states.size();
            if (position_sequences_.size() % joint_num != 0)
            {
                ROS_ERROR("Position array size must be a multiple of joint number.");
                return;
            }
            int points_per_joint = position_sequences_.size() / joint_num;

            std::vector<std::vector<double>> joint_points(joint_num);
            for (int j = 0; j < joint_num; ++j)
            {
                for (int i = 0; i < points_per_joint; ++i)
                {
                    joint_points[j].push_back(position_sequences_[j + i * joint_num]);
                }
            }

            int executed_loops = 0;
            bool saved_csv = false;
            std::ofstream csv;
            double global_time = 0.0;
            std::vector<KDL::VelocityProfile_Spline> splines(joint_num);

            do
            {
                if (!saved_csv)
                {
                    csv.open(params.csv_path);
                    csv << "time";
                    for (int j = 0; j < joint_num; ++j)
                    {
                        csv << ",exp_pos_" << j << ",exp_vel_" << j << ",act_pos_" << j << ",act_vel_" << j;
                    }
                    csv << "\n";
                }

                size_t seg_len = joint_points[0].size() - 1;
                for (size_t seg = 0; seg < seg_len; ++seg)
                {
                    for (int j = 0; j < joint_num; ++j)
                    {
                        splines[j].SetProfileDuration(joint_points[j][seg], 0.0, 0.0,
                                                      joint_points[j][seg + 1], 0.0, 0.0,
                                                      params.segment_time);
                    }

                    for (double t = 0.0; t <= params.segment_time; t += params.dt)
                    {
                        ros::spinOnce();
                        double time_now = global_time + t;

                        hy_hardware_interface::test_dm_4dof_control cmd;
                        cmd.joint_control_frames.resize(joint_num);

                        for (int j = 0; j < joint_num; ++j)
                        {
                            double pos = splines[j].Pos(t);
                            double vel = splines[j].Vel(t);
                            double err_pos = pos - latest_state_.joint_states[j].position;
                            double err_vel = vel - latest_state_.joint_states[j].velocity;

                            cmd.joint_control_frames[j].p_des = static_cast<float>(pos);
                            cmd.joint_control_frames[j].kp = static_cast<float>(params.kp);
                            cmd.joint_control_frames[j].kd = static_cast<float>(params.kd);
                            cmd.joint_control_frames[j].t_ff = static_cast<float>(params.ff_gain * vel);

                            if (params.mode == POSITION)
                            {
                                cmd.joint_control_frames[j].v_des = 0.0f;
                                cmd.joint_control_frames[j].kd = 0.0f;
                            }
                            else
                            {
                                double v_ctrl = params.ff_gain * vel + params.pos_gain * err_pos + params.vel_gain * err_vel;
                                v_ctrl = std::max(-params.v_max, std::min(v_ctrl, params.v_max));
                                cmd.joint_control_frames[j].v_des = static_cast<float>(v_ctrl);
                            }
                        }

                        pub.publish(cmd);

                        if (!saved_csv)
                        {
                            csv << time_now;
                            for (int j = 0; j < joint_num; ++j)
                            {
                                csv << "," << splines[j].Pos(t) << "," << splines[j].Vel(t);
                                csv << "," << latest_state_.joint_states[j].position << "," << latest_state_.joint_states[j].velocity;
                            }
                            csv << "\n";
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
            ROS_INFO("Waiting for joint state and position sequence...");
            while (ros::ok() && (!joint_state_received_ || position_sequences_.empty()))
            {
                ros::spinOnce();
                loop_rate.sleep();
            }
            return joint_state_received_;
        }

    private:
        hy_hardware_interface::test_dm_4dof_state latest_state_;
        std::vector<double> position_sequences_;
        bool joint_state_received_ = false;
    };

} // namespace dm_motor_trajectory_node

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dm_4dof_control_hwi_node");
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
    ros::Subscriber pos_sub = nh.subscribe("/position", 10, &dm_motor_trajectory_node::TrajectoryExecutor::PositionArrayCallback, &executor);

    std::cout << "Press ENTER to start control..." << std::endl;
    std::cin.get();

    if (!executor.WaitForState())
        return 1;
    executor.Execute(mit_pub, params);
    return 0;
}
