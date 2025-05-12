#!/usr/bin/env python3
import rospy
import yaml
from dm_motor_controller.msg import MITCommand

def main():
    rospy.init_node('trajectory_publisher')

    pub = rospy.Publisher('/dm_motor/mit_command', MITCommand, queue_size=10)
    rospy.sleep(1.0)

    path = rospy.get_param('~trajectory_file', 'config/trajectory.yaml')
    target_mode = rospy.get_param('~mode', 'p')  # 'p' for position, 'v' for velocity

    with open(path, 'r') as f:
        traj_data = yaml.safe_load(f)

    filtered = [p for p in traj_data['params'] if p['mode'] == target_mode]
    if not filtered:
        rospy.logwarn("No points found for mode '%s'", target_mode)
        return

    rate = rospy.Rate(200)  # default 200hz
    for point in filtered:
        cmd = MITCommand()
        cmd.kp = point['kp']
        cmd.kd = point['kd']
        cmd.position = point['position']
        cmd.velocity = point['velocity']
        cmd.torque = point['torque']
        rospy.loginfo("Mode [%s] → Sending: %s", target_mode, cmd)
        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    main()
