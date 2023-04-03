import rosbag
import numpy as np
from tf.transformations import euler_from_quaternion
from matplotlib import pyplot as plt




if __name__ == '__main__':
    bag = rosbag.Bag('pure_pursuit.bag')

    topics = ['/odometry/filtered', '/cmd_vel']
    info = bag.get_type_and_topic_info(topics)

    cmd_vel = np.zeros((info[1]['/cmd_vel'].message_count,2)) # linear.x, angular.z
    odom = np.zeros((info[1]['/odometry/filtered'].message_count,4)) # x, y, yaw1, yaw2

    i, j = 0, 0
    for topic, msg, t in bag.read_messages(topics=topics):
        if topic == '/odometry/filtered':
            robot_pose = msg.pose.pose.position
            robot_orientation = msg.pose.pose.orientation
            r_quaternion_list = [robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w]
            _, _, yaw1 = euler_from_quaternion([robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w])
            _, _, yaw2 = euler_from_quaternion([robot_orientation.w, robot_orientation.x, robot_orientation.y, robot_orientation.z])
            odom[i] = np.array([robot_pose.x, robot_pose.y, yaw1, yaw2])
            i += 1

        if topic == '/cmd_vel':
            cmd_vel[j] = np.array([msg.linear.x, msg.angular.z])
            j += 1

    
    fix, (ax1, ax2) = plt.subplots(1,2)
    ax1.plot(odom[:,0], label='x')
    ax1.plot(odom[:,1], label='y')
    ax1.plot(odom[:,2], label='yaw1')
    ax1.plot(odom[:,3], label='yaw2')
    ax1.legend()

    ax2.plot(cmd_vel[:,0], label='x_vel')
    ax2.plot(cmd_vel[:,1], label='curvature')
    ax2.legend()

    plt.show()
    