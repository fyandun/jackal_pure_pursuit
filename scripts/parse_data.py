import rosbag
import numpy as np
from tf.transformations import euler_from_quaternion
from matplotlib import pyplot as plt
import sys

def cmd_vel_to_arr(msg):
    return np.array([
        msg.linear.x,
        msg.linear.y,
        msg.linear.z,
        msg.angular.x,
        msg.angular.y,
        msg.angular.z
    ])

def odometry_to_arr(msg):
    return np.array([
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w,
        msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.linear.z,
        msg.twist.twist.angular.x,
        msg.twist.twist.angular.y,
        msg.twist.twist.angular.z
    ])

def gps_to_arr(msg):
    return np.array([
        msg.latitude,
        msg.longitude,
        msg.altitude
    ])

def imu_to_arr(msg):
    return np.array([
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w,
        msg.angular_velocity.x,
        msg.angular_velocity.y,
        msg.angular_velocity.z,
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z
    ])

def passthrough(msg):
    return msg

extractors = {
    '/cmd_vel': cmd_vel_to_arr,
    '/odometry/filtered': odometry_to_arr,
    '/odom':odometry_to_arr,
    '/gps/rtkfix': odometry_to_arr,
    '/gps/fix': gps_to_arr,
    '/imu/data': imu_to_arr,
    '/imu/data_raw': imu_to_arr
}

def read_bagfile(bagfile,topics=None):
    bag = rosbag.Bag(bagfile)
    topics = list(bag.get_type_and_topic_info()[1].keys()) if topics is None else topics
    data = {k: [] for k in topics}
    for topic, msg, t in bag.read_messages(topics=topics):
        formatter = extractors.get(topic,passthrough)
        data[topic].append(formatter(msg))
    for k,v in data.items():
        if k in extractors and len(v) > 0:
            data[k] = np.stack(v)
    return data

def plot_odom_vs_waypoints(bagfile, waypoints):
    data = read_bagfile(bagfile,topics=['/odometry/filtered','/gps/rtkfix'])
    w = np.loadtxt(waypoints, delimiter=',')
    _, ax = plt.subplots(1,1)
    plt.scatter(w[:,0],w[:,1],color='r',marker='x',label='waypoints')
    ax.plot(data['/gps/rtkfix'][:,0],data['/gps/rtkfix'][:,1],label='GPS')
    # ax.plot(data['/odometry/filtered'][:,0],data['/odometry/filtered'][:,1],label='EKF')
    plt.legend()
    plt.ion()
    plt.show()

def plot_odom_vs_waypoints_data(data, wf):
    w = np.loadtxt(wf, delimiter=',')
    _, ax = plt.subplots(1,1)
    plt.scatter(w[:,0],w[:,1],color='r',marker='x',label='waypoints')
    ax.plot(data['/gps/rtkfix'][:,0],data['/gps/rtkfix'][:,1],label='GPS')
    # ax.plot(data['/odom'][:,0],data['/odom'][:,1],label='GPS')
    # ax.plot(data['/odometry/filtered'][:,0],data['/odometry/filtered'][:,1],label='EKF')
    plt.legend()
    plt.ion()
    plt.show()

def hacky_cte(data,w):
    n = 0
    total_cte = 0
    for i, waypoint in enumerate(w):
        # if i < 3 or (i > 18 and i < 35) or i > 50:
        # if i > 22:
        #     continue
        total_cte += np.min(np.linalg.norm(data-waypoint,axis=1))
        n += 1
    return total_cte/n

def clean_odom(odom):
    cleaned_odom = odom[:,:2].copy()
    mask = np.where(np.linalg.norm(np.diff(odom[:,:2],axis=0),axis=1) > 5)[0]
    mask = mask[::2]
    cleaned_odom[mask+1] = odom[mask]
    return cleaned_odom

if __name__ == '__main__':

    bf = sys.argv[1]
    wf = sys.argv[2]
    plot_odom_vs_waypoints(bf,wf)
    # bag = rosbag.Bag('../data/pure_pursuit.bag')
    bf = '../data/2023-04-14-Florida/2023-04-14_l1p5_odomfilt_offaxis_loop_socat_driver_shifted_2.bag'
    wf = '../data/2023-04-14-Florida/2023-04-14_misaligned_loop_socat_driver_shifted.txt'

    data = read_bagfile(bf, topics=['/odometry/filtered','/gps/rtkfix'])
    plot_odom_vs_waypoints(bf,wf)
    # topics = ['/odometry/filtered', '/cmd_vel']
    # info = bag.get_type_and_topic_info(topics)

    # cmd_vel = np.zeros((info[1]['/cmd_vel'].message_count,2)) # linear.x, angular.z
    # odom = np.zeros((info[1]['/odometry/filtered'].message_count,4)) # x, y, yaw1, yaw2

    # i, j = 0, 0
    # for topic, msg, t in bag.read_messages(topics=topics):
    #     if topic == '/odometry/filtered':
    #         robot_pose = msg.pose.pose.position
    #         robot_orientation = msg.pose.pose.orientation
    #         r_quaternion_list = [robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w]
    #         _, _, yaw1 = euler_from_quaternion([robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w])
    #         _, _, yaw2 = euler_from_quaternion([robot_orientation.w, robot_orientation.x, robot_orientation.y, robot_orientation.z])
    #         odom[i] = np.array([robot_pose.x, robot_pose.y, yaw1, yaw2])
    #         i += 1

    #     if topic == '/cmd_vel':
    #         cmd_vel[j] = np.array([msg.linear.x, msg.angular.z])
    #         j += 1

    
    # fig, (ax1, ax2) = plt.subplots(1,2)
    # ax1.plot(odom[:,0], label='x')
    # ax1.plot(odom[:,1], label='y')
    # ax1.plot(odom[:,2], label='yaw1')
    # ax1.plot(odom[:,3], label='yaw2')
    # ax1.legend()

    # ax2.plot(cmd_vel[:,0], label='x_vel')
    # ax2.plot(cmd_vel[:,1], label='curvature')
    # ax2.legend()

    # plt.show()

    def extract_odom(bagfile):
        bag = rosbag.Bag(bagfile)
        info = bag.get_type_and_topic_info('/odometry/filtered')
        odom = np.zeros((info[1]['/odometry/filtered'].message_count,3)) # x, y, yaw
        i = 0
        for topic, msg, t in bag.read_messages(topics='/odometry/filtered'):
            robot_pose = msg.pose.pose.position
            robot_orientation = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w])
            odom[i] = np.array([robot_pose.x, robot_pose.y, yaw])
            i += 1

        return odom

    
    # code up a pure pursuit line follower

    ######
    #w = np.loadtxt('../data/apr4_manual.txt',delimiter=',')
    # odom1 = extract_odom('../data/apr4_oval_3_manual_waypoints_l1p5.bag')
    #odom2 = extract_odom('../data/apr4_oval_3_manual_waypoints.bag')
    # odom3 = extract_odom('../data/apr4_oval_4_manual_waypoints_l1p5.bag')
    #odom4 = extract_odom('../data/apr4_oval_5_manual_waypoints_l1p5.bag')

    #odom5 = extract_odom('../data/pure_pursuit.bag')
    #fig, ax2 = plt.subplots(1,1)
    #ax2.plot(odom5[:,0],odom5[:,1])
    #plt.show()


    #fig1, ax1 = plt.subplots(1,1)
    #ax1.scatter(w[:,0],w[:,1],marker='x',color='r')
    ## ax1.plot(odom1[:,0],odom1[:,1],label='1')
    #ax1.plot(odom2[:,0],odom2[:,1],label='l=1')
    # for row in odom2:
    #     ax1.arrow(row[0],row[1],np.cos(row[2]),np.sin(row[2]))
    # ax1.plot(odom3[:,0],odom3[:,1],label='3')
    #ax1.plot(odom4[:,0],odom4[:,1],label='l=1.5')
    # ax1.arrow(odom4[:,0],odom4[:,1],np.cos(odom4[:,2]),np.sin(odom4[:,2]))
    #plt.legend()
    #plt.show()

#w2 = np.loadtxt('../data/apr3.txt',delimiter=',')
#fig1, ax3 = plt.subplots(1,1)
#ax1.scatter(w2[:,0],w2[:,1],marker='x',color='r')
    
    
