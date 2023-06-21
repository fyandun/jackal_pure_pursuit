import numpy as np
from matplotlib import pyplot as plt

import rospy
from nav_msgs.msg import Odometry

coords = []

class Listener:
    def __init__(self):
        self.robot_position = None
        self.ignore = False
        self.sub = rospy.Subscriber('/odometry/filtered', Odometry, self.callback)

    def callback(self, data):
        if self.ignore:
            return

        self.robot_position = [data.pose.pose.position.x, data.pose.pose.position.y]
        coords.append(self.robot_position)
        self.ignore = True

def onclick(event):
    global coords
    ix, iy = event.xdata, event.ydata
    coords = np.vstack((coords, [ix, iy]))
    ax1.clear()
    ax1.plot(coords[:,0],coords[:,1])
    ax1.set_xlim(listener.robot_position[0] - field_size, listener.robot_position[0] + field_size)
    ax1.set_ylim(listener.robot_position[1] - field_size, listener.robot_position[1] + field_size)
    print('All waypoints: ', coords)

if __name__ == '__main__':
    coords = []
    listener = Listener()
    rospy.init_node('collector',anonymous=True)
    while not listener.ignore:
        pass
    print('start position: x:', coords)
    field_size = 20 # meters
    fig, ax1 = plt.subplots(1,1)
    ax1.set_xlim(listener.robot_position[0]-field_size, listener.robot_position[0]+field_size)
    ax1.set_ylim(listener.robot_position[1]-field_size, listener.robot_position[1]+field_size)
    ax1.scatter(coords[0][0],coords[0][1])
    ax1.grid(axis='both')
    cid = fig.canvas.mpl_connect('button_press_event', onclick)
    plt.ion()
    plt.show()

    waypt_file = 'apr4.txt'
    save_and_exit = input()
    if save_and_exit == 'y':
        coords = np.stack(coords)
        np.savetxt(waypt_file, coords, delimiter=',')

    print('saved file. exiting...')


