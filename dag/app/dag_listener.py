# -*- coding: utf-8 -*-
import rospy
import rosbag
from dag.msg import Dag
import numpy as np

def bag_recorder(data):
    bag = rosbag.Bag('dag.bag', 'w')

    try:
        bag.write('bag', data)
    finally:
        bag.close()

def bag_reader():
    bag = rosbag.Bag('dag.bag')
    for topic, msg, t in bag.read_messages():
        print t
        # print msg
    bag.close()

def callback(data):
    print "I heard dag info: number of tasks: %d" % data.num_of_tasks 
    print data.num_of_processors
    stride1 = data.num_of_processors 
    stride0 = data.num_of_tasks 
    comp_cost = np.zeros((data.num_of_processors, data.num_of_tasks))
    for i in range(data.comp_cost.layout.dim[1].size ):
        for j in range(data.comp_cost.layout.dim[0].size):
            comp_cost[i,j] = data.comp_cost.data[i*stride1 + j]
    print comp_cost  
    
    comm_cost = np.zeros((data.num_of_tasks, data.num_of_tasks))
    for i in range(data.data.layout.dim[1].size):
        for j in range(data.data.layout.dim[0].size):
            comm_cost[i,j] = data.data.data[i*stride0 + j]
    print comm_cost

    bag_recorder(data)
    bag_reader()

def listener():
    rospy.init_node('dag_listener', anonymous=True)
    rospy.Subscriber("dag", Dag, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
