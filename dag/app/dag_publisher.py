#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cpop
from dag.msg import Dag
import rospy
from std_msgs.msg import String
import create_input
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Int32MultiArray
from numpy import matrix

def talker():
    pub = rospy.Publisher('dag', Dag, queue_size=10)
    rospy.init_node('dag_publisher', anonymous=True)
    pub_rate = rospy.Rate(0.5)   #Hz

    dag = Dag()
    dag.num_of_tasks, dag.num_of_processors, comp_cost, rate, comm_cost = create_input.init('../tgff/input_0.tgff')
    
    dag.comp_cost.layout.dim.append(MultiArrayDimension())
    dag.comp_cost.layout.dim.append(MultiArrayDimension())
    dag.comp_cost.layout.dim[0].size = dag.num_of_tasks 
    dag.comp_cost.layout.dim[1].size = dag.num_of_processors 
    dag.comp_cost.layout.dim[0].stride = dag.num_of_tasks * dag.num_of_processors
    dag.comp_cost.layout.dim[1].stride = dag.num_of_processors 
    dag.comp_cost.layout.data_offset = 0

    stride0 = dag.num_of_tasks * dag.num_of_processors 
    stride1 = dag.num_of_processors 
    offset = dag.comp_cost.layout.data_offset
    
    print 'cost in comp_cost'
    for i in range(dag.num_of_tasks):
        print 'line %d in comp_cost' % i
        cost = comp_cost[i]
        print cost
        for j in range(dag.num_of_processors):
            dag.comp_cost.data.insert(offset + stride1*i + j, cost[j])
    print 'dag.comp_cost.data:'
    print dag.comp_cost.data

    # add communication matrix
    dag.data.layout.dim.append(MultiArrayDimension())
    dag.data.layout.dim.append(MultiArrayDimension())
    dag.data.layout.dim[0].size = dag.num_of_tasks
    dag.data.layout.dim[1].size = dag.num_of_processors
    dag.data.layout.dim[0].stride = dag.num_of_tasks * dag.num_of_processors
    dag.data.layout.dim[1].stride = dag.num_of_processors 
    dag.data.layout.data_offset = 0

    for i in range(dag.num_of_tasks):
        for j in range(dag.num_of_processors):
            dag.data.data.insert(dag.data.layout.dim[1].stride * i + j, comm_cost[i][j])
        print dag.data.data

    # add rate matrix
    dag.rate.layout.dim.append(MultiArrayDimension())
    dag.rate.layout.dim.append(MultiArrayDimension())
    dag.rate.layout.dim[0].size = dag.num_of_processors
    dag.rate.layout.dim[1].size = dag.num_of_processors
    dag.rate.layout.dim[0].stride = dag.num_of_processors * dag.num_of_processors
    dag.rate.layout.dim[1].stride = dag.num_of_processors
    
    for i in range(dag.num_of_processors):
        for j in range(dag.num_of_processors):
            dag.rate.data.insert(dag.rate.layout.dim[1].stride * i + j, rate[i][j])
        print dag.rate.data

    """
    for i in range(dag.num_of_tasks):
        print 'line %d in comp_cost' % i
        dag.comp_cost.append(comp_cost[i])
        print dag.comp_cost[i]
        
        dag.data.append(data[i])
        print dag.data[i]
    for i in range(dag.num_of_processors):
        dag.rate.append(rate[i])
        print dag.rate[i]
    """
    """
    test the schedulers
    """
    
    cpop_scheduler = cpop.CPOP('../tgff/input_0.tgff')
    cpop_scheduler.run()
    cpop_scheduler.display_result()
    
    while not rospy.is_shutdown():
        pub.publish(dag)
        pub_rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
