from dag.srv import *
from dag.msg import Node
from dag.msg import Edge as Arc
import csv
import rospy
import cpop
import heft
import create_input
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import MultiArrayDimension
from igraph import *

result = None

def handle_offloading_request(req):
    print('Offloading request received with filename ' + req.random_dag_filename)

    filename = '../tgff/' + req.random_dag_filename
    #Read dag file to create the computation cost matrix, communication matrix etc.
    g = generate_graph(filename)

    #Call scheduler 
    scheduler_name = req.scheduler
    global result
    result = scheduling(filename, scheduler_name)
    # Get the allocation matrix, 1 denotes the task should run on that robot
    # allocation_matrix = get_allocation(result)

    # publish the scheduled decision
    # schedule_publisher(result)
    '''
    Or call the service to the corresponding robot, along with the allocation decision.
    '''
    robots, nodes, makespan = result
    comp = []
    avg_comp = []
    min_comp = []
    cps = [0]*len(nodes[0].comp_cost)
    for task in nodes:
        comp.append(task.comp_cost)
        avg_comp.append(task.avg_comp)
        min_comp.append(min(task.comp_cost))
        for i in range(len(task.comp_cost)):
            cps[i] += task.comp_cost[i] 

    slr = makespan/sum(min_comp)
    speedup = min(cps)/makespan
    print('Schedule Length Ratio:')
    print(slr)
    print('Speedup: ')
    print(speedup)
    
    txt = 'result.txt'
    f = open(txt, "a")
    f.write(str(len(nodes)-2)+' ' + repr(slr)+' '+repr(speedup)+'\n')
    f.close()
    
    all_nodes = []
    for r in robots:
        nodes = [] 
        nodelets = []
        
        for duration in r.time_line:
            node = Node() 
            nodelets.append(duration.task_num)
            v_id = duration.task_num
            node.id = v_id 
            node.weight = g.vs[v_id]["weight"]
            node.indegree = g.vs[v_id].indegree()
            node.outdegree = g.vs[v_id].outdegree()

            predecessors = g.vs[v_id].predecessors()
            for pre in predecessors:
                income = Arc()
                income.id = g.get_eid(pre.index, v_id)
                income.weight = g.es[income.id]["weight"]
                node.incomes.append(income)
            successors = g.vs[v_id].successors()
            for suc in successors:
                outcome = Arc()
                outcome.id = g.get_eid(v_id, suc.index)
                outcome.weight = g.es[outcome.id]["weight"]
                node.outcomes.append(outcome)
            nodes.append(node)
            all_nodes.append(node)

        flag = send_decision(r.number, nodes, filename)
    return OffloadingRequestResponse(flag)
    
    '''
    try:
        flag = send_decision(1, all_nodes, filename)
        print('Successfully initiate nodelets in robot:' + str(r.number))
                
    except rospy.ServiceException, e:
        print('Service call failed with exception: ')
        print(e)
    
    if flag == True:   
        return OffloadingRequestResponse(True)
    else:
        print('initiate some nodes failed')
        return OffloadingRequestResponse(False)
    '''
def generate_graph(filename):
    num_of_tasks, num_of_processors, comp_costs, rate, comm_cost = create_input.init(filename)
    avg_comp_costs = []
    for row in comp_costs:
        avg_comp_costs.append(sum(row)/len(row))

    print('avg_comp_costs: ')
    print(avg_comp_costs)
    g = Graph(directed=True)
    print('node weithts: ')
    g.es["weight"] = 1.0
    g.add_vertices(num_of_tasks)

    g.vs["weight"] = avg_comp_costs

    print(g.vs["weight"])
    j=0
    for line in comm_cost:
        for i in range(num_of_tasks):
            if line[i] != -1:
                g.add_edge(j, i)
                g[j,i] = line[i]
        j +=1
    print(g.es["weight"])
    '''
    g.vs["label"] = g.vs["weight"]
    g.es["label"] = g.es["weight"]
    g.vs["name"] ="Node "+ str( i in range(g.vcount()))
    layout1 = g.layout("kk")
    plot(g, layout=layout1, autocurve=True)
    layout2 = g.layout("fr")
    plot(g, layout = layout2)
    '''
    return g
    
    
def send_decision(robot_id, nodes, filename):
    service_name = 'ros_'+str(robot_id) + '/init_nodelets_service'
    rospy.wait_for_service(service_name)
    try:
        init_nodelets_service = rospy.ServiceProxy(service_name, InitiateNodelets)
        node_initiated = init_nodelets_service(nodes)
        if node_initiated == True:
            return True
        else:
            return False
    except rospy.ServiceException, e:
        print('Service call failed with exception: ')
        print(e)
    return False

def offloading_manager():
    rospy.init_node('offloading_manager')
    s = rospy.Service('offloading_service', OffloadingRequest, handle_offloading_request)
    print('Ready to accept offloading requests.') 
    print(result)

    # schedule_publisher(result)
    rospy.spin()

def scheduling(filename, scheduler_name):
    if scheduler_name == 'heft':
        scheduler = heft.HEFT(filename)
        scheduler.run()
        scheduler.display_result()
        return scheduler.result()
    else:
        scheduler = cpop.CPOP(filename)
        scheduler.run()
        scheduler.display_result()
        return scheduler.result()
    
def get_allocation(result):
    processors = []
    tasks = []
    processors, tasks = result
    num_of_tasks = len(tasks)
    num_of_processors = len(processors)
    allocation_matrix = [[0 for i in range (num_of_tasks)] for i in range(num_of_processors)]
    for p in processors:
        for duration in p.time_line:
            allocation_matrix[p.number][duration.task_num] = 1
        print allocation_matrix[p.number]
    return allocation_matrix 

def schedule_publisher(allocation_matrix):   
    pub = rospy.Publisher('allocation', Int32MultiArray, queue_size = 10) 
    pub_rate = rospy.Rate(0.5)

    num_of_processors = len(allocation_matrix)
    num_of_tasks = len(allocation_matrix[0])

    allocation = Int32MultiArray()
    allocation.layout.dim.append(MultiArrayDimension())
    allocation.layout.dim.append(MultiArrayDimension())
    allocation.layout.dim[0].size = num_of_processors 
    allocation.layout.dim[1].size = num_of_tasks 
    allocation.layout.dim[1].stride = num_of_tasks
    stride = num_of_tasks 
    for i in range(num_of_processors ):
        for j in range(num_of_tasks ):
            allocation.data.insert(i*stride + j, allocation_matrix[i][j])
    
    while not rospy.is_shutdown():
        pub.publish(allocation)
        pub_rate.sleep()
    print('The allocation matrix has been published')
    print(allocation.data)

if __name__ == "__main__":
    offloading_manager()
