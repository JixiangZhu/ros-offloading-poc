import matplotlib.pyplot as plt

def plot_result():

    f=open("results_heft.txt", 'r')
    heft = f.readlines()
    f2 = open("results_cpop.txt", 'r')
    cpop = f2.readlines()

    node_num = []
    slr = []
    speedup = []
    slr2 = []
    speedup2 = []

    for line in heft:
        line.strip()
        row = line.split()
        node_num.append(row[0])
        slr.append(row[1])
        speedup.append(row[2])
    
    for line in cpop:
        line.strip()
        row = line.split()
        slr2.append(row[1])
        speedup2.append(row[2])
    
    HEFT,= plt.plot(node_num, slr, 'rs--', label='HEFT')
    CPOP,= plt.plot(node_num,slr2, 'b^--', label = 'CPOP')
    plt.xlabel('Node Number')
    plt.ylabel('Avg SLR')
    plt.title('Average SLR with respect to graph size')
    plt.axis([0,110, 0.4, 1])
    plt.legend([HEFT,CPOP],['HEFT','CPOP'])
    plt.show()
    
    plt.plot(node_num, speedup, 'rs--', node_num,speedup2, 'b^--')
    HEFT,= plt.plot(node_num, speedup, 'rs--', label='HEFT')
    CPOP,= plt.plot(node_num,speedup2, 'b^--', label = 'CPOP')
    plt.xlabel('Node Number')
    plt.ylabel('Avg SLR')
    plt.title('Average SLR with respect to graph size')
    plt.axis([0,110, 1,3])
    plt.legend([HEFT,CPOP],['HEFT','CPOP'])
    plt.show()

if __name__ == "__main__":
    plot_result()
