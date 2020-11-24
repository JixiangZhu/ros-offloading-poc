import sys
import rospy
from dag.srv import *

def offloading_client(filename, scheduler):
    rospy.wait_for_service('offloading_service')
    try:
        offloading_service = rospy.ServiceProxy('offloading_service', OffloadingRequest)
        offloading = offloading_service(filename, scheduler)
        if offloading.offloading == True:
            print('Offloading available')
            return offloading
        else:
            print('offloading requst declined by the offloading manager')
    except rospy.ServiceException, e:
        print('Service call failed: ')
        print(e)

if __name__ == "__main__":
    filename = sys.argv[1]
    scheduler = sys.argv[2]
    offloading_client(filename, scheduler)
    print('Sent the offloading request for dag: ' + filename
            + ', Scheduler: ' + scheduler)





