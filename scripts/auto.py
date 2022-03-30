#!/usr/bin/python3
import math
from xmlrpc.client import Boolean
import rospy
from std_msgs.msg import Float64, Bool
#from geometry_msgs.msg import Twist, Pose
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Range
from std_srvs.srv import SetBool, SetBoolResponse

# to set an default initial state
L = 0.4
R = 0.04
current_velocity = 0.0
current_orientation = 0.0
reference_velocity = 0.0
reference_orientation = 0.0
ROBOTNAME = "uavcar"
start = False

def startAutonomous(data):
    if not start:
        # Please start the robot
        rospy.loginfo("Please start the service")
        rospy.sleep(1)
        return 
    range = data.range 
    max_range = data.max_range # 1
    min_range = data.min_range # 0.2
    threshold = 0.5 # TO BE CHANGE
    if range < 0.5: 
        # STOP AND TURN LEFT
        pass 
    else:
        # GO STRAIGHT
        pass 


def start_server(req):
    global start
    if req.data:
        start = True
        return SetBoolResponse(True, "The robot has started")
    else:
        start = False
        return SetBoolResponse(False, "The robot has halted")


if __name__ == '__main__':
    # name of this node(controller_system)
    rospy.init_node('controller_system', anonymous=True)

    s = rospy.Service('LetsGo', SetBool, start_server)
    # subscribe to three nodes ( cmd_vel, cmd_orientation, gazebo_model_state )
    rospy.Subscriber("/ultrasonic_feedback", Range, startAutonomous)
   
    # publish to two nodes ( left_wheel_speed, right_wheel_speed )
    left_wheel_pub = rospy.Publisher('/uavcar/jointL_velocity_controller/command', Float64, queue_size=2)
    right_wheel_pub = rospy.Publisher('/uavcar/jointR_velocity_controller/command', Float64, queue_size=2)

    rospy.spin()  # simply keeps python from exiting until this node is stopped
