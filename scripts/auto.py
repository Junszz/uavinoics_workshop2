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
start = False
current_velocity = 0.0
current_orientation = 0.0
ROBOTNAME = "uavcar"
reference_orientation = 0
current_distance = 1

def ultrasonic_state_callback(data):
    global current_distance 
    current_distance = data.range 
     

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

    # subscribe to two nodes (  gazebo_model_state, ultrasonic_feedback )
    rospy.Subscriber("/ultrasonic_feedback", Range, ultrasonic_state_callback)

    # publish to two nodes ( cmd_vel, cmd_orientation)
    speed_pub = rospy.Publisher('/cmd_vel', Float64, queue_size=2)
    rotate_pub = rospy.Publisher('/cmd_orientation', Float64, queue_size=2)
    
    while not rospy.is_shutdown():
        print("HI")
        if current_distance < 0.5:
            print("Turn")
            speed_pub.publish(0.1)
            rotate_pub.publish(math.pi/2)
            rospy.sleep(10)
        else:
            # GO STRAIGHT
            print("Straight")
            speed_pub.publish(0.4)
        
