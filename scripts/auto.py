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

def startAutonomous(data):
    global reference_orientation
    if not start:
        # Please start the robot
        rospy.loginfo("Please start the service")
        rospy.sleep(5)
        return 

    range = data.range 
    max_range = data.max_range # 1
    min_range = data.min_range # 0.2
    threshold = 0.5 # TO BE CHANGE
    if range < 0.5:
        reference_orientation = reference_orientation +math.pi/180
        speed_pub.publish(0.1)
        rotate_pub.publish(reference_orientation)
    else:
        # GO STRAIGHT
        speed_pub.publish(0.2)
    


def start_server(req):
    global start
    if req.data:
        start = True
        return SetBoolResponse(True, "The robot has started")
    else:
        start = False
        return SetBoolResponse(False, "The robot has halted")

def current_state_callback(data):
    #callback function to get the current states (velocity, position and etc) of our robot
    global current_velocity, current_orientation, linear_velocity
    robots = data.name
    robot_idx=0

    #in gazebo model state, different "index" represent different robot, thus, we need to find the correspoinding robot index to extract the correct values.
    for idx, robot in enumerate(list(robots)):
        if robot == ROBOTNAME:
            robot_idx = idx
            break
    
    #Then, calculate the linear velocity with Pythagoras' theorem
    linear_velocity = math.sqrt(pow(data.twist[robot_idx].linear.x, 2)+pow(data.twist[robot_idx].linear.y, 2))
    #the orientation is in Quaternion Form, we need to convert it to Euler Form (The one we familiar with) before we proceed
    x_o = data.pose[robot_idx].orientation.x
    y_o = data.pose[robot_idx].orientation.y
    z_o = data.pose[robot_idx].orientation.z
    w = data.pose[robot_idx].orientation.w

    #yaw, rotation along z-axis is the one that we interested, another two should be close to zero all the time
    (roll, pitch, yaw) = euler_from_quaternion([x_o,y_o,z_o,w]) 
    current_orientation = yaw


if __name__ == '__main__':
    # name of this node(controller_system)
    rospy.init_node('controller_system', anonymous=True)

    s = rospy.Service('LetsGo', SetBool, start_server)

    # subscribe to two nodes (  gazebo_model_state, ultrasonic_feedback )
    rospy.Subscriber("/ultrasonic_feedback", Range, startAutonomous)
    rospy.Subscriber("/gazebo/model_states", ModelStates,current_state_callback)

    # publish to two nodes ( cmd_vel, cmd_orientation)
    speed_pub = rospy.Publisher('/cmd_vel', Float64, queue_size=2)
    rotate_pub = rospy.Publisher('/cmd_orientation', Float64, queue_size=2)

    rospy.spin()  # simply keeps python from exiting until this node is stopped
