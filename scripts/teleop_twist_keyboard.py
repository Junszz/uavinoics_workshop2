#!/usr/bin/env python3

from cmath import nan
import threading

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Float64

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
        w
    a   s    d
                                                                                           
change turning and speed step size
---------------------------
   y         u
   j         j

y : increase linear step size
j : decrease linear step size
u : increase angular step size
j : decrease angular step size
r: reset

anything else : reset
CTRL-C or c to quit
"""

speed_limit = 1
turn_limit = 0.5

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.cmd_publisher = rospy.Publisher('/cmd_vel', Float64, queue_size = 1)
        self.orien_publisher = rospy.Publisher('/cmd_orientation', Float64, queue_size = 1)
        self.speed = 0.0
        self.turn = 0.0
        self.speed_step_size = 0.0
        self.angular_step_size = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.cmd_publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.cmd_publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, linear_step, angular_step, reset_state = False):
        self.condition.acquire()
        if reset_state:
            self.speed = 0.0
            self.turn = 0.0
        else:
            self.speed_step_size = linear_step
            self.angular_step_size = angular_step
            self.speed += self.speed_step_size
            self.turn = self.angular_step_size
            self.speed = max(min(speed_limit,self.speed),-1*speed_limit)
            self.turn = max(min(turn_limit,self.turn),-1*turn_limit)
        # Notify publish thread that we have a new message.
        print(self.vels()+"\r")

        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, False)
        self.join()

    def run(self):
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)
            # Publish.
            self.orien_publisher.publish(self.turn)
            self.cmd_publisher.publish(self.speed)
            self.condition.release()
    
    def vels(self):
        return "currently:\tspeed %s\tturn %s " % (self.speed,self.turn)
            

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.0)
    turn = rospy.get_param("~turn", 0.0)
    linear_step = rospy.get_param("~speed", 0.1)
    angular_step = rospy.get_param("~turn", 0.1)
    
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.2)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    try:
        pub_thread.wait_for_subscribers()
 
        #initial vel and turn
        pub_thread.update(speed, turn)

        print(msg)
        # print(vels(speed,turn))
        linear_increase = 0
        angular_increase = 0
        reset_state = False
        while(1):
            key = getKey(key_timeout)
            if key == "w":
                linear_increase = linear_step
            elif key == "s":
                linear_increase = -1*linear_step
            elif key == "a":
                angular_increase = angular_step
            elif key == "d":
                angular_increase = -1*angular_step
            elif key == "y":
                linear_step *= 1.1
            elif key == "h":
                linear_step /= 1.1
            elif key == "u":
                angular_step *= 1.1
            elif key == "j":
                angular_step /= 1.1
            elif key == 'r':
                reset_state = True
            elif key == 'c':
                break
            pub_thread.update(linear_increase, angular_increase, reset_state)
            linear_increase = angular_increase = 0.0
            reset_state = False

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)