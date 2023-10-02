#!/usr/bin/env python

import rospy
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys, select, termios, tty
import random
import numpy as np

# Save the terminal settings
settings = termios.tcgetattr(sys.stdin)

class Teleop():
    def __init__(self):
        rospy.init_node('turtlebot_teleop') # initialize ROS node twice if needed
        # Set the update rate
        self.rate = rospy.Rate(10) # 10 Hz
        self.pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.bumped = False
        self.bumper = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.bumper_halt)
        self.moveBindings = {
            'i':(1,0),
            'o':(1,-1),
            'j':(0,1),
            'l':(0,-1),
            'u':(1,1),
            ',':(-1,0),
            '.':(-1,1),
            'm':(-1,-1),
        }

        self.speedBindings = {
            'q':(1.1,1.1),
            'z':(.9,.9),
            'w':(1.1,1),
            'x':(.9,1),
            'e':(1,1.1),
            'c':(1,.9),
        }

        self.settings = termios.tcgetattr(sys.stdin)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        self.keyPressed = False
        self.x = 0
        self.th = 0
        self.status = 0
        self.count = 0
        self.acc = 0.1
        self.target_speed = 0
        self.target_turn = 0
        self.control_speed = 0
        self.control_turn = 0
        self.speed = rospy.get_param("~speed", 0.5)
        self.turn = rospy.get_param("~turn", 1.0)
        self.msg = """
            Control Your Turtlebot!
            ---------------------------
            Moving around:
            u    i    o
            j    k    l
            m    ,    .

            q/z : increase/decrease max speeds by 10%
            w/x : increase/decrease only linear speed by 10%
            e/c : increase/decrease only angular speed by 10%
            space key, k : force stop
            anything else : stop smoothly

            CTRL-C to quit
            """
        print("Teleop Initialized")

    def bumper_halt(self, bump):
        # Check to see if the bumper has been hit, if it is hit then stop
        if bump.PRESSED == 1:
            self.bumped = True
        else:
            self.bumped = False

    def move(self):
        # default movement from original teleop, modified to include automatic movement
        key = self.getKey()
        if key in self.moveBindings.keys():
            self.x = self.moveBindings[key][0]
            self.th = self.moveBindings[key][1]
            self.count = 0
            self.keyPressed = True
        elif key in self.speedBindings.keys():
            self.speed = self.speed * self.speedBindings[key][0]
            self.turn = self.turn * self.speedBindings[key][1]
            self.count = 0

            print(vels(self.speed,self.turn))
            if (self.status == 14):
                print(self.msg)
            self.status = (self.status + 1) % 15
        elif key == ' ' or key == 'k' :
            self.x = 0
            self.th = 0
            self.control_speed = 0
            self.control_turn = 0
            self.count = 0
            self.keyPressed = True
            
            # begin automatic movement
        else:
            # Automatically move the robot
            if self.count > 50:
                print("AUTOMODE")
                self.keyPressed = False
                # Generate a random number between -15 and 15 degrees
                rand = random.uniform(-3.14, 3.14)
                # This is the speed that will be used to move
                auto_speed = 0.3048
                auto_turn = rand
                twist = Twist()
                # Set the linear and velocities
                twist.linear.x = auto_speed; twist.linear.y = 0; twist.linear.z = 0
                self.pub.publish(twist)
                rospy.sleep(1.0)
                # reset the linear velocity
                twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
                self.pub.publish(twist)
                # Set the angular velocity
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = auto_turn
                self.pub.publish(twist)
                return
            if (key == '\x03'):
                return

        # resume manual control
        self.target_speed = self.speed * self.x
        self.target_turn = self.turn * self.th

        if self.target_speed > self.control_speed:
            self.control_speed = min( self.target_speed, self.control_speed + 0.02 )
        elif self.target_speed < self.control_speed:
            self.control_speed = max( self.target_speed, self.control_speed - 0.02 )
        else:
            self.control_speed = self.target_speed

        if self.target_turn > self.control_turn:
            self.control_turn = min( self.target_turn, self.control_turn + 0.1 )
        elif self.target_turn < self.control_turn:
            self.control_turn = max( self.target_turn, self.control_turn - 0.1 )
        else:
            self.control_turn = self.target_turn
        twist = Twist()
        twist.linear.x = self.control_speed; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = self.control_turn
        self.pub.publish(twist)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

class ObstacleAvoidance():
    def __init__(self):
        # Initialize ROS node and global variables
        self.twist_vel = 5
        self.rotate_time = 1
        self.laser = rospy.Subscriber('/scan', LaserScan, self.fake_laser_scan)
        self.pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.stopAuto = False
        self.global_min = 100
        self.endpoints = []

    def twisting(self):
        # Stop the robot from moving
        self.stopAuto = True
        rotate = Twist()
        rotate.linear.x = 0; rotate.linear.y = 0; rotate.linear.z = 0
        # Rotate the robot in place to avoid obstacle
        rotate.angular.z = self.twist_vel; rotate.angular.x = 0.0; rotate.angular.y = 0.0
        self.pub.publish(rotate)
        rospy.sleep(self.rotate_time) #used to be rotate_time
        # Stop the robot from rotating
        rotate.angular.z = 0.0
        rotate.linear.x = 0.5
        self.pub.publish(rotate)
        rospy.sleep(1.0)
        # reset movement
        rotate.linear.x = 0
        self.pub.publish(rotate)
        self.stopAuto = False

    def fake_laser_scan(self, laser):
        # Get the minimum distance from the laser scan
        left = min(laser.ranges[420:], key=abs)
        center = min(laser.ranges[210:420], key=abs)
        right = min(laser.ranges[0:210], key=abs)
        # Set the endpoints with minimum distances
        self.endpoints = [left, center, right]
        # Set the endpoints to 0 if they are NaN
        for i in range(len(self.endpoints)):
            if self.endpoints[i] != self.endpoints[i]:
                self.endpoints[i] = 10
        # Get the global minimum distance
        self.global_min = min(self.endpoints)
       
    def avoid(self, threshold):
        print("OBSTACLE")
        rad = 3.14
        # Check to see which endpoint is the global minimum
        if self.global_min == self.endpoints[1] and self.global_min < threshold:
            # Turn away from obstacle if heading straight for it
            self.twist_vel = 5
            self.rotate_time = rad / abs(self.twist_vel)
            self.twisting()
        elif self.global_min == self.endpoints[2] and self.global_min < threshold:
            # Turn right if obstacle is on the left
            self.twist_vel = 3.14
            self.rotate_time = rad / abs(self.twist_vel)
            self.twisting()
        elif self.global_min == self.endpoints[0] and self.global_min < threshold:
            # Turn left if obstacle is on the right
            self.twist_vel = -3.14
            self.rotate_time = rad / abs(self.twist_vel)
            self.twisting()
        else:
            pass


def main():
    # Initialize the teleop and obstacle avoidance classes
    teleop = Teleop()
    obstacle_avoidance = ObstacleAvoidance()

    # print the default teleop message
    print(teleop.msg)

    threshold = 0.6

    try:
        while(1):   
            # Check to see if the robot is clear to move     
            while obstacle_avoidance.global_min < threshold:
                obstacle_avoidance.avoid(threshold)
            
            # Check to see if user has requested manual override
            if teleop.keyPressed == False: 
                if obstacle_avoidance.stopAuto == True:
                    continue
                
            # Move the robot
            teleop.move()
            teleop.count = teleop.count + 1
            # Check to see if the robot has been bumped
            if teleop.bumped == True:
                break
    except Exception as e:
        print(e)
    finally:
        # Stop the robot completely
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.z = 0; twist.angular.x = 0.0; twist.angular.y = 0.0
        teleop.pub.publish(twist)

if __name__=="__main__":
    main()

    
