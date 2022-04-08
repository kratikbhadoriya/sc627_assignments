#!/usr/bin/env python

import rospy
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
import matplotlib.pyplot as plt

# ANG_MAX = math.pi/18
# VEL_MAX = 0.15

class balance:
    def __init__(self):

        self.pos = [0.0,0.0]
        self.vel = [0.0,0.0]

        self.lpos = [0.0,0.0]
        self.lvel = [0.0,0.0]

        self.rpos = [0.0,0.0]
        self.rvel = [0.0,0.0]

        self.pos_err = [0.0,0.0]

        self.ANG_MAX = math.pi/18
        self.VEL_MAX = 0.15

        rospy.Subscriber('/odom', Odometry, self.callback_odom) #topic name fixed
        rospy.Subscriber('/left_odom', Odometry, self.callback_left_odom) #topic name fixed
        rospy.Subscriber('/right_odom', Odometry, self.callback_right_odom) #topic name fixed

        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        self.r = rospy.Rate(30)

        self.botpos = [[], []]
        self.start_time = time.time()
        self.time_elapsed = []


    def velocity_convert(self, vel_x, vel_y):
        '''
        Robot pose (x, y, theta)  Note - theta in (0, 2pi)
        Velocity vector (vel_x, vel_y)
        '''

        # gain_ang = 1 #modify if necessary
        
        # ang = math.atan2(vel_y, vel_x)
        
        # ang_err = min(max(ang - theta, -self.ANG_MAX), self.ANG_MAX)

        v_lin =  min(max(vel_x, -self.VEL_MAX), self.VEL_MAX)
        v_ang = 0
        return v_lin, v_ang

    def callback_odom(self,data):
        '''
        Get robot data
        '''
        self.pos[0] = data.pose.pose.position.x
        self.pos[1] = data.pose.pose.position.y

        self.vel[0] = data.twist.twist.linear.x

        #self.botpos.append((self.pos[0],self.pos[1]))
        self.botpos[0].append(self.pos[0])
        self.botpos[1].append(self.pos[1])
        self.time_elapsed.append(time.time()-self.start_time)
        # print(data)
        pass

    def callback_left_odom(self,data):
        '''
        Get left robot data
        '''
        self.lpos[0] = data.pose.pose.position.x
        self.lpos[1] = data.pose.pose.position.y

        self.lvel[0] = data.twist.twist.linear.x

        # print('left robot')
        # print(data)
        pass

    def callback_right_odom(self,data):
        '''
        Get right robot data
        '''
        self.rpos[0] = data.pose.pose.position.x
        self.rpos[1] = data.pose.pose.position.y

        self.rvel[0] = data.twist.twist.linear.x

        # print('right robot')
        # print(data)
        pass

    def balancing(self):
        self.pos_err[0] = 0
        k_p = 2
        count = 0
        while (abs(self.vel[0]) > 0.0002 and abs(self.rvel[0]) > 0.0002 and abs(self.lvel[0])>0.0002) or count<800 :
            self.pos_err[0] = self.pos[0]-(self.lpos[0]+self.rpos[0])/2
            vel_x = -k_p * (self.pos_err[0])
            vel_y = 0
            v_lin,v_ang = self.velocity_convert(vel_x,vel_y)
            vel_msg = Twist()
            vel_msg.linear.x = v_lin
            vel_msg.angular.z = v_ang
            # print(vel_x)
            self.pub_vel.publish(vel_msg)
            self.r.sleep()
            count += 1
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        # print(vel_x)
        self.pub_vel.publish(vel_msg)
        self.r.sleep()
        return self.botpos,self.time_elapsed
        

while __name__ == '__main__':
    rospy.init_node('assign4_balance', anonymous = True)
    bot = balance()
    botpos,time_elapsed = bot.balancing()
    # bot_pos_x = [i[0] for i in botpos]
    # bot_pos_y = [i[1] for i in botpos]

    # plt.plot(bot_pos_x, bot_pos_y)
    plt.plot(botpos[0], botpos[1])
    plt.title("Robot Path")
    plt.xlabel("x-Coordinate")
    plt.ylabel("y-Coordinate")
    plt.grid()
    plt.show()

    # plt.plot(time_elapsed, bot_pos_x)
    plt.plot(time_elapsed, botpos[0])
    plt.title("x vs t")
    plt.xlabel("Time")
    plt.ylabel("x-Coordinate")
    plt.grid()
    plt.show()

