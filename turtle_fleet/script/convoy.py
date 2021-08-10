#!/usr/bin/env python

# Inspirado em:
# https://gitlab.com/cursoseaulas/ros_examples/-/blob/master/scripts/position_control_TB3.py

import rospy
import numpy as np
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion as q2e

class Wagon():

    def __init__(self, name):
        self.name = name
        self.max_lin = 0.22
        self.max_ang = 2.84
        self.pose = Pose()
        self.cmd_vel = Twist()
        self.rate = rospy.Rate(1)
        self.pub = rospy.Publisher(self.name + "/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber(self.name + "/odom", Odometry, self.track)

    # rastreia a própria posição
    def track(self, msg):
        orientation = msg.pose.pose.orientation
        (_, _, yaw) = q2e([orientation.x, orientation.y, orientation.z, orientation.w])
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        self.pose.theta =  yaw
    
    # se movimenta conforme o comando
    def move(self, msg):
        self.cmd_vel = msg
        self.pub.publish(self.cmd_vel)
        #self.status()
    
    # informa posição e deslocamento
    def status(self):
        print(self.name + ':')
        print(f'\t> linear={self.cmd_vel.linear}')
        print(f'\t> angular={self.cmd_vel.angular}')
    
    # fica em standby
    def run(self):
        rospy.spin()


class Convoy():

    def __init__(self, leader, followers=[]):
        rospy.init_node("convoy", anonymous=False)
        self.convoy = {}
        self.leader = Wagon(leader)
        self.addFollowers(followers)
        self.machinist = rospy.Rate(10)
        self.min_radius = 0.5
        rospy.Subscriber("/cmd_vel", Twist, self.teleop)
    
    # intercepta o comando de teleop e move o líder
    def teleop(self, msg):
        self.leader.move(msg)

    # adiciona mais robôs seguidores ao comboio
    def addFollowers(self, robots):
        new_bots = list(set(robots) - set(self.convoy.keys()))
        for robot in new_bots:
            print('adding wagon \"' + robot + '\"')
            self.convoy[robot] = Wagon(robot)
    
    # obtém o vetor de posição do vagão em relação ao líder
    def relPos(self, front_wagon, back_wagon):
        ref = front_wagon.pose
        wgn = back_wagon.pose
        module = np.sqrt((ref.x - wgn.x)**2 + (ref.y - wgn.y)**2)
        angle = np.arctan2(ref.y - wgn.y,  ref.x - wgn.x) - wgn.theta
        return (module, angle)

    # ação de controle baseada no vetor de posição relativa
    def control(self, vector, kp=(1.5, 6)):
        u = Twist()
        flag = (vector[0] > self.min_radius)
        # ação de controle linear
        u.linear.x  = kp[0] * vector[0] * flag
        u.linear.y = 0
        u.linear.z = 0
        # ação de controle angular
        u.angular.z = kp[1] * vector[1] * flag
        u.angular.x = 0
        u.angular.y = 0
        return u
    
    # saturação da ação de controle
    def sat(self, u, wagon):
        # velocidade linear máxima
        if abs(u.linear.x) > wagon.max_lin:
            u.linear.x = wagon.max_lin * np.sign(u.linear.x)
        # velocidade angular máxima
        if abs(u.angular.z) > wagon.max_ang:
            u.angular.z = wagon.max_ang * np.sign(u.angular.z)
        return u

    # comanda o comboio para seguir o líder
    def run(self):
        while not rospy.is_shutdown():
            front_wagon = self.leader

            for robot in self.convoy:
                
                # posição relativa do vagão
                wagon = self.convoy[robot]
                (dist, ang) = self.relPos(front_wagon, wagon)
                # ação de controle com saturação
                cmd = self.control((dist, ang))
                cmd = self.sat(cmd, wagon)
                wagon.move(cmd)
                # atualização do vagão da frente
                front_wagon = wagon
            
            self.machinist.sleep()

if __name__ == "__main__":
    try:
        tb3_fleet = Convoy(leader='tb3_0')
        tb3_fleet.addFollowers(robots=['tb3_1', 'tb3_2'])
        tb3_fleet.run()
    except rospy.ROSInterruptException:
        pass