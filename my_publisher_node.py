#!/usr/bin/env python3
import os
import numpy as np
import rospy
import smbus2
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from sensor_msgs.msg import Range
import time
speed = WheelsCmdStamped()


class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        # initialize parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # Publisherid ja subscriberid
        self.pub = rospy.Publisher('/shelby/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.tof = rospy.Subscriber('/shelby/front_center_tof_driver_node/range', Range, self.callback)
        self.rwheel = rospy.Subscriber('/shelby/right_wheel_encoder_node/tick', WheelEncoderStamped ,self.rightwheel)
        self.lwheel = rospy.Subscriber('/shelby/left_wheel_encoder_node/tick', WheelEncoderStamped, self.leftwheel)
        self.seqLeft = rospy.Subscriber('/shelby/left_wheel_encoder_node/tick', WheelEncoderStamped, self.time_leftwheel)
        self.seqRight = rospy.Subscriber('/shelby/right_wheel_encoder_node/tick', WheelEncoderStamped ,self.time_rightwheel)

        #--------------------------MUUTUJAD----------------------------------------------------------------------------------
        self.bus = SMBus(12)
        self.range = 1
        self.right = 0
        self.left = 0
        self.ticks_left = 0
        self.prev_tick_left = 0
        self.ticks_right = 0
        self.prev_tick_right = 0
        self.rotation_wheel_left = 0
        self.rotation_wheel_right = 0
        self.delta_ticks_left = 0
        self.delta_ticks_right = 0
        self.n_tot = 135
        self.last_error = 0
        self.delta_time = 1/20
        self.prev_integral = 0
        self.previous_left = 0
        self.previous_right = 0
        self.wtraveltmp = 0
        self.kaugus_cm = 0
        self.wtravel = 0
        #---------------------------------------------------------------------------------------------------------------------
    
    def callback(self, data):
        self.range = data.range

    def rightwheel(self, data):
        self.right = data.data

    def leftwheel(self, data):
        self.left = data.data

    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        time.sleep(0.2)
        self.bus.close()
        rospy.on_shutdown()

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            read = self.bus.read_byte_data(62,17)
            read = bin(read)[2:].zfill(8) #joonelugeri andmete lugemine binaaris nii, et väljastatav väärtus oleks alati 8-kohaline
            R = 0.0318 #ratta raadius
            kp = rospy.get_param("/p")
            ki = rospy.get_param("/i")
            kd = rospy.get_param("/d")


            #-----------------------------------------ODOMEETRIA VALEMID------------------------------------------------------------
            
            
            self.ticks_right = self.right
            self.ticks_left = self.left
            self.delta_ticks_left = self.ticks_left-self.prev_tick_left
            self.delta_ticks_right = self.ticks_right-self.prev_tick_right
            self.rotation_wheel_left = (2*np.pi/self.n_tot)*self.delta_ticks_left #vasaku ratta pöördenurk (rad)
            self.rotation_wheel_right = (2*np.pi/self.n_tot)*self.delta_ticks_right #parema ratta pöördenurk (rad)
            d_left = R * self.rotation_wheel_left #vasaku ratta läbitud vahemaa (cm)
            d_right = R * self.rotation_wheel_right #parema ratta läbitud vahemaa (cm)
            self.prev_tick_left = self.ticks_left
            self.prev_tick_right = self.ticks_right
            self.wtravel = round(((d_left + d_right)*100)/2, 1) #roboti läbitud vahemaa (cm)
            self.kaugus_cm = round(self.range*100, 1) #TOF sensori tuvastatud kaugus (cm)

            #-------------------------------------TAKISTUSEST MÖÖDUMINE------------------------------------------------------------
            
            def obstacle():
                while self.kaugus_cm < 35: #tuvastades objekti 35cm kauguselt, pöörab robot paremale
                    speed.vel_left = 0.33
                    speed.vel_right = 0.05
                    self.pub.publish(speed)
                    self.kaugus_cm = round(self.range*100, 1)
                time.sleep(0.2)
                while self.wtraveltmp < 30: #robot sõidab 30cm otse
                    speed.vel_left = 0.3
                    speed.vel_right = 0.3
                    self.pub.publish(speed)
                    self.wtraveltmp = self.wtraveltmp + self.wtravel
                time.sleep(0.2)
                speed.vel_left = 0.05 #robot pöörab vasakule
                speed.vel_right = 0.4
                self.pub.publish(speed)
                time.sleep(1.2)
                speed.vel_left = 0.3  #robot pöörab paremale
                speed.vel_right = 0.05
                time.sleep(0.5)

            if self.kaugus_cm <= 35:
                obstacle()

            #------------------------------------------PID-KONTROLLER----------------------------------------------------------------------------------
            
                
            näidik = [] #joonelugeri tuvastused vahemikus 1-8
            for indx, nr in enumerate(read):
                if nr == "1":
                    näidik.append(indx + 1)

            error = 4.5 - np.average(näidik) #keskpunktist maha lahutatud viimane joonelugeri tuvastus
            integral = self.prev_integral + error*self.delta_time
            integral = max(min(integral,2), -2) #integrali piirang
            derivative = (error - self.last_error)/self.delta_time
            correction = kp * error + ki * integral + kd * derivative #PID kontrolleri lõplik valem 
            speed.vel_left = 0.5 - correction
            speed.vel_right = 0.5 + correction

            if len(näidik) == 0: #joonelt maha sõites, jätkab robot liikumist vastavalt viimasele ette antud kiirusele
                speed.vel_left = self.previous_left
                speed.vel_right = self.previous_right
            self.previous_left = speed.vel_left
            self.previous_right = speed.vel_right

            speed.vel_left = max(0.0, min(speed.vel_left, 0.7)) #kiirusepiirangud
            speed.vel_right = max(0.0, min(speed.vel_right, 0.7)) #kiirusepiirangud
            self.pub.publish(speed)
            self.last_error = error
            rate.sleep()

if __name__ == '__main__':
    node = MyPublisherNode(node_name='my_publisher_node')
    rospy.on_shutdown(node.on_shutdown)
    node.run()
    rospy.spin()