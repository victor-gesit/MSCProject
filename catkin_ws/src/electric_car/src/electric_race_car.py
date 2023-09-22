#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from electric_car.msg import BatteryStatus
import math
import pandas as pd
import rospkg
import os

rospack = rospkg.RosPack()

class ElectricCar:
    def __init__(self):
        self.battery_level = 100
        self.current_velocity = Twist()
        
        
        # Velocity Publisher
        self.vel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.accel_vel_profile = []
        self.circular_motion_vel_profile = []
        self.skid_pad_vel_profile = []
        self.enduration_test_vel_profile = []

        self.data_interval = 0.2

        # Subscribers
        rospy.Subscriber("battery", BatteryStatus, callback=self.battery_level_subscriber_callback)
        
        self.load_accel_vel_profile()
        self.load_circular_motion_vel_profiles()


        # self.publish_acell_vel_profile()
        self.publish_circular_vel_profile()
        
    def battery_level_subscriber_callback(self, battery_level):
        # print("Battery Level: ", battery_level)
        pass

    def start_race(self):
        while not rospy.is_shutdown():
            self.vel_publisher.publish()
        pass

    def get_velocity_from_profile():
        pass

    def load_accel_vel_profile(self):
        vel_profile_path = os.path.join(rospack.get_path("electric_car"), "velocity_profiles", "accel_vel_profile.csv")
        vel_profile_arr = [{'t': row[0], 'v-lin': row[1], 'steer-ang': row[2]} for _, row in pd.read_csv(vel_profile_path).iterrows()]
        self.accel_vel_profile = vel_profile_arr

    def load_circular_motion_vel_profiles(self):
        vel_profile_path = os.path.join(rospack.get_path("electric_car"), "velocity_profiles", "vel_prof_circular_motion.csv")
        vel_profile_circular = [{'t': row[0], 'v-lin': row[1], 'steer-ang': row[2]} for _, row in pd.read_csv(vel_profile_path).iterrows()]
        self.circular_motion_vel_profile = vel_profile_circular


    def publish_acell_vel_profile(self):
        rate = rospy.Rate(1/self.data_interval)
        count = 0
        entries = len(self.accel_vel_profile)


        while not rospy.is_shutdown():
            print("EE: ", count, entries)
            entry = self.accel_vel_profile[count]
            t = entry['t']
            linear_v = entry['v-lin']
            steer_angle = entry['steer-ang']

            vel = Twist()
            vel.linear.x = linear_v
            vel.angular.z = steer_angle

            self.vel_publisher.publish(vel)

            print("T::: ", t, linear_v, steer_angle)
            if(count + 1 < entries):
                count += 1
            rate.sleep()

    def publish_circular_vel_profile(self):
        rate = rospy.Rate(1/self.data_interval)
        count = 0
        entries = len(self.accel_vel_profile)


        while not rospy.is_shutdown():
            entry = self.circular_motion_vel_profile[count]
            t = entry['t']
            linear_v = entry['v-lin']
            steer_angle = entry['steer-ang']

            vel = Twist()
            vel.linear.x = linear_v
            vel.angular.z = steer_angle

            self.vel_publisher.publish(vel)

            if(count + 1 < entries):
                count += 1
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("electric_race_car")
    ElectricCar()
    rospy.spin()
