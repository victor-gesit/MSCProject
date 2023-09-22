#!/usr/bin/env python3

import rospy
from electric_car.msg import SignalMessage
from electric_car.msg import SimulationCommand
from electric_car.msg import VehicleConfiguration

import rospy

import time
import pandas as pd
import rospkg
import os

rospack = rospkg.RosPack()

class SIGNAL_GENERATOR:
    def __init__(self):
        self.data_interval = 0.2
        self.simulation_type = 0
        self.simulation_started = False
        self.simulation_duration = 100 # seconds
        self.vehicle_max_speed = 100 # m/s


        # Vehicle Configuration Subscriber
        rospy.Subscriber("configure_vehicle", VehicleConfiguration, callback=self.configure_vehicle)

        # Simulation Command Subscriber
        rospy.Subscriber("simulation_command", SimulationCommand, callback=self.start_simulation)

        # Velocity Signal Publisher
        self.vel_signal_publisher = rospy.Publisher("velocity_signal", SignalMessage, queue_size=1)
        
        self.accel_vel_profile = []
        self.circular_motion_vel_profile = []
        self.skid_pad_vel_profile = []
        self.enduration_test_vel_profile = []

        self.load_accel_vel_profile()
        self.load_circular_motion_vel_profiles()

        self.simulation_start_time = time.time()
    
    def configure_vehicle(self, configuration):
        self.vehicle_max_speed = configuration.driverVehicleMaxSpeed

    def start_simulation(self, command):
        if(self.simulation_started):
            return
        
        simulationType = command.simulationType
        time.sleep(2)
        self.simulation_start_time = time.time()

        if(simulationType == 1):
            print("IT's a 1")
            self.trapezoid_profile()
        elif(simulationType == 2):
            print("IT's a 2")
            self.high_acceleration_profile()
        elif(simulationType == 3):
            print("IT's a 3")
            self.acceleration_deceleration_profile()
        elif(simulationType == 4):
            print("IT's a 4")
            self.simulate_longitudinal_drive()
        elif(simulationType == 5):
            print("IT's a 5")
            self.simulate_circular_movement()
        else:
            self.stop_simulation()
        
        self.simulation_started = True

    def trapezoid_profile(self):
        while not rospy.is_shutdown():
            time_now = time.time()
            signal = SignalMessage()
            time_elapsed = time_now - self.simulation_start_time
            if(time_elapsed < 30):
                signal.referenceVelocity = (self.vehicle_max_speed/30.0) * time_elapsed
            elif(time_elapsed < 70):
                signal.referenceVelocity = self.vehicle_max_speed
            elif(time_elapsed < 100):
                vel = self.vehicle_max_speed -(self.vehicle_max_speed/30.0) * (time_elapsed - 70)
                signal.referenceVelocity = vel
            if(signal.referenceVelocity < 0):
                signal.referenceVelocity = 0

            self.vel_signal_publisher.publish(signal)

    def high_acceleration_profile(self):
        while not rospy.is_shutdown():
            time_now = time.time()
            signal = SignalMessage()
            time_elapsed = time_now - self.simulation_start_time
            if(time_elapsed < 15):
                signal.referenceVelocity = (self.vehicle_max_speed/15.0) * time_elapsed
            elif(time_elapsed < 70):
                signal.referenceVelocity = self.vehicle_max_speed
            elif(time_elapsed < 100):
                vel = self.vehicle_max_speed -(self.vehicle_max_speed/30.0) * (time_elapsed - 70)
                signal.referenceVelocity = vel
            if(signal.referenceVelocity < 0):
                signal.referenceVelocity = 0

            self.vel_signal_publisher.publish(signal)

    def acceleration_deceleration_profile(self):
        while not rospy.is_shutdown():
            time_now = time.time()
            signal = SignalMessage()
            time_elapsed = time_now - self.simulation_start_time
            if(time_elapsed < 50):
                signal.referenceVelocity = (self.vehicle_max_speed/50) * time_elapsed
            elif(time_elapsed < 100):
                vel = self.vehicle_max_speed -(self.vehicle_max_speed/50.0) * (time_elapsed - 50)
                signal.referenceVelocity = vel
            if(signal.referenceVelocity < 0):
                signal.referenceVelocity = 0

            self.vel_signal_publisher.publish(signal)

    def reference_velocity_callback(self, signal_msg):
        self.desired_velocity = signal_msg.velocity

    def stop_simulation(self):
        pass

    def simulate_longitudinal_drive(self):
        rate = rospy.Rate(1/self.data_interval)
        count = 0
        entries = len(self.accel_vel_profile)

        while not rospy.is_shutdown():
            entry = self.accel_vel_profile[count]
            t = entry['t']
            linear_v = entry['v-lin']
            steer_angle = entry['steer-ang']

            signal = SignalMessage()
            signal.referenceVelocity = linear_v
            signal.referenceSteerAngle = steer_angle

            self.vel_signal_publisher.publish(signal)

            if(count + 1 < entries):
                count += 1
            rate.sleep()

    def simulate_skid_pad(self):
        rate = rospy.Rate(1/self.data_interval)
        count = 0
        entries = len(self.accel_vel_profile)

        while not rospy.is_shutdown():
            entry = self.accel_vel_profile[count]
            t = entry['t']
            linear_v = entry['v-lin']
            steer_angle = entry['steer-ang']

            signal = SignalMessage()
            signal.referenceVelocity = linear_v
            signal.referenceSteerAngle = steer_angle

            self.vel_signal_publisher.publish(signal)

            if(count + 1 < entries):
                count += 1
            rate.sleep()

    def simulate_endurance_test(self):
        rate = rospy.Rate(1/self.data_interval)
        count = 0
        entries = len(self.accel_vel_profile)

        while not rospy.is_shutdown():
            entry = self.accel_vel_profile[count]
            t = entry['t']
            linear_v = entry['v-lin']
            steer_angle = entry['steer-ang']

            signal = SignalMessage()
            signal.referenceVelocity = linear_v
            signal.referenceSteerAngle = steer_angle

            self.vel_signal_publisher.publish(signal)

            if(count + 1 < entries):
                count += 1
            rate.sleep()

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

            signal = SignalMessage()
            signal.referenceVelocity = linear_v
            signal.referenceSteerAngle = steer_angle

            self.vel_signal_publisher.publish(signal)

            if(count + 1 < entries):
                count += 1
            rate.sleep()

    def simulate_circular_movement(self):
        rate = rospy.Rate(1/self.data_interval)
        count = 0
        entries = len(self.accel_vel_profile)


        while not rospy.is_shutdown():
            entry = self.circular_motion_vel_profile[count]
            t = entry['t']
            linear_v = entry['v-lin']
            steer_angle = entry['steer-ang']

            signal = SignalMessage()
            signal.referenceVelocity = linear_v
            signal.referenceSteerAngle = 0.75

            self.vel_signal_publisher.publish(signal)

            if(count + 1 < entries):
                count += 1
            rate.sleep()

    def simulate_constant_rotation(self):
        while not rospy.is_shutdown():
            signal = SignalMessage()
            signal.referenceVelocity = 25
            signal.referenceSteerAngle = 0.75

            self.vel_signal_publisher.publish(signal)




if __name__ == "__main__":
    rospy.init_node("signal_generator_node")
    SIGNAL_GENERATOR()
    rospy.spin()
