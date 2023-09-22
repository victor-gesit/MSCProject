#!/usr/bin/env python3

import rospy
from electric_car.msg import SignalMessage
from electric_car.msg import DriveMessage
from electric_car.msg import SimulationCommand
from electric_car.msg import VehicleConfiguration
from geometry_msgs.msg import Twist

class DRIVER:
    def __init__(self):
        self.simulation_started = False
        # Velocity and steer angles
        self.feedback_velocity = 0
        self.reference_velocity = 0
        self.feedback_steer_angle = 0
        self.reference_steer_angle = 0.785398

        self.minGasPedalAngle = 0
        self.gasPedalAngle = 0
        self.maxGasPedalAngle = 25

        self.kP = 0.5
        self.kI = 1
        self.reset = 0

        self.vehicle_max_speed = 50


        # Vehicle Configuration Subscriber
        rospy.Subscriber("configure_vehicle", VehicleConfiguration, callback=self.configure_vehicle)

        # Simulation Command Subscriber
        rospy.Subscriber("simulation_command", SimulationCommand, callback=self.start_simulation)

        # Signal Subscriber     
        rospy.Subscriber("velocity_signal", SignalMessage, callback=self.reference_velocity_callback)


        # Signal Subscriber     
        rospy.Subscriber("feedback_velocity", Twist, callback=self.feedback_velocity_callback)

        # Drive Command Publisher
        self.drive_command_publisher = rospy.Publisher("drive_command", DriveMessage, queue_size=1)
        
    def start_simulation(self, simulation_command):
        if(self.simulation_started):
            return
        self.publish_drive_command()
        self.simulation_started = True

    def configure_vehicle(self, configuration):
        self.minGasPedalAngle = configuration.driverMinGasPedalAngle
        self.maxGasPedalAngle = configuration.driverMaxGasPedalAngle
        self.kP = configuration.driverKP
        self.kI = configuration.driverKI
        self.vehicle_max_speed = configuration.driverVehicleMaxSpeed

    def reference_velocity_callback(self, signal_msg):
        self.reference_velocity = signal_msg.referenceVelocity
        self.reference_steer_angle = signal_msg.referenceSteerAngle

    def feedback_velocity_callback(self, twist_msg):
        self.feedback_velocity = twist_msg.linear.x
        self.feedback_steer_angle = twist_msg.angular.z
        # print("FFBBBB: ", twist_msg)

    def get_reference_velocity(self):
        return self.reference_velocity
    
    def get_feedback_velocity(self):
        return self.feedback_velocity
    
    def get_reference_steer_angle(self):
        return self.reference_steer_angle
    
    def get_feedback_steer_angle(self):
        return self.feedback_steer_angle
    
    def publish_drive_command(self):
        while not rospy.is_shutdown():
            reference_vel = self.get_reference_velocity()
            reference_angle = self.get_reference_steer_angle()

            feedback_vel = self.get_feedback_velocity()
            feedback_angle = self.get_feedback_steer_angle()

            desired_gas_pedal_angle = self.reference_velocity/self.vehicle_max_speed * self.maxGasPedalAngle
            current_gas_pedal_angle = feedback_vel/self.vehicle_max_speed * self.maxGasPedalAngle

            error = desired_gas_pedal_angle - current_gas_pedal_angle

            gas_pedal_angle = self.kI * error
            # v_target = gas_pedal_target/self.maxGasPedalAngle * self.vehicle_max_speed
            # v_target = self.kP * error # + self.kI * self.reset

            if(gas_pedal_angle >= self.maxGasPedalAngle):
                gas_pedal_angle = self.maxGasPedalAngle
            if(gas_pedal_angle < 0):
                gas_pedal_angle = 0

            # gas_pedal_angle = (self.vehicle_max_speed - v_target)/self.vehicle_max_speed * self.maxGasPedalAngle
            self.reset = self.reset + error

            drive_message = DriveMessage()
            drive_message.gasPedalAngle = gas_pedal_angle
            drive_message.steerAngle = reference_angle
            drive_message.maxGasPedalAngle = self.maxGasPedalAngle

            self.gasPedalAngle = gas_pedal_angle
            self.drive_command_publisher.publish(drive_message)

if __name__ == "__main__":
    rospy.init_node("driver_node")
    DRIVER()
    rospy.spin()
