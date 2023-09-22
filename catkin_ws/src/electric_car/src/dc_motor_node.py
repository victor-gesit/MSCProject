#!/usr/bin/env python3

import rospy
from electric_car.msg import MotorVoltage
from electric_car.msg import MotorTorque
from electric_car.msg import VehicleConfiguration
from electric_car.msg import SimulationCommand
from electric_car.msg import PowerDraw
from geometry_msgs.msg import Twist
import math

# Total Electric Angle
# Reduced Electrical Angle

class DC_MOTOR:
    def __init__(self):
        self.simulation_started = False
        self.torque_constant = 0
        self.armature_resistance = 0
        self.back_emf_constant = 0
        self.wheel_rotational_speed = 0 # rad/s
        self.voltage = 0

        # Motor Details

        # Signal Subscriber     
        rospy.Subscriber("feedback_velocity", Twist, callback=self.feedback_velocity_callback)
        
        # Simulation Command Subscriber
        rospy.Subscriber("simulation_command", SimulationCommand, callback=self.start_simulation)

        # Torque Publisher
        self.motor_torque_publisher = rospy.Publisher("motor_torque", MotorTorque, queue_size=1)
        
        # Power Draw Publisher
        self.power_draw_publisher = rospy.Publisher("power_draw", PowerDraw, queue_size=1)
        

        # H-Bridge Voltage Subscriber
        rospy.Subscriber("motor_voltage", MotorVoltage, callback=self.voltage_callback)
        
        # Vehicle Configuration Subscriber
        rospy.Subscriber("configure_vehicle", VehicleConfiguration, callback=self.configure_vehicle)

    def start_simulation(self, simulation_command):
        if(self.simulation_started):
            return
        self.publish_torque()
        self.simulation_started = True
    
    def configure_vehicle(self, configuration):
        print("Configuring: ", configuration)
        self.armature_resistance = configuration.motorResistance
        self.inductance = configuration.motorInductance
        self.back_emf_constant = configuration.motorBackEMFConstant
        self.torque_constant = configuration.motorTorqueConstant


    def voltage_callback(self, data):
        self.voltage = data.voltage

    def get_voltage(self):
        return self.voltage
    
    def feedback_velocity_callback(self, twist_msg):
        self.wheel_rotational_speed = twist_msg.linear.y
    
    def compute_torque(self):
        # This will use the voltage
        back_emf = self.back_emf_constant * self.wheel_rotational_speed
        voltage = self.voltage
        effective_voltage = voltage - back_emf if voltage - back_emf > 0 else 0
        armature_current = (effective_voltage)/self.armature_resistance if self.armature_resistance > 0 else 0

        torque = self.torque_constant * armature_current
        
        power_draw = PowerDraw()
        power_draw.current = armature_current
        power_draw.voltage = (self.voltage - back_emf)

        self.power_draw_publisher.publish(power_draw)

        return torque
    
    def publish_torque(self):
        while not rospy.is_shutdown():
            torque = MotorTorque()
            torque.torque = self.compute_torque()
            self.motor_torque_publisher.publish(torque)

if __name__ == "__main__":
    rospy.init_node("power_node")
    DC_MOTOR()
    rospy.spin()
