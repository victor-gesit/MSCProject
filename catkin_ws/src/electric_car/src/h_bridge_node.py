#!/usr/bin/env python3

import rospy
from electric_car.msg import PWMVoltageMessage
from electric_car.msg import DriveMessage
from electric_car.msg import MotorVoltage
from electric_car.msg import SimulationCommand
from electric_car.msg import VehicleConfiguration
from electric_car.msg import BatteryStatus

class H_BRIDGE:
    def __init__(self):
        self.simulation_started = False
        self.braking = False

        self.min_pwm_voltage = 0
        self.input_pwm_voltage = 0
        self.max_pwm_voltage = 12

        self.max_output_voltage = 50

        # Simulation Command Subscriber
        rospy.Subscriber("simulation_command", SimulationCommand, callback=self.start_simulation)


        # Drive Command Subscriber     
        rospy.Subscriber("drive_command", DriveMessage, callback=self.driver_command_callback)

        # PWM Voltage Subscriber     
        rospy.Subscriber("pwm_voltage", PWMVoltageMessage, callback=self.pwm_voltage_callback)

        # Motor Voltage Publisher
        self.motor_voltage_publisher = rospy.Publisher("motor_voltage", MotorVoltage, queue_size=1)
        

        # Vehicle Configuration Subscriber
        rospy.Subscriber("configure_vehicle", VehicleConfiguration, callback=self.configure_vehicle)


        # Battery Status Subscriber
        rospy.Subscriber("battery_status", BatteryStatus, callback=self.battery_status_callback)


    def configure_vehicle(self, configuration):
        self.max_output_voltage = configuration.batteryOutputVoltage
        self.max_pwm_voltage = configuration.pwmMaxOutputVoltage

    def battery_status_callback(self, battery_status):
        self.max_output_voltage = battery_status.maxVoltage

    def start_simulation(self, simulationCommand):
        if(self.simulation_started):
            return
        
        self.publish_motor_voltage()
        self.simulation_started = True

    def pwm_voltage_callback(self, pwm):
        self.input_pwm_voltage = pwm.voltage

    def compute_output_voltage(self):
        if(self.max_output_voltage == 0):
            return 0
        output_v = (self.input_pwm_voltage / self.max_pwm_voltage) * self.max_output_voltage
        return output_v if self.max_pwm_voltage > 0 else 0

    def driver_command_callback(self, driver_command):
        if(driver_command.brake):
            self.braking = True
        else:
            self.braking = False

    def publish_motor_voltage(self):
        while not rospy.is_shutdown():
            motor_message = MotorVoltage()
            
            if(self.braking):
                motor_message.voltage = 0
                self.motor_voltage_publisher.publish(motor_message)
            else:
                motor_message.voltage = self.compute_output_voltage()
                self.motor_voltage_publisher.publish(motor_message)

if __name__ == "__main__":
    rospy.init_node("h_bridge_node")
    H_BRIDGE()
    rospy.spin()
