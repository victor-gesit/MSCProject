#!/usr/bin/env python3

import rospy
from electric_car.msg import PWMVoltageMessage
from electric_car.msg import DriveMessage
from electric_car.msg import SimulationCommand
from electric_car.msg import VehicleConfiguration

class CONTROLLED_PWM_VOLTAGE:
    def __init__(self):
        self.min_voltage = 0
        self.max_voltage = 12
        self.voltage = 0

        # Gas Pedal
        self.gas_pedal_angle = 0
        self.min_gas_pedal_angle = 0
        self.max_gas_pedal_angle = 0

        self.simulation_started = False

        # Simulation Command Subscriber
        rospy.Subscriber("simulation_command", SimulationCommand, callback=self.start_simulation)

        # Vehicle Configuration Subscriber
        rospy.Subscriber("configure_vehicle", VehicleConfiguration, callback=self.configure_vehicle)

        # Drive Command Subscriber     
        rospy.Subscriber("drive_command", DriveMessage, callback=self.driver_command_callback)

        # PWM Voltage Publisher
        self.pwm_voltage_publisher = rospy.Publisher("pwm_voltage", PWMVoltageMessage, queue_size=1)
    
    def start_simulation(self, simulation_command):
        if(self.simulation_started):
            return
        
        self.simulation_started = True
        self.publish_pwm_signal()
    
    def configure_vehicle(self, configuration):
        self.min_voltage = configuration.pwmMinOutputVoltage
        self.max_voltage = configuration.pwmMaxOutputVoltage
        self.min_gas_pedal_angle = configuration.driverMinGasPedalAngle
        self.max_gas_pedal_angle = configuration.driverMaxGasPedalAngle
        

    def driver_command_callback(self, driver_command):
        self.gas_pedal_angle = driver_command.gasPedalAngle
        self.min_gas_pedal_angle = driver_command.minGasPedalAngle
        self.max_gas_pedal_angle = driver_command.maxGasPedalAngle

    def publish_pwm_signal(self):
        while not rospy.is_shutdown():
            if(self.max_gas_pedal_angle != 0):
                self.voltage = (self.gas_pedal_angle - self.min_gas_pedal_angle)/self.max_gas_pedal_angle * self.max_voltage + self.min_voltage
            pwm_message = PWMVoltageMessage()
            pwm_message.voltage = self.voltage
            self.pwm_voltage_publisher.publish(pwm_message)

if __name__ == "__main__":
    rospy.init_node("pwm_voltage_node")
    CONTROLLED_PWM_VOLTAGE()
    rospy.spin()
