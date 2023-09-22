#!/usr/bin/env python3

import rospy
from electric_car.msg import SimulationCommand
from electric_car.msg import MotorTorque

class SIMPLE_GEAR:
    def __init__(self):
        self.simulation_started = False
        self.gear_ratio = 0.75
        self.input_torque = 0
        self.efficiency = 0.95

        # Simulation Command Subscriber
        rospy.Subscriber("simulation_command", SimulationCommand, callback=self.start_simulation)


        # Motor Torque Subscriber     
        rospy.Subscriber("motor_torque", MotorTorque, callback=self.motor_torque_callback)

        # Output Gear Torque Publisher
        self.output_gear_torque_publisher = rospy.Publisher("output_gear_torque", MotorTorque, queue_size=1)
        
    def start_simulation(self, _):
        if(self.simulation_started):
            return
        self.publish_output_torque()
        self.simulation_started = True

    def motor_torque_callback(self, torque):
        self.input_torque = torque.torque
        

    def get_input_torque(self):
        return self.input_torque
    
    def compute_output_torque(self):
        return self.get_input_torque() * 12 * self.gear_ratio * self.efficiency
        
    def publish_output_torque(self):
        while not rospy.is_shutdown():
            torque = MotorTorque()
            torque.torque = self.compute_output_torque()
            self.output_gear_torque_publisher.publish(torque)

if __name__ == "__main__":
    rospy.init_node("simple_gear_node")
    SIMPLE_GEAR()
    rospy.spin()
