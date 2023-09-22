#!/usr/bin/env python3

import rospy
from electric_car.msg import SimulationCommand
from electric_car.msg import MotorTorque
from electric_car.msg import DriveMessage
from electric_car.msg import VehicleDrive
from electric_car.msg import VehicleConfiguration

class DIFFERENTIAL:
    def __init__(self):
        self.simulation_started = False
        
        self.input_torque = 0
        self.efficiency = 0.95
        self.number_of_wheels = 2

        self.steer_angle = 0.785398

        # Drive Command Subscriber     
        rospy.Subscriber("drive_command", DriveMessage, callback=self.drive_command_callback)

        # Simulation Command Subscriber
        rospy.Subscriber("simulation_command", SimulationCommand, callback=self.start_simulation)

        # Gear Output Torque Subscriber     
        rospy.Subscriber("output_gear_torque", MotorTorque, callback=self.gear_output_torque_callback)

        # Motor Voltage Publisher
        self.vehicle_drive_command_publisher = rospy.Publisher("vehicle_drive", VehicleDrive, queue_size=1)
        

        # Vehicle Configuration Subscriber
        rospy.Subscriber("configure_vehicle", VehicleConfiguration, callback=self.configure_vehicle)


    def drive_command_callback(self, drive_command):
        pass
        #self.steer_angle = drive_command.steerAngle

    def get_steer_angle(self):
        return self.steer_angle
    
    def start_simulation(self, _):
        if(self.simulation_started):
            return
        self.publish_wheel_torque()
        self.simulation_started = True

    def configure_vehicle(self, configuration):
        self.efficiency = configuration.differentialEfficiency

    def gear_output_torque_callback(self, torque):
        self.input_torque = torque.torque

    def get_input_torque(self):
        return self.input_torque
    
    def compute_output_torque(self):
        return self.get_input_torque() / self.number_of_wheels * self.efficiency


    def publish_wheel_torque(self):
        while not rospy.is_shutdown():
            vehicle_drive = VehicleDrive()
            vehicle_drive.wheelTorque = self.compute_output_torque()
            vehicle_drive.steerAngle = self.get_steer_angle()

            vehicle_drive.wheelTorque = vehicle_drive.wheelTorque

            # print("TTOO: ", vehicle_drive.wheelTorque, vehicle_drive.steerAngle)
            self.vehicle_drive_command_publisher.publish(vehicle_drive)

if __name__ == "__main__":
    rospy.init_node("differential_node")
    DIFFERENTIAL()
    rospy.spin()

