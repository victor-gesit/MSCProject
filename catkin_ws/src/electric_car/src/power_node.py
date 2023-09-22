#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
from electric_car.msg import BatteryStatus
from electric_car.msg import PowerDraw
from electric_car.msg import VehicleConfiguration
from electric_car.msg import SimulationCommand
import time
import csv
import os
import rospkg

rospack = rospkg.RosPack()

# roslaunch rosbridge_server rosbridge_websocket.launch

class BatteryNode:
    def __init__(self):
        self.battery_capacity = 100 # AH
        self.remaining_battery_capacity = 100 #AH
        self.battery_level = 100 # percent
        self.battery_output_voltage = 40 # Volts

        self.simulation_data = []
        self.simulation_start_time = time.time()

        self.sim_type = ""


        # Simulation Command Subscriber
        rospy.Subscriber("simulation_command", SimulationCommand, callback=self.start_simulation)


        # Vehicle Configuration Subscriber
        rospy.Subscriber("configure_vehicle", VehicleConfiguration, callback=self.configure_vehicle)

        # Motor Voltage Publisher
        self.battery_level_publisher = rospy.Publisher("battery_status", BatteryStatus, queue_size=1)
        

        # Simulation Command Subscriber
        rospy.Subscriber("power_draw", PowerDraw, callback=self.compute_battery_level)
        self.prev_time = time.time()
        self.prev_current = 0

        self.simulation_started = False
        self.simulation_completed = False

    def start_simulation(self, simulationCommand):
        if(simulationCommand.simulationType == 1):
            self.sim_type = "trapezoid"
        elif(simulationCommand.simulationType == 2):
            self.sim_type = "high-acc"
        elif(simulationCommand.simulationType == 3):
            self.sim_type = "acc-dec"
        elif(simulationCommand.simulationType == 4):
            self.sim_type = "simulation"


        self.simulation_started = True
        self.simulation_start_time = time.time()

    def save_simulation(self):
        print("Saving: ", self.simulation_data)
        header = ["time_stamp", "percentage_left", "battery_capacity_left"]


        package_path = rospack.get_path('electric_car')
        file_name = os.path.join(package_path, "results", f"{self.sim_type}-{time.time()}.csv")
        absolute_path = os.path.abspath(file_name)
        print("Absolute: ", absolute_path)


        with open(file_name, mode='w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=header)
            writer.writeheader()
            for row in self.simulation_data:
                writer.writerow(row)
            pass
        print("Done Saving")

    def compute_battery_level(self, power_draw):
        if(not self.simulation_started):
            return
        
        current_time = time.time()
        elapsed_time = current_time - self.prev_time
        if(elapsed_time < 0.5):
            return
        
        self.prev_time = time.time()

        current = power_draw.current
        elapsed_time_hour = elapsed_time/60
        average_current = abs((current + self.prev_current))/2




        if(self.prev_current != 0 and current == 0 and not self.simulation_completed):
            self.simulation_started = False
            self.simulation_completed = True
            self.save_simulation()
        




        self.prev_current = current

        power_used = average_current * elapsed_time_hour

        remaining_capacity = self.remaining_battery_capacity - power_used
        self.remaining_battery_capacity = remaining_capacity

        percentage_left = (self.remaining_battery_capacity)/self.battery_capacity * 100

        total_elapsed_time = current_time - self.simulation_start_time

        current_sim_data = {
            'time_stamp': total_elapsed_time,
            'percentage_left': percentage_left,
            'battery_capacity_left': remaining_capacity
        }

        self.simulation_data.append(current_sim_data)


        battery_status = BatteryStatus()
        battery_status.percentageLeft = percentage_left
        battery_status.maxVoltage = self.battery_output_voltage

        print("BAT ", battery_status)
        self.battery_level_publisher.publish(battery_status)

    def configure_vehicle(self, configuration):
        self.battery_capacity = configuration.batteryCapacity
        self.remaining_battery_capacity = configuration.batteryCapacity
        self.battery_level = configuration.batteryPercentage
        self.battery_output_voltage = configuration.batteryOutputVoltage


        
        pass
if __name__ == "__main__":
    rospy.init_node("power_node")
    BatteryNode()
    rospy.spin()
