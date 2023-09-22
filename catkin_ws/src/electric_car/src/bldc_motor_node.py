#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
from electric_car.msg import BatteryStatus
from electric_car.msg import MotorInfo
import math

# Total Electric Angle
# Reduced Electrical Angle

class BLDC_MOTOR:
    def __init__(self, battery_level=100):
        self.voltage_freq = 60 # hz
        self.voltage_period = 1
        self.battery_level = battery_level
        self.running = False
        
        # Battery Level Publisher

        battery_status = BatteryStatus()
        battery_status.percentage_left = 20

        rate = rospy.Rate(self.voltage_freq/12)
        
        current_angle = 0
        angle_fraction = math.pi/6
        while not rospy.is_shutdown():
            pass

    
    def compute_rpm(self):

        pass

    def commutator_table(self, hall_values):
        table = {
            '000': '52',
            '001': '12',
            '010': '50',
            '011': [],
            '100': [],
            '101': '14',
            '110': '30',
            '111': '34'
        }

        transistors_to_activate = table[hall_values]

        return transistors_to_activate
    
    def angle_to_hall_values(self, theta):
        pi = math.pi
        if theta < pi/6:
            return '000'
        if theta < pi/2:
            return '001'
        if theta < 5*pi/6:
            return '101'
        if theta < 7*pi/6:
            return '111'
        if theta < 3*pi/2:
            return '110'
        if theta < 11*pi/6:
            return '010'
        if theta < 2*pi:
            return '000'
    
    def transistors_to_phase_voltage(self, transistors):
        if transistors == '52':
            return [0, -1, 1]
        if transistors == '50':
            return [-1, 0, 1]
        if transistors == '30':
            return [-1, 1, 0]
        if transistors == '34':
            return [0, 1, -1]
        if transistors == '14':
            return [1, 0, -1]
        if transistors == '12':
            return [1, -1, 0]
        pass
        

    def back_emf_to_hall_values(self, back_emfs):
        ea, eb, ec = back_emfs
        line_back_emf_ab = ea - eb
        line_back_emf_bc = eb - ec
        line_back_emf_ca = ec - ea

        hb = 1 if line_back_emf_ab < 0 else 0
        ha = 1 if line_back_emf_bc > 0 else 0
        hc = 1 if line_back_emf_ca < 0 else 0

        return ha, hb, hc

    def compute_back_emf_a(self, theta, v):
        pi = math.pi
        if(theta < pi/6):
            return  v * theta / (pi/6)
        
        if(theta < 5*pi/6):
            return v
        
        if(theta < 7*pi/6):
            return v * (v/(pi/6)) * (theta * 5 * pi/6)
        
        if(theta < 11*pi/6):
            return -v
        
        if(theta < 2*pi):
            return -v * (pi/6) * (theta - 11*pi/6)
        
    def compute_back_emf_b(self, theta, v):
        pi = math.pi
        if(theta < pi/2):
            return  -v
        
        if(theta < 5*pi/6):
            return -v + v * (theta - 3 * pi/6)/(pi/6)
        
        if(theta < 9*pi/6):
            return v
        
        if(theta < 11*pi/6):
            return v - v * (theta - 9*pi/6)/(pi/6)
        
        if(theta < 2*pi):
            return -v
    
    def compute_back_emf_c(self, theta, v):
        pi = math.pi
        if(theta < pi/6):
            return  v
        
        if(theta < 3*pi/6):
            return v - v*(theta - pi/6)/(pi/6)
        
        if(theta < 7*pi/6):
            return -v
        
        if(theta < 9*pi/6):
            return -v + v*(theta - 7*pi/6)/(pi/6)
        
        if(theta < 2*pi):
            v
        pass


if __name__ == "__main__":
    rospy.init_node("power_node")
    BLDC_MOTOR()
    rospy.spin()
