
import ROSLIB from 'roslib';

export const startSimulationTopic = (ros) => {
    return new ROSLIB.Topic({
    ros: ros,
    name: '/simulation_command',
    messageType: 'electric_car/SimulationCommand',
})}

export const velocitySignalTopic = (ros) => {return new ROSLIB.Topic({
    ros,
    name: '/velocity_signal',
    messageType: 'electric_car/SignalMessage',
})}

export const driveCommandTopic = (ros) => {return new ROSLIB.Topic({
    ros,
    name: '/drive_command',
    messageType: 'electric_car/DriveMessage',
})}

export const controlledPwmVoltageTopic = (ros) => {return new ROSLIB.Topic({
    ros,
    name: '/pwm_voltage',
    messageType: 'electric_car/PWMVoltageMessage',
})}

export const motorVoltageTopic = (ros) => {return new ROSLIB.Topic({
    ros,
    name: '/motor_voltage',
    messageType: 'electric_car/MotorVoltage',
})}

export const motorTorqueTopic = (ros) => {return new ROSLIB.Topic({
    ros,
    name: '/motor_torque',
    messageType: 'electric_car/MotorTorque',
})}

export const vehicleConfigurationTopic = (ros) => {return new ROSLIB.Topic({
    ros,
    name: '/configure_vehicle',
    messageType: 'electric_car/VehicleConfiguration',
})}



export const feedbackVelocityTopic = (ros) => {return new ROSLIB.Topic({
    ros,
    name: '/cmd_vel',
    messageType: 'geometry_msgs/Twist',
})}

