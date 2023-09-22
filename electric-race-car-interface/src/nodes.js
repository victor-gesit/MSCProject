import { Handle, Position } from 'reactflow'
import React, { memo } from 'react'
import {CiWavePulse1 } from 'react-icons/ci'
import {GiSteeringWheel } from 'react-icons/gi'
import { TbBuildingBridge } from 'react-icons/tb'
import { SiDiscogs } from 'react-icons/si'
import { FaCog } from 'react-icons/fa'
import { FaCogs } from 'react-icons/fa'
import { AiFillCar } from 'react-icons/ai'

const nodeInputStyle = {
    padding: '5px',
    margin: '50x'
}
export const SignalGeneratorNode = memo(({ data }) => {
    return (
        <>
            <div>
               <CiWavePulse1/> 
               <p>Signal Generator Node</p>
               <select value={data.simulationType} onChange={(e) => data.onChange({ key: 'simulationType', value: e.target.value})}>
                <option value={1}>Trapezoidal Profile</option>
                <option value={2}>High Acceleration Profile</option>
                <option value={3}>Acceleration/Deceleration Profile</option>
               </select>
            </div>
            <Handle type="source" position={Position.Bottom} isConnectable={true} id="signal_bottom" />
        </>
    )
})

export const DriverNode = memo(({ data }) => {
    return (
        <>
            <Handle type="target" position={Position.Top} isConnectable={true} id="driver_top" />
            <GiSteeringWheel/> <p>Driver</p>
            <div style={nodeInputStyle}>
                <label>Min Gas Pedal:</label>
                <input readOnly={data.simulationStarted} type="number" value={data.config.driverMinGasPedalAngle} onChange={(e) => data.onChange({ key: 'driverMinGasPedalAngle', value: e.target.value })} />
            </div>

            <div style={nodeInputStyle}>
                <label>Max Gas Pedal:</label>
                <input  readOnly={data.simulationStarted} type="number" value={data.config.driverMaxGasPedalAngle} onChange={(e) => data.onChange({ key: 'driverMaxGasPedalAngle', value: e.target.value })} />
            </div>

            <div style={nodeInputStyle}>
                <label>Control kP:</label>
                <input  readOnly={data.simulationStarted} type="number" value={data.config.driverKP} onChange={(e) => data.onChange({ key: 'driverKP', value: e.target.value })} />
            </div>

            <div style={nodeInputStyle}>
                <label>Control kI:</label>
                <input  readOnly={data.simulationStarted} type="number" value={data.config.driverKI} onChange={(e) => data.onChange({ key: 'driverKI', value: e.target.value })} />
            </div>

            <div style={nodeInputStyle}>
                <label>Max Speed:</label>
                <input readOnly={data.simulationStarted} type="number" value={data.config.driverVehicleMaxSpeed} onChange={(e) => data.onChange({ key: 'driverVehicleMaxSpeed', value: e.target.value })} />
            </div>
            <Handle type="source" position={Position.Bottom} isConnectable={true} id="driver_bottom" />
        </>
    )
})

export const PWMNode = memo(({ data }) => {
    return (
        <>
            <Handle type="target" position={Position.Top} isConnectable={true} id="pwm_top" />
            <p>Controlled PWM Voltage</p>
            <div style={nodeInputStyle}>
                <label>Min Output Voltage:</label>
                <input readOnly={data.simulationStarted} value={data.config.pwmMinOutputVoltage } type="number" onChange={(e) => data.onChange({ key: 'pwmMinOutputVoltage', value: e.target.value })} />
            </div>

            <div>
                <label>Max Output Voltage:</label>
                <input readOnly={data.simulationStarted} value={data.config.pwmMaxOutputVoltage}  type="number" onChange={(e) => data.onChange({ key: 'pwmMaxOutputVoltage', value: e.target.value })} />
            </div>
            <Handle type="source" position={Position.Bottom} isConnectable={true} id="pwm_bottom" />
        </>
    )
})

export const HBridgeNode = memo(({ data }) => {
    return (
        <div>
            <Handle type="target" position={Position.Top} isConnectable={true} id="h-bridge_top" />
            <TbBuildingBridge/>
            <p>H Bridge</p>
            <div style={nodeInputStyle}>
                <label>Min Output Voltage:</label>
                <input readOnly={data.simulationStarted} type="number" value={data.config.hBridgeMinOutputVoltage} onChange={(e) => data.onChange({ key: 'hBridgeMinOutputVoltage', value: e.target.value })} />
            </div>

            <div style={nodeInputStyle}>
                <label>Max Output Voltage:</label>
                <input readOnly={data.simulationStarted} type="number" value={data.config.hBridgeMaxOutputVoltage} onChange={(e) => data.onChange({ key: 'hBridgeMaxOutputVoltage', value: e.target.value })} />
            </div>
            <Handle type="source" position={Position.Bottom} isConnectable={true} id="h-bridge_bottom" />
        </div>
    )
})

export const MotorNode = memo(({ data }) => {
    return (
        <>
            <Handle type="target" position={Position.Top} isConnectable={true} id="motor_top" />
            <SiDiscogs/>
            <p>Motor</p>
            <div style={nodeInputStyle}>
                <label>Resistance:</label>
                <input readOnly={data.simulationStarted} type="number" value={data.config.motorResistance} onChange={(e) => data.onChange({ key: 'motorResistance', value: e.target.value })} />
            </div>

            <div>
                <label>Inductance:</label>
                <input readOnly={data.simulationStarted} type="number" value={data.config.motorInductance} onChange={(e) => data.onChange({ key: 'motorInductance', value: e.target.value })} />
            </div>


            <div>
                <label>Back EMF Constant:</label>
                <input readOnly={data.simulationStarted} type="number" value={data.config.motorBackEMFConstant} onChange={(e) => data.onChange({ key: 'motorBackEMFConstant', value: e.target.value })} />
            </div>

            <div>
                <label>Motor Torque Constant:</label>
                <input readOnly={data.simulationStarted} type="number" value={data.config.motorTorqueConstant} onChange={(e) => data.onChange({ key: 'motorTorqueConstant', value: e.target.value })} />
            </div>

            
            <Handle type="source" position={Position.Bottom} isConnectable={true} id="motor_bottom" />
        </>
    )
})

export const GearNode = memo(({ data }) => {
    return (
        <>
            <Handle type="target" position={Position.Top} isConnectable={true} id="gear_top" />
            <FaCog/>
            <p>Simple Gear</p>

            <div style={nodeInputStyle}>
                <label>Gear Ratio</label>
                <input readOnly={data.simulationStarted} type="number" value={data.config.gearRatio} onChange={(e) => data.onChange({ key: 'gearRatio', value: e.target.value })} />
            </div>

            <div style={nodeInputStyle}>
                <label>Gear Efficiency</label>
                <input readOnly={data.simulationStarted} type="number" value={data.config.gearEfficiency} onChange={(e) => data.onChange({ key: 'gearEfficiency', value: e.target.value })} />
            </div>
            <Handle type="source" position={Position.Bottom} isConnectable={true} id="gear_bottom" />
        </>
    )
})

export const DifferentialNode = memo(({ data }) => {
    return (
        <>
            <Handle type="target" position={Position.Top} isConnectable={true} id="differential_top" />
            <FaCogs/>
            <p>Differential</p>

            <div style={nodeInputStyle}>
                <label>Differential Efficiency:</label>
                <input readOnly={data.simulationStarted} type="number" value={data.config.differentialEfficiency} onChange={(e) => data.onChange({ key: 'differentialEfficiency', value: e.target.value })} />
            </div>
            <Handle type="source" position={Position.Bottom} isConnectable={true} id="differential_bottom" />
        </>
    )
})

export const BatteryNode = memo(({ data }) => {
    return (
        <>
            <Handle type="target" position={Position.Left} isConnectable={true} id="battery_left" />
            <AiFillCar/>
            <p>Battery</p>

            <div style={nodeInputStyle}>
                <label>Battery Capacity</label>
                <input readOnly={data.simulationStarted} type="number" value={data.config.batteryCapacity} onChange={(e) => data.onChange({ key: 'batteryCapacity', value: e.target.value })} />
            </div>

            <div style={nodeInputStyle}>
                <label>Battery Level</label>
                <input readOnly={data.simulationStarted} type="number" value={data.config.batteryLevel} onChange={(e) => data.onChange({ key: 'batteryLevel', value: e.target.value })} />
            </div>
            <Handle type="source" position={Position.Bottom} isConnectable={true} id="vehicle-body_bottom" />
        </>
    )

})

export const VehicleBodyNode = memo(({ data }) => {
    return (
        <>
            <Handle type="target" position={Position.Top} isConnectable={true} id="vehicle-body_top" />
            <AiFillCar/>
            <p>Vehicle Body</p>

            <div style={nodeInputStyle}>
                <label>Drag Coefficient</label>
                <input readOnly={data.simulationStarted} type="number" value={data.config.vehicleBodyDragCoefficient} onChange={(e) => data.onChange({ key: 'vehicleBodyDragCoefficient', value: e.target.value })} />
            </div>

            <div style={nodeInputStyle}>
                <label>Reference Area</label>
                <input readOnly={data.simulationStarted} type="number" value={data.config.vehicleBodyReferenceArea} onChange={(e) => data.onChange({ key: 'vehicleBodyReferenceArea', value: e.target.value })} />
            </div>
            <Handle type="source" position={Position.Bottom} isConnectable={true} id="vehicle-body_bottom" />
        </>
    )
})
