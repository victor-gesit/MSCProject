import logo from './logo.svg';
import './App.css';
import { useEffect, useState, useCallback } from 'react'
import ROSLIB from 'roslib';
import { startSimulationTopic,
  velocitySignalTopic,
  driveCommandTopic,
  controlledPwmVoltageTopic,
  motorVoltageTopic,
  motorTorqueTopic,
  feedbackVelocityTopic,
  vehicleConfigurationTopic
} from  './topics'

import ReactFlow, { addEdge, useEdgesState, useNodesState } from 'reactflow'
import { SignalGeneratorNode,
  DriverNode,
  PWMNode,
  HBridgeNode,
  MotorNode,
  GearNode,
  DifferentialNode,
  VehicleBodyNode
 } from './nodes';

 const buttonsContainerStyle = {
  width: '100px',
  height: '80px',
  position: 'absolute',
  top: '40px',
  left: '40px',
  display: 'flex',
  justifyContent: 'center',
  flexDirection: 'column',
  backgroundColor: 'rgba(255,0,0, 1)',
  padding: '20px'
 }

const every = './topics'
const rfStyle = {
  backgroundColor: '#D8D6CE'
}

const initBgColor = '#1A192B';
const defaultViewport = { x: 0, y: 0, zoom: 1.5 };

const connectionLineStyle = { stroke: '#fff' };
const snapGrid = [20, 20];


const nodeTypes = {
  signalGen: SignalGeneratorNode,
  driver: DriverNode,
  pwm: PWMNode,
  h_bridge: HBridgeNode,
  motor: MotorNode,
  gear: GearNode,
  differential: DifferentialNode,
  vehicleBody: VehicleBodyNode
}


const defaultVehicleConfig = {
  driverMinGasPedalAngle: 0,
  driverMaxGasPedalAngle: 25,
  driverKP: 0.5,
  driverKI: 1,
  driverVehicleMaxSpeed: 100,

  pwmMinOutputVoltage: 0,
  pwmMaxOutputVoltage: 12, //

  hBridgeMinOutputVoltage: 0,
  hBridgeMaxOutputVoltage: 40,

  motorResistance: 1,
  motorInductance: 0.00012,
  motorBackEMFConstant: 1,
  motorTorqueConstant: 1,

  gearRatio: 0.85, // 3.1 11 to 34
  gearEfficiency: 0.95,

  differentialEfficiency: 0.95,

  vehicleBodyDragCoefficient: 0.25,
  vehicleBodyReferenceArea: 0.85,
  
  batteryCapacity: 100, // AH (30.2 MJ)
  batteryPercentage: 100, // % MOTOR IS EMRAX 228
  batteryOutputVoltage: 50 // Volts // 367.2 Volts Nominal// 380 to 390 - 450
}

const initialEdges = (simulationStarted) => [
  {
    id: 'sigGen-driver', source: 'signalGen', sourceHandle: 'signal_bottom', target: 'driver', 
    animated: simulationStarted,
    style: { stroke: '#fff' },
  },
  {
    id: 'driver-pwm', source: 'driver', sourceHandle: 'driver_bottom', target: 'pwm',
    animated: simulationStarted,
    style: { stroke: '#fff' },
  },
  {
    id: 'pwm-h_bridge', source: 'pwm', target: 'h_bridge', sourceHandle: 'pwm_bottom',
    animated: simulationStarted,
    style: { stroke: '#fff' },
  },
  {
    id: 'h_bridge-motor', source: 'h_bridge', target: 'motor', sourceHandle: 'h-bridge_bottom',
    animated: simulationStarted,
    style: { stroke: '#fff' },
  },
  {
    id: 'motor-gear', source: 'motor', target: 'gear', sourceHandle: 'motor_bottom',
    animated: simulationStarted,
    style: { stroke: '#fff' },
  },
  {
    id: 'gear-differential', source: 'gear', target: 'differential', sourceHandle: 'gear_bottom',
    animated: simulationStarted,
    style: { stroke: '#fff' },
  },
  {
    id: 'differential-vehicleBody', source: 'differential', target: 'vehicleBody', sourceHandle: 'differential_bottom',
    animated: simulationStarted,
    style: { stroke: '#fff' },
  },

]


function App() {
  const [rosInstance, setRosInstance] = useState(null)
  const [rosConnected, setRosConnected] = useState(false)
  const [simulationStarted, setSimulationStarted] = useState(false)
  const [vehicleConfig, setVehicleConfig] = useState(defaultVehicleConfig) 
  const [simulationType, setSimulationType] = useState(1)

  const [nodes, setNodes, onNodesChange] = useNodesState([]);
  const [edges, setEdges, onEdgesChange] = useEdgesState([]);

  const ros = new ROSLIB.Ros({
    url: 'ws://0.0.0.0:9090'
  })

  const startSimTopic = startSimulationTopic(ros)
  const vehicleConfigTopic = vehicleConfigurationTopic(ros)
  const velSignalTopic = velocitySignalTopic(ros)
  const driveCommTopic = driveCommandTopic(ros)
  const controlledPWMVoltTopic = controlledPwmVoltageTopic(ros)
  const motorVoltTopic = motorVoltageTopic(ros)
  const motorTorqTopic = motorTorqueTopic(ros)
  const feedbckVelTopic = feedbackVelocityTopic(ros)

  const onChange = ({ key, value }) => {
    let vConfig = {...vehicleConfig}
    if(key === "simulationType") {
      setSimulationType(value)
    } else {
      vConfig[key] = value
      setVehicleConfig(vConfig)
    }
  }

  useEffect(() => {
    setNodes((nds) => {
      const updatedNodes = nds.map((node) => {
        let value = vehicleConfig[node.id]
        if(node.id == "signalGen") {
          return {
            ...node,
            simulationType: value
          }
        }
        return {
          ...node,
          data: {
            ...node.data,
            config: vehicleConfig
          },
        }
      })
      return updatedNodes
    })
  }, [vehicleConfig])


  useEffect(() => {
    // const onChange = ({ key, value}) => {
    //   let vConfig = {...vehicleConfig}
    //   console.log("CCONNF: ", vehicleConfig, key, value)
    //   vConfig[key] = value
    //   setVehicleConfig(vConfig)


    //   if(key == "simulationType") {
    //     setSimulationType(value)
    //   }

    //   setNodes((nds) => {
    //     const updatedNodes = nds.map((node) => {
    //       if(node.id == "") {
    //         return {
    //           ...node,
    //           simulationType: value
    //         }
    //       }
    //       return {
    //         ...node,
    //         data: {
    //           ...node.data,
    //           config: vConfig
    //         },
    //       }
    //     })
    //     return updatedNodes
    //   })
    // }

    console.log("HHHHHAA")

    const initialNodes = [
      {
        id: 'signalGen',
        type: 'signalGen',
        position: { x: -50, y: 0},
        data: { onChange: onChange, simulationStarted, config: vehicleConfig },
        style: { border: '1px solid #777', padding: 10 },
      },
      {
        id: 'driver', 
        type: 'driver', 
        position: { x: -30, y: 200}, 
        data: { onChange: onChange, simulationStarted, config: vehicleConfig },
        style: { border: '1px solid #777', padding: 10 },
      },
      {
        id: 'pwm', 
        type: 'pwm', 
        position: { x: -10, y: 500}, 
        data: { onChange: onChange, simulationStarted, config: vehicleConfig },
        style: { border: '1px solid #777', padding: 10 },
      },
      {
        id: 'h_bridge', 
        type: 'h_bridge', 
        position: { x: 10, y: 700}, 
        data: { onChange: onChange, simulationStarted, config: vehicleConfig },
        style: { border: '1px solid #777', padding: 10 },
      },
      {
        id: 'motor', 
        type: 'motor', 
        position: { x: 30, y: 900}, 
        data: { onChange: onChange, simulationStarted, config: vehicleConfig },
        style: { border: '1px solid #777', padding: 10 },
      },
      {
        id: 'gear', 
        type: 'gear', 
        position: { x: 50, y: 1100}, 
        data: { onChange: onChange, simulationStarted, config: vehicleConfig },
        style: { border: '1px solid #777', padding: 10 },
      },
      {
        id: 'differential', 
        type: 'differential', 
        position: { x: 70, y: 1300}, 
        data: { onChange: onChange, simulationStarted, config: vehicleConfig },
        style: { border: '1px solid #777', padding: 10 },
      },
      {
        id: 'vehicleBody', 
        type: 'vehicleBody', 
        position: { x: 90, y: 1500}, 
        data: { onChange: onChange, simulationStarted, config: vehicleConfig },
        style: { border: '1px solid #777', padding: 10 },
      }
    ]

    setNodes(initialNodes)
    setEdges(initialEdges(simulationStarted))
  }, [simulationStarted])

  const configureRaceCar = () => {
    let vC = {}
    Object.keys(vehicleConfig).forEach((key) => {
        vC[key] = parseFloat(vehicleConfig[key])
    })
    
    const configuration = new ROSLIB.Message(vC)
    vehicleConfigTopic.publish(configuration)
  }

  const startSimulation = () => {
    // if(simulationStarted) {
    //   return
    // }
    const simType = parseInt(simulationType)
    const strtSim = new ROSLIB.Message({
      simulationType: simType
    })
    
    startSimTopic.publish(strtSim)
    // setSimulationStarted(true)
  }

  const onConnect = useCallback(
    (params) =>
      setEdges((eds) => addEdge({ ...params, animated: true, style: { stroke: '#fff' } }, eds)),
    []
  );

  return (
    <div className="App">
      {/* <header className="App-header">
        <button onClick={() => startSimulation(4)}>
          Start Simulation
        </button>
        
      </header> */}

      <ReactFlow
        nodes={nodes}
        edges={edges}
        onNodesChange={onNodesChange}
        onEdgesChange={onEdgesChange}
        onConnect={onConnect}
        nodeTypes={nodeTypes}
        connectionLineStyle={connectionLineStyle}
        defaultViewport={defaultViewport}
        snapToGrid={true}
        fitView
        style={rfStyle}
      />
      <div style={buttonsContainerStyle}>
        <button style={{marginBottom: '10px'}} onClick={() => configureRaceCar()}>Configure Electric Vehicle</button>
        <button onClick={() => startSimulation()}>Start Simulation</button>
      </div>
      </div>
  );
}

export default App;
