from pydrake.common import FindResourceOrThrow
from pydrake.geometry import (ConnectDrakeVisualizer, SceneGraph)
from pydrake.lcm import DrakeLcm
from pydrake.multibody.rigid_body_tree import (RigidBodyTree, AddModelInstancesFromSdfFile,
                                               FloatingBaseType, AddModelInstanceFromUrdfFile, AddFlatTerrainToWorld,
                                               RigidBodyFrame, AddModelInstanceFromUrdfStringSearchingInRosPackages)
from pydrake.multibody.multibody_tree import UniformGravityFieldElement, MultibodyTree, BodyIndex
from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
from pydrake.multibody.multibody_tree.parsing import AddModelFromSdfFile
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.common import (set_assertion_failure_to_throw_exception)
from pydrake.systems.controllers import (RbtInverseDynamicsController)
from pydrake.systems.primitives import (ConstantVectorSource, SignalLogger, TrajectorySource)
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer
from pydrake.multibody.parsers import PackageMap
from pydrake.systems.framework import LeafSystem, PortDataType, BasicVector

from pydrake.multibody.rigid_body_plant import DrakeVisualizer, RigidBodyPlant
import numpy as np
import matplotlib.pyplot as plt

import os
import pydrake

from pydrake.systems.all import LcmSubscriberSystem, LcmPublisherSystem, AbstractValue
from pydrake.multibody.rigid_body_plant import ContactResults
from pydrake.all import PySerializer
# from pydrake.all import  KinematicsResults

from lcmt import *
model_path = os.path.join(os.getcwd(), 'Model/LittleDog.urdf')

dt = 0

lcm = DrakeLcm()
builder = DiagramBuilder()


class RobotCommandToRigidBodyPlantConverter(LeafSystem):
    """
    A block system that outputs torques of joints to Robot Plant, given a Command subscriber

    Command subscriber  --> this block  --> Robot Plant

    Input Port :
        -- robot_command_port : AbstractValue,  lcm_subscriber type
    Output Port:
        -- desired_effort_port : BasicVector, numpy[n, 1] the torques of actuators
    """
    def __init__(self, actuators):
        LeafSystem.__init__(self)
        self.num_actuators = len(actuators)
        self.actuators_init = np.ones(self.num_actuators) * 0.0

        self.robot_command_port_index = self._DeclareAbstractInputPort('robot_command_port',
                                                                       AbstractValue.Make(
                                                                           littledog_command_t)).get_index()
        self.desired_effort_port_indices = self._DeclareVectorOutputPort(BasicVector(self.num_actuators),
                                                                         self.OutputActuation).get_index()

    def OutputActuation(self, context, output):
        ## OutputDesiredEffort is not equal to output command
        msg = self.EvalAbstractInput(context, self.robot_command_port_index).get_value()

        # TODO : Considering motor model (motor effort length etc.)
        command = msg.joint_command

        if len(command) == 0 :
            output_command =  self.actuators_init
        elif len(command) == self.num_actuators:
            output_command = self.actuators_init + np.array(command)
        else:
            raise("Command size doesn't match the size of actuators")

        output.SetFromVector(output_command)

    def robot_command_input_port(self):
        return self.get_input_port(self.robot_command_port_index)
    def desired_effort_output_port(self):
        return self.get_output_port(self.desired_effort_port_indices)

class RobotStateEncoder(LeafSystem):
    """
    A block system that outputs all state infomation to a lcm publisher.

    Robot Plant --> this block --> lcm Publisher

    Input Port :
        -- joint_state_results_port : kVectorValued  , [n *2 , 1]  the positions and velocties of joints

    Output Port:
        -- lcm_message_port : AbstractValue, lcm publisher type

    """
    def __init__(self, rb_tree):
        LeafSystem.__init__(self)
        self.rb_tree = rb_tree
        self.num_controlled_q_ = self.rb_tree.get_num_positions()

        # Input Port
        self.joint_state_results_port_index = self._DeclareInputPort('joint_state_results_port',
                                                                     PortDataType.kVectorValued,
                                                                     self.num_controlled_q_ * 2).get_index()
        #         self.contact_results_port_index = self._DeclareAbstractInputPort('contact_results_port',
        #                                                                        AbstractValue.Make(ContactResults)).get_index()
        # Output Port
        self.lcm_message_port_index = self._DeclareAbstractOutputPort('state_output_port',
                                                                      self._Allocator,
                                                                      self._OutputRobotState,
                                                                      ).get_index()
    def _Allocator(self):
        return AbstractValue.Make(robot_state_t)

    def _OutputRobotState(self, context, output):
        message = robot_state_t()
        message.timestamp = context.get_time() * 1e3  # milliseconds

        # contact_result = EvalAbstractInput(context, contact_results_port_index).get_value()
        joint_state_result = self.EvalVectorInput(context, self.joint_state_results_port_index).get_value()

        # get all information
        message.num_joints = self.num_controlled_q_
        message.joint_position = joint_state_result[:self.num_controlled_q_]
        message.joint_velocity = joint_state_result[self.num_controlled_q_:]
        output.set_value(message)

    def joint_state_results_input_port(self):
        return self.get_input_port(self.joint_state_results_port_index)
    def lcm_message_output_port(self):
        return self.get_output_port(self.lcm_message_port_index)

# Build up your Robot World
rb_tree = RigidBodyTree()
world_frame = RigidBodyFrame("world_frame", rb_tree.world(), [0, 0, 0], [0, 0, 0])
AddFlatTerrainToWorld(rb_tree, 1000, 10)
robot_frame = RigidBodyFrame("robot_frame", rb_tree.world(), [0, 0, 0.5], [0, 0, 0])

# insert a robot from urdf files
pmap = PackageMap()
pmap.PopulateFromFolder(os.path.dirname(model_path))
AddModelInstanceFromUrdfStringSearchingInRosPackages(
        open(model_path, 'r').read(),
        pmap,
        os.path.dirname(model_path),
        FloatingBaseType.kRollPitchYaw,
        robot_frame,
        rb_tree)

# RigidBodyPlant
plant = builder.AddSystem(RigidBodyPlant(rb_tree, dt))
plant.set_name('plant')

# Robot command subscriber
robot_command_subscriber = builder.AddSystem(LcmSubscriberSystem.Make('ROBOT_COMMAND', littledog_command_t, lcm))
robot_command_subscriber.set_name('robot_command_subscriber')

# Robot command to Plant Converter
robot_command_to_rigidbodyplant_converter = builder.AddSystem(
    RobotCommandToRigidBodyPlantConverter(rb_tree.actuators))  # input argu: rigidbody actuators
robot_command_to_rigidbodyplant_converter.set_name('robot_command_to_rigidbodyplant_converter')

# Visualizer
visualizer_publisher = builder.AddSystem(DrakeVisualizer(rb_tree, lcm))
visualizer_publisher.set_name('visualizer_publisher')
visualizer_publisher.set_publish_period(1e-3);

# Robot State Encoder
robot_state_encoder = builder.AddSystem(RobotStateEncoder(rb_tree))  # force_sensor_info
robot_state_encoder.set_name('robot_state_encoder')

# Robot State Publisher
robot_state_publisher = builder.AddSystem(LcmPublisherSystem.Make('EST_ROBOT_STATE', robot_state_t, lcm))
robot_state_publisher.set_name('robot_state_publisher')
robot_state_publisher.set_publish_period(1e-3)

# Connect everything
builder.Connect(robot_command_subscriber.get_output_port(0),
                robot_command_to_rigidbodyplant_converter.robot_command_input_port())
builder.Connect(robot_command_to_rigidbodyplant_converter.desired_effort_output_port(),
                plant.get_input_port(0))
builder.Connect(plant.state_output_port(),
                visualizer_publisher.get_input_port(0))
builder.Connect(plant.state_output_port(),
                robot_state_encoder.joint_state_results_input_port())
builder.Connect(robot_state_encoder.lcm_message_output_port(),
                robot_state_publisher.get_input_port(0))


diagram = builder.Build()

diagram_context = diagram.CreateDefaultContext()
simulator = Simulator(diagram, diagram_context)
simulator.set_publish_every_time_step(False)
simulator.set_target_realtime_rate(1)
simulator.Initialize()

simulation_time = 10

lcm.StartReceiveThread()
simulator.StepTo(simulation_time)
lcm.StopReceiveThread()