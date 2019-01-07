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

from systems.robot_coomand_to_rigidbody_converter import RobotCommandToRigidBodyPlantConverter
from systems.robot_state_encoder import RobotStateEncoder
from lcmt import *

def joints_PID_params(rbtree):
    num_joints = rbtree.get_num_positions()

    kp = np.ones(num_joints ) *100
    ki = np.zeros(num_joints)
    kd = 2* np.sqrt(kp)

    assert kp.shape[0] == num_joints

    return kp, ki, kd

class RobotPDAndFeedForwardController(LeafSystem):
    """

    """

    def __init__(self,  rbtree, kp, ki, kd):
        LeafSystem.__init__(self)
        self.rb_tree = rbtree
        self.num_controlled_q_ = self.rb_tree.get_num_positions()

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.robot_state_port_index = self._DeclareAbstractInputPort('robot_state_port',
                                                                     AbstractValue.Make(
                                                                         littledog_command_t)).get_index()
        self.state_ref_port_index = self._DeclareInputPort('State_Ref_Port', PortDataType.kVectorValued,
                                                           self.num_controlled_q_ *2).get_index()
        self.robot_command_port_index = self._DeclareAbstractOutputPort('robot_command_port',
                                                                        self._Allocator,
                                                                        self._OutputCommand,
                                                                        ).get_index()

    def _Allocator(self):
        return AbstractValue.Make(littledog_command_t)

    def _OutputCommand(self, context, output):
        ## OutputDesiredEffort is not equal to output command
        msg = self.EvalAbstractInput(context, self.robot_state_port_index).get_value()

        print(msg.num_joints)
        q = np.array(msg.joint_position)
        qv = np.array(msg.joint_velocity)

        if msg.num_joints != 0:
            command_msg = littledog_command_t()
            command_msg.timestamp = context.get_time() * 1e3  # milliseconds
            command_msg.num_joints = self.num_controlled_q_

            state_d = self.EvalVectorInput(context, self.state_ref_port_index).get_value()
            controlled_state_diff = state_d - np.concatenate((q, qv), axis=0)
            state_block = context.get_continuous_state_vector().get_value()

            command_msg.joint_command = self.kp * (controlled_state_diff[:self.num_controlled_q_]) + self.kd * (
            controlled_state_diff[self.num_controlled_q_:]) + self.ki * (state_block)

            output.set_value(command_msg)

    def robot_state_input_port(self):
        return self.get_input_port(self.robot_state_port_index)

    def state_ref_input_port(self):
        return self.get_input_port(self.state_ref_port_index)

    def robot_command_output_port(self):
        return self.get_output_port(self.robot_command_port_index)


lcm = DrakeLcm()
builder = DiagramBuilder()
model_path = os.path.join(os.getcwd(), 'Model/LittleDog.urdf')

dt = 0
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

# Robot state Subscriber
robot_state_subscriber = builder.AddSystem(LcmSubscriberSystem.Make('EST_ROBOT_STATE', robot_state_t, lcm))
robot_state_subscriber.set_name('robot_state_subscriber')

# Robot command Publisher
robot_state_publisher = builder.AddSystem(LcmPublisherSystem.Make('ROBOT_COMMAND', littledog_command_t, lcm))
robot_state_publisher.set_name('robot_command_publisher')
robot_state_publisher.set_publish_period(1e-3)

# controller
kp,ki,kd = joints_PID_params(rb_tree)
controller = builder.AddSystem(RobotPDAndFeedForwardController(rb_tree,kp,ki,kd))


builder.Connect(robot_state_subscriber.get_output_port(0),
               controller.robot_state_input_port())
builder.Connect(controller.robot_command_output_port(),
               robot_state_publisher.get_input_port(0))

diagram = builder.Build()

diagram_context = diagram.CreateDefaultContext()
controller_context = diagram.GetMutableSubsystemContext(controller, diagram_context)

controller_context.FixInputPort(
    controller.state_ref_port_index, np.zeros(controller.num_controlled_q_*2))
#

simulator = Simulator(diagram, diagram_context)
simulator.set_publish_every_time_step(False)
simulator.set_target_realtime_rate(1)
simulator.Initialize()
simulator.StepTo(10)