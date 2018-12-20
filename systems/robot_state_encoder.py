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