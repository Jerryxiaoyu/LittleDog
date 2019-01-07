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


class RobotStateLCMEncoder(LeafSystem):
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
        self.num_position = self.rb_tree.get_num_positions()
        self.num_controlled_q_ = self.rb_tree.get_num_actuators()


        # Input Port
        self.joint_state_results_port_index = self._DeclareInputPort('joint_state_results_port',
                                                                     PortDataType.kVectorValued,
                                                                     self.num_position * 2).get_index()
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

        #print('joint_state_result', joint_state_result)
        # get all information
        message.num_joints = self.num_controlled_q_
        message.joint_position = joint_state_result[self.num_position - self.num_controlled_q_:self.num_position]
        message.joint_velocity = joint_state_result[self.num_position *2 - self.num_controlled_q_:]

        #print('t = {}  joint_pos = {}'.format(message.timestamp, message.joint_position))

        print(message.timestamp)
        #print('t = {} '.format(message.timestamp ))
        output.set_value(message)

    def joint_state_results_input_port(self):
        return self.get_input_port(self.joint_state_results_port_index)
    def lcm_message_output_port(self):
        return self.get_output_port(self.lcm_message_port_index)



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
        self.num_position = self.rb_tree.get_num_positions()
        self.num_controlled_q_ = self.rb_tree.get_num_actuators()

        self.n = 0

        # Input Port
        self.joint_state_results_port_index = self._DeclareInputPort('joint_state_results_port',
                                                                     PortDataType.kVectorValued,
                                                                     self.num_position * 2).get_index()
        #         self.contact_results_port_index = self._DeclareAbstractInputPort('contact_results_port',
        #                                                                        AbstractValue.Make(ContactResults)).get_index()
        # Output Port

        self.joint_state_outport_index = self._DeclareVectorOutputPort('state_output_port', BasicVector(self.num_controlled_q_ *2 ),
                                                                       self._OutputRobotState).get_index()


    def _OutputRobotState(self, context, output):


        states = self.EvalVectorInput(context, self.joint_state_results_port_index).get_value()


        # get joint state from full state
        joint_state = np.concatenate((states[self.num_position - self.num_controlled_q_:self.num_position],
                                      states[self.num_position *2 - self.num_controlled_q_:] ), axis=0)

        #print(joint_state)

        # self.n = self.n +1
        # if self.n % 100 ==0:
        #     print self.n

        output.SetFromVector(joint_state)

    def joint_state_results_input_port(self):
        return self.get_input_port(self.joint_state_results_port_index)
    def joint_state_outport_port(self):
        return self.get_output_port(self.joint_state_outport_index)