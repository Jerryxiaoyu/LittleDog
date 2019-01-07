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


def joints_PID_params(rbtree):
    num_joints = rbtree.get_num_actuators()

    kp = np.ones(num_joints) * 100
    ki = np.zeros(num_joints)
    kd = 2 * np.sqrt(kp)

    assert kp.shape[0] == num_joints

    return kp, ki, kd


class RobotPDAndFeedForwardController(LeafSystem):

    def __init__(self, rbtree, kp, ki, kd):
        LeafSystem.__init__(self)
        self.rb_tree = rbtree
        self.num_controlled_q_ = self.rb_tree.get_num_actuators()

        self.kp = kp
        self.ki = ki
        self.kd = kd

        
        self.input_index_state = self._DeclareInputPort('robot_state_port', PortDataType.kVectorValued,
                                                        self.num_controlled_q_ * 2).get_index()

        self.input_index_desired_state = self._DeclareInputPort('state_ref_port', PortDataType.kVectorValued,
                                                                self.num_controlled_q_ * 2).get_index()

        self.robot_command_port_index = self._DeclareVectorOutputPort(BasicVector(self.num_controlled_q_),
                                      self._OutputCommand).get_index()
       

    def _OutputCommand(self, context, output):
        state = self.EvalVectorInput(context, self.input_index_state).get_value()
        state_d = self.EvalVectorInput(context, self.input_index_desired_state).get_value()

        controlled_state_diff = state_d - state


        # TODO  no integral processing 
        ctrl_output = self.kp * (controlled_state_diff[:self.num_controlled_q_]) + self.kd * (controlled_state_diff[self.num_controlled_q_:])  
        
        # ctrl_output = np.clip(ctrl_output, -50, 50)

        #print(context.get_time())
        
        output.SetFromVector(ctrl_output)

    # Port Declaration
    def robot_state_input_port(self):
        return self.get_input_port(self.input_index_state)

    def state_ref_input_port(self):
        return self.get_input_port(self.input_index_desired_state)

    def robot_command_output_port(self):
        return self.get_output_port(self.robot_command_port_index)

class Robot_controller_test(LeafSystem):

    def __init__(self, rbtree, kp, ki, kd):
        LeafSystem.__init__(self)
        self.rb_tree = rbtree
        self.num_controlled_q_ = self.rb_tree.get_num_actuators()

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.input_index_state = self._DeclareInputPort('robot_state_port', PortDataType.kVectorValued,
                                                        self.num_controlled_q_  ).get_index()

        self.input_index_desired_state = self._DeclareInputPort('state_ref_port', PortDataType.kVectorValued,
                                                                self.num_controlled_q_ * 2).get_index()

        self.robot_command_port_index = self._DeclareVectorOutputPort(BasicVector(self.num_controlled_q_),
                                                                      self._OutputCommand).get_index()

    def _OutputCommand(self, context, output):
        state = self.EvalVectorInput(context, self.input_index_state).get_value()
        state_d = self.EvalVectorInput(context, self.input_index_desired_state).get_value()

        controlled_state_diff =   state_d - np.concatenate((state,state))

        # TODO  no integral processing
        ctrl_output = self.kp * (controlled_state_diff[:self.num_controlled_q_]) + self.kd * (
        controlled_state_diff[self.num_controlled_q_:])

        # ctrl_output = np.clip(ctrl_output, -50, 50)

        # print(context.get_time())

        ctrl_output = state

        output.SetFromVector(ctrl_output)

    # Port Declaration
    def robot_state_input_port(self):
        return self.get_input_port(self.input_index_state)

    def state_ref_input_port(self):
        return self.get_input_port(self.input_index_desired_state)

    def robot_command_output_port(self):
        return self.get_output_port(self.robot_command_port_index)