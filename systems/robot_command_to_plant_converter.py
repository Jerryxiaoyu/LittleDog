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



class RobotCommandToRigidBodyPlantLCMConverter(LeafSystem):
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
        """

        :type output: object
        """
        ## OutputDesiredEffort is not equal to output command
        msg = self.EvalAbstractInput(context, self.robot_command_port_index).get_value()
        #print('t = {}  command = {}'.format(msg.timestamp, msg.joint_command))
        # TODO : Considering motor model (motor effort length etc.)
        command = msg.joint_command
        num_actuators = msg.num_joints
        if num_actuators != 0 :
            if num_actuators == self.num_actuators:
                output_command = self.actuators_init + np.array(command)
            else:
                assert("Command size ( {} )doesn't match the size of actuators".format(len(command)))
        else:
            output_command = self.actuators_init

        #print('actuators = {}'.format(output_command[0] ))

        #output_command2 = np.ones(9) * 20 #self.actuators_init
        output.SetFromVector(output_command)



    def robot_command_input_port(self):
        return self.get_input_port(self.robot_command_port_index)
    def desired_effort_output_port(self):
        return self.get_output_port(self.desired_effort_port_indices)


class RobotCommandToRigidBodyPlantConverter(LeafSystem):
    """
    A block system that outputs torques of joints to Robot Plant, given a Command subscriber

    Command subscriber  --> this block  --> Robot Plant

    Input Port :
        -- robot_command_port : AbstractValue,  lcm_subscriber type
    Output Port:
        -- desired_effort_port : BasicVector, numpy[n, 1] the torques of actuators
    """
    def __init__(self, actuators, motor_gain = None):
        LeafSystem.__init__(self)
        self.num_actuators = len(actuators)
        self.actuators_init = np.ones(self.num_actuators) * 0.0
        if motor_gain is None:
            motor_gain = np.ones(self.num_actuators)

        assert motor_gain.shape == (self.num_actuators, )

        self.motor_gain = motor_gain
        self.robot_command_port_index = self._DeclareInputPort('robot_command_port', PortDataType.kVectorValued,
                                                        self.num_actuators  ).get_index()

        self.desired_effort_port_index = self._DeclareVectorOutputPort(BasicVector(self.num_actuators),
                                                                         self.OutputActuation).get_index()

    def OutputActuation(self, context, output):
        """

        :type output: object
        """
        robot_command = self.EvalVectorInput(context, self.robot_command_port_index).get_value()

        # TODO: ignoring Motor dynamics, and assume the gains of motor are constant

        motor_torque = self.motor_gain * robot_command

        output.SetFromVector(motor_torque)

    def robot_command_input_port(self):
        return self.get_input_port(self.robot_command_port_index)
    def desired_effort_output_port(self):
        return self.get_output_port(self.desired_effort_port_index)