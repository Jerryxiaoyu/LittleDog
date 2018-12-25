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


from lcmt import *
model_path = os.path.join(os.getcwd(), 'Model/LittleDog.urdf')

dt = 0

lcm = DrakeLcm()
builder = DiagramBuilder()

# # Robot state Subscriber
# robot_state_subscriber = builder.AddSystem(LcmSubscriberSystem.Make('EST_ROBOT_STATE', robot_state_t, lcm))
#
#
# diagram = builder.Build()
#
# diagram_context = diagram.CreateDefaultContext()

# import pydrake.systems.lcm as mut
# from pydrake.lcm import DrakeMockLcm
# lcm = DrakeMockLcm()
# dut = mut.LcmSubscriberSystem.Make(
#     channel="EST_ROBOT_STATE", lcm_type=robot_state_t, lcm=lcm)
#
# context = dut.CreateDefaultContext()
# while True:
#     output = dut.AllocateOutput()
#     dut.CalcOutput(context, output)
#     actual = output.get_data(0).get_value()
#     #self.assert_lcm_equal(actual, model)
#     print('t = {}  joint_pos = {}'.format(actual.timestamp, actual.joint_position))

class RobotRecieve(LeafSystem):

    def __init__(self ):
        LeafSystem.__init__(self)

        self.num_actuators =13
        self.robot_state_port_index = self._DeclareAbstractInputPort('robot_state_port',
                                                                     AbstractValue.Make(
                                                                         robot_state_t)).get_index()

        self.desired_effort_port_indices = self._DeclareVectorOutputPort(BasicVector(self.num_actuators),
                                                                     self._OutputCommand).get_index()

    def _OutputCommand(self, context, output):

        msg = self.EvalAbstractInput(context, self.robot_state_port_index).get_value()

        print('t = {}  joint_pos = {}'.format(msg.timestamp, msg.joint_position))

        # command_msg = littledog_command_t()
        # command_msg.timestamp = context.get_time() * 1e3  # milliseconds
        # command_msg.num_joints = self.num_controlled_q_
        #
        #
        # if msg.num_joints == self.num_controlled_q_:
        #     q = np.array(msg.joint_position)
        #     qv = np.array(msg.joint_velocity)
        #     print(msg.num_joints)
        #
        #     state_d = self.EvalVectorInput(context, self.state_ref_port_index).get_value()
        #
        #     controlled_state_diff = state_d - np.concatenate((q, qv), axis=0)
        #     state_block = context.get_continuous_state_vector().get_value()
        #
        #     command_msg.joint_command = self.kp * (controlled_state_diff[:self.num_controlled_q_]) + self.kd * (
        #     controlled_state_diff[self.num_controlled_q_:]) + self.ki * (state_block)
        #
        #
        # else:
        #     command_msg.joint_command = np.zeros(self.num_controlled_q_)
        # output.set_value(command_msg)


    # Port Declaration
    def robot_state_input_port(self):
        return self.get_input_port(self.robot_state_port_index)


robot_state_subscriber = builder.AddSystem(LcmSubscriberSystem.Make('EST_ROBOT_STATE', robot_state_t, lcm))

# Robot command Publisher
robot_test = builder.AddSystem(RobotRecieve() )
robot_test.set_name('robot_test')



builder.Connect(robot_state_subscriber.get_output_port(0),
                robot_test.robot_state_input_port())

diagram = builder.Build()

diagram_context = diagram.CreateDefaultContext()
simulator = Simulator(diagram, diagram_context)
simulator.set_publish_every_time_step(False)
simulator.set_target_realtime_rate(1)
simulator.Initialize()

simulation_time = 100

lcm.StartReceiveThread()
simulator.StepTo(simulation_time)
lcm.StopReceiveThread()


