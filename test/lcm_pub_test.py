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

class RobotStateOutput(LeafSystem):
    """
    A block system that outputs all state infomation to a lcm publisher.

    Robot Plant --> this block --> lcm Publisher

    Input Port :
        -- joint_state_results_port : kVectorValued  , [n *2 , 1]  the positions and velocties of joints

    Output Port:
        -- lcm_message_port : AbstractValue, lcm publisher type

    """
    def __init__(self ):
        LeafSystem.__init__(self)

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

        # get all information
        message.num_joints = 13
        message.joint_position = np.ones(13)* 1.5
        message.joint_velocity = np.ones(13)* 0.5

        #print('t = {}  joint_pos = {}'.format(message.timestamp, message.joint_position))
       # print(message.joint_position.shape)
        output.set_value(message)


    def lcm_message_output_port(self):
        return self.get_output_port(self.lcm_message_port_index)


# dut = mut.LcmPublisherSystem.Make(
#     channel="EST_ROBOT_STATE", lcm_type=robot_state_t, lcm=lcm)

lcm = DrakeLcm()
builder = DiagramBuilder()


# Robot State Publisher
robot_state_publisher = builder.AddSystem(LcmPublisherSystem.Make('EST_ROBOT_STATE', robot_state_t, lcm))
robot_state_publisher.set_name('robot_state_publisher')
robot_state_publisher.set_publish_period(1e-3)


# Robot State Encoder
robot_state_output = builder.AddSystem(RobotStateOutput())  # force_sensor_info
robot_state_output.set_name('robot_state_output')

builder.Connect(robot_state_output.lcm_message_output_port(),
                    robot_state_publisher.get_input_port(0))


diagram = builder.Build()


# while(True):
#     pub_context = diagram.GetMutableSubsystemContext(robot_state_publisher, diagram_context)
#     robot_state_publisher.Publish(pub_context)

# simulation setting
diagram_context = diagram.CreateDefaultContext()


simulator = Simulator(diagram, diagram_context)
simulator.set_publish_every_time_step(False)
simulator.set_target_realtime_rate(1)
simulator.Initialize()

simulation_time = 10

lcm.StartReceiveThread()
simulator.StepTo(100)
lcm.StopReceiveThread()




