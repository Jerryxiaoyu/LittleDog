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

from lcmt import *
from robot_command_to_plant_converter import RobotCommandToRigidBodyPlantLCMConverter
from robot_state_encoder import RobotStateLCMEncoder

from util.meshcat_rigid_body_visualizer import MeshcatRigidBodyVisualizer

model_path = os.path.join(os.path.dirname(__file__), '../Model/LittleDog.urdf')


def LittleDogSimulationDiagram(lcm,rb_tree,  dt, drake_visualizer):
    builder = DiagramBuilder()



    num_pos = rb_tree.get_num_positions()

    # RigidBodyPlant
    plant = builder.AddSystem(RigidBodyPlant(rb_tree, dt))
    plant.set_name('plant')

    # Robot command subscriber
    robot_command_subscriber = builder.AddSystem(LcmSubscriberSystem.Make('ROBOT_COMMAND', littledog_command_t, lcm))
    robot_command_subscriber.set_name('robot_command_subscriber')

    # Robot command to Plant Converter
    robot_command_to_rigidbodyplant_converter = builder.AddSystem(
        RobotCommandToRigidBodyPlantLCMConverter(rb_tree.actuators))  # input argu: rigidbody actuators
    robot_command_to_rigidbodyplant_converter.set_name('robot_command_to_rigidbodyplant_converter')

    # Visualizer
    visualizer_publisher = builder.AddSystem(drake_visualizer)
    visualizer_publisher.set_name('visualizer_publisher')
    visualizer_publisher.set_publish_period(0.02)

    # Visualize
    #visualizer_publisher = builder.AddSystem(MeshcatRigidBodyVisualizer(rb_tree))

    # Robot State Encoder
    robot_state_encoder = builder.AddSystem(RobotStateLCMEncoder(rb_tree))  # force_sensor_info
    robot_state_encoder.set_name('robot_state_encoder')

    # Robot State Publisher
    robot_state_publisher = builder.AddSystem(LcmPublisherSystem.Make('EST_ROBOT_STATE', robot_state_t, lcm))
    robot_state_publisher.set_name('robot_state_publisher')
    robot_state_publisher.set_publish_period(0.005)



    # Connect everything
    builder.Connect(robot_command_subscriber.get_output_port(0),
                    robot_command_to_rigidbodyplant_converter.robot_command_input_port())
    builder.Connect(robot_command_to_rigidbodyplant_converter.desired_effort_output_port(),
                    plant.get_input_port(0))
    builder.Connect(plant.state_output_port(),
                    visualizer_publisher.get_input_port(0))
    # builder.Connect(plant.get_output_port(0),
    #                 visualizer_publisher.get_input_port(0))

    builder.Connect(plant.state_output_port(),
                    robot_state_encoder.joint_state_results_input_port())
    builder.Connect(robot_state_encoder.lcm_message_output_port(),
                    robot_state_publisher.get_input_port(0))

    # Signal logger
    logger = builder.AddSystem(SignalLogger(num_pos*2))
    builder.Connect(plant.state_output_port(), logger.get_input_port(0))


    return builder.Build(), logger,plant


