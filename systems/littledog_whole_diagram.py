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


from util.meshcat_rigid_body_visualizer import MeshcatRigidBodyVisualizer

from controllers.PID_controller import joints_PID_params, RobotPDAndFeedForwardController
from robot_command_to_plant_converter import RobotCommandToRigidBodyPlantConverter
from robot_state_encoder import RobotStateEncoder
from pydrake.multibody.parsing import Parser

model_path = os.path.join(os.path.dirname(__file__), '../Model/LittleDog.urdf')
#model_path = os.path.join(pydrake.getDrakePath(), 'manipulation/models/jaco_description/urdf/j2n6s300.urdf')




def LittleDogSimulationDiagram(lcm, rb_tree,  dt, drake_visualizer):
    builder = DiagramBuilder()

    num_pos = rb_tree.get_num_positions()
    num_actuators = rb_tree.get_num_actuators()

    zero_config = np.zeros((rb_tree.get_num_actuators() *2 , 1))   # just for test
    #zero_config =  np.concatenate((np.array([0, 1, 1, 1,   1.7, 0.5, 0, 0, 0]),np.zeros(9)), axis=0)
    #zero_config = np.concatenate((rb_tree.getZeroConfiguration(),np.zeros(9)), axis=0)
    zero_config = np.concatenate((np.array([ 1,  0, 0,
                                            0, 0, 0,
                                            0, 0, 0,
                                            0, 0,  0]), np.zeros(12)), axis=0)
    # zero_config[0] = 1.7
    # zero_config[1] = 1.7
    # zero_config[2] = 1.7
    # zero_config[3] = 1.7
    # RigidBodyPlant
    plant = builder.AddSystem(RigidBodyPlant(rb_tree, dt))
    plant.set_name('plant')

    # Visualizer
    visualizer_publisher = builder.AddSystem(drake_visualizer)
    visualizer_publisher.set_name('visualizer_publisher')
    visualizer_publisher.set_publish_period(0.02)

    builder.Connect(plant.state_output_port(),
                    visualizer_publisher.get_input_port(0))

    # Robot State Encoder
    robot_state_encoder = builder.AddSystem(RobotStateEncoder(rb_tree))  # force_sensor_info
    robot_state_encoder.set_name('robot_state_encoder')

    builder.Connect(plant.state_output_port(),
                    robot_state_encoder.joint_state_results_input_port())

    # Robot command to Plant Converter
    robot_command_to_rigidbodyplant_converter = builder.AddSystem(
        RobotCommandToRigidBodyPlantConverter(rb_tree.actuators, motor_gain=None))  # input argu: rigidbody actuators
    robot_command_to_rigidbodyplant_converter.set_name('robot_command_to_rigidbodyplant_converter')

    builder.Connect(robot_command_to_rigidbodyplant_converter.desired_effort_output_port(),
                    plant.get_input_port(0))

    # PID controller
    kp, ki, kd = joints_PID_params(rb_tree)
    #PIDcontroller = builder.AddSystem(RobotPDAndFeedForwardController(rb_tree, kp, ki, kd))
    #PIDcontroller.set_name('PID_controller')

    rb =   RigidBodyTree()
    robot_frame = RigidBodyFrame("robot_frame", rb_tree.world(), [0, 0, 0], [0, 0, 0])
    # insert a robot from urdf files
    pmap = PackageMap()
    pmap.PopulateFromFolder(os.path.dirname(model_path))
    AddModelInstanceFromUrdfStringSearchingInRosPackages(
        open(model_path, 'r').read(),
        pmap,
        os.path.dirname(model_path),
        FloatingBaseType.kFixed,  # FloatingBaseType.kRollPitchYaw,
        robot_frame,
        rb)

    PIDcontroller = builder.AddSystem(RbtInverseDynamicsController(rb, kp, ki, kd, False))
    PIDcontroller.set_name('PID_controller')

    builder.Connect(robot_state_encoder.joint_state_outport_port(),
                    PIDcontroller.get_input_port(0))

    builder.Connect(PIDcontroller.get_output_port(0),
                    robot_command_to_rigidbodyplant_converter.robot_command_input_port())
    

    # Ref trajectory


    traj_src = builder.AddSystem(ConstantVectorSource(zero_config))

    builder.Connect(traj_src.get_output_port(0),
                    PIDcontroller.get_input_port(1))


    # Signal logger
    signalLogRate = 60
    logger = builder.AddSystem(SignalLogger(num_pos*2, batch_allocation_size = 1000))
    logger._DeclarePeriodicPublish(1. / signalLogRate, 0.0)
    builder.Connect(plant.get_output_port(0), logger.get_input_port(0))


    return builder.Build(), logger, plant


def LittleDogSimulationDiagram2(lcm, rb_tree, dt, drake_visualizer):
    builder = DiagramBuilder()

    num_pos = rb_tree.get_num_positions()
    num_actuators = rb_tree.get_num_actuators()

    zero_config = np.zeros((rb_tree.get_num_actuators() * 2, 1))  # just for test
    # zero_config =  np.concatenate((np.array([0, 1, 1, 1,   1.7, 0.5, 0, 0, 0]),np.zeros(9)), axis=0)
    # zero_config = np.concatenate((rb_tree.getZeroConfiguration(),np.zeros(9)), axis=0)
    zero_config = np.concatenate((np.array([1, 0, 0,
                                            0, 0, 0,
                                            0, 0, 0,
                                            0, 0, 0]), np.zeros(12)), axis=0)
    # zero_config[0] = 1.7
    # zero_config[1] = 1.7
    # zero_config[2] = 1.7
    # zero_config[3] = 1.7

    # 1.SceneGraph system
    scene_graph = builder.AddSystem(SceneGraph())
    scene_graph.RegisterSource('scene_graph_n')

    plant = builder.AddSystem(MultibodyPlant())
    plant_parser = Parser(plant, scene_graph)
    plant_parser.AddModelFromFile(file_name=model_path,
                                  model_name="littledog")
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("body"))

    # Add gravity to the model.
    plant.AddForceElement(UniformGravityFieldElement([0, 0, -9.81]))
    # Model is completed
    plant.Finalize()

    builder.Connect(scene_graph.get_query_output_port(),
                    plant.get_geometry_query_input_port())
    builder.Connect(plant.get_geometry_poses_output_port(),
                    scene_graph.get_source_pose_port(plant.get_source_id()))
    ConnectDrakeVisualizer(builder=builder, scene_graph=scene_graph)


    # Robot State Encoder
    robot_state_encoder = builder.AddSystem(RobotStateEncoder(rb_tree))  # force_sensor_info
    robot_state_encoder.set_name('robot_state_encoder')

    builder.Connect(plant.get_continuous_state_output_port(),
                    robot_state_encoder.joint_state_results_input_port())

    # Robot command to Plant Converter
    robot_command_to_rigidbodyplant_converter = builder.AddSystem(
        RobotCommandToRigidBodyPlantConverter(rb_tree.actuators, motor_gain=None))  # input argu: rigidbody actuators
    robot_command_to_rigidbodyplant_converter.set_name('robot_command_to_rigidbodyplant_converter')

    builder.Connect(robot_command_to_rigidbodyplant_converter.desired_effort_output_port(),
                    plant.get_actuation_input_port())

    # PID controller
    kp, ki, kd = joints_PID_params(rb_tree)
    # PIDcontroller = builder.AddSystem(RobotPDAndFeedForwardController(rb_tree, kp, ki, kd))
    # PIDcontroller.set_name('PID_controller')

    rb = RigidBodyTree()
    robot_frame = RigidBodyFrame("robot_frame", rb_tree.world(), [0, 0, 0], [0, 0, 0])
    # insert a robot from urdf files
    pmap = PackageMap()
    pmap.PopulateFromFolder(os.path.dirname(model_path))
    AddModelInstanceFromUrdfStringSearchingInRosPackages(
        open(model_path, 'r').read(),
        pmap,
        os.path.dirname(model_path),
        FloatingBaseType.kFixed,  # FloatingBaseType.kRollPitchYaw,
        robot_frame,
        rb)

    PIDcontroller = builder.AddSystem(RbtInverseDynamicsController(rb, kp, ki, kd, False))
    PIDcontroller.set_name('PID_controller')

    builder.Connect(robot_state_encoder.joint_state_outport_port(),
                    PIDcontroller.get_input_port(0))

    builder.Connect(PIDcontroller.get_output_port(0),
                    robot_command_to_rigidbodyplant_converter.robot_command_input_port())

    # Ref trajectory

    traj_src = builder.AddSystem(ConstantVectorSource(zero_config))

    builder.Connect(traj_src.get_output_port(0),
                    PIDcontroller.get_input_port(1))

    # Signal logger
    logger = builder.AddSystem(SignalLogger(num_pos * 2 ))
    builder.Connect(plant.get_continuous_state_output_port(), logger.get_input_port(0))

    return builder.Build(), logger, plant



