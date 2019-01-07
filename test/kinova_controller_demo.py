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
from pydrake.systems.drawing import plot_system_graphviz


import os
import pydrake

from pydrake.systems.all import LcmSubscriberSystem, LcmPublisherSystem, AbstractValue
from pydrake.multibody.rigid_body_plant import ContactResults
from pydrake.all import PySerializer

from lcmt import *


from util.meshcat_rigid_body_visualizer import MeshcatRigidBodyVisualizer

from controllers.PID_controller import joints_PID_params, RobotPDAndFeedForwardController, Robot_controller_test
from robot_command_to_plant_converter import RobotCommandToRigidBodyPlantConverter
from robot_state_encoder import RobotStateEncoder
from pydrake.multibody.rigid_body_plant import DrakeVisualizer
from systems.setup_kinova import SetupKinova
import time


model_path = os.path.join(pydrake.getDrakePath(), 'manipulation/models/jaco_description/urdf/j2n6s300.urdf')


dt =0
builder = DiagramBuilder()

lcm = DrakeLcm()

#rb_tree = SetupLittleDog()
rb_tree = SetupKinova()
drake_visualizer = DrakeVisualizer(rb_tree, lcm, enable_playback= False)

num_pos = rb_tree.get_num_positions()
num_actuators = rb_tree.get_num_actuators()

zero_config = np.zeros((rb_tree.get_num_actuators() * 2, 1))  # just for test
# zero_config =  np.concatenate((np.array([0, 1, 1, 1,   1.7, 0.5, 0, 0, 0]),np.zeros(9)), axis=0)
zero_config = np.concatenate((rb_tree.getZeroConfiguration(), np.zeros(9)), axis=0)

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

rb = rigid_body_tree = RigidBodyTree()
robot_frame = RigidBodyFrame("robot_frame", rb_tree.world(), [0, 0, 0 ], [0, 0, 0])
# insert a robot from urdf files
pmap = PackageMap()
pmap.PopulateFromFolder(os.path.dirname(model_path))
AddModelInstanceFromUrdfStringSearchingInRosPackages(
        open(model_path, 'r').read(),
        pmap,
        os.path.dirname(model_path),
        FloatingBaseType.kFixed,  #FloatingBaseType.kRollPitchYaw,
        robot_frame,
    rb)

#PIDcontroller = builder.AddSystem(RobotPDAndFeedForwardController(rb_tree, kp, ki, kd))
controller = RbtInverseDynamicsController(rb, kp, ki, kd, False)
PIDcontroller = builder.AddSystem(controller)
PIDcontroller.set_name('PID_controller')

controller_test = builder.AddSystem(Robot_controller_test(rb_tree, kp, ki, kd))
controller_test.set_name('controller_test')
builder.Connect(controller.get_output_port(0),
                controller_test.robot_state_input_port())

builder.Connect(controller_test.robot_command_output_port(),
                robot_command_to_rigidbodyplant_converter.robot_command_input_port())

ref_conf = np.concatenate((np.array([0, 1, 1, 1, 1.7, 0.5, 0, 0, 0]),np.zeros(9)), axis=0)
traj_src = builder.AddSystem(ConstantVectorSource(ref_conf))
builder.Connect(traj_src.get_output_port(0),
                controller_test.get_input_port(1))


builder.Connect(robot_state_encoder.joint_state_outport_port(),
                PIDcontroller.get_input_port(0))

# builder.Connect(PIDcontroller.get_output_port(0),
#                 robot_command_to_rigidbodyplant_converter.robot_command_input_port())

# Ref trajectory

ref_conf = np.concatenate((np.array([0, 1, 1, 1,   1.7, 0.5, 0, 0, 0]),np.zeros(9)), axis=0)
traj_src = builder.AddSystem(ConstantVectorSource(ref_conf))

builder.Connect(traj_src.get_output_port(0),
                PIDcontroller.get_input_port(1))

# Signal logger
logger = builder.AddSystem(SignalLogger(num_pos * 2))
builder.Connect(plant.state_output_port(), logger.get_input_port(0))



diagram =   builder.Build()

# init_state =  np.concatenate((rb_tree.getZeroConfiguration(),np.zeros(9)), axis=0)#.reshape((-1,1))
# print(init_state.shape)


# simulation setting
diagram_context = diagram.CreateDefaultContext()
plant_context = diagram.GetMutableSubsystemContext(plant,diagram_context)
#plant.set_state_vector(diagram_context, init_state)



simulator = Simulator(diagram, diagram_context)
simulator.set_publish_every_time_step(False)
simulator.set_target_realtime_rate(1)
simulator.Initialize()
simulator.get_mutable_integrator().set_target_accuracy(1e-3)


simulation_time = 5

t0 = time.time()
lcm.StartReceiveThread()
simulator.StepTo(simulation_time)
lcm.StopReceiveThread()
tf = time.time()




print("t = ", tf-t0)

#drake_visualizer.ReplayCachedSimulation()
# plot graph
plot_system_graphviz(diagram, max_depth=2147483647)
plt.show()