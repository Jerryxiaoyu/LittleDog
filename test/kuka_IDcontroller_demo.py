from pydrake.common import FindResourceOrThrow
from pydrake.geometry import (ConnectDrakeVisualizer, SceneGraph)
from pydrake.lcm import DrakeLcm
from pydrake.multibody.rigid_body_tree import (RigidBodyTree, AddModelInstancesFromSdfFile,
                                               FloatingBaseType,AddModelInstanceFromUrdfFile)
from pydrake.multibody.multibody_tree import UniformGravityFieldElement, MultibodyTree,BodyIndex
from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
from pydrake.multibody.multibody_tree.parsing import AddModelFromSdfFile
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.common import (set_assertion_failure_to_throw_exception)
from pydrake.systems.controllers import (RbtInverseDynamicsController)
from pydrake.systems.primitives import (ConstantVectorSource, SignalLogger, TrajectorySource)
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer

from pydrake.systems.framework import LeafSystem, PortDataType,BasicVector

import numpy as np
import matplotlib.pyplot as plt
from kuka_trajectory import MakeControlledKukaPlan
import time

kSdfPath ="drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf"


builder = DiagramBuilder()
# 1.SceneGraph system
scene_graph = builder.AddSystem(SceneGraph())
scene_graph.RegisterSource('scene_graph_n')

# 2.Kuka robot system
robot_plant = builder.AddSystem(MultibodyPlant())
AddModelFromSdfFile(file_name=FindResourceOrThrow(kSdfPath),
                    plant = robot_plant,
                    scene_graph= scene_graph )
robot_plant.WeldFrames(robot_plant.world_frame(),
                      robot_plant.GetFrameByName("iiwa_link_0"))
# Add gravity to the model.
robot_plant.AddForceElement(UniformGravityFieldElement([0, 0, -9.81]))
# Model is completed
robot_plant.Finalize()

# Sanity check on the availability of the optional source id before using it.
assert robot_plant.geometry_source_is_registered()
assert robot_plant.num_positions() == 7
print('get_source_id : ', robot_plant.get_source_id())


# Boilerplate used to connect the plant to a SceneGraph for visualization.
lcm = DrakeLcm
builder.Connect(scene_graph.get_query_output_port(),
                robot_plant.get_geometry_query_input_port())
builder.Connect(robot_plant.get_geometry_poses_output_port(),
                scene_graph.get_source_pose_port(robot_plant.get_source_id()))
ConnectDrakeVisualizer(builder=builder, scene_graph=scene_graph)

# 3.Signal logger
logger = builder.AddSystem(SignalLogger(14))
builder.Connect(robot_plant.get_continuous_state_output_port(), logger.get_input_port(0))


class PoseOutput(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)

        self.num_vec = 3

        # self._DeclareContinuousState(num_joints, num_particles, 0)
        self._DeclareVectorOutputPort(BasicVector(self.num_vec),
                                      self.CalcOutput)

    def CalcOutput(self, context, output):
        vec = robot_plant.tree().EvalBodyPoseInWorld(plant_context, robot_plant.GetBodyByName('iiwa_link_7')).matrix()

        ret = vec[:, 3][:3]

        print(context.get_time())

        output.SetFromVector(ret)


poseOutput = builder.AddSystem(PoseOutput())
# 3.Signal logger
logger_pose = builder.AddSystem(SignalLogger(3))
builder.Connect(poseOutput.get_output_port(0), logger_pose.get_input_port(0))

kSdfPath = "drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf"

rigid_body_tree = RigidBodyTree()
AddModelInstancesFromSdfFile(FindResourceOrThrow(kSdfPath),
                             FloatingBaseType.kFixed,
                             None,
                             rigid_body_tree
                             )


def kuka_PID_params():
    num_joint_kuka = 7
    kp = np.array([100, 100, 100, 100, 100, 100, 100])
    ki = np.zeros(num_joint_kuka)
    kd = 2 * np.sqrt(kp)

    assert kp.shape[0] == num_joint_kuka

    return kp, ki, kd

iiwa_kp,iiwa_ki,iiwa_kd = kuka_PID_params()
controller = builder.AddSystem(RbtInverseDynamicsController(rigid_body_tree, iiwa_kp, iiwa_ki, iiwa_kd, False))

builder.Connect(robot_plant.get_continuous_state_output_port(),
               controller.get_input_port(0))
builder.Connect(controller.get_output_port(0),
               robot_plant.get_actuation_input_port())


traj_pp = MakeControlledKukaPlan()

traj_src = builder.AddSystem(TrajectorySource(traj_pp, 1))  #outputs q + v
traj_src.set_name("trajectory_source")

builder.Connect(traj_src.get_output_port(0),
                  controller.get_input_port(1))

diagram = builder.Build()

diagram_context = diagram.CreateDefaultContext()
plant_context = diagram.GetMutableSubsystemContext(robot_plant,diagram_context)


simulator = Simulator(diagram, diagram_context)
simulator.set_target_realtime_rate(1)
simulator.Initialize()
simulator.get_mutable_integrator().set_target_accuracy(1e-3)

simulation_time = 10

t0 = time.time()
simulator.StepTo(simulation_time)

tf = time.time()



print("t = ", tf-t0)
