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
from robot_coomand_to_rigidbody_converter import RobotCommandToRigidBodyPlantConverter
from robot_state_encoder import RobotStateEncoder

from util.meshcat_rigid_body_visualizer import MeshcatRigidBodyVisualizer

model_path = os.path.join(os.path.dirname(__file__), '../Model/LittleDog.urdf')


def SetupLittleDog():
    # Build up your Robot World
    rb_tree = RigidBodyTree()
    world_frame = RigidBodyFrame("world_frame", rb_tree.world(), [0, 0, 0], [0, 0, 0])
    AddFlatTerrainToWorld(rb_tree, 1000, 10)
    robot_frame = RigidBodyFrame("robot_frame", rb_tree.world(), [0, 0, 0.2], [0, 0, 0])

    # insert a robot from urdf files
    pmap = PackageMap()
    pmap.PopulateFromFolder(os.path.dirname(model_path))
    AddModelInstanceFromUrdfStringSearchingInRosPackages(
            open(model_path, 'r').read(),
            pmap,
            os.path.dirname(model_path),
            FloatingBaseType.kFixed,  #FloatingBaseType.kRollPitchYaw,
            robot_frame,
            rb_tree)
    return rb_tree