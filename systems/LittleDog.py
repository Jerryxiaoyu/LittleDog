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

class Robot(object):
    def __init__(self):
        pass


class LittleDog(object):
    def __init__(self, file_name , Is_fixed = True ):
        rb_tree = RigidBodyTree()
        world_frame = RigidBodyFrame("world_frame", rb_tree.world(), [0, 0, 0], [0, 0, 0])
        AddFlatTerrainToWorld(rb_tree, 1000, 10)
        robot_frame = RigidBodyFrame("robot_frame", rb_tree.world(), [0, 0, 0.2], [0, 0, 0])

        if Is_fixed is True:
            self.fixed_type = FloatingBaseType.kFixed
        else:
            self.fixed_type = FloatingBaseType.kRollPitchYaw
        # insert a robot from urdf files
        pmap = PackageMap()
        pmap.PopulateFromFolder(os.path.dirname(file_name))
        AddModelInstanceFromUrdfStringSearchingInRosPackages(
            open(file_name, 'r').read(),
            pmap,
            os.path.dirname(file_name),
            self.fixed_type,  # FloatingBaseType.kRollPitchYaw,
            robot_frame,
            rb_tree)

        self.rb_tree = rb_tree
        self.joint_limit_max = self.rb_tree.joint_limit_max
        self.joint_limit_min = self.rb_tree.joint_limit_min

        print(self.joint_limit_max)
        print(self.joint_limit_min)


# 6 front_left_hip_roll
# 7 front_left_hip_pitch
# 8 front_left_knee
# 9 front_right_hip_roll
# 10 front_right_hip_pitch
# 11 front_right_knee
# 12 back_left_hip_roll
# 13 back_left_hip_pitch
# 14 back_left_knee
# 15 back_right_hip_roll
# 16 back_right_hip_pitch
# 17 back_right_knee
joint_limits_h = [0.6,2.4,1.0,
                  0.6, 2.4, 1.0,
                  0.6, 3.5 ,3.1,
                  0.6, 3.5, 3.1]
joint_limits_l = [-0.6,-3.5, -3.1,
                  -0.6, -3.5 , -3.1,
                  -0.6,-2.4, -1.0,
                  -0.6, -2.4 ,-1.0]