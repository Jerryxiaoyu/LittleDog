from pydrake.common import FindResourceOrThrow
from pydrake.geometry import (ConnectDrakeVisualizer, SceneGraph)
from pydrake.lcm import DrakeLcm
from pydrake.multibody.rigid_body_tree import (RigidBodyTree, AddModelInstancesFromSdfFile,
                                               FloatingBaseType,AddModelInstanceFromUrdfFile)
from pydrake.multibody.multibody_tree import UniformGravityFieldElement
from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
from pydrake.multibody.multibody_tree.parsing import AddModelFromSdfFile
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.common import (set_assertion_failure_to_throw_exception)
from pydrake.systems.controllers import (RbtInverseDynamicsController)
from pydrake.systems.primitives import (ConstantVectorSource, SignalLogger, TrajectorySource)
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer
from pydrake.solvers.ik import (PostureConstraint, 
                                WorldPositionConstraint,
                                RigidBodyConstraint,
                               IKoptions,
                               InverseKinPointwise)
from pydrake.trajectories import PiecewisePolynomial 

import numpy as np
import matplotlib.pyplot as plt


def MakeControlledKukaPlan():

    kSdfPath ="drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf"
    num_q =7
    rigid_body_tree = RigidBodyTree()
    AddModelInstancesFromSdfFile(FindResourceOrThrow(kSdfPath),
                                 FloatingBaseType.kFixed,
                                 None,
                                 rigid_body_tree
                                )

    zero_conf = rigid_body_tree.getZeroConfiguration()
    joint_lb = zero_conf - np.ones(num_q) * 0.01
    joint_ub = zero_conf + np.ones(num_q) * 0.01


    pc1 = PostureConstraint(rigid_body_tree, np.array([0,0.5]))
    joint_idx = np.arange(num_q)
    pc1.setJointLimits(joint_idx, joint_lb, joint_ub)

    pos_end= np.array([0.6, 0, 0.325])
    pos_lb = pos_end - np.ones(3)*0.005
    pos_ub = pos_end + np.ones(3)*0.005

    wpc1 = WorldPositionConstraint(rigid_body_tree,
                                   rigid_body_tree.FindBodyIndex("iiwa_link_7"),
                                   np.array([0,0,0]),
                                  pos_lb,
                                  pos_ub,
                                  np.array([1,3]))

    pc2 = PostureConstraint(rigid_body_tree, np.array([4,5.9]))
    pc2.setJointLimits(joint_idx, joint_lb, joint_ub)

    wpc2 = WorldPositionConstraint(rigid_body_tree,
                                   rigid_body_tree.FindBodyIndex("iiwa_link_7"),
                                   np.array([0,0,0]),
                                  pos_lb,
                                  pos_ub,
                                  np.array([6,9]))

    joint_position_start_idx = rigid_body_tree.FindChildBodyOfJoint("iiwa_joint_2").get_position_start_index()

    pc3 = PostureConstraint(rigid_body_tree, np.array([6, 8]))
    pc3.setJointLimits(np.array([joint_position_start_idx]), np.ones(1)*0.7, np.ones(1)*0.8)

    kTimes = np.array([0.0, 2.0, 5.0, 7.0, 9.0])
    q_seed = np.zeros((rigid_body_tree.get_num_positions(), kTimes.size))
    q_norm = np.zeros((rigid_body_tree.get_num_positions(), kTimes.size))

    for i in range(kTimes.size):
        q_seed[:,i] = zero_conf + 0.1* np.ones(rigid_body_tree.get_num_positions())
        q_norm[:,i] = zero_conf

    ikoptions = IKoptions(rigid_body_tree)
    constraint_array = [pc1, wpc1, pc2, pc3, wpc2]
    ik_results = InverseKinPointwise(rigid_body_tree,
                       kTimes,
                        q_seed,q_norm,
                        constraint_array,
                        ikoptions
                       )

    q_sol = ik_results.q_sol
    ik_info = ik_results.info
    infeasible_constraint = ik_results.infeasible_constraints

    info_good =True
    for i in range(kTimes.size):
        print('IK ik_info[{}] = {}'.format(i, ik_info[i]))
        if ik_info[i] != 1:
                info_good = False

    if not info_good:
        raise Exception("inverseKinPointwise failed to compute a valid solution.")

    knots = np.array(q_sol).transpose()
    traj_pp = PiecewisePolynomial.FirstOrderHold(kTimes,knots)

    return traj_pp