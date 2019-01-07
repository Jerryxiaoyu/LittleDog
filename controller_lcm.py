
import lcm

from lcmt import *
#from pydrake.lcm import DrakeLcm
import select
import time
import numpy as np
from systems.setup_kinova import SetupKinova



lc = lcm.LCM()


simulation_end_time = 10 # in seconds
t_s  = 0.005   # sampling time in ms

rb_tree = SetupKinova()
num_controlled_q_ = rb_tree.get_num_actuators()


ref_state = np.concatenate((rb_tree.getZeroConfiguration(),np.zeros(9)), axis=0)
#ref_state =  np.concatenate((np.array([0, 1, 1, 1, 1.7, 0.5, 0, 0, 0]),np.zeros(9)), axis=0)

def joints_PID_params(rbtree):
    num_joints = rbtree.get_num_actuators()

    kp = np.ones(num_joints) * 10
    ki = np.zeros(num_joints)
    kd = 2 * np.sqrt(kp)

    assert kp.shape[0] == num_joints
    return kp, ki, kd


def PIDcontroller(state, state_d , kp, ki, kd):
    controlled_state_diff = state_d - state

    # TODO  no integral processing
    ctrl_output =  kp * (controlled_state_diff[: num_controlled_q_]) +  kd * (controlled_state_diff[ num_controlled_q_:])

    ctrl_output = np.clip(ctrl_output, -1, 1 )
    return ctrl_output

def state_decoder(msg):
    if msg.num_joints == num_controlled_q_:
        q = np.array(msg.joint_position)
        qv = np.array(msg.joint_velocity)
    else:
        raise("error num of q")

    return   np.concatenate((q, qv))


class lcm_publisher(object):
    def __init__(self, channel, lcm_type, lc):
        self.channel = channel
        self.lcm_type = lcm_type
        self.lcm = lc

    def Publish(self, msg):

        self.lcm.publish(self.channel,  msg.encode())


class lcm_subscriptor(object):
    def __init__(self, channel, lcm_type, lc):
        self.subscriptor = lc.subscribe(channel,self._msg_handle)
        self.lcm_type = lcm_type
        self.msg = lcm_type()
    def _msg_handle(self, channel, message):

        self.msg = self.lcm_type.decode(message)
        # handling ...
        #print("msg: {}".format(msg.timestamp))
        #print(self.msg.timestamp)



subscription = lcm_subscriptor("EST_ROBOT_STATE" ,robot_state_t, lc)
t_data =[]
t = time.time()

publisher = lcm_publisher("ROBOT_COMMAND", littledog_command_t, lc)
kp,ki,kd = joints_PID_params(rb_tree)

try:
    while True:
        lc.handle()

        # controller handle
        t_data.append(time.time())

        state = state_decoder(subscription.msg)


        command = PIDcontroller(state, ref_state, kp,ki,kd)

        command_msg = littledog_command_t()
        command_msg.timestamp = 0  #context.get_time() * 1e3  # milliseconds
        command_msg.num_joints = num_controlled_q_
        command_msg.joint_command = command

        publisher.Publish(command_msg)

        # end fun
        if subscription.msg.timestamp >= simulation_end_time * 1e3:
            break
except KeyboardInterrupt:
    pass

#
# try:
#     timeout = t_s * 1.2  # amount of time to wait, in seconds
#     while True:
#         rfds, wfds, efds = select.select([lc.fileno()], [], [], timeout)
#         if rfds:
#             lc.handle()
#             # t_data.append(time.time())  #just for test
#         if subscription.msg.timestamp >= simulation_end_time * 1e3:
#             break
#
# except KeyboardInterrupt:
#     pass

t_data2 = []
for i in range(1, len(t_data)):
    tmp = t_data[i - 1]
    t_data2.append(t_data[i] - tmp)

import matplotlib.pyplot as plt

plt.plot(t_data2)
plt.show()

plt.plot(t_data)
plt.show()
