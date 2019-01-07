
from pydrake.lcm import DrakeLcm
from pydrake.systems.analysis import Simulator
from pydrake.systems.drawing import plot_system_graphviz

import os
import numpy as np
import matplotlib.pyplot as plt
from systems.littledog_whole_diagram import LittleDogSimulationDiagram
#from systems.littledog_simulation_diagram import LittleDogSimulationDiagram

from systems.setup_littledog import SetupLittleDog
from pydrake.multibody.rigid_body_plant import DrakeVisualizer
from systems.setup_kinova import SetupKinova
import time

dt = 0
lcm = DrakeLcm()

rb_tree = SetupLittleDog()
#rb_tree = SetupKinova()
drake_visualizer = DrakeVisualizer(rb_tree, lcm, enable_playback= True)
diagram, logger,plant = LittleDogSimulationDiagram(lcm, rb_tree, dt, drake_visualizer)

num_pos = rb_tree.get_num_positions()
init_state =  np.concatenate((np.zeros(num_pos), np.zeros(num_pos)), axis=0)
# simulation setting
diagram_context = diagram.CreateDefaultContext()

plant_context = diagram.GetMutableSubsystemContext(plant,diagram_context)
plant.set_state_vector(plant_context, init_state)


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

# drake_visualizer.ReplayCachedSimulation()
# # plot graph
# plot_system_graphviz(diagram, max_depth=2147483647)
# plt.show()

plot_system_graphviz(diagram, max_depth=2147483647)
plt.show()

print("t = ", tf-t0)
t = logger.sample_times()
q = logger.data()

plt.figure(figsize=(15, 10))
for i in range(12):
    plt.subplot(4, 3, i + 1)
    plt.plot(t, logger.data()[i, :], 'b', label='id')

    plt.title('joint positon {}'.format(i))
    plt.legend(loc='best')

plt.show()