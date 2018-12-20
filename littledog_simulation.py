
from pydrake.lcm import DrakeLcm
from pydrake.systems.analysis import Simulator
from pydrake.systems.drawing import plot_system_graphviz

import os
import numpy as np
import matplotlib.pyplot as plt
from systems.littledog_simulation_diagram import LittleDogSimulationDiagram

dt = 0
lcm = DrakeLcm()

diagram, logger = LittleDogSimulationDiagram(lcm, dt)

# simulation setting
diagram_context = diagram.CreateDefaultContext()
simulator = Simulator(diagram, diagram_context)
simulator.set_publish_every_time_step(False)
simulator.set_target_realtime_rate(1)
simulator.Initialize()

simulation_time = 10

lcm.StartReceiveThread()
simulator.StepTo(simulation_time)
lcm.StopReceiveThread()

# plot graph
plot_system_graphviz(diagram, max_depth=2147483647)
plt.show()
