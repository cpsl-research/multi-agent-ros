"""
VEHICLE-INDIVIDUAL SCENARIO

This scenario has no use of infrastructure agents.
The agents solely rely on their own sensors for obtaining
situational awareness. There is no collaboration between agents.
"""

import os
from inspect import getsourcefile
from os.path import abspath


root = os.path.dirname(os.path.dirname(abspath(getsourcefile(lambda: 0))))
world = os.path.join(root, "./_base_/base_world.py")

# -- invidual agents
n_indiv_agents = 5
indiv_agents = [
    os.path.join(root, "./scenarios/sec-aware-vehicle-infra/agent_individual.py")
    for _ in range(n_indiv_agents)
]

# -- collaborative agents
collab_agents = []

# -- infrastructure agents
infra_agents = []

agents = collab_agents + indiv_agents + infra_agents
commandcenter = os.path.join(root, "./command_center/point_based_ci_fusion.py")