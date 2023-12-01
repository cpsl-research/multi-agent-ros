# model for an aggregate scenario
import os
from inspect import getsourcefile
from os.path import abspath


root = os.path.dirname(os.path.dirname(abspath(getsourcefile(lambda: 0))))
world = os.path.join(root, "./_base_/base_world.py")

n_untrusted_agents = 1
agents = [
    os.path.join(root, "./agent/untrusted_static_agent.py")
    for _ in range(n_untrusted_agents)
]

n_objects = 5
objects = [os.path.join(root, "./_base_/base_object.py") for _ in range(n_objects)]

commandcenter = os.path.join(root, "./_base_/base_command_center.py")
