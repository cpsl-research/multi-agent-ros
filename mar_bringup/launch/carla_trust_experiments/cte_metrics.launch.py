import os
import sys

from launch import LaunchDescription
from launch.actions import OpaqueFunction


dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)
from utils import get_metrics_evaluator, get_metrics_visualizer


def generate_launch_description():
    metrics_eval_node = OpaqueFunction(function=get_metrics_evaluator)
    metrics_viz_node = OpaqueFunction(function=get_metrics_visualizer)

    return LaunchDescription(
        [
            metrics_eval_node,
            metrics_viz_node,
        ]
    )
