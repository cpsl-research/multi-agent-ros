import argparse
import os
import pickle
import shutil
from copy import deepcopy
from typing import TYPE_CHECKING, Dict

import numpy as np
from tqdm import tqdm


if TYPE_CHECKING:
    from avtrust_bridge.hooks import TrustFusionRosbagHook
    from avstack.datastructs import DataContainer

import avtrust  # noqa # pylint: disable=unused-import
import avtrust_bridge  # noqa # pylint: disable=unused-import
from avstack.config import Config
from avstack.metrics import get_instantaneous_metrics
from avstack_rosbag import DatasetPostprocessor
from cfg_utils import parse_mc_cfgs, print_cfg


sep = "======================================================\n"


def metrics_single_frame(
    agents_all: Dict[int, "DataContainer"],
    truths_all: "DataContainer",
    truths_agents: Dict[int, "DataContainer"],
    fusion_hook: "TrustFusionRosbagHook",
):
    """Compute the trust based on the hook information"""
    metrics = {
        "tracks_all": None,
        "tracks_trusted": None,
        "trust_agents": None,
        "trust_tracks": None,
    }

    # metric 1: single frame metrics on all the tracks
    if fusion_hook.hook.tracks_fused is not None:
        metrics["tracks_all"] = get_instantaneous_metrics(
            tracks=fusion_hook.hook.tracks_fused,
            truths=truths_all,
            assign_radius=2.0,
            timestamp=truths_all.timestamp,
        )

    # metric 2: single frame metrics on the trusted tracks
    if fusion_hook._metrics_assignment is not None:
        metrics["tracks_trusted"] = fusion_hook._metrics_assignment

    # metric 3: trust on the agents/tracks
    if fusion_hook._metrics_trust is not None:
        metrics["trust_agents"] = fusion_hook._metrics_trust["agents"]
        metrics["trust_tracks"] = fusion_hook._metrics_trust["tracks"]

    return metrics


def main(args):
    # initialize the postprocessing hooks
    rng = np.random.RandomState(args.mc_seed)
    hook_config_base = Config.fromfile(args.hook_config)
    os.makedirs(args.metrics_save_dir, exist_ok=True)

    # search for MC configurations in the config
    print(f"Generating {args.n_mcs} configs from the base config")
    all_hook_cfgs = [
        parse_mc_cfgs(deepcopy(hook_config_base), rng=rng) for _ in range(args.n_mcs)
    ]

    # we can set all hooks at once if they are independent
    all_hooks_remap = [h for hook in all_hook_cfgs for h in hook["other_hooks"]]
    print_cfg(all_hooks_remap)

    # metrics directory
    metrics_dir_path = os.path.join(
        args.metrics_save_dir,
        args.hook_config.replace("configs/", "").replace(".py", ""),
    )
    if os.path.exists(metrics_dir_path):
        shutil.rmtree(metrics_dir_path)
    os.makedirs(metrics_dir_path)

    # loop over the rosbags
    subdirs = [f.path for f in os.scandir(args.rosbag_directory) if f.is_dir()]
    for i_trial, subdir in tqdm(enumerate(subdirs)):
        # get bag filepath
        bag_name = subdir.split("/")[-1]
        bag_path = os.path.join(subdir, bag_name + "_0.db3")
        if os.path.exists(bag_path):
            # load postprocessor
            postproc = DatasetPostprocessor(
                bag_folder=args.rosbag_directory,
                bag_name=bag_name,
                other_hooks=all_hooks_remap,
            )
            print(
                f"\n{sep}Loaded postprocessor of length {len(postproc)} from {bag_path} for trial {i_trial}\n{sep}\n"
            )

            # create metrics folder for this run
            metrics_dir_trial = os.path.join(
                metrics_dir_path,
                f"trial_{i_trial}",
            )
            os.makedirs(metrics_dir_trial)

            # save hooks in the metrics folder
            with open(os.path.join(metrics_dir_trial, f"hooks_config.p"), "wb") as f:
                pickle.dump(all_hooks_remap, f)

            # save dataset config in the metrics folder
            # with open(os.path.join(metrics_dir_path, f"dataset_config.p"), "wb") as f:
            #     pickle.dump()

            # run all files in postprocessing
            metrics_all = []
            for i_frame, (truths_all, truths_agents, all_agents) in tqdm(
                enumerate(postproc), total=len(postproc)
            ):

                # compute per-frame metrics for each hook
                if truths_all is not None:

                    # compute single frame metrics
                    metrics_hooks = [
                        metrics_single_frame(
                            agents_all=all_agents,
                            truths_all=truths_all,
                            truths_agents=truths_agents,
                            fusion_hook=hook,
                        )
                        for hook in postproc.hooks["other"]
                    ]
                    metrics_all.append(metrics_hooks)

                    # save this frame's metrics
                    with open(
                        os.path.join(metrics_dir_trial, f"metrics_{i_frame:04d}.p"),
                        "wb",
                    ) as f:
                        pickle.dump(metrics_hooks, f)
        else:
            print(f"{bag_path} not found!!!!")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("rosbag_directory", type=str)
    parser.add_argument("hook_config", type=str)
    parser.add_argument("n_mcs", type=int)
    parser.add_argument("--mc_seed", type=int, default=0)
    parser.add_argument("--metrics_save_dir", type=str, default="./metrics/")
    args = parser.parse_args()

    main(args)
