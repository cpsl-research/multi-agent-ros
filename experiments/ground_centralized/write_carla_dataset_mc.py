from argparse import ArgumentParser
from copy import deepcopy

import avapi  # noqa # pylint: disable=unused-import
import avsec  # noqa # pylint: disable=unused-import
import avsec_bridge  # noqa # pylint: disable=unused-import
import avstack  # noqa # pylint: disable=unused-import
import avtrust  # noqa # pylint: disable=unused-import
import avtrust_bridge  # noqa # pylint: disable=unused-import
import numpy as np
from avstack.config import Config
from avstack_rosbag.config import ROSBAG
from cfg_utils import parse_mc_cfgs, print_cfg
from tqdm import tqdm


def main(args):
    # build from config
    rng = np.random.RandomState(args.mc_seed)
    cfg_base = Config.fromfile(args.config_filepath)

    # search for MC configurations in the config
    print(f"Generating {args.n_mcs} configs from the base config")
    all_cfgs = [parse_mc_cfgs(deepcopy(cfg_base), rng=rng) for _ in range(args.n_mcs)]

    # loop over all configs
    for i, cfg_run in enumerate(all_cfgs):
        print(f"Running {i} config of {len(all_cfgs)}")
        print_cfg(cfg_run)
        # run all frames for this config
        dataset_roswriter = ROSBAG.build(cfg_run["rosbag_writer"])
        for _ in tqdm(dataset_roswriter):
            pass
        del dataset_roswriter


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("config_filepath", type=str)
    parser.add_argument("n_mcs", type=int)
    parser.add_argument("--mc_seed", type=int, default=0)
    args = parser.parse_args()
    main(args)
