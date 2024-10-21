from argparse import ArgumentParser

import avapi  # noqa # pylint: disable=unused-import
import avsec  # noqa # pylint: disable=unused-import
import avsec_bridge  # noqa # pylint: disable=unused-import
import avstack  # noqa # pylint: disable=unused-import
import avtrust  # noqa # pylint: disable=unused-import
import avtrust_bridge  # noqa # pylint: disable=unused-import
from avstack.config import Config
from avstack_rosbag.config import ROSBAG
from tqdm import tqdm


def main(args):
    # build from config
    cfg = Config.fromfile(args.config_filepath)
    dataset_roswriter = ROSBAG.build(cfg["rosbag_writer"])

    # run all frames
    for _ in tqdm(dataset_roswriter):
        pass


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("config_filepath", type=str)
    args = parser.parse_args()
    main(args)
