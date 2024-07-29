import argparse


def main(args):
    pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--base_config", type=str, default="multi_agent_config.rviz")
    parser.add_argument("--n_agents", type=int, default=4)

    args = parser.parse_args()
    main(args)