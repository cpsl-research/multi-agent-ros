import argparse


def main(args):
    with open(args.base_config, "r") as f:
        lines = f.readlines()
    
    for i_agent in range(args.n_agents):
        lines_agent = lines.copy()
        check_next_enable = False

        # loop over lines in *reverse*
        for i_line, line in reversed(list(enumerate(lines_agent))):
            # first we need ot
            if "Name: Agent" in line:
                agent_ID = int(line.split(" ")[-1].split("-")[0].replace("Agent", ""))
                check_next_enable = True
                next_agent_enabled = 'true\n' if agent_ID == i_agent else 'false\n'
            elif "Enabled" in line:
                if check_next_enable:
                    enabled = line.split(' ')[-1]
                    lines_agent[i_line] = line.replace(enabled, next_agent_enabled)
                    check_next_enable = False
        
        # write the config
        with open(f"agent{i_agent}_config.rviz", "w") as f:
            f.write("".join(lines_agent))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--base_config", type=str, default="multi_agent_config.rviz")
    parser.add_argument("--n_agents", type=int, default=4)

    args = parser.parse_args()
    main(args)