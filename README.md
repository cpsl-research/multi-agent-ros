# Multi-Agent ROS Experiments


## Installation

This project uses submodules instead of system installations because it is under such heavy development. As a result, the nodes must be launched from inside a virtual environment to support local paths to projects/dependencies. The procedure for this is as follows:

### Option 1: Using poetry
1. Configure your poetry projects to be able to use the system packages with `poetry config virtualenvs.options.system-site-packages true`. This must be set BEFORE the poetry project is installed.
1. Install the virtual environment with `poetry install`
1. Activate the virtual environemnt with `poetry shell`
1. `cd` back to the project workspace (where you can see the `src` folder)
1. Build the project with `python -m colcon build --symlink-install`
1. Source the project with `source install/setup.bash` or `source install/setup.zsh`
1. Run your launch file, e.g., `ros2 launch <launch file here>`

### Option 2: Using conda + poetry

TODO

### Option 3: Using virtualenv

TODO
