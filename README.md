# Multi-Agent ROS Experiments


## Installation

This project uses submodules instead of system installations because it is under such heavy development. As a result, the nodes must be launched from inside a virtual environment to support local paths to projects/dependencies. The procedure for this is as follows:

### Option 1: Using poetry
1. Configure your poetry projects to be able to use the system packages with `poetry config virtualenvs.options.system-site-packages true`. This must be set BEFORE the poetry project is installed.
1. Install the virtual environment with `poetry install --no-root`
1. Activate the virtual environemnt with `poetry shell`
1. `cd` back to the project workspace (where you can see the `src` folder)
1. Build the project with `python -m colcon build --symlink-install` (the `symlink-install` turns out to be important for path management)
1. Source the project with `source install/setup.bash` or `source install/setup.zsh`
1. Run your launch file, e.g., `ros2 launch mar_bringup <launch file here>`, for example: `ros2 launch mar_bringup vehiclesec_infra.launch.py`

### Option 2: Using conda + poetry

TODO

### Option 3: Using virtualenv

TODO


## AVstack submodules -- **IMPORTANT**

This project makes heavy use of the AVstack submodules (`avstack` and `avapi`). Unfortunately, the other submodules (`avstack-rosbridge` and `multi-agent-trust-estimator`) ALSO make use of those same submodules, making `avstack` and `avapi` second-order submodules. This can lead to version conflicts. The ideal solution to this problem would be to install a version of `avstack` and `avapi` with `pip` and have each of the other submodules use a symbiotic version of `avstack` and `avapi`. This solution is not possible because `avstack` and `avapi` are still under development and cannot be released yet to `pypi`. The solution we came up with is to add `lib-avstack-core` and `lib-avstack-api` as first-order submodules to this project and to change the paths of the dependencies of `avstack-rosbridge` and `multi-agent-trust-estimator` on `avstack` and `avapi` to point to the first-order version and NOT their own second-order submodules. 

All this is to say...when making changes to `lib-avstack-core` and `lib-avstack-api`, make the changes in `multi-agent-ros/submodules/lib-avstack-core(api)` and NOT in e.g., `multi-agent-ros/submodules/avstack-rosbridge/submodules/lib-avstack-core`. 