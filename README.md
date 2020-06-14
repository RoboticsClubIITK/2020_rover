# 2020_rover
Summer Project 2020

## Setup and Installation

Ensure that you have a working ROS Melodic installation.
Install dependencies by:
```bash
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers
```

If you don't have a catkin workspace, create one as follows:
```bash
mkdir -p ~/rover_ws/src
cd ~/rover_ws
catkin init
```
Clone `catkin_simple` for build dependency into your workspace:
```bash
cd ~/rover_ws/src
git clone https://github.com/catkin/catkin_simple
```

You can either clone this repository directory or use your own personal fork(recommended, see below):
```
cd ~/rover_ws/src
git clone https://github.com/RoboticsClubIITK/2020_rover
```

Build all packages in the workspace using `catkin build`.

## Usage

In a new terminal, source your workspace:
```bash
source ~/rover_ws/devel/setup.bash
```

Launch the environment using:
```
roslaunch rover_sim default.launch
```

## Maintaining your Fork

Set up an `upstream` remote so that you can keep your fork updated with this repository. Follow these steps:
1. First, create a fork using the `Fork` button on the top-right. This will create a copy of the same repo on your GitHub account.

2. Clone your fork into your workspace
```bash
cd ~/rover_ws/src
git clone https://github.com/$YOUR_USERNAME/2020_rover
```
3. The default remote that points to your repository and allows you to push your changes to your repository is `origin`. We'll add another remote `upstream` that will keep track of changes made to this repository.
```
cd ~/rover_ws/src/2020_rover
git remote add upstream https://github.com/RoboticsClubIITK/2020_rover
```

4. To fetch changes from `upstream` to your local copy, do `git fetch upstream`.

5. To update your fork with changes from this repository, do the following:
```bash
git merge upstream/master # Merge the upstream's master into your own origin/master
git push origin master    # Push local changes to GitHub
```


## Credits

Meshes and Gazebo objects sourced from https://github.com/Christopheraburns/AWS-JPL-OSR-Challenge
