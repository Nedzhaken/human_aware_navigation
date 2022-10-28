# human_aware_navigation

## Software architecture

<img src="software_arch.jpg" alt="software_arch" width="480"/>

## Hardware platform

<img src="Isaac.jpg" alt="Isaac" width="480"/>

The instrumented CAD designs and 3D models of sensor mounts can be found [here](hardware).

## Build
```sh
sudo apt install ros-melodic-navigation ros-melodic-people ros-melodic-navigation-layers
cd catkin_ws/src
git clone https://github.com/yzrobot/human_aware_navigation
cd catkin_ws
catkin_make
```

## Run
```sh
roslaunch human_aware_navigation human_aware_navigation.launch
roslaunch human_aware_navigation experiment.launch
```

## Tested environment
```
Jetson Xavier AGX
Ubuntu 18.04
ROS Melodic
Jetpack 4.4.1
CUDA 10.2
PCL 1.8
Eigen 3
```

## The example of the questionnare for the experiment

### Could you rate your impression of the agent’s* behavior with the following criteria (Godspeed)?

|  |  |  |  |  |  |  |
| :--- | :---: | :---: | :---: | :---: | :---: | ---: |
| Moving ridigly | 1 | 2 | 3 | 4 | 5 | &nbsp;&nbsp;&nbsp;&nbsp; Moving elegantly |
| Dislike | 1 | 2 | 3 | 4 | 5 | Like |
| Incompetent &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;  | 1 | 2 | 3 | 4 | 5 | Competent |
| Unconscious  | 1 | 2 | 3 | 4 | 5 | Conscious |
| Unfriendly  | 1 | 2 | 3 | 4 | 5 | Friendly |
| Unintelligent  | 1 | 2 | 3 | 4 | 5 | Intelligent |
| Machinelike  | 1 | 2 | 3 | 4 | 5 | Humanlike |
| Unpleasant  | 1 | 2 | 3 | 4 | 5 | Pleasant |
| Foolish  | 1 | 2 | 3 | 4 | 5 | Sensible |

### Could you rate your emotional state with the following criteria (Godspeed)?

|  |  |  |  |  |  |  |
| :--- | :---: | :---: | :---: | :---: | :---: | ---: |
| Anxious | 1 | 2 | 3 | 4 | 5 | Relaxed |
| Agitated  | 1 | 2 | 3 | 4 | 5 | Calm |
| Quiescent &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | 1 | 2 | 3 | 4 | 5 | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Surprised |

### Could you rate the general behavior of the agent* (13th question)?

|  |  |  |  |  |  |  |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| I do not accept<br />the agent's behavior  | 1 | 2 | 3 | 4 | 5 | I accept<br />the agent's behavior |

\* - during the experiment the agent was a robot or another person.

## Citation

If you are considering using this repository, please reference the following:

```
@article{io22software,
   author  = {Iaroslav Okunevich and Vincent Hilaire and Stephane Galland and Olivier Lamotte and Liubov Shilova and Yassine Ruichek and Zhi Yan},
   title   = {Software-hardware Integration and Human-centered Benchmarking for Socially-compliant Robot Navigation},
   journal = {CoRR},
   volume = {abs/2210.15628},
   year = {2022},
   url = {http://arxiv.org/abs/2210.15628},
   archivePrefix = {arXiv},
   eprint = {2210.15628}
}
```
