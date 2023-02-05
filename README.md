# human_aware_navigation

## Software architecture

<img src="software_arch.jpg" alt="software_arch" width="480"/>

The socially-compliant (SC) navigation module is designed as a plug-and-play component, which can be deployed on different computing units or different robots as needed. What we've tested so far:

1. [Social Navigation Layers](https://github.com/DLu/navigation_layers) (deployed on the onboard PC side, CPU-based)
2. [Time Dependent Planning](https://github.com/marinaKollmitz/human_aware_navigation) (deployed on the onboard PC side, CPU-based)
3. [Collision Avoidance with Deep Reinforcement Learning](https://github.com/mit-acl/cadrl_ros) (deployed on the AGX side, GPU-based, SC-ready)

## Hardware platform

<img src="Isaac.jpg" alt="Isaac" width="480"/>

The instrumented CAD designs and 3D models of sensor mounts can be found [here](hardware).

## Build
```sh
sudo apt install ros-melodic-navigation ros-melodic-people ros-melodic-navigation-layers
cd ~/catkin_ws/src/
git clone https://github.com/Nedzhaken/human_aware_navigation
cd ~/catkin_ws
catkin_make
```

## Run
```sh
// Just to see the effect:
roslaunch human_aware_navigation human_aware_navigation.launch
// Want to do benchmarking:
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

## The Robotic Social Attributes Scale (RoSAS)

| Warmth | Competence | Discomfort |  
| :--- | :---: | :---: |
| Happy | Capable | Scary |
| Feeling  | Responsive | Strange |
| Social  | Interactive | Awkward |
| Organic  | Reliable | Dangerous |
| Compassionate  | Competent | Awful |
| Emotional  | Knowledgeable | Aggressive |

<!-- ### Could you rate your impression of the agent’s* behavior with the following criteria (RoSAS)?

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
 -->
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
