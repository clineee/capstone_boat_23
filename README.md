# AVVS Capstone

The Autonomous Vessel Vision System (AVVS) is designed to take a pre-existing oceanic research vessel owned by the Oregon State University College of Earth, Ocean, and Atmospheric Sciences and make it autonomous. The vessel is for studying the interactions of salt-fresh water interfaces, often in areas with irregular magnetic fields making standard radio control systems difficult to operate. So the AVVS needs to be able to identify and navigate around glaciers, other boats, and contend with currents and winds without constant communication with the researchers overseeing it. The goal of this system is to have full pathfinding and navigation based on environmental input.

The 2022 computer science capstone group has taken the work camera based object detection done by a 2020 capstone group and have added a lidar sensor and improved mapping to implement a pathfinding algorithm intended to content with the real world conditions. To help with testing and simulation the 2022 group is using Robot Operating System 2 (ROS2), a professional open source toolchain, to do simulation and development.
To make a robust system the 2022 group is using code from the 2020 group, original code, and ROS2 packages. In addition, some of the ROS2 packages needed to be updated and refactored by the group to work with maintained versions of ROS2.

In the current build there are systems for detecting objects with lidar, cameras, as well as mapping and pathfinding. The sensor systems are not integrated with the mapping and pathfinding quite yet, and the pathfinding is still the ROS2 basic parameters and needs to be more finely tuned.

Future goals include:
	-Improved navigation
	
	-Total system integration of all modules
	
	-Improved documentation
	
	-Real world testing of the system

## Authors

- [Parker Carlson](https://www.github.com/thefxperson)
- [Sammy Jazdak](https://www.github.com/SamuelJazdak2)
- [Ethan Cline](https://www.github.com/clineee)


## Acknowledgements

 - [Johnathan Nash](nashj@oregonstate.edu)
 - [Jasmine Nahorniak](jasmine.nahorniak@oregonstate.edu)
