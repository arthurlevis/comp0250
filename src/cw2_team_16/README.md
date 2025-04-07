# COMP0250: Robot Sensing, Manipulation and Interaction
## Coursework 2: Pick and Place, Object Detection and Localization

Authors: Yun-Chen Lin, Xinyang Huang, Arthur Levisalles


Create a workspace:
1. in your home directory, `mkdir your_workspace`
2. `cd ~/your_workspace`
3. `catkin build`
4. `source devel/setup.bash`

Clone the comp0250_s25_labs in your workspace's src folder:
1. `cd ~/your_workspace/src`
2. `git clone https://github.com/surgical-vision/comp0250_s25_labs.git`

Add our cw2_team_16 package in comp0250_s25_labs/src and make sure to replace x by 16 in the following locations:
- launch/run_solution.launch line 17
- package.xml line 3
- CMakeLists.txt line 2

You can now run the coursework with:
1. `cd ~/your_workspace`
2. `catkin build`
3. `source devel/setup.bash`
4. `roslaunch cw1_team_16 run_solution.launch`


And begin task 1, 2 or 3 in another terminal tab:
> `rosservice call /task 1`

Number of working hours:
- Task 1: 10
- Task 2: 8
- Task 3: 7
- Total time: 25

Group members contribution:
- Yun-Chen Lin: 30%
- Xinyang Huang: 40% 
- Arthur Levisalles: 30%