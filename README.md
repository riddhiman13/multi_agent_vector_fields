# multi_agent_vector_fields
## Circular Fields based (almost global) motion planner

### Used Agents:
   1. `REAL_AGENT`
   2. `GOAL_HEURISTIC`
   3. `OBSTACLE_HEURISTIC`
   4. `GOAL_OBSTACLE_HEURISTIC`
   5. `VEL_HEURISTIC`
   6. `RANDOM_AGENT`

### In order to run it:

`roslaunch multi_agent_vector_fields main_demo.launch`

### In order to switch between different obstacles scenarios:

Change path name of `obstacles_X.yaml` in `main()` in `main.cpp` and rebuild the workspace `catkin_make` or `catkin build` 
![image](https://github.com/user-attachments/assets/fde16450-744a-4e30-ae3c-5e423ab7a692)

`obstacles_1.yaml`

![image](https://github.com/user-attachments/assets/04ecbb64-f5ba-46c4-a3a0-3629fb6688cc)


`obstacles_2.yaml`

![image](https://github.com/user-attachments/assets/f2647f36-f48c-44f3-b5bb-559ca8ee0bf4)

### Updated History
`28.12.2024`: First with Multi-agent(4) version

`29.12.2024`: Add launch file and twists publisher

`02.01.2025`: Add new agents(7) and change obstacle description

`04.01.2025`: Add rotational part in force attractor and combine the two parts to the twist
