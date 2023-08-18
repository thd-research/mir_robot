This PID point2point controller uses a navigation stack and its DWB local planner for finding the best trajectory.
This approach provides obstacle avoidance while the velocity commands are calculated by the PID controller.
Following steps are followed for running the PID point2point controller:


1. in lines 19 and 46 of "mir_empty_world.launch" file (located in "/mir_gazebo/launch" folder) find

        <remap from="mobile_base_controller/cmd_vel" to="cmd_vel" />

    and modify it to:

        <remap from="mobile_base_controller/cmd_vel" to="cmd_vel_user_defined" />

    This "cmd_vel_user_defined" (that gazebo subscribes to it) is a new defined topic for publishing velocity commands of the point2point controller.

2. run gazebo, localization, navigation, and rviz with one of the methods:
    
    a. run them all with this command

        roslaunch mir_driver mir_run_gaz_rviz_maze_world.launch

    b. or alternately run them separatly

        #gazebo:

        roslaunch mir_gazebo mir_maze_world.launch
        rosservice call /gazebo/unpause_physics   # or click the "start" button in the Gazebo GUI

        #localization:
        roslaunch mir_navigation amcl.launch initial_pose_x:=10.0 initial_pose_y:=10.0
        or alternatively: roslaunch mir_gazebo fake_localization.launch delta_x:=-10.0 delta_y:=-10.0

        #navigation:
        roslaunch mir_navigation start_planner.launch \
            map_file:=$(rospack find mir_gazebo)/maps/maze.yaml \
            virtual_walls_map_file:=$(rospack find mir_gazebo)/maps/maze_virtual_walls.yaml
            
        rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz

3. run the PID point2point controller:

        rosrun mir_driver point2point_PID_controller.py 

4. run the plotting for global and local planner:

        rosrun mir_driver plot_plan.py 

5. set an arbitrary "2D Nav Goal" with rviz
