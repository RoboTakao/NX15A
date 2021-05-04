# NX15A

//Launch simulation

roslaunch nx15a nx15a_simulation.launch

//Initial pose

rosrun nx15a nx15a_start_point.py

//Forward

rosrun nx15a nx15a_walk_fwd.py

//Back

rosrun nx15a nx15a_walk_back.py

//Left turn

rosrun nx15a nx15a_walk_leftturn.py

//Right turn

rosrun nx15a nx15a_walk_rightturn.py
w
//Left side

rosrun nx15a nx15a_walk_leftside.py

//Right side

rosrun nx15a nx15a_walk_rightside.py

//Left Head

rosrun nx15a nx15a_left_head_.py

//Right Head

rosrun nx15a nx15a_right_head_.py

//Key controllers  It's necessary to install pynput

rosrun nx15a nx15a_key_controller.py