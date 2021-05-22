# NX15A
Summary:
http://robotakao.jp/NX15/e/index.html

//Launch simulation

roslaunch nx15a nx15a_simulation.launch

//Initial pose (stand up)

rosrun nx15a nx15a_stand_up.py

//Sit down

rosrun nx15a nx15a_sit_down.py

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

rosrun nx15a nx15a_left_head.py

//Right Head

rosrun nx15a nx15a_right_head.py

//Left rolling

rosrun nx15a nx15a_left_roll.py

//Right rolling

rosrun nx15a nx15a_right_roll.py

//Head up

rosrun nx15a nx15a_head_up.py

//Head down

rosrun nx15a nx15a_head_down.py

//Key controllers  It's necessary to install pynput

rosrun nx15a nx15a_key_controller.py
