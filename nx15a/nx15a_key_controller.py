#!/usr/bin/env python
# license removed for brevity
import rospy
from pynput import keyboard
import nx15a_walk_fwd, nx15a_walk_leftturn, nx15a_walk_rightturn, nx15a_walk_leftside, nx15a_walk_rightside, nx15a_walk_back, nx15a_start_point, nx15a_left_head, nx15a_right_head

print("u :Left Turn i: Forward o: Right Turn")
print("j :Left Side k: Back    l: Right Side")

nx15a_start_point.talker()

def on_press(key):
    keyinput = key.char
    if keyinput == "i":
        nx15a_walk_fwd.talker()
    if keyinput == "j":
        nx15a_walk_leftside.talker()
    if keyinput == "l":
        nx15a_walk_rightside.talker()
    if keyinput == "k":
        nx15a_walk_back.talker()
    if keyinput == "u":
        nx15a_walk_leftturn.talker()
    if keyinput == "o":
        nx15a_walk_rightturn.talker()
    if keyinput == "m":
        nx15a_left_head.talker()
    if keyinput == ".":
        nx15a_right_head.talker()

# Collect events until released
with keyboard.Listener(on_press=on_press) as listener:
    listener.join()