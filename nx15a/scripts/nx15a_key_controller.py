#!/usr/bin/env python
# license removed for brevity
import rospy
from pynput import keyboard
import nx15a_walk_fwd, nx15a_walk_leftturn, nx15a_walk_rightturn, nx15a_walk_leftside, nx15a_walk_rightside, nx15a_walk_back, nx15a_stand_up, nx15a_left_head, nx15a_right_head, nx15a_sit_down, nx15a_right_roll, nx15a_left_roll, nx15a_head_down, nx15a_head_up
print("                          8: StandUp")
print("y :Head Down u :Left Turn i: Forward o: Right Turn")
print("h :Left Roll j :Left Side k: Back    l: Right Side +: Right Roll")
print("n :Head Up   m :Left Head <: SitDown >: Right Head")

nx15a_stand_up.talker()

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
    if keyinput == "8":
        nx15a_stand_up.talker()
    if keyinput == ",":
        nx15a_sit_down.talker()
    if keyinput == ";":
        nx15a_right_roll.talker()
    if keyinput == "h":
        nx15a_left_roll.talker()
    if keyinput == "y":
        nx15a_head_down.talker()
    if keyinput == "n":
        nx15a_head_up.talker()

# Collect events until released
with keyboard.Listener(on_press=on_press) as listener:
    listener.join()