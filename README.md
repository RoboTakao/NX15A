# NX15A
Summary:
http://robotakao.jp/NX15/e/index.html


[Key Controll]
 When controlling with the keyboard.

 M5Atom sketch : nx15_rosserial_wifi.ino

 roslaunch nx15a nx15a_simulation_rosserial.launch
 rosrun nx15a nx15a_walk_ik.py

[Blynk Controll]
 When controlling with the Blynk.

 M5Atom sketch : nx15_rosserial_wifi_blynk_BT.ino

 roslaunch nx15a nx15a_simulation_rosserial.launch
 rosrun nx15a nx15a_walk_ik_blynk.py

[Wii Nunchunck Controll]
 When controlling with the Wii controller (M5Atom).
 Use of IMU.

 M5Atom sketch
  For robot : nx15_rosserial_wifi_IMU.ino
  For wii controller : nx15_rosserial_wifi_wii.ino

 roslaunch nx15a nx15a_simulation_rosserial_wii_imu.launch
 rosrun nx15a nx15a_walk_ik_wii_imu.py

[Ultra Sonic Dinstance Sensor with Wii Nunchunck Controll]
 Ultra Sonic Distance Sensor use.
 When controlling with the Wii controller (M5Atom).

 M5Atom sketch
  For robot : nx15_rosserial_wifi_IMU_Sonic.ino
  For wii controller : nx15_rosserial_wifi_wii.ino

 roslaunch nx15a nx15a_simulation_rosserial_wii_imu.launch
 rosrun nx15a nx15a_walk_ik_wii_imu_distancw.py
