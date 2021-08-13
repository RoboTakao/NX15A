# NX15A
Summary:  
http://robotakao.jp/NX15/e/index.html  
  
  
[Key Controll]  
&emsp;When controlling with the keyboard.  
  
&emsp;M5Atom sketch : nx15_rosserial_wifi.ino  
  
&emsp;roslaunch nx15a nx15a_simulation_rosserial.launch  
&emsp;rosrun nx15a nx15a_walk_ik.py  
  
[Blynk Controll]  
&emsp;When controlling with the Blynk.  
  
&emsp;M5Atom sketch : nx15_rosserial_wifi_blynk_BT.ino  
  
&emsp;roslaunch nx15a nx15a_simulation_rosserial.launch  
&emsp;rosrun nx15a nx15a_walk_ik_blynk.py  
  
[Wii Nunchunck Controll]  
&emsp;When controlling with the Wii controller (M5Atom).  
&emsp;Use of IMU.  
  
&emsp;M5Atom sketch  
&emsp;&emsp;For robot : nx15_rosserial_wifi_IMU.ino  
&emsp;&emsp;For wii controller : nx15_rosserial_wifi_wii.ino  
  
&emsp;roslaunch nx15a nx15a_simulation_rosserial_wii_imu.launch  
&emsp;rosrun nx15a nx15a_walk_ik_wii_imu.py  
  
[Ultra Sonic Dinstance Sensor with Wii Nunchunck Controll]  
&emsp;Ultra Sonic Distance Sensor use.  
&emsp;When controlling with the Wii controller (M5Atom).  
  
&emsp;M5Atom sketch  
&emsp;&emsp;For robot : nx15_rosserial_wifi_IMU_Sonic.ino  
&emsp;&emsp;For wii controller : nx15_rosserial_wifi_wii.ino  
  
&emsp;roslaunch nx15a nx15a_simulation_rosserial_wii_imu.launch  
&emsp;rosrun nx15a nx15a_walk_ik_wii_imu_distancw.py  
