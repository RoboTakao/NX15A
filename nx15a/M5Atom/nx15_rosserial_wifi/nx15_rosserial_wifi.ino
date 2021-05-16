/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
#include "M5Atom.h"
#include <Wire.h>          // I2C setting
#include <PCA9685.h>       //for PCA9685
#include <ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <WiFi.h>

bool IMU6886Flag = false;  //今回はなくても良い

//PCA9685のアドレス指定
PCA9685 pwm = PCA9685(0x40);

#define SERVOMIN 104            //Min pulse width (12bit 500μs) 
#define SERVOMAX 512            //Max pulse width (12bit 2500μs) 

float cont_min = -1570;
float cont_max = 1570;
int SrvAng[12] = {4914, 4914, 4914, 4914, 4914, 4914, 4914, 4914, 4914, 4914, 4914, 4914}; //90deg
float TARGET_JOINT_POSITIONS[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float TRIM[12] = {0.03,-0.07,-0.05,0.03,-0.05,0.09,-0.03,0.1,-0.12,0.05,0.12,-0.14};
int target_angle[12];

const char* ssid = "XXXXXXXXXXXXX";
const char* password = "XXXXXXXXXXXXX";

WiFiClient client;
IPAddress server(192, 168, 10, 16); //ROS core IP adress

class WiFiHardware {
  public:
    WiFiHardware() {};
    void init() {
      client.connect(server, 11411);   
    }
    int read() {
      return client.read();      
    }
    void write(uint8_t* data, int length) {
      for (int i = 0; i < length; i++)
        client.write(data[i]);
    }
    unsigned long time() {
      return millis(); // easy; did this one for you
    }
};

ros::NodeHandle_<WiFiHardware> nh;

void servo_cb(const sensor_msgs::JointState& msg){
  
  TARGET_JOINT_POSITIONS[0] = msg.position[6]; //FrontRight
  TARGET_JOINT_POSITIONS[1] = -msg.position[7]; //FrontRight
  TARGET_JOINT_POSITIONS[2] = -msg.position[8]; //FrontRight
  TARGET_JOINT_POSITIONS[3] = msg.position[0]; //FrontLeft
  TARGET_JOINT_POSITIONS[4] = msg.position[1]; //FrontLeft
  TARGET_JOINT_POSITIONS[5] = msg.position[2]; //FrontLeft
  TARGET_JOINT_POSITIONS[6] = msg.position[9]; //RearRight
  TARGET_JOINT_POSITIONS[7] = -msg.position[10]; //RearRight
  TARGET_JOINT_POSITIONS[8] = -msg.position[11]; //RearRight
  TARGET_JOINT_POSITIONS[9] = msg.position[3]; //RearLeft
  TARGET_JOINT_POSITIONS[10] = msg.position[4]; //RearLeft
  TARGET_JOINT_POSITIONS[11] = msg.position[5]; //RearLeft

  servo_set();

  for (int i=0; i <= 10; i++){
    Serial.print(target_angle[i]);
    Serial.print(" ");
  }
  Serial.println(target_angle[11]);

}

ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);

void setupWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if (i == 21) {
    Serial.print("Could not connect to"); Serial.println(ssid);
    while (1) delay(500);
  }
  Serial.print("Ready! Use ");
  Serial.print(WiFi.localIP());
  Serial.println(" to access client");

  M5.dis.drawpix(0, 0x00ff00);  //red
  M5.dis.drawpix(0, 0x00ff00);  //red
}

void servo_set()
{
  for (int i=0; i <= 11; i++){
    target_angle[i] = 1000 * (TARGET_JOINT_POSITIONS[i]+TRIM[i]);
    SrvAng[i] = map(target_angle[i], cont_min, cont_max, SERVOMIN, SERVOMAX); //angle（-1.57～1.57rad）-> pulse width（150～500）
    pwm.setPWM(i, 0, SrvAng[i]);
  }
}

void setup()
{ 
  Serial.begin(9600);
  // void M5Atom::begin(bool SerialEnable , bool I2CEnable , bool DisplayEnable )
  M5.begin(true, false, true);

  setupWiFi();
  
  Wire.begin(19, 22); //SDA-19, SCL-22
  
  pwm.begin();                   //initial setting (for 0x40) (PCA9685)
  pwm.setPWMFreq(50);            //PWM 50Hz (for 0x40) (PCA9685)

  servo_set();
  
  M5.dis.drawpix(0, 0x0000ff);  //blue
  M5.dis.drawpix(0, 0x0000ff);  //blue

  nh.initNode();
  nh.subscribe(sub);

  M5.dis.drawpix(0, 0xff0000); //green
  M5.dis.drawpix(0, 0xff0000); //green

}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
