// wii nunchuck adapter is based on https://github.com/todbot/wiichuck_adapter.git
#include "M5Atom.h"
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <WiFi.h>

int loop_cnt=0;

static uint8_t nunchuck_buf[6];   // array to store nunchuck data,

WiFiClient client;
IPAddress server(192, 168, 10, 19); //ROS core IP adress

const char* ssid = "XXXXXXXXXXXXX";
const char* password = "XXXXXXXXXXXXX";

// initialize the I2C system, join the I2C bus,and tell the nunchuck we're talking to it
static void nunchuck_init()
{ 
    Wire.begin(19, 22);                // join i2c bus as master; for M5Atom SDA-19, SCL-22
    Wire.beginTransmission(0x52);// transmit to device 0x52
    Wire.write((uint8_t)0x40);// sends memory address
    Wire.write((uint8_t)0x00);// sends sent a zero.  
    Wire.endTransmission();// stop transmitting
}

// Send a request for data to the nunchuck was "send_zero()"
static void nunchuck_send_request()
{
    Wire.beginTransmission(0x52);// transmit to device 0x52

    Wire.write((uint8_t)0x00);// sends one byte

    Wire.endTransmission();// stop transmitting
}

// Encode data to format that most wiimote drivers except only needed if you use one of the regular wiimote drivers
static char nunchuk_decode_byte (char x)
{
    x = (x ^ 0x17) + 0x17;
    return x;
}

// Receive data back from the nunchuck, returns 1 on successful read. returns 0 on failure
static int nunchuck_get_data()
{
    int cnt=0;
    Wire.requestFrom (0x52, 6);// request data from nunchuck
    while (Wire.available ()) {
        // receive byte as an integer
        nunchuck_buf[cnt] = nunchuk_decode_byte( Wire.read() );
        cnt++;
    }
    nunchuck_send_request();  // send request for next data payload
    // If we recieved the 6 bytes, then go print them
    if (cnt >= 5) {
        return 1;   // success
    }
    return 0; //failure
}

// returns zbutton state: 1=pressed, 0=notpressed
static int nunchuck_zbutton()
{
    return ((nunchuck_buf[5] >> 0) & 1) ? 0 : 1;  // voodoo
}

// returns zbutton state: 1=pressed, 0=notpressed
static int nunchuck_cbutton()
{
    return ((nunchuck_buf[5] >> 1) & 1) ? 0 : 1;  // voodoo
}

// returns value of x-axis joystick
static int nunchuck_joyx()
{
    return nunchuck_buf[0]; 
}

// returns value of y-axis joystick
static int nunchuck_joyy()
{
    return nunchuck_buf[1];
}

// returns value of x-axis accelerometer
static int nunchuck_accelx()
{
    return nunchuck_buf[2];   // FIXME: this leaves out 2-bits of the data
}

// returns value of y-axis accelerometer
static int nunchuck_accely()
{
    return nunchuck_buf[3];   // FIXME: this leaves out 2-bits of the data
}

// returns value of z-axis accelerometer
static int nunchuck_accelz()
{
    return nunchuck_buf[4];   // FIXME: this leaves out 2-bits of the data
}

class WiFiHardware {
  public:
    WiFiHardware() {};
    void init() {
      client.connect(server, 11412);   
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

std_msgs::Int32MultiArray Wii_joy;

ros::Publisher chatter("Wii_joystick", &Wii_joy);

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

void setup()
{
    Serial.begin(19200);
    // void M5Atom::begin(bool SerialEnable , bool I2CEnable , bool DisplayEnable )
    M5.begin(true, false, true);
    
    //nunchuck_setpowerpins();
    nunchuck_init(); // send the initilization handshake
    
    setupWiFi();

    Serial.println("Waiting for connections...");

    Wii_joy.data_length = 4;
    Wii_joy.data = (int32_t *)malloc(sizeof(int32_t)*3);
  
    nh.initNode();
    nh.advertise(chatter);

    M5.dis.drawpix(0, 0xff0000); //green
}

void loop()
{
    nunchuck_get_data();

    int zbut = nunchuck_zbutton();
    int cbut = nunchuck_cbutton(); 
    int joyx = map(nunchuck_joyx(),23,225,-15,15);
    if(abs(joyx)<=1) joyx=0;
    int joyy = map(nunchuck_joyy(),20,221,-15,15);
    if(abs(joyy)<=1) joyy=0;

    Serial.print(" zbut: "); Serial.print(zbut);
    Serial.print(" cbut: "); Serial.print(cbut);
    Serial.print(" joyx: "); Serial.print(joyx);
    Serial.print(" joyy: "); Serial.println(joyy);

    Wii_joy.data[0] = zbut;
    Wii_joy.data[1] = cbut;
    Wii_joy.data[2] = joyx;
    Wii_joy.data[3] = joyy;

    chatter.publish( &Wii_joy );
  
    nh.spinOnce();

    delay(50);
}
