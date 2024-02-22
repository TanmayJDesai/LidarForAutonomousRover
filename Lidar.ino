#include <RPLidar.h>
#include <SoftwareSerial.h>
RPLidar lidar;

#define LED_ENABLE  12
#define LED_R       9
#define LED_G       11
#define LED_B       10
#define RPLIDAR_MOTOR 5

#define RPLIDAR_RX 0 // RPLIDAR RX pin
#define RPLIDAR_TX 1 // RPLIDAR TX pin

#define LAPTOP_RX 2// Laptop RX pin
#define LAPTOP_TX 3 // Laptop TX pin

//SoftwareSerial mySerial(3, 4);

#define NEO_RGBSPACE_MAX (byte)(200L*255/360)

int _r, _g, _b;

void hue_to_rgb( _u8 hue)
{
    if (hue < 120L*255/360)
    {
        _g = hue;
        _r = NEO_RGBSPACE_MAX - hue;
        _b = 0;
    }else if (hue < 240L*255/360)
    {
        hue -= 120L*255/360;
        _b = hue;
        _g = NEO_RGBSPACE_MAX - hue;
        _r = 0;
    }else
    {
        hue -= 240L*255/360;
        _r = hue;
        _b = NEO_RGBSPACE_MAX - _r;
        _g = 0;
    }
}

void displayColor(float angle, float distance)
{
    byte hue = angle*255/360;
    hue_to_rgb(hue);
    
    int lightFactor = (distance>500.0)?0:(255-distance*255/500);
    _r *=lightFactor;
    _g *=lightFactor;
    _b *=lightFactor;
    
    _r /= 255;
    _g /= 255;
    _b /= 255;    
    
    analogWrite(LED_R, 255-_r);
    analogWrite(LED_G, 255-_g);
    analogWrite(LED_B, 255-_b);   
}

void setup() {
  Serial.begin(9600);
  Serial2.begin(115200);

  lidar.begin(Serial2);
  
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  
  pinMode(LED_ENABLE, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  
  digitalWrite(LED_ENABLE, HIGH);
  
  analogWrite(LED_R,255);
  analogWrite(LED_G,255);
  analogWrite(LED_B,255);

  Serial.print("hello");
  //Serial2.print("hello");
}

float minDistance = 100000;
float angleAtMinDist = 0;

void loop() {
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance;
    float angle = lidar.getCurrentPoint().angle;
    
    if (lidar.getCurrentPoint().startBit) {
      displayColor(angleAtMinDist, minDistance);
      minDistance = 100000;
      angleAtMinDist = 0;
    } else {
       if ( distance > 0 &&  distance < minDistance) {
          minDistance = distance;
          angleAtMinDist = angle;
       }
    }
    Serial.print(angle);
    Serial.print(",");
    Serial.println(distance);

  } else {
    analogWrite(RPLIDAR_MOTOR, 0);
    
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       lidar.startScan();
       analogWrite(RPLIDAR_MOTOR, 255);
       delay(1000);
    }
  }
}
