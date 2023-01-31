#include <DFRobot_BMP280.h>
#include <Wire.h>
#include <KalmanFilter.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include<TinyGPS++.h>
#include <SoftwareSerial.h>



int mpu_sayac = 0;
float previous_acc_z;
bool is_launched = false;
bool apogee_detected_velocity = false;



TinyGPSPlus gps;
Adafruit_MPU6050 mpu;

int buzzer = PD2;
int Dep_1 = 10;
int Dep_2 = 11;

static const int RXPin = 6, TXPin = 5;
SoftwareSerial ss(RXPin, TXPin);

String paket = "";

bool dep_1_state = false;
bool dep_2_state = false;

unsigned long previous_millis_deployment = 0;
const long interval_deployment = 2000;
int deployment_state = 0;

unsigned long previous_millis_buzzer = 0;
const long interval_buzzer = 500;
int buzzer_state = LOW;

typedef DFRobot_BMP280_IIC BMP;

BMP   bmp(&Wire, BMP::eSdoLow);


float baseline_pressure, first_alt;
float alt;



void printLastOperateStatus(BMP::eStatus_t eStatus)
{
  switch (eStatus) {
    case BMP::eStatusOK:    Serial.println("everything ok"); break;
    case BMP::eStatusErr:   Serial.println("unknow error"); break;
    case BMP::eStatusErrDeviceNotDetected:    Serial.println("device not detected"); break;
    case BMP::eStatusErrParameter:    Serial.println("parameter error"); break;
    default: Serial.println("unknow status"); break;
  }
}




float bmp280_read_altitude(float first_pressure, float dynamic_pressure ) {

  return 44330.0 * (1 - pow((float)dynamic_pressure / (float)first_pressure, 1 / 5.255));
}


bool apogee_detected(float first_altitude, float second_altitude) {
  float difference = second_altitude - first_altitude;
  delay(50);
  if (difference <= 10 && second_altitude > 150) {
    return true;
  } else
    return false;

}




void setup() {
  Serial.begin(9600);
  Wire.begin();
  ss.begin(9600);
  pinMode(buzzer, OUTPUT);
  pinMode(Dep_1, OUTPUT);
  pinMode(Dep_2, OUTPUT);


  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("");
  delay(100);

  bmp.reset();
  Serial.println("bmp read data test");
  while (bmp.begin() != BMP::eStatusOK) {
    Serial.println("bmp begin faild");
    printLastOperateStatus(bmp.lastOperateStatus);
    delay(2000);
  }
  Serial.println("bmp begin success");
  delay(100);

  baseline_pressure = bmp.getPressure();


}






float velocity = 0;
float currAccel;
float prevAccel = 0;
unsigned long prevTime = millis();
unsigned long currTime;







void loop() {
  sensors_event_t a, g, temp_mpu;
  mpu.getEvent(&a, &g, &temp_mpu);



  if (mpu_sayac == 0) {
    previous_acc_z = a.acceleration.z;
    mpu_sayac = 1;
  }


  currAccel = a.acceleration.z - previous_acc_z ;


  currTime = millis();
  velocity = (currAccel + prevAccel) / 2 * (currTime - prevTime) / 1000; //1000 added to get the same units
  prevAccel = currAccel;
  prevTime = currTime;
  if (velocity < 0) {
    velocity = velocity * -1;
  }

  if (velocity > 1) {
    is_launched = true;
  }

  if (is_launched && velocity < 0.1) {

    apogee_detected_velocity = true;

  }





  GPSDelay(500);

  unsigned long start;
  double lat_val, lng_val;
  uint8_t sat_val;
  bool loc_valid;
  int hour_val, minute_val, sec_val;


  loc_valid = gps.location.isValid();//gps lokasyonu var mı yok mu ona göre true false döndürür

    lat_val = gps.location.lat();//latitude alır
    lng_val = gps.location.lng();//longitude alır
    sat_val = gps.satellites.value();
    hour_val = gps.time.hour();
    minute_val = gps.time.minute();
    sec_val = gps.time.second();
    hour_val = hour_val + 3;
  






  unsigned long current_millis_buzzer = millis();
  if (current_millis_buzzer - previous_millis_buzzer >= interval_buzzer) {
    previous_millis_buzzer = current_millis_buzzer;
    if (buzzer_state == LOW) {
      buzzer_state = HIGH;
    } else {
      buzzer_state = LOW;
    }
    digitalWrite(buzzer, buzzer_state);
  }




  delay(50);


  float   temp = bmp.getTemperature();
  uint32_t    press = bmp.getPressure();

  first_alt = alt;
  delay(10);
  alt = bmp280_read_altitude(baseline_pressure, press);

  if (apogee_detected(first_alt, alt)) {
    unsigned long current_millis_deployment = millis();
    if (!dep_1_state) {
      if (current_millis_deployment - previous_millis_deployment >= interval_deployment) {
        previous_millis_deployment = current_millis_deployment;
        if (deployment_state == 0) {
          deployment_state = 255;
        } else {
          deployment_state = 0;
        }
        analogWrite(Dep_1, deployment_state);
        deployment_state == 0 ? dep_1_state = true : dep_1_state = false;
      }
    }
  }
  delay(10);


  if (alt < 550.0 && dep_1_state && !dep_2_state) {
    delay(20);
    analogWrite(Dep_2, 255);
    dep_2_state = true;
  } else {
    analogWrite(Dep_2, 0);
  }

  // Serial.println("alt:" + String(alt) + "\n");
  // Serial.println(dep_1_state == 0 ? "birinci ayrilma: - \n" : "birinci ayrilma: + \n");
  //Serial.println(dep_2_state == 0 ? "ikinci ayrilma: - \n"  : "ikinci ayrilma: + \n" );

  String ayrilma_0 = String(!dep_1_state ? "-" : "+");
  String ayrilma_1 = String(!dep_2_state ? "-" : "+");
  
  delay(10);

  paket = paket + String(hour_val);
  paket = paket + ":";
  paket = paket + String(minute_val);
  paket = paket + ":";
  paket = paket + String(sec_val);
  paket = paket + ",";
  paket = paket + ayrilma_0;
  paket = paket + ",";
  paket = paket + ayrilma_1;
  paket = paket + ",";
  paket = paket + String(alt);
  paket = paket + ",";
  paket = paket + String(g.gyro.x);
  paket = paket + ",";
  paket = paket + String(g.gyro.y);
  paket = paket + ",";
  paket = paket + String(g.gyro.z);
  paket = paket + ",";
  paket = paket + String(lat_val, 6);
  paket = paket + ",";
  paket = paket + String(lng_val, 6);
  paket = paket + ",";
  paket = paket + String(sat_val);
  paket = paket + ",";
  paket = paket + String(temp);
 paket = paket + "\n";
  

  delay(50);

  Serial.println(paket);




  delay(100);


}





static void GPSDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());

  } while (millis() - start < ms);
}
