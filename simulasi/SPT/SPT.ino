#include <Servo.h>
#include <Math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define RAD_TO_DEG 57.295779513082320876798154814105

Adafruit_MPU6050 mpu;
Servo servoYaw;  
Servo servoPitch;
int roll = 0;
int pitch = 0;

void setup(){
  servoYaw.attach(D8);
  servoPitch.attach(D7);

  Serial.begin(115200);
  if (!mpu.begin()) {
    Serial.println("Sensor init failed");
    while (1)
      yield();
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

// void loop(){
  //baca acc dan gyro
  // sensors_event_t a, g, temp;
  // mpu.getEvent(&a, &g, &temp);

  // //tampilkan acc
  // Serial.print("Acceleration X: ");
  // Serial.print(a.acceleration.x);
  // Serial.print(", Y: ");  
  // Serial.print(a.acceleration.y);
  // Serial.print(", Z: ");
  // Serial.print(a.acceleration.z);
  // Serial.println(" m/s^2");
  // //tampilkan gyro
  // Serial.print("Rotation X: ");
  // Serial.print((g.gyro.x + 0.02)* RAD_TO_DEG );
  // Serial.print(", Y: ");
  // Serial.print((g.gyro.y - 0.01)* RAD_TO_DEG);
  // Serial.print(", Z: ");
  // Serial.print((g.gyro.z + 0.01)* RAD_TO_DEG);
  // Serial.println(" rad/s");

//   Serial.println("");
//   delay(1000);
// }

void loop()
{  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float accelX = a.acceleration.x;
  float accelY = a.acceleration.y;
  float accelZ = a.acceleration.z;
  float gyroX = g.gyro.x;
  float gyroY = g.gyro.y;
  float gyroZ = g.gyro.z;

  Serial.println("Accel: " + String(accelX) + ", " + String(accelY) + ", " + String(accelZ) + " g");
  Serial.println("Gyro: " + String(gyroX) + ", " + String(gyroY) + ", " + String(gyroZ) + " dps");

//Euler angle from accel

 
   pitch = atan2 (accelY ,( sqrt ((accelX * accelX) + (accelZ * accelZ))));
   roll = atan2(-accelX ,( sqrt((accelY * accelY) + (accelZ * accelZ))));

   // yaw from mag

  //  float Yh = (magY * cos(roll)) - (magZ * sin(roll));
  //  float Xh = (magX * cos(pitch))+(magY * sin(roll)*sin(pitch)) + (magZ * cos(roll) * sin(pitch));

  //  yaw =  atan2(Yh, Xh);


    roll = roll*57.3;
    pitch = pitch*57.3;
    // yaw = yaw*57.3;
   
  Serial.println("pitch"  + String( pitch) );
  Serial.println("roll" + String( roll));
  // Serial.println("yaw" + String( yaw ));
  delay (1000);

}