#include "I2Cdev.h"
#include "Servo.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


// bagian IoT
#include <ESP8266WiFi.h>
const char* host = "gimbal.galium.ga";
const int httpPort = 80;
const char* ssid     = "apoci";
const char* password = "apoci123";
long last_millis_wifi;
String wifi="OFF";
long last_millis;
WiFiClient client;


//bagian kontrol 
MPU6050 mpu;

Servo servo1; //servo bagian bawah (roll)   
Servo servo2; //servo bagian atas (pitch)

//keperluan kalibrasi
#define ERROR_MINIMAL 0.001
#define WAKTU_CEK 1000 //dalam milisekon

long int tSekarang;
long int tSebelum;
long int dT;
double yawSekarang;
double yawSebelum;
double dYaw;
double pitchSekarang;
double pitchSebelum;
double dPitch;
double rollSekarang;
double rollSebelum;
double dRoll;
double bufferDataKalibrasi[10];
char counterKalibrasi;

double dataYaw;
double dataPitch;
double dataRoll;
char dataYawKirim[4];
char statusKalibrasi = 0;
double offsetSudutYaw;
double offsetSudutPitch;
double offsetSudutRoll;

#define INTERRUPT_PIN D8  

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void ICACHE_RAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    Serial.begin(115200);
    Serial.print("void setup initialized");

    servo1.attach(D6);
    servo2.attach(D5);

    servo1.write(0);          //ATUR POSISI AWAL SERVO
    servo2.write(0);

    delay(1000);

    // konek wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting.....");
  Serial.print(ssid);

  Serial.print("SUCCESS!");
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // nilai offset hasil kalibrasi
    mpu.setXGyroOffset(45);
    mpu.setYGyroOffset(-27);
    mpu.setZGyroOffset(18);
    mpu.setXAccelOffset(-1690);
    mpu.setYAccelOffset(1157);
    mpu.setZAccelOffset(2742); 
    
    if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(7);  //Kalibrasi Pertama kali
      mpu.CalibrateGyro(7);
      mpu.PrintActiveOffsets();
        
      mpu.setDMPEnabled(true);
        
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
       
      dmpReady = true;
       
      packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else{ 
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
}

// untuk kalibrasi awal
// void loop(){

// }

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    if(statusKalibrasi == 0){  //proses kalibrasi
      dataYaw = ypr[0] * 180/M_PI;
      dataPitch= ypr[1] * 180/M_PI;
      dataRoll= ypr[2] * 180/M_PI;
      tSekarang = millis();
      dT = tSekarang - tSebelum;
      if(dT < WAKTU_CEK){
        yawSekarang=ypr[0];
        pitchSekarang=ypr[1];
        rollSekarang=ypr[2];
        }
      else{           
        dYaw = abs(yawSekarang - yawSebelum);
        yawSebelum = yawSekarang;
        dRoll = abs(rollSekarang - rollSebelum);
        rollSebelum = rollSekarang;
        dPitch = abs(pitchSekarang - pitchSebelum);
        pitchSebelum = pitchSekarang;
        if(dYaw<ERROR_MINIMAL || dRoll<ERROR_MINIMAL || dPitch<ERROR_MINIMAL ){
          counterKalibrasi++ ;
          }
        else counterKalibrasi = 0;
        if (counterKalibrasi>6){
            statusKalibrasi = 1;
            offsetSudutYaw = yawSekarang;
            offsetSudutPitch = pitchSekarang;
            offsetSudutRoll = rollSekarang;
          }
        tSebelum = tSekarang;
      }   
    }  
    else{
      dataYaw = (ypr[0]-offsetSudutYaw) * 180/M_PI;
      dataPitch = (ypr[1]-offsetSudutPitch) * 180/M_PI;
      dataRoll = (ypr[2]-offsetSudutRoll) * 180/M_PI;


      servo1.write(dataRoll);
      servo2.write(dataPitch);
    }
    // kirim data setiap 5 detik
    if ((millis()-last_millis)>5000){
      last_millis=millis();
      String protokol = "/update.php?sudutroll="+String(dataRoll)+
      "&sudutpitch="+String(dataPitch);
      Serial.println( "Send Data To Server: Data Roll = "+String(dataRoll)+
      " Data Pitch = "+String(dataPitch));
  }
  
    }
}


String send(String url) {
  if (!client.connect(host, httpPort)) {
    return "Error Request Confirm";
  }
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "User-Agent: BuildFailureDetectorESP8266\r\n" +
               "Connection: close\r\n\r\n");
  
  String body = "";
   long last_millis_x=millis();
  long next_millis_x=millis();
  while (client.connected()) {
      if ((millis()-last_millis_x)>10000){
        goto next3;
      }
      next_millis_x=millis();
    while (client.available()) {
       if ((millis()-next_millis_x)>10000){
        goto next3;
      }
      char c = client.read();
      body += String(c);
    }
  }
  next3:
  client.stop();
  int a = body.indexOf("~");
  int b = body.indexOf("!");
  int c = body.indexOf("@");
  
  String data_dataRoll =  body.substring(a+1,b);
  String data_dataPitch =  body.substring(b+1,c);
  
  if (data_dataRoll!=""&&data_dataPitch!=""){
      //Serial.println(body);
      Serial.println("SP="+data_dataRoll+" "+data_dataPitch);
      dataRoll = data_dataRoll.toInt();
      dataPitch = data_dataPitch.toInt();
      body="OK";
  }else{
    body ="ERROR";
  }
   
  return body;
}
