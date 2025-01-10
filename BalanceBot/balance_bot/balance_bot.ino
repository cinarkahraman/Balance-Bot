#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>
#include <Wire.h>

#define Motor_A1 9  // IN1
#define Motor_A2 6  // IN2
#define Motor_C1 5  // IN3
#define Motor_C2 3  // IN4
#define MotorENA 10 // ENA (Motor 1 hızı)
#define MotorENB 11 // ENB (Motor 2 hızı)

// MPU6050 Sensor
MPU6050 mpu;
bool dmpReady = false;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

// PID variables
double setpoint = 182.00;
double input, output;
double Kp = 10.0;
double Ki = 0.1;
double Kd = 3.0;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// MPU6050 Interrupt flag
volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  pinMode(Motor_A1, OUTPUT);
  pinMode(Motor_A2, OUTPUT);
  pinMode(Motor_C1, OUTPUT);
  pinMode(Motor_C2, OUTPUT);
  pinMode(MotorENA, OUTPUT);
  pinMode(MotorENB, OUTPUT);

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  Serial.println("MPU6050 connection successful");

  mpu.dmpInitialize();
  mpu.setXAccelOffset(-1767);
  mpu.setYAccelOffset(1037);
  mpu.setZAccelOffset(827);
  mpu.setXGyroOffset(99);
  mpu.setYGyroOffset(-22);
  mpu.setZGyroOffset(24);

  mpu.setDMPEnabled(true);
  attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-150, 150); 
}

void loop() {
  if (!dmpReady) return;

  if (mpuInterrupt) {
    mpuInterrupt = false;

    // FIFO kontrolü
    if (mpu.getFIFOCount() < packetSize) return; 
    if (mpu.getFIFOCount() >= 1024) {
      mpu.resetFIFO(); // FIFO'yu sıfırla
      Serial.println("FIFO overflow!"); // FIFO taşması
      return;
    }

    // FIFO'dan veri al
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    input = ypr[1] * 180 / M_PI + 180;  // YPR'yi açıya dönüştür
    double hata = setpoint - input;     // Hata hesapla

    pid.Compute(); 

    // Durum analizi
    String durum;
    if (abs(hata) < 5) { // Daha geniş tolerans
      durum = "Dengeleme";
      stop();  // Durum dengede ise motorları durdur
    } else if (hata > 5) {
      durum = "Geri";
      geri(output);  // PID'e göre geri hareket et
    } else {
      durum = "İleri";
      ileri(output);  // PID'e göre ileri hareket et
    }

    // Seri monitör çıktısı
    Serial.print("Açı: "); Serial.print(input);
    Serial.print(" | Hata: "); Serial.print(hata);
    Serial.print(" | PID Çıkışı: "); Serial.print(output);
    Serial.print(" | Durum: "); Serial.println(durum);
  }
}

void ileri(int hiz) {
  hiz = constrain(abs(hiz), 0, 255); // Negatif değerleri pozitif yap
  analogWrite(MotorENA, hiz);
  analogWrite(MotorENB, hiz);
  digitalWrite(Motor_A1, HIGH);
  digitalWrite(Motor_A2, LOW);
  digitalWrite(Motor_C1, HIGH);
  digitalWrite(Motor_C2, LOW);
}

void geri(int hiz) {
  hiz = constrain(abs(hiz), 0, 255); // Negatif değerleri pozitif yap
  analogWrite(MotorENA, hiz);
  analogWrite(MotorENB, hiz);
  digitalWrite(Motor_A1, LOW);
  digitalWrite(Motor_A2, HIGH);
  digitalWrite(Motor_C1, LOW);
  digitalWrite(Motor_C2, HIGH);
}

void stop() {
  analogWrite(MotorENA, 10); // Minimum hız
  analogWrite(MotorENB, 10); // Minimum hız
  digitalWrite(Motor_A1, LOW);
  digitalWrite(Motor_A2, LOW);
  digitalWrite(Motor_C1, LOW);
  digitalWrite(Motor_C2, LOW);
}
