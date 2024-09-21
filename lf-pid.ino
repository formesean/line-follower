#include <QTRSensors.h>
#include <SparkFun_TB6612.h>

// Motor Driver Properties
#define PWMA 5
#define AIN2 6
#define AIN1 7
#define BIN1 8
#define BIN2 9
#define PWMB 10
#define STBY 3

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// PID Controller Properties
QTRSensors qtr;
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
uint16_t position;

float Kp = 0.25; // 0.305
float Ki = 0.0029;
float Kd = 2; // 5.05

int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;

int centerPosition = 2000;
int minSpeed = -100;
int maxSpeed = 255;
int baseSpeed = 100;
// 127

// Button Properties
#define START_BUTTON 4

void setup()
{
  Serial.begin(9600);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A4, A3, A2, A1, A0}, SensorCount);

  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  calibrate();
}

void loop()
{
  bool start_button = digitalRead(START_BUTTON);

  while (start_button == LOW)
  {
    robot_control();
    delay(1);
  }
}

void calibrate()
{
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 25; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void robot_control()
{
  position = qtr.readLineBlack(sensorValues);

  if (position >= 3400)
  {
    motor1.drive(minSpeed);
    motor2.drive(maxSpeed);
  }
  else if (position <= 600)
  {
    motor1.drive(maxSpeed);
    motor2.drive(minSpeed);
  }
  else
  {
    error = centerPosition - position;

    P = error;
    I = constrain(I + error, minSpeed, maxSpeed);
    D = error - previousError;
    previousError = error;

    int motorSpeed = (Kp * P) + (Ki * I) + (Kd * D);

    int lsp = constrain(baseSpeed + motorSpeed, minSpeed, maxSpeed);
    int rsp = constrain(baseSpeed - motorSpeed, minSpeed, maxSpeed);

    motor1.drive(lsp);
    motor2.drive(rsp);
  }
}
