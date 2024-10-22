#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <VL53L0X.h>
#include <Servo.h>

// ======== SETTING UP I2C LCD ================================
const byte LCDAddress  = 0x27;
const byte LCDRows     = 16;
const byte LCDColumns  = 2;

LiquidCrystal_I2C lcd(LCDAddress , LCDRows, LCDColumns);
// ============================================================

// ======= SETTING UP VL53L0X DISTANCE SENSOR =================
#define GET_DISTANCE distanceSensor.readRangeContinuousMillimeters()
VL53L0X distanceSensor;

const int sensorTimeout = 500;
// ============================================================

// ======= SETTING UP SERVO ===================================
#define SERVOPIN 4

Servo servo;
// ============================================================

// ======= SETTING UP BLUETOOTH UART ==========================
#define BT_RX_PIN 3
#define BT_TX_PIN 2

SoftwareSerial BTSerial(BT_RX_PIN, BT_TX_PIN); 
// ============================================================

// ======= OTHER VARIABLES AND PROTOTYPES =====================
#define BUTTON_PIN 5

String inputString;
const int sensorThreshold = 300;

long startTime = 0;
long endTime = 0;
// ============================================================

void setup(){

  // LCD SETUP
  lcd.init();
  lcd.backlight();

  // SERVO SETUP
  servo.attach(SERVOPIN);
  servo.write(0);

  // BT UART SETUP
  BTSerial.begin(9600);

  // VL53L0X SETUP
  distanceSensor.setTimeout(sensorTimeout);
  
  while(!distanceSensor.init()){
    BTSerial.println("NOSENSOR");
  }

  distanceSensor.startContinuous();

  // OTHER SETUP
  pinMode(BUTTON_PIN, INPUT);

  BTSerial.println("WAITING");

  lcd.print("Waiting for");
  lcd.setCursor(0, 1);
  lcd.print("BT Connection...");

}

void loop(){

  bool button = digitalRead(BUTTON_PIN);

  if (BTSerial.available() || button){
  
    if (BTSerial.available()){
      inputString = BTSerial.readStringUntil('\n');
    }

    if (inputString == "GO" || button){

      BTSerial.println("WAITING");
      lcd.clear();
      lcd.print("Waiting for");
      lcd.setCursor(0, 1);
      lcd.print("Player");

      while(GET_DISTANCE > sensorThreshold){}

      startTime = millis();
      BTSerial.print("START ");
      BTSerial.println(startTime);

      lcd.clear();
      lcd.print("Player passed!");
      lcd.setCursor(0, 1);
      lcd.print("Recording time...");

      delay(2000);

      while(GET_DISTANCE < sensorThreshold){}
      while(GET_DISTANCE > sensorThreshold){
        if (digitalRead(BUTTON_PIN))
          break;
      }

      endTime = millis();
      BTSerial.print("STOP ");
      BTSerial.println(endTime);

      long actualTime = endTime - startTime;
      long minutes = actualTime / 60000;
      actualTime %= 60000;
      long seconds = actualTime / 1000;
      actualTime %= 1000;

      lcd.clear();
      lcd.print("Recorded Time:");
      lcd.setCursor(0,1);
 
      lcd.print(minutes);
      lcd.print(":");

      if(seconds < 10) {
        lcd.print("0"); 
      }
      lcd.print(seconds);
      lcd.print(":");

      if(actualTime < 100) {
        lcd.print("0");
      }
      if(actualTime < 10) {
        lcd.print("0");
      }
      lcd.print(actualTime);

      congratulateServo();
    }
  }
}

void congratulateServo(){

  for (int i = 0; i < 4; i++){
    servo.write(45);
    delay(500);
    servo.write(135);
    delay(500);
  }

  return;
}