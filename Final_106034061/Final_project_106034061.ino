#include <Wire.h>
#include <SoftwareSerial.h>
#include <Arduino_FreeRTOS.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MotorShield.h>
#include <queue.h>
#include "PS2X_lib.h"

#define carSpeed 200
#define DirectionButton 0
#define JoyStick 1
#define Automation 2
PS2X ps2x;
LiquidCrystal_I2C lcd(0x3F, 16, 2);

int Control_Method = 0;

int error;
int leftSpeed = 0;  //左馬達速度
int rightSpeed = 0;  //右馬達速度
bool Is_On_Obstacle = false;
int Obstacle_limit = 60;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(4);

int pr_min = 10;
int LightPin = 2;
int Time_limit = 200;
int Previous_time;
long duration, distance;
int trigPin = 8, echoPin = 7;
int redPin = 6, greenPin = 5, bluePin = 3;


TaskHandle_t ControlHandle;
QueueHandle_t Global_Queue_Handle = 0; //Global Handler

void setup() {
  Previous_time = millis();
  pinMode(echoPin, INPUT);
  pinMode(A0, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(LightPin, OUTPUT);
  
  digitalWrite(LightPin, LOW);
  
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Start");

  Serial.begin(9600);
  do {
    error = ps2x.config_gamepad(13, 11, 10, 12, true, true);  
    if (error == 0) {Serial.print("Gamepad found!");break;}
    else { delay(100); } 
  }while(1);

  Serial.println("DC Motor test!");
  AFMS.begin(50);
  myMotor->setSpeed(carSpeed);
  myMotor2->setSpeed(carSpeed);

  Global_Queue_Handle = xQueueCreate(3, sizeof(int));
  xTaskCreate(ControlTask, "ControlTask", 128, NULL, 1, &ControlHandle);

}

void forward() {
  Photoresistor();
  UltrasonicDetection();
//  delay(10);
  if(!Is_On_Obstacle){
  analogWrite(redPin, 0);
  analogWrite(greenPin, 255);
  analogWrite(bluePin, 0);
  myMotor->run(FORWARD);
  myMotor2->run(FORWARD);
  Serial.println("FORWARD");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Forward");
  }
}
void backward() {
  analogWrite(redPin, 0);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 255);
  myMotor->run(BACKWARD);
  myMotor2->run(BACKWARD);
  Serial.println("BACKWARD");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Backward");
}
void stopcar() {
  analogWrite(redPin, 255);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 0);
  if (Control_Method == JoyStick) {
    myMotor->setSpeed(leftSpeed);
    myMotor2->setSpeed(rightSpeed);
  }
  myMotor->run(RELEASE);
  myMotor2->run(RELEASE);
  Serial.println("STOP");
  //  lcd.clear();
  //  lcd.print("Stop");
}
void turnLeft() {
  analogWrite(redPin, 0);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 255);
  myMotor->run(FORWARD);
  myMotor2->run(RELEASE);
  Serial.println("LEFT");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Turn Left");
}
void turnRight () {
  analogWrite(redPin, 0);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 255);
  myMotor->run(RELEASE);
  myMotor2->run(FORWARD );
  Serial.println("RIGHT");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Turn Right");
}
bool UltrasonicDetection() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2); digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  Serial.print("Distance:"); Serial.print(distance);
  if (distance < Obstacle_limit) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Obstacle:");
    Serial.println("Obstacle");
    Is_On_Obstacle = true;
  }
  else {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("No Obstacle:");
    Serial.println("No Obstacle");
    Is_On_Obstacle = false;
  }
  delay(10);
  return Is_On_Obstacle;
}
void Photoresistor() {
  int pr = analogRead(A0);
  Serial.println(pr);
  if (pr < pr_min) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Dark");
    digitalWrite(LightPin, HIGH);
  }
  else {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Light");
    digitalWrite(LightPin, LOW);
  }
}
void DirectionButtonControl(){
//    Serial.println("DirectionButton");
//    ps2x.read_gamepad(false, 0);
    myMotor->setSpeed(carSpeed);
    myMotor2->setSpeed(carSpeed);
    if (ps2x.Button(PSB_START)) Serial.println("Start is being held");
    if (ps2x.Button(PSB_SELECT)) Serial.println("Select is being held");
    
    if (ps2x.Button(PSB_PAD_UP)) {forward();}
    else if (ps2x.Button(PSB_PAD_RIGHT)) { turnLeft();}
    else if (ps2x.Button(PSB_PAD_LEFT)) { turnRight();}
    else if (ps2x.Button(PSB_PAD_DOWN)) { backward();}
    else { stopcar();}
    
    if (ps2x.NewButtonState(PSB_L3)) Serial.println("L3 pressed");
    if (ps2x.NewButtonState(PSB_R3)) Serial.println("R3 pressed");
    if (ps2x.NewButtonState(PSB_L2)) Serial.println("L2 pressed");
    if (ps2x.NewButtonState(PSB_R2)) Serial.println("R2 pressed");
    
    if (ps2x.NewButtonState(PSB_TRIANGLE)){
      if(Control_Method == DirectionButton){ 
        lcd.setCursor(0,1);
        lcd.print("JoyStick");
        Control_Method = JoyStick;
      }
      else if(Control_Method == JoyStick){ 
        lcd.setCursor(0,1);
        lcd.print("Automation");
        Control_Method = Automation;
      }
      else if(Control_Method == Automation){ 
        lcd.setCursor(0,1);
        lcd.print("Button");
        Control_Method = DirectionButton;
      }
    }
    if (ps2x.NewButtonState(PSB_CIRCLE)) { Photoresistor(); }
    if (ps2x.NewButtonState(PSB_CROSS)){ UltrasonicDetection(); }
    if (ps2x.NewButtonState(PSB_SQUARE)) Serial.println("Square pressed");
    if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) {
      Serial.print("Stick Values:");
    }
//    JoyStickControl();
//    delay(10);
}
void JoyStickControl(){
    Serial.println("JoyStick");
    //    ps2x.read_gamepad(false, 0);
    int joyLeft = ps2x.Analog(PSS_LY);
    int joyRight = ps2x.Analog(PSS_RX);
    if (joyLeft < 125) {
      if (joyRight > 130) {
        int turnSpeed = map(joyRight, 131, 255, 0, 255);
        rightSpeed = map(joyLeft, 124, 0, 0, 255) - turnSpeed;
        leftSpeed = map(joyLeft, 124, 0, 0, 255);
      }
      else if (joyRight < 125) {
        int turnSpeed = map(joyRight, 124, 0, 0, 255);
        rightSpeed = map(joyLeft, 124, 0, 0, 255);
        leftSpeed = map(joyLeft, 124, 0, 0, 255) - turnSpeed;
      }
      else {
        rightSpeed = map(joyLeft, 124, 0, 0, 255);
        leftSpeed = map(joyLeft, 124, 0, 0, 255);
      }
      myMotor->setSpeed(leftSpeed);
      myMotor2->setSpeed(rightSpeed);
      forward();
    }
    else if (joyLeft > 130) {
      if (joyRight > 130) {
        int turnSpeed = map(joyRight, 131, 255, 0, 255);
        rightSpeed = map(joyLeft, 131, 255, 0, 255) - turnSpeed;
        leftSpeed = map(joyLeft, 131, 255, 0, 255);
      }
      else if (joyRight < 125) {
        int turnSpeed = map(joyRight, 124, 0, 0, 255);
        rightSpeed = map(joyLeft, 131, 255, 0, 255);
        leftSpeed = map(joyLeft, 131, 255, 0, 255) - turnSpeed;
      }
      else {
        rightSpeed = map(joyLeft, 131, 255, 0, 255);
        leftSpeed = map(joyLeft, 131, 255, 0, 255);
      }
      myMotor->setSpeed(leftSpeed);
      myMotor2->setSpeed(rightSpeed);
      backward();
    } else {
      leftSpeed = 0;
      rightSpeed = 0;
      myMotor->setSpeed(leftSpeed);
      myMotor2->setSpeed(rightSpeed);
      stopcar();
    }
    if (ps2x.NewButtonState(PSB_TRIANGLE)){
      if(Control_Method == DirectionButton){ 
        lcd.setCursor(0,1);
        lcd.print("JoyStick");
        Control_Method = JoyStick;
      }
      else if(Control_Method == JoyStick){ 
        lcd.setCursor(0,1);
        lcd.print("Automation");
        Control_Method = Automation;
      }
      else if(Control_Method == Automation){ 
        lcd.setCursor(0,1);
        lcd.print("Button");
        Control_Method = DirectionButton;
      }
    }
    if (ps2x.NewButtonState(PSB_CIRCLE)) { Photoresistor(); }
    if (ps2x.NewButtonState(PSB_CROSS)){ UltrasonicDetection(); }
}
void AutomationControl(){
    myMotor->setSpeed(carSpeed);
    myMotor2->setSpeed(carSpeed);
    Serial.println("Automation");
    if (ps2x.NewButtonState(PSB_TRIANGLE)){
      if(Control_Method == DirectionButton){ 
        lcd.setCursor(0,1);
        lcd.print("JoyStick");
        Control_Method = JoyStick;
      }
      else if(Control_Method == JoyStick){ 
        lcd.setCursor(0,1);
        lcd.print("Automation");
        Control_Method = Automation;
      }
      else if(Control_Method == Automation){ 
        lcd.setCursor(0,1);
        lcd.print("Button");
        Control_Method = DirectionButton;
      }
    }
    if (ps2x.NewButtonState(PSB_CIRCLE)) { Photoresistor(); }
    if (ps2x.NewButtonState(PSB_CROSS)){ UltrasonicDetection(); }
  if(millis() - Previous_time > Time_limit){
      Previous_time = millis();
      UltrasonicDetection();
    }
//    UltrasonicDetection();
        if(Is_On_Obstacle){
//          stopcar();
          int seed = random(0,2);
          if(seed == 1){
            turnLeft();   
          }
          else{
            turnRight();
          }
          delay(1000);
        }
        else{
          forward();
        }
  
}
void loop() {
//  if(millis() - Previous_time > Time_limit){
//      Previous_time = millis();
////      UltrasonicDetection();
//    }
//  ps2x.read_gamepad(false, 0);
//  if (Control_Method == DirectionButton ) {
//    DirectionButtonControl();
//  }
//  else if (Control_Method == JoyStick) {
//    JoyStickControl();
//  }
//  else if (Control_Method == Automation) {
//    AutomationControl();
//  }
//    delay(50);

}
void ControlTask(void *pvParameters)  {
  (void) pvParameters;
  for (;;) {
    ps2x.read_gamepad(false, 0);
    if (Control_Method == DirectionButton ) {
      DirectionButtonControl();
    }
    else if (Control_Method == JoyStick) {
      JoyStickControl();
    }
    else if (Control_Method == Automation) {
      AutomationControl();
    }
    vTaskDelay(10); 
  }
}
