#include <esp_now.h>
#include <WiFi.h>
#include <vector>
#include <ESP32Servo.h>   // Added for servo

#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define FORWARD_LEFT 5
#define FORWARD_RIGHT 6
#define BACKWARD_LEFT 7
#define BACKWARD_RIGHT 8
#define TURN_LEFT 9
#define TURN_RIGHT 10
#define STOP 0

#define BACK_RIGHT_MOTOR 0
#define BACK_LEFT_MOTOR 1
#define FRONT_RIGHT_MOTOR 2
#define FRONT_LEFT_MOTOR 3

// ------------------------ MOTOR PINS ------------------------
struct MOTOR_PINS
{
  int pinIN1;
  int pinIN2;
  int pinEn; 
  int pwmSpeedChannel;
};

std::vector<MOTOR_PINS> motorPins = 
{
  {16, 17, 22, 4},  // BACK_RIGHT_MOTOR
  {18, 19, 23, 5},  // BACK_LEFT_MOTOR
  {26, 27, 14, 6},  // FRONT_RIGHT_MOTOR
  {33, 25, 32, 7},  // FRONT_LEFT_MOTOR
};

#define MAX_MOTOR_SPEED 50
const int PWMFreq = 1000;      // 1 KHz
const int PWMResolution = 8;   // 8-bit PWM

#define SIGNAL_TIMEOUT 1000  // in milliseconds
unsigned long lastRecvTime = 0;

// ------------------------ PACKET DATA ------------------------
typedef struct PacketData
{
  byte xAxisValue;
  byte yAxisValue;
  byte zAxisValue;
  byte logic;     // Added new field for logic
} PacketData;

PacketData receiverData;

// ------------------------ SERVO SETUP ------------------------
Servo myServo;               
#define SERVO_PIN 21          // Servo connected here
int lastLogicState = 0;       // Track last logic state

// ------------------------ CALLBACK ------------------------
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) 
{
  if (len == 0) return;

  memcpy(&receiverData, incomingData, sizeof(receiverData));

  // Print received values
  Serial.printf("values %d  %d  %d  Logic: %d\n", 
                receiverData.xAxisValue, 
                receiverData.yAxisValue, 
                receiverData.zAxisValue,
                receiverData.logic);

  // ----------------- Servo Logic -----------------

  if (receiverData.logic == 1 && lastLogicState == 0) {
    myServo.write(0);   // move once to 90° when logic changes 0 → 1
  }
  if (receiverData.logic == 0 && lastLogicState == 1) {
    myServo.write(90);    // return once to 0° when logic changes 1 → 0
  }
  lastLogicState = receiverData.logic;

  // ----------------- Movement Logic -----------------
  if (receiverData.xAxisValue < 75 && receiverData.yAxisValue < 75)
    processCarMovement(FORWARD_LEFT);    
  else if (receiverData.xAxisValue > 175 && receiverData.yAxisValue < 75)
    processCarMovement(FORWARD_RIGHT);    
  else if (receiverData.xAxisValue < 75 && receiverData.yAxisValue > 175)
    processCarMovement(BACKWARD_LEFT);    
  else if (receiverData.xAxisValue > 175 && receiverData.yAxisValue > 175)
    processCarMovement(BACKWARD_RIGHT);  
  else if (receiverData.zAxisValue > 175)
    processCarMovement(TURN_RIGHT);
  else if (receiverData.zAxisValue < 75)
    processCarMovement(TURN_LEFT);
  else if (receiverData.yAxisValue < 75)
    processCarMovement(FORWARD);  
  else if (receiverData.yAxisValue > 175)
    processCarMovement(BACKWARD);     
  else if (receiverData.xAxisValue > 175)
    processCarMovement(RIGHT);   
  else if (receiverData.xAxisValue < 75)
    processCarMovement(LEFT);    
  else
    processCarMovement(STOP);     

  lastRecvTime = millis();   
}

// ------------------------ MOTOR CONTROL ------------------------
void processCarMovement(int inputValue)
{
  switch(inputValue)
  {
    case FORWARD:
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);                  
      break;

    case BACKWARD:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);   
      break;

    case LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);   
      break;

    case RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);  
      break;

    case FORWARD_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, 0);
      rotateMotor(FRONT_LEFT_MOTOR, 0);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);  
      break;

    case FORWARD_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, 0);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, 0);  
      break;

    case BACKWARD_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, 0);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, 0);   
      break;

    case BACKWARD_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, 0);
      rotateMotor(FRONT_LEFT_MOTOR, 0);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);   
      break;

    case TURN_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);  
      break;

    case TURN_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);   
      break;

    case STOP:
    default:
      rotateMotor(FRONT_RIGHT_MOTOR, 0);
      rotateMotor(BACK_RIGHT_MOTOR, 0);
      rotateMotor(FRONT_LEFT_MOTOR, 0);
      rotateMotor(BACK_LEFT_MOTOR, 0);    
      break;
  }
}

// ------------------------ ROTATE MOTOR ------------------------
void rotateMotor(int motorNumber, int motorSpeed)
{
  if (motorSpeed < 0)
  {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, HIGH);    
  }
  else if (motorSpeed > 0)
  {
    digitalWrite(motorPins[motorNumber].pinIN1, HIGH);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);       
  }
  else
  {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);      
  }
  
  ledcWrite(motorPins[motorNumber].pinEn, abs(motorSpeed));
}

// ------------------------ SETUP PINS ------------------------
void setUpPinModes()
{
  for (int i = 0; i < motorPins.size(); i++)
  {
    pinMode(motorPins[i].pinIN1, OUTPUT);
    pinMode(motorPins[i].pinIN2, OUTPUT);

    ledcAttach(motorPins[i].pinEn, PWMFreq, PWMResolution);

    rotateMotor(i, 0);
  }
}

// ------------------------ SETUP ------------------------
void setup() 
{
     // Read logic signal
  myServo.attach(SERVO_PIN);
  Serial.begin(115200);

  setUpPinModes();

  // Setup servo
  //myServo.attach(SERVO_PIN, 500, 2400);
  myServo.write(0);  // start at 0°

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}
 
// ------------------------ LOOP ------------------------
void loop() 
{
  if (millis() - lastRecvTime > SIGNAL_TIMEOUT) 
  {
    processCarMovement(STOP); 
  }
}