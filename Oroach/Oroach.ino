#include <Servo.h>
#include <math.h>

#define SERVOMIN 180
#define SERVOMAX 550
#define HEAD 88
#define LEFTLEG 86
#define RIGHTLEG 94
#define MIDLEG 87
#define UART_RATE 9600

#define AVOIDANCE_SAT 16.0
#define PHOTO_SAT 25.0
#define AVOIDANCE_GAIN 0.5
#define PHOTO_GAIN 1
//Manual Sat
#define WALK_SAT 5
#define TURN_SAT 15

Servo headServo;
Servo rightLegServo;
Servo leftLegServo;
Servo midLegServo;


const uint8_t headPin = 3; // D3 head
const uint8_t leftLegPin = 5; // D5 Left leg
const uint8_t rightLegPin = 6; // D9 Right leg
const uint8_t midLegPin = 9; // D9 head

const uint8_t left_photoresistor_pin = A7;
const uint8_t right_photoresistor_pin = A6;
const uint8_t trigPin = 4;
const uint8_t echoPin = 2;

enum States {
  IDL,
  AUTONOMOUS, 
  SCAN, 
  MANUAL
  };
enum ManualCmds {
  PASS,
  FORWARD,
  BACK, 
  RIGHT, 
  LEFT,
  TOGGLE
  };
struct ControlStates {
  int walk_g = 3;
  //int walk_delay = 200;
  int stride_ang = 20;
  int height_ang = 25;
  float photo_g = 0.0;
  float avoidance_g = 0.0;
  float turn_g = 0.0;
  uint8_t n_stride = 0;
  uint8_t n_stride_cycle = 3;
  States state = IDL; 

};

int center;
int left;
int right;
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement
String inString = "";

ManualCmds serial_input = 0;
ControlStates cs;

void setup() {
  headServo.attach(headPin);
  rightLegServo.attach(rightLegPin);
  leftLegServo.attach(leftLegPin);
  midLegServo.attach(midLegPin);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(UART_RATE);
}
void scan() {
  int d_head = 800;
  headServo.write(HEAD);
  delay(200);
  //Serial.println("Center");
  center = get_ultrasonic_reading();
  delay(d_head);
  headServo.write(HEAD+45);
  //Serial.println("Left");
  delay(d_head);
  left = get_ultrasonic_reading();
  delay(d_head);
  headServo.write(HEAD-45);
  //Serial.println("Right");
  delay(d_head);
  right= get_ultrasonic_reading();
  delay(d_head);
  headServo.write(HEAD);
}
void stand_stance() {
  midLegServo.write(MIDLEG);
  rightLegServo.write(RIGHTLEG);
  leftLegServo.write(LEFTLEG);
  headServo.write(HEAD);
  delay(100);
}
int get_ultrasonic_reading(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  //Displays the distance on the Serial Monitor
  //Serial.print("Distance: ");
  //Serial.print(distance);
  //Serial.println(" cm");
  return distance;
}

void walk(int speed_g, int turn) {
  int delay_time = map(speed_g, 0, 5, 1000, 100);
  if (cs.walk_g != 0) {
    Serial.print("n Strides");
    Serial.println(cs.n_stride);
    midLegServo.write(MIDLEG - cs.height_ang);
    delay(delay_time);
    rightLegServo.write(RIGHTLEG + cs.stride_ang - turn);
    leftLegServo.write(LEFTLEG + cs.stride_ang + turn);
    delay(delay_time);
    midLegServo.write(MIDLEG + cs.height_ang);
    delay(delay_time);
    rightLegServo.write(RIGHTLEG - cs.stride_ang + turn);
    leftLegServo.write(LEFTLEG - cs.stride_ang - turn);
    delay(delay_time);
    cs.n_stride++;
  }
}

void get_light_diff() {
  
  float photores_right =25.0/173.0 * analogRead(right_photoresistor_pin);
  float photores_left = 25.0/231.0 * analogRead(left_photoresistor_pin);
  cs.photo_g = photores_right - photores_left;
}

void update_avoidance(){
  float right_turn = 0;
  float left_turn = 0;
  scan();
  if (left < 30){
    left_turn = AVOIDANCE_GAIN*(30 - left);    
  }
  if (right < 30){
    right_turn = -AVOIDANCE_GAIN*(30 - right);    
  }
  if (left_turn > abs(right_turn)){
    cs.avoidance_g = min(AVOIDANCE_SAT, left_turn);
  } else {
    cs.avoidance_g = max(-AVOIDANCE_SAT, right_turn);
  }
}

void update_turn_gains(){
  if (abs(cs.avoidance_g) > 15){
    cs.turn_g =  cs.avoidance_g;
  } else  {
    cs.turn_g = cs.avoidance_g + cs.photo_g;
  }
}

void state_machine(){
  switch (cs.state) {
    case IDL:
      //Serial.println("Idl");
      stand_stance();
      cs.state = AUTONOMOUS;
      cs.n_stride = 0;
      break;
    case SCAN:
      //Serial.println("Scan");
      update_avoidance();
      update_turn_gains();
      cs.state = AUTONOMOUS;
      cs.n_stride = 0;
      break; 
    case MANUAL:
      //Serial.println("Manual");
      manual_control();
      walk(cs.walk_g, cs.turn_g);
      break;
    case AUTONOMOUS:
      //Serial.println("Auto");
      get_light_diff();
      update_turn_gains();
      walk(cs.walk_g, cs.turn_g);
      //Serial.print("Front Scan");
      int forward_dis = get_ultrasonic_reading();
      //Serial.println(forward_dis);
      if (forward_dis < 15 || cs.n_stride_cycle <= cs.n_stride) {
        // GOTO Avoidance 
        cs.state = SCAN;
      }
      break; 

    default:
      Serial.println("Default");
    // statements
      break;
  }
}

void check_toggle(){
  if (serial_input == TOGGLE){
    serial_input = 0;
    if (cs.state !=MANUAL){
      cs.state = MANUAL;
      cs.walk_g = 0;
      cs.turn_g = 0;
    }
    else{
      cs.state = IDL;
      cs.walk_g = 3;
    }
  }
}
void manual_control(){
// 0 No new input
// 1 forward
// 2 back
// 3 right
// 4 left
  if (cs.state == MANUAL){
    switch (serial_input) {
      case FORWARD:
        cs.walk_g+=1;
        cs.walk_g = min(cs.walk_g, WALK_SAT);
        serial_input = 0;
        break;
      case BACK:
        cs.walk_g-=1;
        cs.walk_g = max(cs.walk_g, 0);
        serial_input = 0;
        break;
      case RIGHT:
        cs.turn_g+=10;
        cs.turn_g = min(cs.turn_g, TURN_SAT);
        serial_input = 0;
        break;
      case LEFT:
        cs.turn_g-=10;
        cs.turn_g = min(cs.turn_g, -TURN_SAT);
        serial_input = 0;
        break;
      default:
        // if nothing else matches, do the default
        // default is optional
        break;
    }
  } else {
    cs.state = IDL;
  }
}
void loop() {
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString = (char)inChar;
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n') {
      //Serial.print("Value:");
      //Serial.println(inString.toInt());
      serial_input = inString.toInt();
    }
  }
  delay(10);
  check_toggle();
  state_machine();
  delay(10);
  Serial.print("Walk gain: ");
  Serial.print(cs.walk_g);
  Serial.print(" Turn gain: ");
  Serial.println(cs.turn_g);
}
