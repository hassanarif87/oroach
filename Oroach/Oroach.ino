#include <Servo.h>
#include <math.h>

#define SERVOMIN 180
#define SERVOMAX 550
#define HEAD 88
#define LEFTLEG 86
#define RIGHTLEG 94
#define MIDLEG 87
#define UART_RATE 9600
//Auto
#define AVOIDANCE_SAT 20.0
#define PHOTO_SAT 20.0
#define AVOIDANCE_GAIN 0.5
#define PHOTO_GAIN 1.5
//Manual
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
  uint8_t walk_g = 3;
  uint8_t stride_ang = 20;
  uint8_t height_ang = 25;
  float photo_g = 0.0;
  float avoidance_g = 0.0;
  float turn_g = 0.0;
  uint8_t n_stride = 0;
  uint8_t n_stride_cycle = 3;
  int8_t head_cmd = 0;
  int8_t rightLeg_cmd= 0;
  int8_t leftLeg_cmd = 0;
  int8_t midLeg_cmd= 0;
  uint8_t walk_seq = 0;
  States state = IDL; 
};

struct ScanStates {
  const int8_t seq[3] = {45, -45, 0};   //{"Left", Right", "Center"}
  int distance[3];
  uint8_t head_seq = 0;
  const uint8_t seq_n = 3;

};
//int center;
//int left;
//int right;
//long duration; // variable for the duration of sound wave travel
//int distance; // variable for the distance measurement
String inString = "";

ManualCmds serial_input = 0;
ControlStates ctrl_s;
ScanStates scan_s;
void setup() {
  headServo.attach(headPin);
  rightLegServo.attach(rightLegPin);
  leftLegServo.attach(leftLegPin);
  midLegServo.attach(midLegPin);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(UART_RATE);
}


void update_servos(){
  headServo.write(HEAD + ctrl_s.head_cmd);
  rightLegServo.write(RIGHTLEG + ctrl_s.rightLeg_cmd);
  leftLegServo.write(LEFTLEG + ctrl_s.leftLeg_cmd);
  midLegServo.write(MIDLEG + ctrl_s.midLeg_cmd);
}
void scan() {
  const int d_head = 800;
  while( scan_s.head_seq < scan_s.seq_n ){
    ctrl_s.head_cmd = scan_s.seq[scan_s.head_seq]; //+45
    update_servos();
    delay(d_head);
    scan_s.distance[scan_s.head_seq] = get_ultrasonic_reading();
    delay(d_head/2);
    scan_s.head_seq ++;
  }
}
void stand_stance() {
  ctrl_s.rightLeg_cmd= 0;
  ctrl_s.leftLeg_cmd = 0;
  ctrl_s.midLeg_cmd= 0;
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
  // duration * Speed of sound wave divided by 2 (go and back)
  return  pulseIn(echoPin, HIGH) * 0.034 / 2;
}

void walk(int speed_g, int turn) {
  int delay_time = map(speed_g, 0, 5, 800, 200);
  if (ctrl_s.walk_g != 0) {
    switch (ctrl_s.walk_seq) {
      case 0:
        ctrl_s.midLeg_cmd = - ctrl_s.height_ang;
      case 1:
        ctrl_s.rightLeg_cmd = ctrl_s.stride_ang - turn;
        ctrl_s.midLeg_cmd = ctrl_s.stride_ang + turn;
      case 2:
        ctrl_s.midLeg_cmd = ctrl_s.height_ang;
      case 3:
        ctrl_s.rightLeg_cmd= - ctrl_s.stride_ang + turn;
        ctrl_s.midLeg_cmd = - ctrl_s.stride_ang - turn;
        ctrl_s.walk_seq = 0;
        ctrl_s.n_stride++;
    }
  ctrl_s.walk_seq ++;
  update_servos();
  delay(delay_time);
  }
}

void get_light_diff() {
  
  float photores_right =25.0/173.0 * analogRead(right_photoresistor_pin);
  float photores_left = 25.0/231.0 * analogRead(left_photoresistor_pin);
  ctrl_s.photo_g = photores_right - photores_left;
}

void update_avoidance(){
  float right_turn = 0;
  float left_turn = 0;
  scan();
  if (scan_s.distance[0] < 30){
    left_turn = AVOIDANCE_GAIN*(30 - scan_s.distance[0]);    
  }
  if (scan_s.distance[2] < 30){
    right_turn = -AVOIDANCE_GAIN*(30 - scan_s.distance[1]);    
  }
  if (left_turn > abs(right_turn)){
    ctrl_s.avoidance_g = min(AVOIDANCE_SAT, left_turn);
  } else {
    ctrl_s.avoidance_g = max(-AVOIDANCE_SAT, right_turn);
  }
}

void update_turn_gains(){
  if (abs(ctrl_s.avoidance_g) > 15){
    ctrl_s.turn_g =  ctrl_s.avoidance_g;
  } else  {
    ctrl_s.turn_g = ctrl_s.avoidance_g + ctrl_s.photo_g;
  }
}

void state_machine(){
  switch (ctrl_s.state) {
    case IDL:
      //Serial.println("Idl");
      stand_stance();
      update_servos();
      ctrl_s.state = AUTONOMOUS;
      ctrl_s.n_stride = 0;
      break;
    case SCAN:
      //Serial.println("Scan");
      update_avoidance();
      update_turn_gains();
      ctrl_s.state = AUTONOMOUS;
      ctrl_s.n_stride = 0;
      break; 
    case MANUAL:
      //Serial.println("Manual");
      manual_control();
      walk(ctrl_s.walk_g, ctrl_s.turn_g);
      break;
    case AUTONOMOUS:
      //Serial.println("Auto");
      get_light_diff();
      update_turn_gains();
      walk(ctrl_s.walk_g, ctrl_s.turn_g);
      //Serial.print("Front Scan");
      int forward_dis = get_ultrasonic_reading();
      //Serial.println(forward_dis);
      if (forward_dis < 15 || ctrl_s.n_stride_cycle <= ctrl_s.n_stride) {
        // GOTO Avoidance 
        ctrl_s.state = SCAN;
      }
      break; 
    default:
      Serial.println("Default");
      break;
  }
}

void check_toggle(){
  if (serial_input == TOGGLE){
    serial_input = 0;
    if (ctrl_s.state !=MANUAL){
      ctrl_s.state = MANUAL;
      ctrl_s.walk_g = 0;
      ctrl_s.turn_g = 0;
    }
    else{
      ctrl_s.state = IDL;
      ctrl_s.walk_g = 3;
    }
  }
}
void manual_control(){
// 0 No new input
// 1 forward
// 2 back
// 3 right
// 4 left
  if (ctrl_s.state == MANUAL){
    switch (serial_input) {
      case FORWARD:
        ctrl_s.walk_g+=1;
        ctrl_s.walk_g = min(ctrl_s.walk_g, WALK_SAT);
        serial_input = 0;
        break;
      case BACK:
        ctrl_s.walk_g-=1;
        ctrl_s.walk_g = max(ctrl_s.walk_g, 0);
        serial_input = 0;
        break;
      case RIGHT:
        ctrl_s.turn_g+=10;
        ctrl_s.turn_g = min(ctrl_s.turn_g, TURN_SAT);
        serial_input = 0;
        break;
      case LEFT:
        ctrl_s.turn_g-=10;
        ctrl_s.turn_g = min(ctrl_s.turn_g, -TURN_SAT);
        serial_input = 0;
        break;
      default:
        // if nothing else matches, do the default
        // default is optional
        break;
    }
  } else {
    ctrl_s.state = IDL;
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
  Serial.print(ctrl_s.walk_g);
  Serial.print(" Turn gain: ");
  Serial.println(ctrl_s.turn_g);
}
