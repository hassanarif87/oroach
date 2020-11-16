#include <Servo.h>
#include <math.h>
#include <SchedTask.h>
// Servo Parameters 
#define SERVOMIN 180
#define SERVOMAX 550
#define HEAD 88
#define LEFTLEG 86
#define RIGHTLEG 94
#define MIDLEG 87
#define UART_RATE 9600
//Auto Control parameters 
#define AVOIDANCE_SAT 20.0
#define PHOTO_SAT 20.0
#define AVOIDANCE_GAIN 0.5
#define PHOTO_GAIN 1.5
//Manual Control parameters 
#define WALK_SAT 5
#define TURN_SAT 15

// Sch update rates 
#define SERVO_UPDATE 15 // ms
#define SENSOR_UPDATE 10 // ms
#define CTRL_UPDATE 20 // ms
#define SERIAL_UPDATE 50 // ms

// Servo instances 
Servo headServo;
Servo rightLegServo;
Servo leftLegServo;
Servo midLegServo;

// Defining i/o pins
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

// Robot control structure
struct ControlStates {
  uint8_t walk_g = 3;
  uint8_t stride_ang = 20;
  uint8_t height_ang = 25;
  float photo_g = 0.0;
  float avoidance_g = 0.0;
  float turn_g = 0.0;
  uint8_t n_stride = 0;
  const uint8_t n_stride_cycle = 3;
  int8_t head_cmd = 0;
  int8_t rightLeg_cmd= 0;
  int8_t leftLeg_cmd = 0;
  int8_t midLeg_cmd= 0;
  int8_t walk_seq = 0;
  States state = IDL; 
};

// Scan states 
struct ScanStates {
  const int8_t seq[3] = {45, -45, 0};   //{"Left", Right", "Center"}
  int distance[3];
  uint8_t head_seq = 0;
  const uint8_t seq_n = 3;
  bool actuate_motor = 1;

};

String inString = "";
unsigned long time;
ManualCmds serial_input = 0;
ControlStates ctrl_s;
ScanStates scan_s;

// forward declarations
void update_servos();
void stand_stance();
int get_ultrasonic_reading();
void walk();
void get_light_diff();
void update_avoidance();
void scan();
void update_turn_gains();
void state_machine();
void check_toggle();
void manual_cotrol();
void check_serial();

// Tasks
SchedTask UpdateServoTask(20, SERVO_UPDATE, update_servos);              // define the turn on task (dispatch now, every 3 sec)
SchedTask StateMachineTask (0, CTRL_UPDATE, state_machine);         // define the turn off task (dispatch in 1 sec, every 3 sec)
SchedTask LightDiffTask (15, SENSOR_UPDATE, get_light_diff);         // define the turn off task (dispatch in 1 sec, every 3 sec)        
SchedTask CheckSerialTask (400, SERIAL_UPDATE, check_serial);
SchedTask WalkTask (NEVER, ONESHOT, walk);
SchedTask ScanTask (NEVER, ONESHOT, scan);
SchedTask CtrlToggleTask (NEVER, ONESHOT, check_toggle);   
void setup() {
  headServo.attach(headPin);
  rightLegServo.attach(rightLegPin);
  leftLegServo.attach(leftLegPin);
  midLegServo.attach(midLegPin);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(UART_RATE);
}

void loop() {
  SchedBase::dispatcher();  
}

/* Serial Inputs */
void check_serial(){
    while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString = (char)inChar;
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n') {
      serial_input = inString.toInt();
      // If there is a new input toggle is checked 
      CtrlToggleTask.setNext(NOW);
    }
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
/* Servo Update */
void update_servos(){
  //Serial.println("Servo Update");
  //Serial.println(ctrl_s.rightLeg_cmd);
  //Serial.println(ctrl_s.leftLeg_cmd);
  //Serial.println(ctrl_s.midLeg_cmd);
  headServo.write(HEAD + ctrl_s.head_cmd);
  rightLegServo.write(RIGHTLEG + ctrl_s.rightLeg_cmd);
  leftLegServo.write(LEFTLEG + ctrl_s.leftLeg_cmd);
  midLegServo.write(MIDLEG + ctrl_s.midLeg_cmd);
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

void stand_stance() {
  ctrl_s.rightLeg_cmd= 0;
  ctrl_s.leftLeg_cmd = 0;
  ctrl_s.midLeg_cmd= 0;
}

void walk(){
  //time=millis();
  //Serial.print("Walk: ");
  //Serial.println(time);

  int turn = ctrl_s.turn_g;
  //Serial.print("Turn: ");
  //Serial.println(turn);
  uint16_t next_time = map(ctrl_s.walk_g, 0, 5, 800, 200);
  if (ctrl_s.walk_g != 0) {
    switch (ctrl_s.walk_seq) {
      case 0:
        // Serial.print("Step0: "); Serial.println(ctrl_s.walk_seq);
        ctrl_s.midLeg_cmd = - ctrl_s.height_ang;
        break;
      case 1:
        // Serial.print("Step1: "); Serial.println(ctrl_s.walk_seq);
        ctrl_s.rightLeg_cmd = ctrl_s.stride_ang - turn;
        ctrl_s.leftLeg_cmd = ctrl_s.stride_ang + turn;
        break;
      case 2:
        // Serial.print("Step2: "); Serial.println(ctrl_s.walk_seq);
        ctrl_s.midLeg_cmd = ctrl_s.height_ang;
        break;
      case 3:
        // Serial.print("Step3: "); Serial.println(ctrl_s.walk_seq);
        ctrl_s.rightLeg_cmd= - ctrl_s.stride_ang + turn;
        ctrl_s.leftLeg_cmd = - ctrl_s.stride_ang - turn;
        ctrl_s.walk_seq = -1;
        // One stride seq complete
        ctrl_s.n_stride++;
        break;
    }
  ctrl_s.walk_seq ++;
  WalkTask.setNext(next_time);
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
  ScanTask.setNext(NOW);
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

void scan(){
  const int d_head = 800;
  while( scan_s.head_seq < scan_s.seq_n ){
    if (scan_s.actuate_motor){
      ctrl_s.head_cmd = scan_s.seq[scan_s.head_seq]; //+45
      scan_s.actuate_motor = 0;
      update_servos();
      ScanTask.setNext(d_head);
    } else {
      scan_s.distance[scan_s.head_seq] = get_ultrasonic_reading();
      scan_s.actuate_motor = 1;
      scan_s.head_seq ++;
      ScanTask.setNext(d_head/2);
    }
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
      Serial.println("Idl");
      stand_stance();
      update_servos();
      ctrl_s.state = AUTONOMOUS;
      WalkTask.setNext(600);
      ctrl_s.n_stride = 0;
      break;
    case SCAN:
      //Serial.println("Scan");
      update_avoidance();
      update_turn_gains();
      if(scan_s.head_seq >= scan_s.seq_n){ 
        ctrl_s.state = AUTONOMOUS;
        scan_s.head_seq = 0;
      }
      break; 
    case MANUAL:
      //Serial.println("Manual");
      manual_control();
      break;
    case AUTONOMOUS:
      //Serial.println("Auto")
      update_turn_gains(); // turn gains are only updtes in Auto 
      //Serial.print("Front Scan");
      int forward_dis = get_ultrasonic_reading();
      Serial.println(forward_dis);
      if (forward_dis < 15 || ctrl_s.n_stride_cycle <= ctrl_s.n_stride) {
        ctrl_s.state = SCAN;
      }
      break; 
    default:
      Serial.println("Default");
      break;
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
