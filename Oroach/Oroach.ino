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
#define PHOTO_SAT 16.0
#define AVOIDANCE_GAIN 0.5
#define PHOTO_GAIN 0.5


// map(x, 1, 50, 50, -100)
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

float photo_g = 0.0;
float avoidance_g = 0.0;
float turn_g = 0.0;

long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

int stride_ang = 30;
int height_ang = 25;
uint8_t n_stride = 3;

int center;
int left;
int right;
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
  Serial.println("Center");
  center = get_ultrasonic_reading();
  delay(d_head);
  headServo.write(HEAD+45);
  Serial.println("Left");
  delay(d_head);
  left = get_ultrasonic_reading();
  delay(d_head);
  headServo.write(HEAD-45);
  Serial.println("Right");
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
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  return distance;
}

void walk(int delay_time, int turn) {
  midLegServo.write(MIDLEG - height_ang);
  delay(delay_time);
  rightLegServo.write(RIGHTLEG + stride_ang - turn);
  leftLegServo.write(LEFTLEG + stride_ang + turn);
  delay(delay_time);
  midLegServo.write(MIDLEG + height_ang);
  delay(delay_time);
  rightLegServo.write(RIGHTLEG - stride_ang + turn);
  leftLegServo.write(LEFTLEG - stride_ang - turn);
  delay(delay_time);
}

void get_light_diff() {
  int saturation = 30;
  //photores_d = analogRead(right_photoresistor_pin) - analogRead(left_photoresistor_pin);
  float photores_right =25.0/173.0 * analogRead(right_photoresistor_pin);
  float photores_left = 25.0/231.0 * analogRead(left_photoresistor_pin);
  photo_g = photores_right - photores_left;
  Serial.print("Light diff: ");
  Serial.println(photo_g);
}

void avoidance(){
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
    avoidance_g = min(AVOIDANCE_SAT, left_turn);
  } else {
    avoidance_g = max(-AVOIDANCE_SAT, right_turn);
  }
}

void update_turn_gains(){
  if (abs(avoidance_g) > 15){
    turn_g =  avoidance_g;
  } else  {
    turn_g = avoidance_g + photo_g;
  }
}
void loop() {
  stand_stance();  
  delay(100);
  avoidance();
  update_turn_gains();
  for (int i = 0; i < n_stride ; i++)
  {
    walk(200, turn_g);
    delay(500);
    get_light_diff();
    delay(500);
    int forward_dis = get_ultrasonic_reading();
    if (forward_dis < 15) {
      Serial.println("Front");
      avoidance();
    }
  }


}
