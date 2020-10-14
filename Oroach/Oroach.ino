

#define SERVOMIN 180
#define SERVOMAX 550

#include <Servo.h>
#include <math.h>

Servo headServo;  
Servo rightLegServo;  
Servo leftLegServo;  
Servo midLegServo;  

int pos =0;
const uint8_t headPin = 3; // D3 head  
const uint8_t leftLegPin = 5; // D5 Left leg
const uint8_t rightLegPin = 6; // D9 Right leg
const uint8_t midLegPin = 9; // D9 head 

const uint8_t left_photoresistor_pin = A7; 
const uint8_t right_photoresistor_pin = A6; 
const uint8_t trigPin = 4;
const uint8_t echoPin= 2;
int photores_d;
int;

long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

int stride_ang = 30; 
int height_ang = 25;

void setup() {
  headServo.attach(headPin);
  rightLegServo.attach(rightLegPin); 
  leftLegServo.attach(leftLegPin); 
  midLegServo.attach(midLegPin);   
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 
  Serial.begin(9600);

}
void scan(){}

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

void walk(int delay_time){
    
    delay(delay_time); 
    rightLegServo.write(stride_ang);
    leftLegServo.write(-stride_ang);
    delay(delay_time); 

}

void get_light_diff(){
  photores_d = analogRead(right_photoresistor_pin) - analogRead(left_photoresistor_pin);
}
void loop() {



  
  /*rightLegServo.write(pos)
  for (pos = 0; pos <= 90; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    //headServo.write(pos);
    //rightLegServo.write(pos); 
    //leftLegServo.write(pos); 
    //midLegServo.write(pos);   
    //right_photores_val = analogRead(right_photoresistor_pin);
    Serial.println(right_photores_val);
    delay(15);                       //
  }
  for (pos = 90; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    //headServo.write(pos);
    //rightLegServo.write(pos); 
    //leftLegServo.write(pos); 
    //midLegServo.write(pos);       
    delay(15);                // waits 15ms for the servo to reach the position
  }*/
}
