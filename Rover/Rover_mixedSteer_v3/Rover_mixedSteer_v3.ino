/**
*This RC rover operating code is based on the ExplorerBot source code in Chapter 8 
*of Arduino Robotics by Warren, JD et al. 
*Used with permsision.
*
**********************************************************************************
*Title: ExplorerBot_mixedSteer    
*Author: Warren, JD
*Date: 1-8-2010  
*Code version: 1.0    
*Availability: https://sites.google.com/site/arduinorobotics/home/chapter8
***********************************************************************************
*
*Controls 3x L298N motor controllers connected to 5x 24 Volt motors for RC operation of 
*a Mars Regolith Collector prototype.
*The 4 drive motor channels have full 0-100% high-resolution pwm speed control
*The Scoop channel is bi-directional ON/OFF.

***********************************************************************************
*Title: Rover_mixedSteer_v3  
*Author: Sharp, D
*Date: 15-4-2020   
*Code version: 3.0    
***********************************************************************************
*
*/

// Inputs from RC receiver
int ppm1 = 47; 
int ppm2 = 49;
int ppm3 = 51;

// connect motor controller pins to Arduino digital pins
// Linear Actuator - Motor B

int LIN_enB = 46;
int LIN_in3 = 50;
int LIN_in4 = 48;

// LEFT
// Left Front - Motor A
int LF_enA = 3; // PWM
int LF_in1 = 2;
int LF_in2 = 4;

// Left Back - Motor B
int LB_in3 = 5;
int LB_in4 = 6;
int LB_enB = 9; // PWM

// RIGHT
// Right Front - Motor B
int RB_enB = 10; //PWM
int RB_in4 = 7;
int RB_in3 = 8;

// Right Back - Motor A
int RF_in2 = 12;
int RF_in1 = 13;
int RF_enA = 11; //PWM

unsigned int servo1_val; // unsigned integer simply means that it cannot be negative.
int adj_val1;  
int servo1_Ready;

unsigned int servo2_val; 
int adj_val2;  
int servo2_Ready;

unsigned int servo3_val; 
int adj_val3;  
int servo3_Ready;

unsigned int servo4_val; 
int adj_val4;  
int servo4_Ready;

////////////////////////////////

int deadband = 100; // sets the total deadband - this number is divided by 2 to get the deadband for each direction. Higher value will yield larger neutral band.
int deadband_high = deadband / 2; // sets deadband_high to be half of deadband (ie. 10/2 = 5)
int deadband_low = deadband_high * -1; // sets deadband_low to be negative half of deadband (ie. 5 * -1 = -5)

// You can adjust these values to calibrate the code to your specific radio - check the Serial Monitor to see your values.
int low1 = 900;
int high1 = 2000;
int low2 = 900;
int high2 = 2000;

int x; // this will represent the x coordinate
int y; // this will represent the y coordinate

int left; // X and y will be converted into left and right values
int right; // X and y will be converted into left and right values

int speed_low; 
int speed_high;

int speed_limit = 255;

int speed_max = 255;
int speed_min = 0;


void setup() {

  TCCR1B = TCCR1B & 0b11111000 | 0x01; // change PWM frequency on pins 9 and 10 to 32kHz
  TCCR2B = TCCR2B & 0b11111000 | 0x01; // change PWM frequency on pins 3 and 11 to 32kHz

  Serial.begin(9600);

  // set all the motor control pins to outputs
  pinMode(LIN_enB, OUTPUT);
  pinMode(LIN_in3, OUTPUT);
  pinMode(LIN_in4, OUTPUT);
  
  pinMode(RF_enA, OUTPUT);
  pinMode(RF_in1, OUTPUT);
  pinMode(RF_in2, OUTPUT);
  
  pinMode(RB_in3, OUTPUT);
  pinMode(RB_in4, OUTPUT);
  pinMode(RB_enB, OUTPUT);

  pinMode(LF_enA, OUTPUT);
  pinMode(LF_in1, OUTPUT);
  pinMode(LF_in2, OUTPUT);
  
  pinMode(LB_in3, OUTPUT);
  pinMode(LB_in4, OUTPUT);
  pinMode(LB_enB, OUTPUT); 
  
  //PPM inputs from RC receiver
  pinMode(ppm1, INPUT);
  pinMode(ppm2, INPUT); 
  pinMode(ppm3, INPUT);

  delay(1000);

}


void pulse(){

  servo1_val = pulseIn(ppm1, HIGH, 20000); // read pulse from channel 1
  // make sure servo 1 value is within range (between 800 and 2200 microseconds)
  if (servo1_val > 800 && servo1_val < 2200){	
    servo1_Ready = true;
  }
  else {
    servo1_Ready = false;
    servo1_val = 1500;
  }

  servo2_val = pulseIn(ppm2, HIGH, 20000); // read pulse from channel 2
  if (servo2_val > 800 && servo2_val < 2200){	
    servo2_Ready = true;
  }
  else {
    servo2_Ready = false;
    servo2_val = 1500;
  }

    servo3_val = pulseIn(ppm3, HIGH, 20000); // read pulse from channel 3
  if (servo3_val > 1800){
    raise_scoop();
  }

  else if (servo3_val < 1200){
    lower_scoop();
  }
  else{
    stop_scoop();
  }
}



void loop() {

  pulse(); // read pulses

  ///////////////////////////////

  if (servo1_Ready) {

    servo1_Ready = false;  
    // map servo value from 1500 microseconds (neutral) to a value of 0. 
    //If pulse is above neutral (forward) value will be 0 to 255, otherwise it will be 0 to -255 for reverse
    adj_val1 = map(servo1_val, low1, high1, -speed_limit, speed_limit); 
    adj_val1 = constrain(adj_val1, -speed_limit, speed_limit);

    x = adj_val1;

  }
  if (servo2_Ready) {

    servo2_Ready = false;

    adj_val2 = map(servo2_val ,low2, high2, -speed_limit, speed_limit); 
    adj_val2 = constrain(adj_val2, -speed_limit, speed_limit);

    y = adj_val2;

  }



  if (x > deadband_high) {  // if the Up/Down R/C input is above the upper threshold, go FORWARD 

    // now check to see if left/right input from R/C is to the left, to the right, or centered.

    if (y > deadband_high) { // go forward while turning right proportional to the R/C left/right input
      left = x;
      right = x - y;
      test(); // make sure signal stays within range of the Arduino capable values
      left_forward(left);
      right_forward(right);
      // quadrant 1
    }

    else if (y < deadband_low) {   // go forward while turning left proportional to the R/C left/right input
      left = x - (y * -1);  // remember that in this case, y will be a negative number
      right = x;
      test(); 
      left_forward(left);
      right_forward(right);
      // quadrant 2
    }

    else {   // left/right stick is centered, go straight forward
      left = x;
      right = x;
      test();
      left_forward(left);
      right_forward(right);
    }
  }

  else if (x < deadband_low) {    // otherwise, if the Up/Down R/C input is below lower threshold, go BACKWARD

    // remember that x is below deadband_low, it will always be a negative number, we need to multiply it by -1 to make it positive.
    // now check to see if left/right input from R/C is to the left, to the right, or centered.

    if (y > deadband_high) { // // go backward while turning right proportional to the R/C left/right input
      left = (x * -1);
      right = (x * -1) - y;
      test();
      left_reverse(left);
      right_reverse(right);
      // quadrant 4
    }

    else if (y < deadband_low) {   // go backward while turning left proportional to the R/C left/right input
      left = (x * -1) - (y * -1);
      right = x * -1;
      test();
      left_reverse(left);
      right_reverse(right);   
      // quadrant 3
    }		

    else {   // left/right stick is centered, go straight backwards
      left = x * -1; 
      right = x * -1; 
      test();
      left_reverse(left);
      right_reverse(right);
    }

  }

  else {     // if neither of the above 2 conditions is met, the Up/Down R/C input is centered (neutral)

    if (y > deadband_high) { //spin right

      left = y;
      right = y;
      test();
      left_forward(left);
      right_reverse(right);

    }  

    else if (y < deadband_low) { //spin left

      left = (y * -1);
      right = (y * -1);
      test();
      left_reverse(left);
      right_forward(right);

    }  

    else {

      left = 0;
      right = 0;
      left_stop();
      right_stop();
    }

  }

  Serial.print(left);
  Serial.print("   ");
  Serial.print(right);
  Serial.print("    ");

  Serial.print(servo1_val);
  Serial.print("   ");
  Serial.print(servo2_val);
  Serial.print("    ");
  Serial.print(servo3_val);
  Serial.println("   ");
}


int test() {

  // make sure we don't try to write any invalid PWM values to the h-bridge, ie. above 255 or below 0.

  if (left > 254) {
    left = 255;
  }
  if (left < 1) {
    left = 0; 
  }
  if (right > 254) {
    right = 255;
  }
  if (right < 1) {
    right = 0; 
  } 

}


// Create single instances for each motor direction, so we don't accidentally write a shoot-through condition to the H-bridge.

void right_forward(int m2_speed){
  analogWrite(RF_enA, m2_speed);
  digitalWrite(RF_in1, LOW);
  digitalWrite(RF_in2, HIGH);
  digitalWrite(RB_in3, HIGH);
  digitalWrite(RB_in4, LOW);
  analogWrite(RB_enB, m2_speed);    
}

void right_reverse(int m2_speed){
  analogWrite(RF_enA, m2_speed);
  digitalWrite(RF_in1, HIGH);
  digitalWrite(RF_in2, LOW);
  digitalWrite(RB_in3, LOW);
  digitalWrite(RB_in4, HIGH);
  analogWrite(RB_enB, m2_speed);
}

void left_forward(int m1_speed){
  analogWrite(LF_enA, m1_speed);
  digitalWrite(LF_in1, LOW);
  digitalWrite(LF_in2, HIGH);
  digitalWrite(LB_in3, HIGH);
  digitalWrite(LB_in4, LOW);
  analogWrite(LB_enB, m1_speed);
   
}

void left_reverse(int m1_speed){
  analogWrite(LF_enA, m1_speed);
  digitalWrite(LF_in1, HIGH);
  digitalWrite(LF_in2, LOW);
  digitalWrite(LB_in3, LOW);
  digitalWrite(LB_in4, HIGH);
  analogWrite(LB_enB, m1_speed); 
} 

void right_stop(){    
  digitalWrite(RF_in1, LOW);
  digitalWrite(RF_in2, LOW);
  digitalWrite(RB_in3, LOW);
  digitalWrite(RB_in4, LOW);
}

void left_stop(){
  digitalWrite(LF_in1, LOW);
  digitalWrite(LF_in2, LOW);
  digitalWrite(LB_in3, LOW);
  digitalWrite(LB_in4, LOW);  
}

void raise_scoop(){
  digitalWrite(LIN_in3, HIGH);
  digitalWrite(LIN_in4, LOW);
  analogWrite(LIN_enB, 255); 
}

void lower_scoop(){
  digitalWrite(LIN_in3, LOW);
  digitalWrite(LIN_in4, HIGH);
  analogWrite(LIN_enB, 255); 
}

void stop_scoop(){
  digitalWrite(LIN_in3, LOW);
  digitalWrite(LIN_in4, LOW);
  analogWrite(LIN_enB,0); 
}
