//Imports
#include <Wire.h>
#include <Servo.h>

//ESCs
Servo esc1, esc2, esc3, esc4;
float esc1_pulse,esc2_pulse,esc3_pulse,esc4_pulse;
//Variables for reading gyroscope and accelerometer
float accX, accY, accZ, gyroX, gyroY, gyroZ;
float accX_angle, accY_angle, accZ_angle, gyroX_angle, gyroY_angle, gyroZ_angle;
float accX_angle_error = -0.12;
float accY_angle_error = 5.41;
float gyroX_error = -2.60;
float gyroY_error = 0.26;
float gyroZ_error = 1.76;
float roll, pitch, yaw;
float accX_error, accY_error, gyroX_error, gyroY_error, gyroZ_error;
float time_ellp, time_curr, time_prev;

//Kalman Filter
float K_roll_angle = 0;
float last_K_roll = 0;
float K_roll_angle_u = 4;
float K_pitch_angle = 0;
float K_pitch_angle_u = 4;
float last_K_pitch = 0;
float K_output[] = {0,0};

//Variables for reading Channels
byte CH1_prev, CH2_prev, CH3_prev, CH4_prev;
unsigned long ISR_timer1, ISR_timer2, ISR_timer3, ISR_timer4;
volatile int CH1_input, CH2_input, CH3_input, CH4_input;

//Variables for PID controller
//Roll X-axis
float p_gain_roll = 0.0;
float i_gain_roll = 0.0;
float d_gain_roll = 0.0;
float roll_output = 0.0;
float roll_p = 0.0;
float roll_i = 0.0;
float roll_d = 0.0;
float roll_curr_error = 0.0;
float roll_last_error = 0.0;
int max_roll = 200; 
//Pitch Y-axis
float p_gain_pitch = p_gain_roll;
float i_gain_pitch = i_gain_roll;
float d_gain_pitch = d_gain_roll;
float pitch_p = 3.0;
float pitch_i = 0.01;
float pitch_d = 0.0;
float pitch_output = 0;
float pitch_curr_error = 0.0;
float pitch_last_error = 0.0;
int max_pitch = 200; 
//Yaw Z-axis
float p_gain_yaw = 0.0;
float i_gain_yaw = 0.0;
float d_gain_yaw = 0.0;
float yaw_p = 0.0;
float yaw_i = 0.0;
float yaw_d = 0.0;
float yaw_output = 0;
float yaw_curr_error = 0.0;
float yaw_last_error = 0.0;
int max_yaw = 200; 
//Reciever
float recX_angle,recY_angle,recZ_angle;
//Time Ellapsed
float pid_time_prev;
float pid_time_elp;
float time = 0;

void setup() {
 //Activate PCI and Port B
 PCICR |= (1 << PCIE0); //Activating Port B for PCI
 PCMSK0 |= (1 << PCINT0); //pin 8, CH1
 PCMSK0 |= (1 << PCINT1); //pin 9, CH2
 PCMSK0 |= (1 << PCINT2); //pin 10, CH3
 PCMSK0 |= (1 << PCINT3); //pin 11, CH4
 Serial.begin(9600);
 Wire.begin();
 Wire.beginTransmission(0x68);
 Wire.write(0x6B);
 Wire.write(0x00);
 Wire.endTransmission(true);
 delay(10);
 //Setting up ESCs
 DDRD |= B11110000;
 ESC_setup();
 delay(10);
}

void loop() {
 readIMU();
 readSignal();
 PID_Control();
 ESC_pulse();
}

ISR(PCINT0_vect) {
  if (CH1_prev == 0 && (PINB & B00000001)) {
    CH1_prev = 1;
    ISR_timer1 = micros();
  } else if (CH1_prev == 1 && !(PINB & B00000001)){
    CH1_prev = 0;
    CH1_input = micros() - ISR_timer1;
  }
   if (CH2_prev == 0 && (PINB & B00000010)) {
    CH2_prev = 1;
    ISR_timer2 = micros();
  } else if (CH2_prev == 1 && !(PINB & B00000010)){
    CH2_prev = 0;
    CH2_input = micros() - ISR_timer2;
  }
   if (CH3_prev == 0 && (PINB & B00000100)) {
    CH3_prev = 1;
    ISR_timer3 = micros();
  } else if (CH3_prev == 1 && !(PINB & B00000100)){
    CH3_prev = 0;
    CH3_input = micros() - ISR_timer3;
  }
   if (CH4_prev == 0 && (PINB & B00001000)) {
    CH4_prev = 1;
    ISR_timer4 = micros();
  } else if (CH4_prev == 1 && !(PINB & B00001000)){
    CH4_prev = 0;
    CH4_input = micros() - ISR_timer4;
  }
}

void ESC_setup() {
 esc1.attach(4,850,1860);
 esc1.write(0);
 delay(2000);
 esc2.attach(5,850,1860);
 esc2.write(0);
 delay(2000);
 esc3.attach(6,850,1860);
 esc3.write(0);
 delay(2000);
 esc4.attach(7,850,1860);
 esc4.write(0);
 delay(2000);
}

void readIMU() {
  //Reading accelerometer values
 Wire.beginTransmission(0x68);
 Wire.write(0x3B);
 Wire.endTransmission(false);
 Wire.requestFrom(0x68,6,true);
 accX = (Wire.read() << 8 | Wire.read()) / 16384.0; //Combine the stuff since they're stored in 2 diff registers (both 8 bit)
 accY = (Wire.read() << 8 | Wire.read()) / 16384.0;
 accZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
 accX_angle = (atan(accY / sqrt(pow(accX,2) + pow(accZ,2))))*(180/PI) - accX_angle_error;
 accY_angle = (atan(-1 * accX / sqrt(pow(accY,2) + pow(accZ,2))))*(180/PI) - accY_angle_error;
 //Reading gyroscope values
 time_prev = time_curr;
 time_curr = millis();
 time_ellp =( time_curr - time_prev)/1000;
 Wire.beginTransmission(0x68);
 Wire.write(0x43);
 Wire.endTransmission(false);
 Wire.requestFrom(0x68,6,true);
 gyroX = (Wire.read() << 8 | Wire.read()) / 131.0 - gyroX_error; //Combine the stuff since they're stored in 2 diff registers (both 8 bit)
 gyroY = (Wire.read() << 8 | Wire.read()) / 131.0 - gyroY_error;
 gyroZ = (Wire.read() << 8 | Wire.read()) / 131.0 - gyroZ_error; 
 run_filters();
 roll = K_roll_angle;
 pitch = K_pitch_angle;
 yaw = yaw + gyroZ * time_ellp;
 Serial.println("Angles:");
 Serial.print(roll);
 Serial.print("/");
 Serial.print(pitch);
 Serial.print("/");
 Serial.print(yaw);
 Serial.println("");
}
void readSignal() {
  Serial.print(" Roll: ");
  if (CH1_input - 1520 > 0) {
    Serial.print(" x-Right ");
  } else if (CH1_input - 1480 < 0){
    Serial.print(" x-Left ");
  } else {
    Serial.print(" Stay ");
  }
  Serial.println(CH1_input);
  Serial.print(" Pitch: ");
  if (CH2_input - 1520 > 0) {
    Serial.print("y-Right");
  } else if (CH2_input - 1480 < 0){
    Serial.print("y-Left");
  } else {
    Serial.print("Stay");
  }
  Serial.println(CH2_input);
  Serial.print(" Y-translate: ");
  if (CH3_input - 1520 > 0) {
    Serial.print(" z-Up ");
  } else if (CH3_input - 1480 < 0){
    Serial.print(" z-Down ");
  } else {
    Serial.print(" Stay ");
  }
  Serial.println(CH3_input);
  Serial.print(" Yaw: ");
  if (CH4_input - 1520 > 0) {
    Serial.print(" Rotate Clock ");
  } else if (CH4_input - 1480 < 0){
    Serial.print(" Rotate Anticlock ");
  } else {
    Serial.print(" Stay ");
  }
  Serial.println(CH4_input);
}

void PID_Control() {
  pid_time_prev = time;
  time = millis();
  pid_time_elp = (time - pid_time_prev)/1000;
  
  if (CH1_input < 1480) {
    recX_angle = (1480 - CH1_input)/9.6; //Roll
  } else {
    recX_angle = (CH1_input - 1520)/9.6;
  }
  if (CH2_input < 1480) {
    recY_angle = (1480 - CH2_input)/9.6; //Pitch
  } else {
    recY_angle = (CH2_input - 1520)/9.6;
  }
  if (CH4_input < 1480) {
    recZ_angle = (1480 - CH4_input)/9.6; //Yaw
  } else {
    recZ_angle = (CH4_input - 1520)/9.6;
  }
  //Pitch
  pitch_curr_error = (pitch - recY_angle);
  pitch_p = pitch_curr_error * p_gain_pitch;
  pitch_i += i_gain_pitch * pitch_curr_error;
  if (pitch_i > max_pitch) {
    pitch_i = max_pitch;
  } else if (pitch_i < -1 * max_pitch) {
    pitch_i = -1 * max_pitch;
  };
  pitch_d = d_gain_pitch * (pitch_curr_error - pitch_last_error)/(pid_time_elp);
  pitch_output = pitch_p + pitch_i + pitch_d;
  Serial.println("Pitch output:");
  Serial.print("Error:");
  Serial.print(pitch_curr_error - pitch_last_error);
  Serial.print(pitch_p);
  Serial.print("/");
  Serial.print(pitch_i);
  Serial.print("/");
  Serial.print(pitch_d);
  Serial.print("/");
  Serial.print(pitch_output);
  Serial.println("");
  if (pitch_output > max_pitch) {
    pitch_output = max_pitch;
  } else if (pitch_output < -1 * max_pitch) {
    pitch_output = -1 * max_pitch;
  }
  pitch_last_error = pitch_curr_error;
  
  //Roll
  
  roll_curr_error = (roll - recX_angle);
  roll_p = roll_curr_error * p_gain_roll;
  roll_i += i_gain_roll * roll_curr_error;
  if (roll_i > max_roll) {
    roll_i = max_roll;
  } else if (pitch_i < -1 * max_roll) {
    roll_i = -1 * max_roll;
  };
  roll_d = d_gain_roll * (roll_curr_error - roll_last_error)/(pid_time_elp);
  roll_output = roll_p + roll_i + roll_d;
  Serial.println("Roll output:");
  Serial.print(roll_p);
  Serial.print("/");
  Serial.print(roll_i);
  Serial.print("/");
  Serial.print(roll_d);
  Serial.print("/");
  Serial.print(roll_output);
  Serial.println("");
  if (roll_output > max_roll) {
    roll_output = max_roll;
  } else if (roll_output < -1 * max_roll) {
    roll_output = -1 * max_roll;
  }
  roll_last_error = roll_curr_error;
  
  //Yaw
  
  yaw_curr_error = (yaw - recZ_angle);
  yaw_p = yaw_curr_error * p_gain_yaw;
  yaw_i += yaw_curr_error * i_gain_yaw;
  if (yaw_i > max_yaw) {
    yaw_i = max_yaw;
  } else if (yaw_i < -1 * max_yaw) {
    yaw_i = -1 * max_yaw;
  };
  yaw_d = (yaw_curr_error - yaw_last_error)/(pid_time_elp);
  yaw_output = yaw_p + yaw_i + d_gain_yaw * yaw_d;
  if (yaw_output > max_yaw) {
    yaw_output = max_yaw;
  } else if (yaw_output < -1 * max_yaw) {
    yaw_output = -1 * max_yaw;
  }
  yaw_last_error = yaw_curr_error;
}

void ESC_pulse() {
  CH3_input = map(CH3_input,990,2000,850,1860);
  if (CH3_input < 1000) {
    esc1.writeMicroseconds(0);
    esc2.writeMicroseconds(0);
    esc3.writeMicroseconds(0);
    esc4.writeMicroseconds(0);
  } else {
   esc1_pulse = CH3_input + roll_output + pitch_output + yaw_output;
   esc2_pulse = CH3_input + roll_output - pitch_output - yaw_output;
   esc3_pulse = CH3_input - roll_output - pitch_output + yaw_output;
   esc4_pulse = CH3_input - roll_output + pitch_output - yaw_output;
   if (esc1_pulse < 1000) {
     esc1_pulse = 1000;
   } else if (esc1_pulse < 1500) {
    esc1_pulse = 1500;
   }
   if (esc2_pulse < 1000) {
     esc2_pulse = 1000;
   }else if (esc2_pulse > 1500) {
    esc2_pulse = 1500;
   }
   if (esc3_pulse < 1000) {
     esc3_pulse = 1000;
   }else if (esc3_pulse > 1500) {
    esc3_pulse = 1500;
   }
   if (esc4_pulse < 1000) {
     esc4_pulse = 1000;
   }else if (esc4_pulse > 1500) {
    esc4_pulse = 1500;
   }
   Serial.print("ESC1: ");
   Serial.println(esc1_pulse);
   Serial.print("ESC2: ");
   Serial.println(esc2_pulse);
   Serial.print("ESC3: ");
   Serial.println(esc3_pulse);
   Serial.print("ESC4: ");
   Serial.println(esc4_pulse);
   esc1.writeMicroseconds(esc1_pulse);
   esc2.writeMicroseconds(esc2_pulse);
   esc3.writeMicroseconds(esc3_pulse);
   esc4.writeMicroseconds(esc4_pulse); 
  }
}

void kalman_filter(float K_state,float K_u,float K_rate,float K_angle, float t_ellapsed) {
  K_state += t_ellapsed*K_rate;
  K_u += (t_ellapsed * t_ellapsed)*(4.0 *4.0);
  float K_gain = (K_u)/(K_u + 9);
  K_state += K_gain*(K_angle - K_state);
  K_u = (1 - K_gain)*K_u;
  K_output[0] = K_state;
  K_output[1] = K_u;
}

void run_filters() {
  kalman_filter(K_roll_angle,K_roll_angle, gyroX, accX_angle,0.004);
  K_roll_angle = K_output[0];
  K_roll_angle_u = K_output[1];
  kalman_filter(K_pitch_angle,K_pitch_angle, gyroY, accY_angle,0.004);
  K_pitch_angle = K_output[0];
  K_pitch_angle_u = K_output[1];
}
