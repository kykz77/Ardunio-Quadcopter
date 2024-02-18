//Imports
#include <Wire.h>
#include <Servo.h>

//ESCs
Servo esc1, esc2, esc3, esc4;
float esc1_pulse,esc2_pulse,esc3_pulse,esc4_pulse;

//IMU Variables
float accX, accY, accZ, gyroX, gyroY, gyroZ;
float accX_angle, accY_angle, accZ_angle, gyroX_angle, gyroY_angle, gyroZ_angle;
float accX_angle_error = -3.52;
float accY_angle_error = 7.33;
float gyroX_error = -2.57;
float gyroY_error = 0.19;
float gyroZ_error = 1.57;
float roll, pitch, yaw;
float time_ellp, time_curr, time_prev;

//Kalman Filter
float K_roll_angle = 0;
float last_K_roll = 0;
float K_roll_angle_u = 4;
float K_pitch_angle = 0;
float K_pitch_angle_u = 4;
float last_K_pitch = 0;
float K_output[] = {0,0};

//RC Transmitter and Reciever
//Variables for reading Channels
byte CH1_prev, CH2_prev, CH3_prev, CH4_prev;
unsigned long ISR_timer1, ISR_timer2, ISR_timer3, ISR_timer4;
volatile int CH1_input, CH2_input, CH3_input, CH4_input;
//Variables for reciever Angles
float recX_angle,recY_angle,recZ_angle;

//Variables for PID controller
//Roll X-axis
float p_gain_roll = 1.3;
float i_gain_roll = 0.0;
float d_gain_roll = 15.0;
float roll_output = 0.0;
float roll_p = 0.0;
float roll_i = 0.0;
float roll_d = 0.0;
float roll_curr_error = 0.0;
float roll_last_error = 0.0;
int max_roll = 300; 

//Pitch Y-axis
float p_gain_pitch = p_gain_roll;
float i_gain_pitch = i_gain_roll;
float d_gain_pitch = d_gain_roll;
float pitch_p = 0.0;
float pitch_i = 0.0;
float pitch_d = 0.0;
float pitch_output = 0;
float pitch_curr_error = 0.0;
float pitch_last_error = 0.0;
int max_pitch = 300;

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
int max_yaw = 300; 

void setup() {
  //Turning on Serial 
  Serial.begin(115200);
  delay(10);
  Interrupt_Setup();
  delay(10);
  IMU_Setup();
  delay(10);
  ESC_Setup();
  delay(2000);
}

void loop() {
  read_IMU();
  read_Signal();
  PID_Control();
  ESC_Pulse();
}

////Setup Functions////
void IMU_Setup() {
 Wire.begin();
 //Writing to MPU6050 slave address and activating it
 Wire.beginTransmission(0x68);
 Wire.write(0x6B);
 Wire.write(0x00);
 Wire.endTransmission(true);
}

void Interrupt_Setup() {
 PCICR |= (1 << PCIE0); //Activating Port B for Pin Change Interrupts (PCI)
 PCMSK0 |= (1 << PCINT0); //pin 8, CH1
 PCMSK0 |= (1 << PCINT1); //pin 9, CH2
 PCMSK0 |= (1 << PCINT2); //pin 10, CH3
 PCMSK0 |= (1 << PCINT3); //pin 11, CH4
}

//Interrupt Sub Routine (ISR), will be called whenever PCI occurs
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

void ESC_Setup() {
  DDRD |= B11110000; //Setting the Port D pins (4,5,6,7) to output mode
  //Pairing up pins with ESCs, then providing minimum throttle to arm ESCs
  
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  esc1.attach(4, 1000, 2000);
  esc2.attach(5, 1000, 2000);
  esc3.attach(6, 1000, 2000);
  esc4.attach(7, 1000, 2000);
};

////Loop Functions////

//Reading IMU Functions//

//Reading accelerometer values
void read_Acc() {
 Wire.beginTransmission(0x68);
 Wire.write(0x3B);
 Wire.endTransmission(false);
 Wire.requestFrom(0x68,6,true);
 accX = (Wire.read() << 8 | Wire.read()) / 16384.0; //Combine data since they are stored in 2 different registers (both 8 bit)
 accY = (Wire.read() << 8 | Wire.read()) / 16384.0;
 accZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
 accX_angle = (atan(accY / sqrt(pow(accX,2) + pow(accZ,2))))*(180/PI) - accX_angle_error;
 accY_angle = (atan(-1 * accX / sqrt(pow(accY,2) + pow(accZ,2))))*(180/PI) - accY_angle_error;
}

//Reading gyroscope values
void read_Gyro() {
 time_prev = time_curr;
 time_curr = millis();
 time_ellp =( time_curr - time_prev)/1000;
 Wire.beginTransmission(0x68);
 Wire.write(0x43);
 Wire.endTransmission(false);
 Wire.requestFrom(0x68,6,true);
 gyroX = (Wire.read() << 8 | Wire.read()) / 131.0 - gyroX_error; //Combine data since they are stored in 2 different registers (both 8 bit)
 gyroY = (Wire.read() << 8 | Wire.read()) / 131.0 - gyroY_error;
 gyroZ = (Wire.read() << 8 | Wire.read()) / 131.0 - gyroZ_error;
}

//Kalman Filter Functions
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

void read_IMU() {
  read_Acc();
  read_Gyro();
  run_filters();
  roll = K_roll_angle;
  pitch = K_pitch_angle;
  yaw = yaw + gyroZ * time_ellp;
}

//Reading Transmitter Signal//
void read_Signal() {
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
}
//PID Controller 

void PID_Control() {
  //Pitch
  pitch_curr_error = (pitch - recY_angle);
  pitch_p = pitch_curr_error * p_gain_pitch;
  pitch_i += i_gain_pitch * pitch_curr_error;
  if (pitch_i > max_pitch) {
    pitch_i = max_pitch;
  } else if (pitch_i < -1 * max_pitch) {
    pitch_i = -1 * max_pitch;
  };
  pitch_d = d_gain_pitch * (pitch_curr_error - pitch_last_error);
  pitch_output = pitch_p + pitch_i + pitch_d;
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
  roll_d = d_gain_roll * (roll_curr_error - roll_last_error);
  roll_output = roll_p + roll_i + roll_d;
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
  yaw_d = d_gain_yaw * (yaw_curr_error - yaw_last_error);
  yaw_output = yaw_p + yaw_i + yaw_d;
  if (yaw_output > max_yaw) {
    yaw_output = max_yaw;
  } else if (yaw_output < -1 * max_yaw) {
    yaw_output = -1 * max_yaw;
  }
  yaw_last_error = yaw_curr_error;
}

//Providing Pulses to ESCs
void ESC_Pulse() {
  if (CH3_input < 1075) {
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);
  } else {
   esc1_pulse = CH3_input + roll_output + pitch_output + yaw_output;
   esc2_pulse = CH3_input + roll_output - pitch_output - yaw_output;
   esc3_pulse = CH3_input - roll_output - pitch_output + yaw_output;
   esc4_pulse = CH3_input - roll_output + pitch_output - yaw_output;
   if (esc1_pulse < 1100) {
     esc1_pulse = 1100;
   } else if (esc1_pulse > 1600) {
    esc1_pulse = 1600;
   }
   if (esc2_pulse < 1100) {
     esc2_pulse = 1100;
   }else if (esc2_pulse > 1600) {
    esc2_pulse = 1600;
   }
   if (esc3_pulse < 1100) {
     esc3_pulse = 1100;
   }else if (esc3_pulse > 1600) {
    esc3_pulse = 1600;
   }
   if (esc4_pulse < 1100) {
     esc4_pulse = 1100;
   }else if (esc4_pulse > 1600) {
    esc4_pulse = 1600;
   }
   esc1.writeMicroseconds(esc1_pulse);
   esc2.writeMicroseconds(esc2_pulse);
   esc3.writeMicroseconds(esc3_pulse);
   esc4.writeMicroseconds(esc4_pulse); 
  }
}
