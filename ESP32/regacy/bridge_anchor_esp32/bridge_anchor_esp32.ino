// File name: ESP_master.ino
// Recieve RF signal, encoder, data from ROS2 system, and control the motor
// Written by Yecheol Moon, May 1, 2023

// Setups
// PIN number - Encoder
int mtrA_encA_Pin = 36; int mtrA_encB_Pin = 39; int mtrB_encA_Pin = 34; int mtrB_encB_Pin = 35; 
// PIN number - Motor driver
int IN1_A = 32; int IN2_A = 33; int EN_A= 27; int IN1_B = 25; int IN2_B = 26; int EN_B= 14;  
// PIN number - Voltage sensor
#define VoltagePin 13
// PIN number - RF controller reciever (AUX1, GEAR, RUDD, ELEV, AILE, THRO)
// AUX1 : Not use, GEAR : Modes setup, RUDD : left stick horizontal, ELEV : left stick vertical, AILE : right stick horizontal, THRO : right stick vertical
int PIN_AUX1 = 23;  int PIN_GEAR = 22;  int PIN_RUDD = 21;  int PIN_ELEV = 19;  int PIN_AILE = 18;  int PIN_THRO = 5;
// Max and min value of RF controller
// Threshold value to devide gear mode : MIN_GEAR ~ DOWN_GEAR ~ UP_GEAR ~ MAX_Gear
float MAX_GEAR = 2200;  float MIN_GEAR = 700; float UP_GEAR = 1700;  float DOWN_GEAR = 1200; 
float MAX_RUDD = 1900;  float MIN_RUDD = 1097;
float MAX_ELEV = 1902;  float MIN_ELEV = 1098;
float MAX_AILE = 2018;  float MIN_AILE = 978;
float MAX_THRO = 1903;  float MIN_THRO = 1093;
// incremetal counts when wheel turns 1 round (A : Left, B : Right)
float COUNT_PER_ROUND_A = 603.5;  
float COUNT_PER_ROUND_B = 611.0;
// setups max PWM 230 to saftey
#define MAX_PWM_DUTY 230.0
// Ratio inc/sec to PWM by getting experiments
float INCPERSEC_TO_PWM_L = 1.926;//4.5
float INCPERSEC_TO_PWM_R = 1.926;//4.5//6.9477
// time between loop cycles
float dt = 0.05;
// time threshold check timeout error
#define TIMEOUT 1.0
// Deadzone PWM value
#define DEADZONE_L 42.0
#define DEADZONE_R 42.0
// L/R gain to tunning difference between left and right
#define GAIN_PWM_LR 0.5 
// Gains for PID
float P_GAIN_L = 0.04; //0.036
float P_GAIN_R = 0.04; //0.036
float D_GAIN_L = 0.00005; //0.00005
float D_GAIN_R = 0.00005; //0.00005
// motor control mode. NOT USE ANYMORE
int mtr_advnc = 1;

// Initialize
// [RF controller]
// RF channel values
float AUX1 = 0; float GEAR = 0; float RUDD = 0; float ELEV = 0; float AILE = 0; float THRO = 0;
// RF channel percentage values
float per_RUDD = 0; float per_ELEV = 0; float per_AILE = 0; float per_THRO = 0; 
// time 1,2 are factors to calculate dtime. dtime is loop time in RF calculate
float time1 = 2;  float time2 = 1;  float dtime = 1;

// [Encoder and position]
// encoder raw digital input (HIGH or LOW)
int mtrA_encA = LOW;  int mtrA_encB = LOW;  int mtrB_encA = LOW;  int mtrB_encB = LOW;
// last and temporary digital status to calculate
int mtrA_encA_last = LOW;   int mtrA_encB_last = LOW; int mtrB_encA_last = LOW; int mtrB_encB_last = LOW; 
int mtrA_encA_tmp = LOW;  int mtrA_encB_tmp = LOW;  int mtrB_encA_tmp = LOW;  int mtrB_encB_tmp = LOW;
// target motor position to get by using PID-control
float mtrA_pos_desired = 0; float mtrB_pos_desired = 0;
// Encoder counts and counts to plus of minus. it is for correction of encoder noise
unsigned long countA = 0; unsigned long countB = 0;
unsigned long countA_plus = 0;  unsigned long countB_plus = 0;
unsigned long countA_minus = 0; unsigned long countB_minus = 0;
unsigned long countA_plus_sum = 0;  unsigned long countA_minus_sum = 0;
unsigned long countB_plus_sum = 0;  unsigned long countB_minus_sum = 0;
// motor position data [inc]
float mtrA_pos = 0; float mtrB_pos = 0;
float mtrA_pos_last = 0;  float mtrB_pos_last = 0;
// PWM values of motor, MTRtemp is to change left and right
int MTRLeft = 0;  int MTRRight = 0; int MTRtemp = 0;
// Present angular speed of motor left, right
float w_left = 0.0; float w_right = 0.0;
// inc/sec of left or right motor
float inc_per_sec_left = 0; float inc_per_sec_right = 0;
// Setup threshold to remove noise
#define FILTER_THRESHOLD_ACC 20.0 //[rad/s]

// [Message communcation]
// msg datas to communicate with rpi
// Entire string of data strings 
// msg to rpi : "SRT, mtrA_angle[rad], mtrB_angle[rad], timestamp_to_rpi, mtrA_angle[inc], mtrB_angle[inc], PWMLeft, PWMRight, mtrA_angle_desired[inc], mtrB_angle_desired[inc], e_PID_left[inc], e_PID_right[inc], battery_voltage, END"
// msg from rpi : "SRT, w_left[rad/s], w_right[rad], P_GAIN_L, P_GAIN_R, D_GAIN_L, D_GAIN_R, COUNT_PER_ROUND_A, COUNT_PER_ROUND_B, INCPERSEC_TO_PWM_L, INCPERSEC_TO_PWM_R, mtr_mode, PWM_L_manual, PWM_R_manual, mtr_advnc, END"
// checker for msg
String msg_rpi_start_checker = "SRT"; String msg_rpi_end_checker = "END";
String w_left_string = "Null";  String w_right_string = "Null";
// time to check RPI
float time_from_rpi = 2;
float time_from_rpi_last = 1;
// time stamp to send data to rpi
float time_to_rpi = 2;

// [motor control mode]
// 0.0 - just set w_left,right
// 1.0 - w PID control mode [default]
// 2.0 - to control manual PWM
float mtr_mode = 1.0;
// PWM manual control values for mtr_mode is 2.0
float PWM_L_manual = 0.0; float PWM_R_manual = 0.0;
// PID error terms
float e_term_L = 0; float e_term_R = 0;
float e_term_L_prev = 0;  float e_term_R_prev = 0;
// PID error terms [inc]
float e_inc_left = 0; float e_inc_right = 0;
// PID error terms to calculate D control
float e_term_L_d = 0; float e_term_R_d = 0;

// [Battery voltage] from voltage sensor
float voltage_value = 0.0;

// [Multitask]
TaskHandle_t Task1;
// Starting time of multitask
float time_start = 0; 

void text(String value);
void disp(String name, float var);
void CheckVoltage();
void EncoderReading ( void *param );
void tx_to_rpi();
void rx_from_rpi();
void motor_turn(int speed_pwm_left, int speed_pwm_right);
void motor_stop();
void RF_recieve_values();
void check_gear(float check_count);
void motor_control_pos();

void setup() {
  
  xTaskCreatePinnedToCore(
    EncoderReading,         // task function
    "Task1",           // task name
    10000,             // stack size (word)
    NULL,              // task parameters
    0,                 // task priority
    &Task1,            // task handle
    0);                // exe core

  pinMode (mtrA_encA_Pin, INPUT);  pinMode (mtrA_encB_Pin, INPUT);
  pinMode (mtrB_encA_Pin, INPUT);  pinMode (mtrB_encB_Pin, INPUT);

  //Motor driver
  pinMode (EN_A, OUTPUT);
  pinMode (IN1_A, OUTPUT);
  pinMode (IN2_A, OUTPUT);
  pinMode (EN_B, OUTPUT);
  pinMode (IN1_B, OUTPUT);
  pinMode (IN2_B, OUTPUT);

  //RF
  pinMode(PIN_AUX1, INPUT);
  pinMode(PIN_GEAR, INPUT);
  pinMode(PIN_RUDD, INPUT);
  pinMode(PIN_ELEV, INPUT);
  pinMode(PIN_AILE, INPUT);
  pinMode(PIN_THRO, INPUT);

  Serial.begin(115200);  
  Serial2.begin(115200);  

  // ENC Reading
  mtrA_encA = digitalRead(mtrA_encA_Pin);
  mtrA_encB = digitalRead(mtrA_encB_Pin);
  mtrB_encA = digitalRead(mtrB_encA_Pin);
  mtrB_encB = digitalRead(mtrB_encB_Pin);
  
  time_start = 0.001*millis();
}

//============================================================================================

void loop()
{
  CheckVoltage();
  
  int MAX_PWM = MAX_PWM_DUTY;
  //Check GEAR mode
  check_gear(0.5);

  // RF off - automatic system
  if (!(GEAR > MIN_GEAR) && (GEAR < MAX_GEAR))
  {
    time_from_rpi = 0.001*millis();
    //ESP - RPI data coummnication  
    tx_to_rpi();
    rx_from_rpi();
    motor_control_pos();
  }

  // RF mode
  else
  {
    text("RF Mode on");
    RF_recieve_values();  

    int error_count = 0;
    
    if( (RUDD < (MIN_RUDD-200) ) || (RUDD > (MAX_RUDD+200) ) )     {text("RUDD ERROR !!");  disp("RUDD",RUDD);   error_count += 1 ;    }
    if( (ELEV < (MIN_ELEV-200) ) || (ELEV > (MAX_ELEV+200) ) )     {text("ELEV ERROR !!");  disp("ELEV",ELEV);   error_count += 1 ;    }
    if( (AILE < (MIN_AILE-200) ) || (AILE > (MAX_AILE+200) ) )     {text("AILE ERROR !!");  disp("AILE",AILE);   error_count += 1 ;    }
    if( (THRO < (MIN_THRO-200) ) || (THRO > (MAX_THRO+200) ) )     {text("THRO ERROR !!");  disp("THRO",THRO);   error_count += 1 ;    }
    if(error_count>3) {text("RF FATAL ERROR !!");  disp("error_count",error_count);   GEAR = 0.0;   RUDD = 0.0;   ELEV = 0.0;   AILE = 0.0;   THRO = 0.0;    }
    
    // GEAR 1 :
    // Normal operating mode (GEAR most down)
    if (GEAR>UP_GEAR && GEAR<MAX_GEAR)
    {
      text("RF Gear 1 : Automatic control mode");
      //THRO (right vertical stick) power limit
      disp("MAX_PWM", MAX_PWM);
      MAX_PWM = MAX_PWM*(per_THRO+1)/2;
      disp("per_THRO", MAX_PWM);
      disp("MAX_PWM", MAX_PWM);
      //PWM saturation to 0 if PWM is too low
      if(MAX_PWM < 10) {MAX_PWM=0;}

      //MTR power control based on left stick vertical and horizontal movement
      MTRLeft = int(MAX_PWM*(per_ELEV-per_RUDD*2));
      MTRRight = int(MAX_PWM*(per_ELEV+per_RUDD*2));
      disp("per_ELEV", per_ELEV);
      disp("per_RUDD", per_RUDD);
      disp("MTRLeft", MTRLeft);
      disp("MTRRight", MTRRight);

      //PWM saturation when PWM is over
      if (MTRLeft >= MAX_PWM) {MTRLeft = MAX_PWM;}
      if (MTRRight >= MAX_PWM) {MTRRight = MAX_PWM;}
      if (MTRLeft <= -MAX_PWM) {MTRLeft = -MAX_PWM;}
      if (MTRRight <= -MAX_PWM) {MTRRight = -MAX_PWM;}
      disp("MTRLeft", MTRLeft);
      disp("MTRRight", MTRRight);
      
      //if left stick is backward direction, setting motor values as opposite to forward
      if (per_ELEV < 0) 
      {
        MTRtemp = MTRLeft; 
        MTRLeft = MTRRight; 
        MTRRight = MTRtemp;
      }

      // Motor turning
      disp("AUX1", AUX1);
      disp("GEAR", GEAR);
      disp("RUDD", RUDD);
      disp("ELEV", ELEV);
      disp("AILE", AILE);
      disp("THRO", THRO);
      disp("MTRLeft", MTRLeft);
      disp("MTRRight", MTRRight);
    }

    // GEAR 3 : 
    // Manual motor steering mode - simple forward
    else if (GEAR<DOWN_GEAR && GEAR>MIN_GEAR)
    {
      text("RF Gear 3 : Manual control mode");
      // calibration controller zero point - set to 0 if the values is similar to 0
      if (ELEV== 0)  {per_ELEV = 0;}
      if ((THRO < 988) && (THRO > 968))  {per_THRO = 0;}
      // Set motor duty based on left stick vertical value
      MTRLeft = int(MAX_PWM*per_ELEV);
      MTRRight = int(MAX_PWM*per_THRO);
      
    }

    // GEAR 2 : 
    // Stop mode
    else
    {
      text("RF Gear 2 : Stop mode");
      motor_stop();
      MTRLeft = 0;
      MTRRight = 0;
    }

    MTRLeft = GAIN_PWM_LR * MTRLeft;
    MTRRight = 1 * MTRRight;
    motor_turn(MTRLeft,MTRRight);
  }
}

//============================================================


void text(String value){  Serial.println(value);}
void disp(String name, float var){  String text = "";  text = name;  text = text + " : ";  Serial.print(text);  Serial.println(var,6);}
void CheckVoltage (){  voltage_value = analogRead( VoltagePin );}

void EncoderReading ( void *param )
{
  float t_dif_enc = 0.0;
  float time_enc = 0.0;
  float time_enc_last = 0.0;
  mtrA_encA_last = digitalRead(mtrA_encA_Pin);
  mtrA_encB_last = digitalRead(mtrA_encB_Pin);
  mtrB_encA_last = digitalRead(mtrB_encA_Pin);
  mtrB_encB_last = digitalRead(mtrB_encB_Pin);
  while (1)
  {
    time_enc = 0.001*millis();
    t_dif_enc = time_enc - time_enc_last;
    t_dif_enc = 0.012;
    // ENC Reading
    mtrA_encA_tmp = mtrA_encA;
    mtrA_encB_tmp = mtrA_encB;
    mtrB_encA_tmp = mtrB_encA;
    mtrB_encB_tmp = mtrB_encB;
    
    float mtrA_pos_tmp = mtrA_pos;
    float mtrB_pos_tmp = mtrB_pos;

    mtrA_encA = digitalRead(mtrA_encA_Pin);
    mtrA_encB = digitalRead(mtrA_encB_Pin);
    mtrB_encA = digitalRead(mtrB_encA_Pin);
    mtrB_encB = digitalRead(mtrB_encB_Pin);

    if ((mtrA_encA != mtrA_encA_last) || (mtrA_encB != mtrA_encB_last) || (mtrB_encA != mtrB_encA_last) || (mtrB_encB != mtrB_encB_last))
    {
      if (mtrA_encA != mtrA_encA_last) 
      {
        countA++;
        // CCW
        if (mtrA_encB != mtrA_encA) {mtrA_pos--; countA_minus ++;countA_minus_sum++;}
        //CW
        else                        {mtrA_pos++ ; countA_plus++; countA_plus_sum++;}
      }

      else if (mtrB_encA != mtrB_encA_last) 
      {
        countB++;
        // CW
        if (mtrB_encB != mtrB_encA) {mtrB_pos--;  countB_minus++;  countB_minus_sum++;}
        //CCW
        else                        {mtrB_pos++;  countB_plus++; countB_plus_sum++;}
      } 

      int error_compare_const = 10; 
      int rate_pm_A = countA_plus - countA_minus;
      int rate_pm_B = countB_plus - countB_minus;

      // 노이즈 필터링.
      float limit_motorA = FILTER_THRESHOLD_ACC / (2*PI) * COUNT_PER_ROUND_A ;
      float limit_motorB = FILTER_THRESHOLD_ACC / (2*PI) * COUNT_PER_ROUND_B ;
      float d_mtrA = (mtrA_pos - mtrA_pos_last)/t_dif_enc;
      float d_mtrB = (mtrB_pos - mtrB_pos_last)/t_dif_enc;
      if (( d_mtrA > limit_motorA) || (d_mtrA < -limit_motorA) || (d_mtrB > limit_motorB) || (d_mtrB < -limit_motorB) )
      {
        mtrA_pos = mtrA_pos_tmp;
        mtrB_pos = mtrB_pos_tmp;
        
        mtrA_encA = mtrA_encA_tmp;
        mtrA_encB = mtrA_encB_tmp;
        mtrB_encA = mtrB_encA_tmp;
        mtrB_encB = mtrB_encB_tmp;
      }
      // ENC Remain
      mtrA_encA_last = mtrA_encA;
      mtrA_encB_last = mtrA_encB;
      mtrB_encA_last = mtrB_encA;
      mtrB_encB_last = mtrB_encB;
      mtrA_pos_last = mtrA_pos;
      mtrB_pos_last = mtrB_pos;
      
    }
    time_enc_last = time_enc;
  }
}
void tx_to_rpi()
{
  //Encoder to RPI
  time_to_rpi = 0.001*millis();
  String msg_to_rpi = "SRT";
  msg_to_rpi = msg_to_rpi + ",";
  msg_to_rpi = msg_to_rpi + String(mtrA_pos/COUNT_PER_ROUND_A*2*PI); // [rad]
  msg_to_rpi = msg_to_rpi + ",";
  msg_to_rpi = msg_to_rpi + String(mtrB_pos/COUNT_PER_ROUND_B*2*PI); // [rad]
  msg_to_rpi = msg_to_rpi + ",";
  msg_to_rpi = msg_to_rpi + String(time_to_rpi);
  msg_to_rpi = msg_to_rpi + ",";
  msg_to_rpi = msg_to_rpi + String(mtrA_pos);
  msg_to_rpi = msg_to_rpi + ",";
  msg_to_rpi = msg_to_rpi + String(mtrB_pos);
  msg_to_rpi = msg_to_rpi + ",";
  msg_to_rpi = msg_to_rpi + String(MTRLeft);
  msg_to_rpi = msg_to_rpi + ",";
  msg_to_rpi = msg_to_rpi + String(MTRRight);
  msg_to_rpi = msg_to_rpi + ",";
  msg_to_rpi = msg_to_rpi + String(mtrA_pos_desired);
  msg_to_rpi = msg_to_rpi + ",";
  msg_to_rpi = msg_to_rpi + String(mtrB_pos_desired);
  msg_to_rpi = msg_to_rpi + ",";
  msg_to_rpi = msg_to_rpi + String(e_inc_left);
  msg_to_rpi = msg_to_rpi + ",";
  msg_to_rpi = msg_to_rpi + String(e_inc_right);
  msg_to_rpi = msg_to_rpi + ",";
  msg_to_rpi = msg_to_rpi + String(voltage_value);
  msg_to_rpi = msg_to_rpi + ",";  
  msg_to_rpi = msg_to_rpi + "END";
  Serial2.println (msg_to_rpi);
}
void rx_from_rpi()
{
  //RPI to ESP
  String msg_from_rpi = Serial2.readStringUntil('D');
  msg_rpi_start_checker = "SRT";

  if (msg_from_rpi.startsWith(msg_rpi_start_checker))
  {
    time_from_rpi = 0.001*millis();
    float t_dif = 0;
    t_dif = time_from_rpi - time_from_rpi_last;

    if (t_dif < TIMEOUT)
    {
      dt = t_dif;
      // find the comma position
      int term1 = msg_from_rpi.indexOf(",");         // SRT
      int term2 = msg_from_rpi.indexOf(",",term1+1); // w_left
      int term3 = msg_from_rpi.indexOf(",",term2+1); // w_right
      int term4 = msg_from_rpi.indexOf(",",term3+1); // P_GAIN_L
      int term5 = msg_from_rpi.indexOf(",",term4+1); // P_GAIN_R
      int term6 = msg_from_rpi.indexOf(",",term5+1); // D_GAIN_L
      int term7 = msg_from_rpi.indexOf(",",term6+1); // D_GAIN_R
      int term8 = msg_from_rpi.indexOf(",",term7+1); // COUNT_PER_ROUND_A
      int term9 = msg_from_rpi.indexOf(",",term8+1); // COUNT_PER_ROUND_B
      int term10 = msg_from_rpi.indexOf(",",term9+1); // INCPERSEC_TO_PWM_L
      int term11 = msg_from_rpi.indexOf(",",term10+1); // INCPERSEC_TO_PWM_R
      int term12 = msg_from_rpi.indexOf(",",term11+1); // mtrcontrol
      int term13 = msg_from_rpi.indexOf(",",term12+1); // PWM_L_manual
      int term14 = msg_from_rpi.indexOf(",",term13+1); // PWM_R_manual
      int term15 = msg_from_rpi.indexOf(",",term14+1); // mtr_advnc

      // Split the terms by using comma position
      String w_left_string   = msg_from_rpi.substring(term1+1, term2);
      String w_right_string  = msg_from_rpi.substring(term2+1, term3);
      String P_GAIN_L_string = msg_from_rpi.substring(term3+1, term4);
      String P_GAIN_R_string = msg_from_rpi.substring(term4+1, term5);
      String D_GAIN_L_string = msg_from_rpi.substring(term5+1, term6);
      String D_GAIN_R_string = msg_from_rpi.substring(term6+1, term7);
      String COUNT_PER_ROUND_A_string = msg_from_rpi.substring(term7+1, term8);
      String COUNT_PER_ROUND_B_string = msg_from_rpi.substring(term8+1, term9);
      String INCPERSEC_TO_PWM_L_string = msg_from_rpi.substring(term9+1, term10);
      String INCPERSEC_TO_PWM_R_string = msg_from_rpi.substring(term10+1, term11);
      String mtr_mode_string = msg_from_rpi.substring(term11+1, term12);
      String PWM_L_manual_string = msg_from_rpi.substring(term12+1, term13);
      String PWM_R_manual_string = msg_from_rpi.substring(term13+1, term14);
      String mtr_advnc_string = msg_from_rpi.substring(term14+1, term15);

      // String to float
      w_left   = w_left_string.toFloat();
      w_right  = w_right_string.toFloat();
      P_GAIN_L = P_GAIN_L_string.toFloat();
      P_GAIN_R = P_GAIN_R_string.toFloat();
      D_GAIN_L = D_GAIN_L_string.toFloat();
      D_GAIN_R = D_GAIN_R_string.toFloat();
      COUNT_PER_ROUND_A = COUNT_PER_ROUND_A_string.toFloat();
      COUNT_PER_ROUND_B = COUNT_PER_ROUND_B_string.toFloat();
      INCPERSEC_TO_PWM_L = INCPERSEC_TO_PWM_L_string.toFloat();
      INCPERSEC_TO_PWM_R = INCPERSEC_TO_PWM_R_string.toFloat();
      mtr_mode = mtr_mode_string.toFloat();
      PWM_L_manual = PWM_L_manual_string.toFloat();
      PWM_R_manual = PWM_R_manual_string.toFloat();
      mtr_advnc = mtr_advnc_string.toFloat();
      
      if(w_left == 0 && w_right== 0)
      {
        mtrA_pos_desired = mtrA_pos;
        mtrB_pos_desired = mtrB_pos;        
      }

      mtrA_pos_desired = mtrA_pos_desired + w_left  / (2*PI) * COUNT_PER_ROUND_A * dt;
      mtrB_pos_desired = mtrB_pos_desired + w_right / (2*PI) * COUNT_PER_ROUND_B * dt;
    }
    else // timeout
    {
      Serial.println ("====time out====");
      mtrA_pos_desired = mtrA_pos;
      mtrB_pos_desired = mtrB_pos;
      w_left = 0;
      w_right = 0;
    }
    time_from_rpi_last = time_from_rpi;
  }
  else 
  {
    Serial.println ("Rpi connection lost");
    mtrA_pos_desired = mtrA_pos;
    mtrB_pos_desired = mtrB_pos;
    w_left = 0;
    w_right = 0;
    MTRLeft = 0;
    MTRRight = 0;
    }
}

void motor_turn(int speed_pwm_left, int speed_pwm_right)
{
  //input : -255 ~ 255

  //Deadzone
  if ( speed_pwm_left  > -DEADZONE_L && speed_pwm_left  < DEADZONE_L ) {  speed_pwm_left = 0;}
  if ( speed_pwm_right > -DEADZONE_R && speed_pwm_right < DEADZONE_R ) {  speed_pwm_right = 0;}

  //calibration over values
  if (speed_pwm_left>MAX_PWM_DUTY) {  speed_pwm_left = MAX_PWM_DUTY;}
  if (speed_pwm_left<-MAX_PWM_DUTY) {  speed_pwm_left = -MAX_PWM_DUTY;}
  if (speed_pwm_right>MAX_PWM_DUTY) {  speed_pwm_right = MAX_PWM_DUTY;}
  if (speed_pwm_right<-MAX_PWM_DUTY) {  speed_pwm_right = -MAX_PWM_DUTY;}

  //MTR left
  if (speed_pwm_left>0)
  {
    digitalWrite (EN_A, HIGH);
    analogWrite (IN1_A, abs(speed_pwm_left));
    analogWrite (IN2_A, 0);
  }
  else
  {
    digitalWrite (EN_A, HIGH);
    analogWrite (IN1_A, 0);
    analogWrite (IN2_A, abs(speed_pwm_left));
  }

  //MTR right
  if (speed_pwm_right>0)
  {
    digitalWrite (EN_B, HIGH);
    analogWrite (IN1_B, abs(speed_pwm_right));
    analogWrite (IN2_B, 0);
  }
  else
  {
    digitalWrite (EN_B, HIGH);
    analogWrite (IN1_B, 0);
    analogWrite (IN2_B, abs(speed_pwm_right));
  }
}

void motor_stop()
{
  //MTR left
  digitalWrite (EN_A, LOW);
  analogWrite (IN1_A, 0);
  analogWrite (IN2_A, 0);
  //MTR right
  digitalWrite (EN_B, LOW);
  analogWrite (IN1_B, 0);
  analogWrite (IN2_B, 0);
}


void RF_recieve_values()
{
  //Reading RF controller and calculate to percentage

  AUX1 = pulseIn(PIN_AUX1, HIGH,50000);
  RUDD = pulseIn(PIN_RUDD, HIGH,50000);
  ELEV = pulseIn(PIN_ELEV, HIGH,50000);
  AILE = pulseIn(PIN_AILE, HIGH,50000);
  THRO = pulseIn(PIN_THRO, HIGH,50000);

  per_RUDD = (RUDD-(MAX_RUDD+MIN_RUDD)/2)/((MAX_RUDD-MIN_RUDD)/2);
  per_ELEV = (ELEV-(MAX_ELEV+MIN_ELEV)/2)/((MAX_ELEV-MIN_ELEV)/2);
  per_AILE = (AILE-(MAX_AILE+MIN_AILE)/2)/((MAX_AILE-MIN_AILE)/2);
  per_THRO = (THRO-(MAX_THRO+MIN_THRO)/2)/((MAX_THRO-MIN_THRO)/2);
  float percent_zero = 0.2;

  // RF controller calibration
  if (abs(per_RUDD) < percent_zero)  {per_RUDD = 0;}
  if (abs(per_ELEV) < percent_zero)  {per_ELEV = 0;}
  if (abs(per_AILE) < percent_zero)  {per_AILE = 0;}
  if (abs(per_THRO) < percent_zero)  {per_THRO = 0;} 
  
  if (RUDD== 0.00)  {per_RUDD = 0;}
  if (ELEV== 0.00)  {per_ELEV = 0;}
  if (AILE== 0.00)  {per_AILE = 0;}
  if ((THRO < 988) && (THRO > 968))  {per_THRO = 0;}
}

void check_gear(float check_count)
{
  time2 = 0.001*millis();
  if ((time2-time1)>check_count)
  {
    GEAR = pulseIn(PIN_GEAR, HIGH,50000);
    dtime = time2 - time1;
    time1 = 0.001*millis();
    disp("Gear : ",GEAR);
  }
}

void motor_control_pos()
{
  e_inc_left  = (mtrA_pos_desired - mtrA_pos);
  e_inc_right = (mtrB_pos_desired - mtrB_pos);

  disp("e_inc_left",e_inc_left);
  
  e_term_L = INCPERSEC_TO_PWM_L * e_inc_left;
  e_term_R = INCPERSEC_TO_PWM_R * e_inc_right;
  disp("e_term_L",e_term_L);

  e_term_L_d = e_term_L - e_term_L_prev;
  e_term_R_d = e_term_R - e_term_R_prev;
  disp("e_term_L_d",e_term_L_d);
  
  if(mtr_mode == 0.0)
  {
    MTRLeft  = INCPERSEC_TO_PWM_L * w_left  / (2*PI) * COUNT_PER_ROUND_A;
    MTRRight  = INCPERSEC_TO_PWM_R * w_right  / (2*PI) * COUNT_PER_ROUND_B;
  }
  
  if(mtr_mode == 1.0)
  {
    MTRLeft  = P_GAIN_L * e_term_L + D_GAIN_L * e_term_L_d / dt;
    MTRRight = P_GAIN_R * e_term_R + D_GAIN_R * e_term_R_d / dt;
  }
  
  if(mtr_mode == 2.0)
  {
    MTRLeft  = PWM_L_manual;
    MTRRight  = PWM_R_manual;
  }
  disp("mtr_mode",mtr_mode);
  disp("MTRLeft",MTRLeft);
    
  Serial.print ("e_term_L : ");
  Serial.println (e_term_L);
  Serial.print (",");
  Serial.print ("e_term_R : ");
  Serial.println (e_term_R);
  Serial.println ("");
  
  e_term_L_prev = e_term_L;
  e_term_R_prev = e_term_R;

  motor_turn(MTRLeft,MTRRight);
}
