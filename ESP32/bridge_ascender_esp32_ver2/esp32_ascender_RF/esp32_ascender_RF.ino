#include <CAN.h>

#define byteRead(x,n) ( * ( (unsigned char*)(&x) + n) )

// [RF]
int PIN_THRO = 36; //36//vp// controller 3
int PIN_AILE = 39; //39//vn// conttoller 4
int PIN_ELEV = 34; // controller 1
int PIN_RUDD = 35; // controller 2 
int PIN_GEAR = 32; // gear
int PIN_AUX1 = 33; // aux1

// int PIN_THRO = 33; //36//vp// controller 3
// int PIN_AILE = 32; //39//vn// conttoller 4
// int PIN_ELEV = 35; // controller 1
// int PIN_RUDD = 34; // controller 2 
// int PIN_GEAR = 39; // gear
// int PIN_AUX1 = 36; // aux1

float MAX_GEAR = 2000;
float UP_GEAR = 1700;
float DOWN_GEAR = 1300;
float MIN_GEAR = 1000;
float MAX_THRO = 1950;
float UP_THRO = 1800;
float DOWN_THRO = 1200;
float MIN_THRO = 1050;
float MAX_AILE = 1897;
float UP_AILE = 1700;
float DOWN_AILE = 1300;
float MIN_AILE = 1098;
float MAX_ELEV = 1900;
float UP_ELEV = 1700;
float DOWN_ELEV = 1200;
float MIN_ELEV = 1050;
float MAX_RUDD = 1950;
float UP_RUDD = 1700;
float DOWN_RUDD = 1250;
float MIN_RUDD = 1050;

unsigned long THRO = 0.0; 
unsigned long AILE = 0.0; 
unsigned long ELEV = 0.0;
unsigned long RUDD = 0.0;
unsigned long GEAR = 0.0;
unsigned long AUX1 = 0.0;

int Thro_1 = 0;
int Aile_1 = 0;
int Elev_1 = 0;
int Rudd_1 = 0;

// [CAN]
// define CAN struct
struct CanData {
  uint32_t Id;
  int length;
  int Extened;
  int RTR;
  byte Data[8];
};
// pin select
int rx = 18; //CRX
int tx = 19; //CTX
// parameter for CAN
byte candata[8];
CanData MotorCan1, MotorCan2;
CanData SensorCan1;


// [Motor]
float motor_position[2];
float rpm[2];

// [Serial2]
//define what pin rx2 tx2 is going to be
#define rx2 16 //RX2                                       
#define tx2 17 //TX2
unsigned long timeout = 50; //50ms


// [MUX]
//esp32 pins to control pin S1 S2 S3
int s1 = 21;                                           
int s2 = 22;                                           
int s3 = 23;                                           
//what port to open
int port = 0;

// [EBIMU]
// parameter for EMimu
float euler[3];
float quaternion[4];

void open_port();
void serial2Flush();
void rx_from_imu();
void rx_from_imu_quaternion();
void RF_recieve_values();
void Motor_Control_CAN(); 
void motor_control_position_can(CanData CAN, float motor_position_degree);
void wheel_motor_can(CanData CAN, float rpm);
void Motor_Control_Velocity_CAN1(uint32_t ID, int rpm);
void Motor_Control_Velocity_CAN2(uint32_t ID, int rpm); 
void tx_CAN();
void EMImu_CAN();
void EBImu_Quaternion_CAN();

// [Multitask]
TaskHandle_t Task1;

void setup() {

  xTaskCreatePinnedToCore(
    SENDCAN,         // task function
    "Task1",           // task name
    10000,             // stack size (word)
    NULL,              // task parameters
    0,                 // task priority
    &Task1,            // task handle
    0);                // exe core

  //UART Expander S1 S2 S3. 000 -> Port1. 기본적으로 1인상태에서시작.
  pinMode(s1,OUTPUT);
  pinMode(s2,OUTPUT);
  pinMode(s3,OUTPUT);
  open_port();

  Serial.begin(115200);
  // Serial.setTimeout(timeout);
  Serial2.begin(115200, SERIAL_8N1, rx2, tx2);
  Serial2.setTimeout(timeout);

  CAN.setPins(rx, tx);
  CAN.setTimeout(timeout);

  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  // RF receiver
  // ESP32 는 pinmode 인풋지정해줘야한다. 아두이노는 괜춘
  pinMode(PIN_THRO, INPUT); // Thro
  pinMode(PIN_AILE, INPUT); // AILE
  pinMode(PIN_ELEV, INPUT); // ELEV
  pinMode(PIN_RUDD, INPUT); // Rudd
  pinMode(PIN_GEAR, INPUT); // Gear
  pinMode(PIN_AUX1, INPUT); // Aux1

  // CAN setting
  MotorCan1.Id = 0x321;
  MotorCan2.Id = 0x322;
  MotorCan1.length = 8;
  MotorCan2.length = 8;
  SensorCan1.Id = 0x2871;
  SensorCan1.length = 8;

}
void loop() {
//  // EBIMU
//  rx_from_imu_quaternion();

  // RF
  RF_recieve_values();

  vTaskDelay(pdMS_TO_TICKS(1));

  // GEAR = pulseIn(PIN_GEAR, HIGH, 50000);
  // Serial.println(GEAR);
  // vTaskDelay(pdMS_TO_TICKS(1));
  // GEAR = 1900;
// RF off - ROS2 system
  if (!(GEAR > MIN_GEAR) && (GEAR < MAX_GEAR)) {
    // Serial.println("ROS mode");

    //rx_CAN_from_ROS();

    //tx_CAN_to_ROS();

    // tx_CAN_sensor();
  }
  // RF mode
  else {
    // Serial.println("RF mode");

    // GEAR 0 : velocity mode 2
    if (GEAR > UP_GEAR && GEAR < MAX_GEAR) {
      // Serial.println("Gear 0");
      // RF_recieve_values();

      if (ELEV > UP_ELEV && ELEV < MAX_ELEV) {
        // 1 up
        rpm[0] = 10;
        // rpm2 = 10;
        
      }
      else if (ELEV < DOWN_ELEV && ELEV > MIN_ELEV){
        // 1 down
        rpm[0] = -10;
        // rpm2 = -10;

      }
      else {
        // 1 stop
        rpm[0] = 0;
        // rpm2 = 0;
        
      }

      if (THRO > UP_THRO && THRO < MAX_THRO) {
        // 1 up
        // rpm1 = 10;
        rpm[1] = 10;
        
      }
      else if (THRO < DOWN_THRO && THRO > MIN_THRO){
        // 1 down
        // rpm1 = -10;
        rpm[1] = -10;

      }
      else {
        // rpm1 = 0;
        rpm[1] = 0;
        
      }

      // Motor_Control_Velocity_CAN1(301, rpm1);
      // Motor_Control_Velocity_CAN2(302, rpm2);

      
    }
    // GEAR 2 : stop mode
    else if (GEAR < DOWN_GEAR && GEAR > MIN_GEAR) {
      // Serial.println("Gear 2");

      // CAN_send(StopCanData1);
      // CAN_send(StopCanData2);
      //  Motor_Control_CAN1(601);
      // Motor_Control_CAN2(602);

    }

    // GEAR 1 : velocity mode
    else {
      // Serial.println("Gear 1");
      // RF_recieve_values();

      if (ELEV > UP_ELEV && ELEV < MAX_ELEV) {
        // up
        rpm[0] = 10;
        rpm[1] = 10;
        
      }
      else if (ELEV < DOWN_ELEV && ELEV > MIN_ELEV){
        // down
        rpm[0] = -10;
        rpm[1] = -10;

      }
      else {
        if (RUDD > UP_RUDD && RUDD < MAX_RUDD) {
        // up
          rpm[0] = 10;
          rpm[1] = -10;
        }
        else if (RUDD < DOWN_RUDD && RUDD > MIN_RUDD){
        // down
          rpm[0] = -10;
          rpm[1] = 10;
        }
        else {
        // stop
      
          rpm[0] = 0;
          rpm[1] = 0;
        }
      }

      // Motor_Control_Velocity_CAN1(301, rpm1);
      // Motor_Control_Velocity_CAN2(302, rpm2);
    }

  }


}

//====================================================================================
// Functions
void open_port() {                                  //this function controls what UART port is opened.

  if (port < 1 || port > 9)port = 1;                //if the value of the port is within range (1-8) then open that port. If it’s not in range set it to port 0

  digitalWrite(s1, bitRead(port-1, 0));               //Here we have two commands combined into one.
  digitalWrite(s2, bitRead(port-1, 1));               //The digitalWrite command sets a pin to 1/0 (high or low)
  digitalWrite(s3, bitRead(port-1, 2));               //The bitRead command tells us what the bit value is for a specific bit location of a number
                                          
  // delay(2);                                         //this is needed to make sure the channel switching event has completed
}

void serial2Flush(){
  while(Serial2.available() > 0) {
    char t = Serial2.read();
  }
}

void rx_from_imu() {
  if (Serial2.available()>0) {
      String msg_from_imu = Serial2.readStringUntil(0x0a);
    if (msg_from_imu.startsWith("*")) {
      int term1 = msg_from_imu.indexOf("*");               // *
      int term2 = msg_from_imu.indexOf(",", term1 + 1);    // pitch
      int term3 = msg_from_imu.indexOf(",", term2 + 1);    // roll
      int term4 = msg_from_imu.indexOf(",", term3 + 1);    // yaw
  
      String pitch_string = msg_from_imu.substring(term1 + 1, term2);
      String roll_string = msg_from_imu.substring(term2 + 1, term3);
      String yaw_string = msg_from_imu.substring(term3 + 1, term4);
  
      euler[0] = pitch_string.toFloat();
      euler[1] = roll_string.toFloat();
      euler[2] = yaw_string.toFloat();
  
//      Serial.print("Pitch :"); Serial.print(euler[0]);   Serial.print(" ");
//      Serial.print("Roll :"); Serial.print(euler[1]);   Serial.print(" ");
//      Serial.print("Yaw :"); Serial.print(euler[2]);   Serial.println(" ");
    }
    else {
      Serial.println("====== not imu data ======");
    }
  }
  else {
    Serial.println("==== No data Received from imu ====");
  }

}

void rx_from_imu_quaternion() {
  if (Serial2.available()>0) {
      String msg_from_imu = Serial2.readStringUntil(0x0a);
//      Serial.println(msg_from_imu);
    if (msg_from_imu.startsWith("*")) {
      int term1 = msg_from_imu.indexOf("*");               // *
      int term2 = msg_from_imu.indexOf(",", term1 + 1);    // z
      int term3 = msg_from_imu.indexOf(",", term2 + 1);    // y
      int term4 = msg_from_imu.indexOf(",", term3 + 1);    // x
      int term5 = msg_from_imu.indexOf(",", term4 + 1);    // w
  
      String z_string = msg_from_imu.substring(term1 + 1, term2);
      String y_string = msg_from_imu.substring(term2 + 1, term3);
      String x_string = msg_from_imu.substring(term3 + 1, term4);
      String w_string = msg_from_imu.substring(term4 + 1, term5);
//      String CR_string = msg_from_imu.substring(term5 + 1, term5 + 2);

      if (!(-1<=x_string.toFloat() && x_string.toFloat() <= 1)){
//        Serial.println("no");
        return;
      }
      if (!(-1<=y_string.toFloat() && y_string.toFloat() <= 1)){
//        Serial.println("no");
        return;
      }
      if (!(-1<=z_string.toFloat() && z_string.toFloat() <= 1)){
//        Serial.println("no");
        return;
      }
      if (!(-1<=w_string.toFloat() && w_string.toFloat() <= 1)){
//        Serial.println("no");
        return;
      }
 
      quaternion[0] = x_string.toFloat();
      quaternion[1] = y_string.toFloat();
      quaternion[2] = z_string.toFloat();
      quaternion[3] = w_string.toFloat();
  
//      Serial.print("x :"); Serial.print(quaternion[0]);   Serial.print(" ");
//      Serial.print("y :"); Serial.print(quaternion[1]);   Serial.print(" ");
//      Serial.print("z :"); Serial.print(quaternion[2]);   Serial.print(" ");
//      Serial.print("w :"); Serial.print(quaternion[3]);   Serial.println(" ");
    }
    else {
//      Serial.println("====== not imu data ======");
    }
  }
  else {
//    Serial.println("==== No data Received from imu ====");
  }

}

CanData CAN_receive(uint32_t MaskId){
  CanData ReceiveCanData;
  int packetSize = CAN.parsePacket();
  if (packetSize){ // is there CAN data received?
    uint32_t Id_identifier = CAN.packetId();

    // Serial.print(Id_identifier); Serial.print(" ? ");
    // Serial.println(MaskId);
    // Serial.println(Id_identifier == MaskId);

    if (Id_identifier == MaskId) { // has CAN Data right ID? 
      // Serial.print("Received ");
      ReceiveCanData.length = packetSize;
      ReceiveCanData.Id = Id_identifier;
    
      if(CAN.packetExtended()){
        ReceiveCanData.Extened = 1;
        // Serial.print("extended ");
      }
      else{
        ReceiveCanData.Extened = 0;
      }

      if (CAN.packetRtr()) {
        ReceiveCanData.RTR = 1;
        // Serial.print("RTR ");
      }
      else{
        ReceiveCanData.RTR = 0;
      }
      // Serial.print("packet with id 0x");
      // Serial.print(CAN.packetId(), HEX);
      // Serial.print("  ");

      int i = 0;

      while (CAN.available()) {
    
        //Serial.println(CAN.read(), HEX);
        ReceiveCanData.Data[i] = CAN.read();
        i = i+1;
      }

      // Serial.print(ReceiveCanData.Data[0], HEX);
      // Serial.print("  ");
      // Serial.print(ReceiveCanData.Data[1], HEX);
      // Serial.print("  ");
      // Serial.print(ReceiveCanData.Data[2], HEX);
      // Serial.print("  ");
      // Serial.print(ReceiveCanData.Data[3], HEX);
      // Serial.print("  ");
      // Serial.print(ReceiveCanData.Data[4], HEX);
      // Serial.print("  ");
      // Serial.print(ReceiveCanData.Data[5], HEX);
      // Serial.print("  ");
      // Serial.print(ReceiveCanData.Data[6], HEX);
      // Serial.print("  ");
      // Serial.println(ReceiveCanData.Data[7], HEX);

    }
    else {
      // Serial.println("=== Wrong ID ===="); 
      //ReceiveCanData.Extened = 0;
      //ReceiveCanData.RTR = 1;
      //for(int i = 0; i < ReceiveCanData.length ; i++) {
      //  ReceiveCanData.Data[i] = byte(0);
      //}
    }
  }
  else {
    // Serial.println("=== No CAN Received ====");
  }
  return ReceiveCanData;
}

void CAN_send(CanData InputCanData){
  CAN.beginPacket(InputCanData.Id);
  CAN.write(InputCanData.Data, InputCanData.length);
  CAN.endPacket();
}

void CAN_EXTsend(CanData InputCanData){
  CAN.beginExtendedPacket(InputCanData.Id);
  CAN.write(InputCanData.Data, InputCanData.length);
  CAN.endPacket();
}


void RF_recieve_values() { //44ms
  ELEV = 0;
  RUDD = 0;
  THRO = 0;
  AILE = 0;
  THRO = pulseIn(PIN_THRO, HIGH, 50000); //Elev 펄스를 읽습니다
  // Thro_1 = map(THRO, MIN_THRO, MAX_THRO, -255, 255); //읽힌 펄스의 값을 PWM범위로 다시 바꾸어줍니다.  1900 1102

  AILE = pulseIn(PIN_AILE, HIGH, 50000); //Elev 펄스를 읽습니다
  // Aile_1 = map(AILE, MIN_AILE, MAX_AILE, -255, 255); //읽힌 펄스의 값을 PWM범위로 다시 바꾸어줍니다. 1900 1105
  
  ELEV = pulseIn(PIN_ELEV, HIGH, 50000); //Elev 펄스를 읽습니다
  // Elev_1 = map(ELEV, MIN_ELEV, MAX_ELEV, -255, 255); //읽힌 펄스의 값을 PWM범위로 다시 바꾸어줍니다. 1891 1116
  
  RUDD = pulseIn(PIN_RUDD, HIGH, 50000); //Elev 펄스를 읽습니다
  // Rudd_1 = map(RUDD, MIN_RUDD, MAX_RUDD, -255, 255); //읽힌 펄스의 값을 PWM범위로 다시 바꾸어줍니다. 1895 1105
  
  GEAR = pulseIn(PIN_GEAR, HIGH, 50000); 
  // AUX1 = pulseIn(PIN_AUX1, HIGH, 10000); 

//  Serial.print(THRO);
//  Serial.print(" ");
//  Serial.print(AILE);
//  Serial.print(" ");
//  Serial.print(ELEV);
//  Serial.print(" ");
//  Serial.print(RUDD);
//  Serial.print(" ");
//  Serial.println(GEAR);
  // Serial.print(" ");
  // Serial.println(AUX1);
    
  // Serial.print(Thro_1);
  // Serial.print(" ");
  // Serial.print(Aile_1);
  // Serial.print(" ");
  // Serial.print(Elev_1);
  // Serial.print(" ");
  // Serial.println(Rudd_1);
}

void Motor_Control_CAN() {
  float motor_position_degree = map(Thro_1, -255, 255, 0, 180);
  // Serial.print("Desired Degree is : ");
  // Serial.println(motor_position_degree);
  if (motor_position_degree<0) { //saifty issue
    Serial.println("error");
    return;
  }

  //float motor_position_degree = 0.0;
  float motor_position = motor_position_degree * 10000;
  int motor_position_int32 = int(motor_position);
  int motor_position_int32_HIGH = motor_position_int32 >> 16;
  int motor_position_int32_LOW = motor_position_int32-(motor_position_int32_HIGH<<16);
  int motor_position_int32_HIGH_HIGH = motor_position_int32_HIGH>>8;
  int motor_position_int32_HIGH_LOW = motor_position_int32_HIGH-(motor_position_int32_HIGH_HIGH<<8);
  int motor_position_int32_LOW_HIGH = motor_position_int32_LOW>>8;
  int motor_position_int32_LOW_LOW = motor_position_int32_LOW-(motor_position_int32_LOW_HIGH<<8);

  float motor_position_speed = 3*64*21;
  int16_t motor_position_speed_int16 = int16_t(motor_position_speed);
  int16_t motor_position_speed_int16_HIGH = motor_position_speed_int16>>8;
  int16_t motor_position_speed_int16_LOW = motor_position_speed_int16-(motor_position_speed_int16_HIGH<<8);

  float motor_position_acc = 10;
  int16_t motor_position_acc_int16 = int16_t(motor_position_acc);
  int16_t motor_position_acc_int16_HIGH = motor_position_acc_int16>>8;
  int16_t motor_position_acc_int16_LOW = motor_position_acc_int16-(motor_position_acc_int16_HIGH<<8);

  candata[0] = byte(motor_position_int32_HIGH_HIGH);
  candata[1] = byte(motor_position_int32_HIGH_LOW);
  candata[2] = byte(motor_position_int32_LOW_HIGH);
  candata[3] = byte(motor_position_int32_LOW_LOW);
  candata[4] = byte(motor_position_speed_int16_HIGH);
  candata[5] = byte(motor_position_speed_int16_LOW);
  candata[6] = byte(motor_position_acc_int16_HIGH);
  candata[7] = byte(motor_position_acc_int16_LOW);

  // Serial.print(candata[0], HEX);
  // Serial.print(" ");
  // Serial.print(candata[1], HEX);
  // Serial.print(" ");
  // Serial.print(candata[2], HEX);
  // Serial.print(" ");
  // Serial.print(candata[3], HEX);
  // Serial.print(" ");
  // Serial.print(candata[4], HEX);
  // Serial.print(" ");
  // Serial.print(candata[5], HEX);
  // Serial.print(" ");
  // Serial.print(candata[6], HEX);
  // Serial.print(" ");
  // Serial.print(candata[7], HEX);
  // Serial.println(" ");
}

void motor_control_position_can(CanData CAN, float motor_position_degree) {

  float motor_position = motor_position_degree * 10000;
  int motor_position_int32 = int(motor_position);
  int motor_position_int32_HIGH = motor_position_int32 >> 16;
  int motor_position_int32_LOW = motor_position_int32-(motor_position_int32_HIGH<<16);
  int motor_position_int32_HIGH_HIGH = motor_position_int32_HIGH>>8;
  int motor_position_int32_HIGH_LOW = motor_position_int32_HIGH-(motor_position_int32_HIGH_HIGH<<8);
  int motor_position_int32_LOW_HIGH = motor_position_int32_LOW>>8;
  int motor_position_int32_LOW_LOW = motor_position_int32_LOW-(motor_position_int32_LOW_HIGH<<8);

  float motor_position_speed = 3*64*21;
  int16_t motor_position_speed_int16 = int16_t(motor_position_speed);
  int16_t motor_position_speed_int16_HIGH = motor_position_speed_int16>>8;
  int16_t motor_position_speed_int16_LOW = motor_position_speed_int16-(motor_position_speed_int16_HIGH<<8);

  float motor_position_acc = 10;
  int16_t motor_position_acc_int16 = int16_t(motor_position_acc);
  int16_t motor_position_acc_int16_HIGH = motor_position_acc_int16>>8;
  int16_t motor_position_acc_int16_LOW = motor_position_acc_int16-(motor_position_acc_int16_HIGH<<8);

  CAN.Data[0] = byte(motor_position_int32_HIGH_HIGH);
  CAN.Data[1] = byte(motor_position_int32_HIGH_LOW);
  CAN.Data[2] = byte(motor_position_int32_LOW_HIGH);
  CAN.Data[3] = byte(motor_position_int32_LOW_LOW);
  CAN.Data[4] = byte(motor_position_speed_int16_HIGH);
  CAN.Data[5] = byte(motor_position_speed_int16_LOW);
  CAN.Data[6] = byte(motor_position_acc_int16_HIGH);
  CAN.Data[7] = byte(motor_position_acc_int16_LOW);

  CAN_send(CAN);
}

void wheel_motor_can(CanData CAN, float rpm) {
  float motor_velocity = rpm * 64*21;
  int motor_velocity_int32 = int(motor_velocity);

  CAN.Data[0] = byteRead(motor_velocity_int32,3);
  CAN.Data[1] = byteRead(motor_velocity_int32,2);
  CAN.Data[2] = byteRead(motor_velocity_int32,1);
  CAN.Data[3] = byteRead(motor_velocity_int32,0);
  CAN.Data[4] = byte(0);
  CAN.Data[5] = byte(0);
  CAN.Data[6] = byte(0);
  CAN.Data[7] = byte(0);

  // float motor_position = motor_position_degree * 10000;
  // int motor_position_int32 = int(motor_position);
  // int motor_position_int32_HIGH = motor_position_int32 >> 16;
  // int motor_position_int32_LOW = motor_position_int32-(motor_position_int32_HIGH<<16);
  // int motor_position_int32_HIGH_HIGH = motor_position_int32_HIGH>>8;
  // int motor_position_int32_HIGH_LOW = motor_position_int32_HIGH-(motor_position_int32_HIGH_HIGH<<8);
  // int motor_position_int32_LOW_HIGH = motor_position_int32_LOW>>8;
  // int motor_position_int32_LOW_LOW = motor_position_int32_LOW-(motor_position_int32_LOW_HIGH<<8);

  // float motor_position_speed = 3*64*21;
  // int16_t motor_position_speed_int16 = int16_t(motor_position_speed);
  // int16_t motor_position_speed_int16_HIGH = motor_position_speed_int16>>8;
  // int16_t motor_position_speed_int16_LOW = motor_position_speed_int16-(motor_position_speed_int16_HIGH<<8);

  // float motor_position_acc = 10;
  // int16_t motor_position_acc_int16 = int16_t(motor_position_acc);
  // int16_t motor_position_acc_int16_HIGH = motor_position_acc_int16>>8;
  // int16_t motor_position_acc_int16_LOW = motor_position_acc_int16-(motor_position_acc_int16_HIGH<<8);

  // CAN.Data[0] = byte(motor_position_int32_HIGH_HIGH);
  // CAN.Data[1] = byte(motor_position_int32_HIGH_LOW);
  // CAN.Data[2] = byte(motor_position_int32_LOW_HIGH);
  // CAN.Data[3] = byte(motor_position_int32_LOW_LOW);
  // CAN.Data[4] = byte(motor_position_speed_int16_HIGH);
  // CAN.Data[5] = byte(motor_position_speed_int16_LOW);
  // CAN.Data[6] = byte(motor_position_acc_int16_HIGH);
  // CAN.Data[7] = byte(motor_position_acc_int16_LOW);

  CAN_send(CAN);

}

void Motor_Control_Velocity_CAN1(uint32_t ID, int rpm) { 

  //float motor_position_degree = 0.0;
  float motor_velocity = - rpm * 64*21;
  int motor_velocity_int32 = int(motor_velocity);

  // int motor_velocity_int32_HIGH = motor_velocity_int32 >> 16;
  // int motor_velocity_int32_LOW = motor_velocity_int32-(motor_velocity_int32_HIGH<<16);
  // int motor_velocity_int32_HIGH_HIGH = motor_velocity_int32_HIGH>>8;
  // int motor_velocity_int32_HIGH_LOW = motor_velocity_int32_HIGH-(motor_velocity_int32_HIGH_HIGH<<8);
  // int motor_velocity_int32_LOW_HIGH = motor_velocity_int32_LOW>>8;
  // int motor_velocity_int32_LOW_LOW = motor_velocity_int32_LOW-(motor_velocity_int32_LOW_HIGH<<8);

  // MotorCan1.Data[0] = byte(motor_velocity_int32_HIGH_HIGH);
  // MotorCan1.Data[1] = byte(motor_velocity_int32_HIGH_LOW);
  // MotorCan1.Data[2] = byte(motor_velocity_int32_LOW_HIGH);
  // MotorCan1.Data[3] = byte(motor_velocity_int32_LOW_LOW);
  // MotorCan1.Data[4] = byte(0);
  // MotorCan1.Data[5] = byte(0);
  // MotorCan1.Data[6] = byte(0);
  // MotorCan1.Data[7] = byte(0);

  MotorCan1.Data[0] = byteRead(motor_velocity_int32,3);
  MotorCan1.Data[1] = byteRead(motor_velocity_int32,2);
  MotorCan1.Data[2] = byteRead(motor_velocity_int32,1);
  MotorCan1.Data[3] = byteRead(motor_velocity_int32,0);
  MotorCan1.Data[4] = byte(0);
  MotorCan1.Data[5] = byte(0);
  MotorCan1.Data[6] = byte(0);
  MotorCan1.Data[7] = byte(0);

  CAN_send(MotorCan1);

  // CanData SendCanData;
  // SendCanData.Id = uint32_t(strtol(String(ID).c_str(), NULL, 16));
  // SendCanData.length = 8;
  // for (int i = 0; i <= 7; i++) {
  //   SendCanData.Data[i] = candata[i];
  // }
  // Serial.print("....");
  // CAN_send(SendCanData);
  // Serial.println(" done"); 

}

void Motor_Control_Velocity_CAN2(uint32_t ID, int rpm) { 

  //float motor_position_degree = 0.0;
  float motor_velocity = rpm * 64*21;
  int motor_velocity_int32 = int(motor_velocity);
  // int motor_velocity_int32_HIGH = motor_velocity_int32 >> 16;
  // int motor_velocity_int32_LOW = motor_velocity_int32-(motor_velocity_int32_HIGH<<16);
  // int motor_velocity_int32_HIGH_HIGH = motor_velocity_int32_HIGH>>8;
  // int motor_velocity_int32_HIGH_LOW = motor_velocity_int32_HIGH-(motor_velocity_int32_HIGH_HIGH<<8);
  // int motor_velocity_int32_LOW_HIGH = motor_velocity_int32_LOW>>8;
  // int motor_velocity_int32_LOW_LOW = motor_velocity_int32_LOW-(motor_velocity_int32_LOW_HIGH<<8);

  // MotorCan2.Data[0] = byte(motor_velocity_int32_HIGH_HIGH);
  // MotorCan2.Data[1] = byte(motor_velocity_int32_HIGH_LOW);
  // MotorCan2.Data[2] = byte(motor_velocity_int32_LOW_HIGH);
  // MotorCan2.Data[3] = byte(motor_velocity_int32_LOW_LOW);
  // MotorCan2.Data[4] = byte(0);
  // MotorCan2.Data[5] = byte(0);
  // MotorCan2.Data[6] = byte(0);
  // MotorCan2.Data[7] = byte(0);

  MotorCan2.Data[0] = byteRead(motor_velocity_int32,3);
  MotorCan2.Data[1] = byteRead(motor_velocity_int32,2);
  MotorCan2.Data[2] = byteRead(motor_velocity_int32,1);
  MotorCan2.Data[3] = byteRead(motor_velocity_int32,0);
  MotorCan2.Data[4] = byte(0);
  MotorCan2.Data[5] = byte(0);
  MotorCan2.Data[6] = byte(0);
  MotorCan2.Data[7] = byte(0);

  CAN_send(MotorCan2);

  // CanData SendCanData;
  // SendCanData.Id = uint32_t(strtol(String(ID).c_str(), NULL, 16));
  // SendCanData.length = 8;
  // for (int i = 0; i <= 7; i++) {
  //   SendCanData.Data[i] = candata[i];
  // }
  // Serial.print("....");
  // CAN_send(SendCanData);
  // Serial.println(" done"); 
  // StopCanData1 = SendCanData;

}

// Task1 function
void SENDCAN(void * parameter) {
  for(;;) {
    tx_CAN();
    // vTaskDelay(0.001);
  }
}
