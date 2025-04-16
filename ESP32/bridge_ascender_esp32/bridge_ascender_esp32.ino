#include <CAN.h>
#include <HX711.h>

#define byteRead(x,n) ( * ( (unsigned char*)(&x) + n) )

// [RF]
int PIN_THRO = 33; // controller 3
int PIN_AILE = 32; // conttoller 4
int PIN_ELEV = 35; // controller 1
int PIN_RUDD = 34; // controller 2 
int PIN_GEAR = 39; //39//vn// gear
int PIN_AUX1 = 36; //36//vp// aux1

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
int rx = 18;
int tx = 19;
// parameter for CAN
byte candata[8];
CanData MotorCan1, MotorCan2;
CanData SensorCan1, SensorCan2, SensorCan3, SensorCan4;


// [Motor]
int rpm1 = 0;
int rpm2 = 0;

// [Serial2]
//define what pin rx2 tx2 is going to be
#define rx2 16                                          
#define tx2 17
unsigned long timeout = 50; //50ms


// [Encoder]
float Rotary_Encoder_pos1 = 0.0;
float Rotary_Encoder_pos2 = 0.0;
float Absolute_Encoder_pos1 = 0.0;
float Absolute_Encoder_pos2 = 0.0;

// [LoadCell]
#define calibration_factor1 -2180
#define dtout1 26  //DT 전선 연결
#define clk1 25    // SCLK 전선 연결
#define calibration_factor2 -2180
#define dtout2 14  //DT 전선 연결
#define clk2 27    // SCLK 전선 연결
HX711 scale1;
HX711 scale2;
float LoadCell_value1 = 0.0;
float LoadCell_value2 = 0.0;

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

// [RS422 abs Encoder]
struct SensorData {
  byte buffer1;
  byte buffer2;
  byte buffer3;
}data;
SensorData data1, data2;
float RS422_Encoder_pos1 = 0.0;
float RS422_Encoder_pos2 = 0.0;


void open_port();
void serial2Flush();
void rx_from_imu();
void rx_from_esp32();
void tx_to_esp32();
bool rx_CAN_from_ROS();
void tx_CAN_to_ROS();
void RF_recieve_values();
void Motor_Control_CAN(); 
void Motor_Control_CAN1(uint32_t ID);
void Motor_Control_CAN2(uint32_t ID); 
void Motor_Control_Velocity_CAN1(uint32_t ID, int rpm);
void Motor_Control_Velocity_CAN2(uint32_t ID, int rpm); 
void tx_CAN_to_Motor(uint32_t ID);
float rx_from_RS422();
void tx_CAN();
void AbsEnc_CAN();
void RotEnc_CAN();
void RS422Enc_CAN();
void LoadCell_CAN();
void EMImu_CAN();
void GPS_CAN();

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
  pinMode(PIN_THRO, INPUT_PULLUP); // Thro
  pinMode(PIN_AILE, INPUT_PULLUP); // AILE
  pinMode(PIN_ELEV, INPUT_PULLUP); // ELEV
  pinMode(PIN_RUDD, INPUT_PULLUP); // Rudd
  pinMode(PIN_GEAR, INPUT); // Gear
  pinMode(PIN_AUX1, INPUT); // Aux1

  // Loadcell
  scale1.begin(dtout1, clk1);
  scale1.set_scale(calibration_factor1);
  scale1.tare();

  scale2.begin(dtout2, clk2);
  scale2.set_scale(calibration_factor2);
  scale2.tare();

  MotorCan1.Id = 0x301;
  MotorCan2.Id = 0x302;
  SensorCan1.Id = 0x2381;
  SensorCan2.Id = 0x2382;
  SensorCan3.Id = 0x2383;
  SensorCan4.Id = 0x2384;
  MotorCan1.length = 8;
  MotorCan2.length = 8;
  SensorCan1.length = 8;
  SensorCan2.length = 8;
  SensorCan3.length = 8;
  SensorCan4.length = 8;

}
void loop() {
  //EBIMU
  // port = 1;         //port select
  // open_port();       //port open
  // serial2Flush();    //reset serial2 buffer
  // // Serial.println("Port : 1");
  // delay(10);        //delay for receiving new Serial data
  // delayMicroseconds(500);

  // rx_from_imu();

  //GPS620
  // port = 2;
  // open_port();
  // serial2Flush();
  // // Serial.println("Port : 2");
  // delay(5);


  // ESP32S3
  port = 3;         //port select
  open_port();      //port open
  serial2Flush();   //reset Serial buffer
  // // Serial.println("Port : 3");
  // delay(10);        //delay for receiving new Serial data
  // delayMicroseconds(5000);
  vTaskDelay(pdMS_TO_TICKS(10));

  rx_from_esp32(); //0ms

  //RS422 abs ENC1
  port = 4;         //port select
  open_port();      //port open
  serial2Flush();   //reset Serial buffer
  // Serial.println("Port : 4");
  // delay(1);        //delay for receiving new Serial data
  // delayMicroseconds(500);
  vTaskDelay(pdMS_TO_TICKS(1));

  RS422_Encoder_pos1 = rx1_from_RS422(); //0ms



  // RS422 abs ENC2 & Loadcell & RF
  port = 5;         //port select
  open_port();      //port open
  serial2Flush();   //reset Serial buffer
  // Serial.println("Port : 5");
  // delay(1);        //delay for receiving new Serial data
  // delayMicroseconds(500);
  vTaskDelay(pdMS_TO_TICKS(1));

  RS422_Encoder_pos2 = rx2_from_RS422(); //0ms

  LoadCell_value1 = scale1.get_units(1); //22ms-44ms
  LoadCell_value2 = scale2.get_units(1);
  // // Serial.print("Load cell value : ");
  // // Serial.print(LoadCell_value1);
  // // Serial.print(" , ");
  // // Serial.println(LoadCell_value2);
  RF_recieve_values();
  // GEAR = pulseIn(PIN_GEAR, HIGH, 50000);
  // Serial.println(GEAR);

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
      RF_recieve_values();

      if (ELEV > UP_ELEV && ELEV < MAX_ELEV) {
        // 1 up
        rpm1 = 10;
        // rpm2 = 10;
        
      }
      else if (ELEV < DOWN_ELEV && ELEV > MIN_ELEV){
        // 1 down
        rpm1 = -10;
        // rpm2 = -10;

      }
      else {
        // 1 stop
        rpm1 = 0;
        // rpm2 = 0;
        
      }

      if (THRO > UP_THRO && THRO < MAX_THRO) {
        // 1 up
        // rpm1 = 10;
        rpm2 = 10;
        
      }
      else if (THRO < DOWN_THRO && THRO > MIN_THRO){
        // 1 down
        // rpm1 = -10;
        rpm2 = -10;

      }
      else {
        // rpm1 = 0;
        rpm2 = 0;
        
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
      RF_recieve_values();

      if (ELEV > UP_ELEV && ELEV < MAX_ELEV) {
        // up
        rpm1 = 10;
        rpm2 = 10;
        
      }
      else if (ELEV < DOWN_ELEV && ELEV > MIN_ELEV){
        // down
        rpm1 = -10;
        rpm2 = -10;

      }
      else {
        if (RUDD > UP_RUDD && RUDD < MAX_RUDD) {
        // up
          rpm1 = 10;
          rpm2 = -10;
        }
        else if (RUDD < DOWN_RUDD && RUDD > MIN_RUDD){
        // down
          rpm1 = -10;
          rpm2 = 10;
        }
        else {
        // stop
      
          rpm1 = 0;
          rpm2 = 0;
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
                                          
  vTaskDelay(pdMS_TO_TICKS(1));;                                         //this is needed to make sure the channel switching event has completed
}

void serial2Flush(){
  while(Serial2.available() > 0) {
    char t = Serial2.read();
    // Serial.println(t);
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
  
      Serial.print("Pitch :"); Serial.print(euler[0]);   Serial.print(" ");
      Serial.print("Roll :"); Serial.print(euler[1]);   Serial.print(" ");
      Serial.print("Yaw :"); Serial.print(euler[2]);   Serial.println(" ");
    }
    else {
      // Serial.println("====== not imu data ======");
    }
  }
  else {
    // Serial.println("==== No data Received from imu ====");
  }

}

// rx serial from esp32s3
void rx_from_esp32() { 
  if (Serial2.available()>0) {
      String msg_from_esp = Serial2.readStringUntil('D');
      // Serial.print("Received Data is : ");
      // Serial.println(msg_from_esp);
    //   int termx = msg_from_esp.find("SRT");
    // if (termx) {
      
    // }
    if (msg_from_esp.startsWith("SRT")) {
      int term1 = msg_from_esp.indexOf(",");               // SRT
      int term2 = msg_from_esp.indexOf(",", term1 + 1);    // Rotary_Encoder1_pos
      int term3 = msg_from_esp.indexOf(",", term2 + 1);    // Rotary_Encoder2_pos
      int term4 = msg_from_esp.indexOf(",", term3 + 1);    // Absolute_Encoder1_pos
      int term5 = msg_from_esp.indexOf(",", term4 + 1);    // Absolute_Encoder2_pos
      // int term6 = msg_from_esp.indexOf(",", term5 + 1);    // RS232_Encoder1_pos
      // int term7 = msg_from_esp.indexOf(",", term6 + 1);    // RS232_Encoder2_pos
      // int term8 = msg_from_esp.indexOf(",", term7 + 1);    // LoadCell1_value
      // int term9 = msg_from_esp.indexOf(",", term8 + 1);    // LoadCelll2_value
  
      String Rotary_Encoder_pos1_string = msg_from_esp.substring(term1 + 1, term2);
      String Rotary_Encoder_pos2_string = msg_from_esp.substring(term2 + 1, term3);
      String Absolute_Encoder_pos1_string = msg_from_esp.substring(term3 + 1, term4);
      String Absolute_Encoder_pos2_string = msg_from_esp.substring(term4 + 1, term5);
      // String RS232_Encoder_pos1_string = msg_from_esp.substring(term5 + 1, term6);
      // String RS232_Encoder_pos2_string = msg_from_esp.substring(term6 + 1, term7);
      // String LoadCell_value1_string = msg_from_esp.substring(term7 + 1, term8);
      // String LoadCell_value2_string = msg_from_esp.substring(term8 + 1, term9);

      
      Rotary_Encoder_pos1 = Rotary_Encoder_pos1_string.toFloat();
      Rotary_Encoder_pos2 = Rotary_Encoder_pos2_string.toFloat();
      Absolute_Encoder_pos1 = Absolute_Encoder_pos1_string.toFloat();
      Absolute_Encoder_pos2 = Absolute_Encoder_pos2_string.toFloat();
      // RS232_Encoder_pos1 = RS232_Encoder1_pos_string.toFloat();
      // RS232_Encoder_pos2 = RS232_Encoder2_pos_string.toFloat();
      // LoadCell_value1 = LoadCell1_value_string.toFloat();
      // LoadCell_value2 = LoadCell2_value_string.toFloat();

      // Serial.print(Rotary_Encoder_pos1);
      // Serial.print(" , ");
      // Serial.print(Rotary_Encoder_pos2);
      // Serial.print(" , ");
      // Serial.print(Absolute_Encoder_pos1);
      // Serial.print(" , ");
      // Serial.println(Absolute_Encoder_pos2);
      

    }
    else {
      // Serial.println("====== not esp data ======");
    }
  }
  else {
    // Serial.println("==== No data Received from esp32 ====");
  }

}

void tx_to_esp32() {
  Serial2.print("SRT,Hello,ESP32S3,END");
  //Serial2.print("SRT,1,2,3,4,5,6,7,8,9,10,11,12,13,14,END");
  delay(2);
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
        // ReceiveCanData.Extened = 1;
        // Serial.print("extended ");
      }
      else{
        ReceiveCanData.Extened = 0;
      }

      if (CAN.packetRtr()) {
        // ReceiveCanData.RTR = 1;
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

bool rx_CAN_from_ROS() {
  CanData ReceiveCanData = CAN_receive(0x2999);
  if(ReceiveCanData.Data[0] == 0x5F){
    return 0;
  }
  else{
    return 1;
  }
}

void tx_CAN_to_ROS() {
  // Serial.print("CAN data send to ROS ");
  // CanData SendCanData;
  // SendCanData.Id = 101; 
  // SendCanData.length = 8;
  // for (int i = 0; i <= 7; i++) {
  //   SendCanData.Data[i] = ReceiveCanData.Data[i];
  // }
  // //SendCanData.Data = [ 0x01, 0x02, 0x03, 0x04, 0x01, 0x02, 0x03, 0x04 ];
  // //SendCanData.Data[0] = 0x34;
  // //SendCanData.Data[1] = 0x11;
  // //SendCanData.Data[2] = 0x12;
  // //SendCanData.Data[3] = 0x13;
  // //SendCanData.Data[4] = 0x23;
  // //SendCanData.Data[5] = 0x43;
  // //SendCanData.Data[6] = 0x10;
  // //SendCanData.Data[7] = 0x06;

  // // Serial.print("....");
  // CAN_send(SendCanData);
  // // Serial.println(" done");
}

void RF_recieve_values() { //44ms
  ELEV = 0;
  RUDD = 0;
  THRO = 0;
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

  Serial.print(THRO);
  Serial.print(" ");
  Serial.print(AILE);
  Serial.print(" ");
  Serial.print(ELEV);
  Serial.print(" ");
  Serial.print(RUDD);
  Serial.print(" ");
  Serial.println(GEAR);
  Serial.print(" ");
  Serial.println(AUX1);
    
  // Serial.print(Thro_1);
  // Serial.print(" ");
  // Serial.print(Aile_1);
  // Serial.print(" ");
  // Serial.print(Elev_1);
  // Serial.print(" ");
  // Serial.println(Rudd_1);
}

void tx_CAN_to_Motor(uint32_t ID) {
  // Serial.print("CAN data send to Motor ");
  CanData SendCanData;
  SendCanData.Id = uint32_t(strtol(String(ID).c_str(), NULL, 16));
  SendCanData.length = 8;
  for (int i = 0; i <= 7; i++) {
    SendCanData.Data[i] = candata[i];
  }

  // Serial.print("....");
  CAN_send(SendCanData);
  // Serial.println(" done"); 
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

void Motor_Control_CAN1(uint32_t ID) {
  float motor_position_degree = map(Thro_1, -255, 255, 0, 180);
  // Serial.print("Desired Degree is : ");
  // Serial.println(motor_position_degree);
  if (motor_position_degree<0) { //saifty issue
    Serial.println("error");
    return;
  }

  //float motor_position_degree = 0.0;
  float motor_position = - motor_position_degree * 10000;
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

  CanData SendCanData;
  SendCanData.Id = uint32_t(strtol(String(ID).c_str(), NULL, 16));
  SendCanData.length = 8;
  for (int i = 0; i <= 7; i++) {
    SendCanData.Data[i] = candata[i];
  }
  // Serial.print("....");
  CAN_send(SendCanData);
  // Serial.println(" done"); 
  // StopCanData1 = SendCanData;

}

void Motor_Control_CAN2(uint32_t ID) {
  float motor_position_degree = map(Thro_1, -255, 255, 0, 180);
  // Serial.print("Desired Degree is : ");
  // Serial.println(motor_position_degree);
  if (motor_position_degree<0) { //saifty issue
    Serial.println("error");
    return;
  }

  //float motor_position_degree = 0.0;
  float motor_position =  motor_position_degree * 10000;
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

  CanData SendCanData;
  SendCanData.Id = uint32_t(strtol(String(ID).c_str(), NULL, 16));
  SendCanData.length = 8;
  for (int i = 0; i <= 7; i++) {
    SendCanData.Data[i] = candata[i];
  }

  // Serial.print("....");
  CAN_send(SendCanData);
  // Serial.println(" done"); 
  // StopCanData2 = SendCanData;
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

float rx_from_RS422() {
  Serial2.write("3");
  delay(1);
  if(Serial2.available()){
    Serial2.readBytes((uint8_t*)&data, sizeof(SensorData));

    float AbsoluteEncoderPos = (float)((data.buffer1>>5)+(data.buffer2<<11)+(data.buffer3<<3))*360/524288;

    // Serial.print("Recieved RS422 value : ");
    // Serial.print(data.buffer1, HEX);
    // Serial.print("  ");
    // Serial.print(data.buffer2, HEX);
    // Serial.print("  ");
    // Serial.print(data.buffer3, HEX);
    // Serial.print("  ");
    // Serial.println(AbsoluteEncoderPos);
    return AbsoluteEncoderPos;
  }
  else {
    // Serial.println("==== RS422 not Received ===");
    return 0;
  }
}

float rx1_from_RS422() {
  Serial2.write("3");
  // delayMicroseconds(500);
  delay(2);
  if(Serial2.available()){
    Serial2.readBytes((uint8_t*)&data1, sizeof(data1));

    float AbsoluteEncoderPos = (float)((data1.buffer1>>5)+(data1.buffer2<<11)+(data1.buffer3<<3))*360/524288;

    // Serial.print("RS422 1 : ");
    // Serial.print(data1.buffer1, HEX);
    // // Serial.print("  ");
    // Serial.print(data1.buffer2, HEX);
    // // Serial.print("  ");
    // Serial.println(data1.buffer3, HEX);
    // // Serial.print("  ");
    // // Serial.print(data1.buffer4);
    // // Serial.print("  ");
    // // Serial.println(data1.buffer5);
    // // Serial.println("  ");
    // Serial.println(AbsoluteEncoderPos);
    
    return 1;
  }
  else {
    // Serial.println("==== RS422 not Received ===");
    return 0;
  }
}

float rx2_from_RS422() {
  Serial2.write("3");
  // delayMicroseconds(500);
  delay(1);
  if(Serial2.available()){
    Serial2.readBytes((uint8_t*)&data2, sizeof(data2));

    float AbsoluteEncoderPos = (float)((data2.buffer1>>5)+(data2.buffer2<<11)+(data2.buffer3<<3))*360/524288;

    // Serial.print("RS422 2 : ");
    // Serial.print(data2.buffer1, HEX);
    // // Serial.print("  ");
    // Serial.print(data2.buffer2, HEX);
    // // Serial.print("  ");
    // Serial.println(data2.buffer3, HEX);
    // // // Serial.println("  ");
    // Serial.println(AbsoluteEncoderPos);
    return 1;
  }
  else {
    // Serial.println("==== RS422 not Received ===");
    return 0;
  }
}
void SENDCAN(void * parameter) {
  for(;;) {
    tx_CAN();
    // vTaskDelay(0.001);
  }
}

