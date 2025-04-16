//Sending CAN sensor data from esp32 to ROS
// float Rotary_Encoder_pos1 = 0.0;
// float Rotary_Encoder_pos2 = 0.0;
// float Absolute_Encoder_pos1 = 0.0;
// float Absolute_Encoder_pos2 = 0.0;
// float RS422_Encoder_pos1 = 0.0;
// float RS422_Encoder_pos2 = 0.0;
// float LoadCell_value1 = 0.0;
// float LoadCell_value2 = 0.0;
// float euler[3];

float AbsEnc_position1 = 0.0;
int AbsEnc_position1_int32 = 0;
float AbsEnc_position2 = 0.0;
int AbsEnc_position2_int32 = 0;
float RotEnc_position1 = 0.0;
int RotEnc_position1_int32 = 0;
float RotEnc_position2 = 0.0;
int RotEnc_position2_int32 = 0;
float loadcell_degree1 = 0.0;
int loadcell_degree1_int32 = 0;
float loadcell_degree2 = 0.0;
int loadcell_degree2_int32 = 0;
 

void tx_CAN() {
  // Serial.println("Sensor CAN data send to ROS ");
  // ID for sensor == 300 + (sensor type)
  // abs encoder    10
  // rot encoder    20
  // RS422 encoder  30
  // LoadCell       40
  // EBIMU          50
  // GPS            60
  // SensorCanData.Data = {data1(4byte), data2(4byte)}
  AbsEnc_CAN();
  RotEnc_CAN();
  // RS422Enc_CAN();
  RS422Enc_CAN2();
  LoadCell_CAN();

  if (GEAR > MIN_GEAR && GEAR < MAX_GEAR) {

  if (GEAR > UP_GEAR && GEAR < MAX_GEAR) {
    // Serial.println("Gear 0");

    Motor_Control_Velocity_CAN1(301, rpm1);
    Motor_Control_Velocity_CAN2(302, rpm2);

      
    }
    // GEAR 2 : stop mode
  else if (GEAR < DOWN_GEAR && GEAR > MIN_GEAR) {
  }

    // GEAR 1 : velocity mode
  else {
    Motor_Control_Velocity_CAN1(301, rpm1);
    Motor_Control_Velocity_CAN2(302, rpm2);
  }

  }

  // EBImu_CAN();
  //GPS_CAN();


  // Serial.println(" done"); 
}
// Converting DEC to HEX functions
void AbsEnc_CAN() {
  // Serial.print("AbsEnc : ");
  // CanData SensorCanData; //Absolute_Encoder_pos
  // SensorCanData.Id = 0x2381;
  // SensorCanData.length = 8;
  
  // Absolute_Encoder_pos1 = 12345.67;   // HEX 00 12 D6 87
  // Absolute_Encoder_pos2 = -12345.67;  // HEX FF ED 29 79  

  AbsEnc_position1 = Absolute_Encoder_pos1 * 100;
  AbsEnc_position1_int32 = int(AbsEnc_position1);
  // int AbsEnc_position1_int32_HIGH = AbsEnc_position1_int32 >> 16;
  // int AbsEnc_position1_int32_LOW = AbsEnc_position1_int32-(AbsEnc_position1_int32_HIGH<<16);
  // int AbsEnc_position1_int32_HIGH_HIGH = AbsEnc_position1_int32_HIGH >> 8;
  // int AbsEnc_position1_int32_HIGH_LOW = AbsEnc_position1_int32_HIGH-(AbsEnc_position1_int32_HIGH_HIGH<<8);
  // int AbsEnc_position1_int32_LOW_HIGH = AbsEnc_position1_int32_LOW >> 8;
  // int AbsEnc_position1_int32_LOW_LOW = AbsEnc_position1_int32_LOW-(AbsEnc_position1_int32_LOW_HIGH<<8);

  AbsEnc_position2 = Absolute_Encoder_pos2 * 100;
  AbsEnc_position2_int32 = int(AbsEnc_position2);
  // int AbsEnc_position2_int32_HIGH = AbsEnc_position2_int32 >> 16;
  // int AbsEnc_position2_int32_LOW = AbsEnc_position2_int32-(AbsEnc_position2_int32_HIGH<<16);
  // int AbsEnc_position2_int32_HIGH_HIGH = AbsEnc_position2_int32_HIGH >> 8;
  // int AbsEnc_position2_int32_HIGH_LOW = AbsEnc_position2_int32_HIGH-(AbsEnc_position2_int32_HIGH_HIGH<<8);
  // int AbsEnc_position2_int32_LOW_HIGH = AbsEnc_position2_int32_LOW >> 8;
  // int AbsEnc_position2_int32_LOW_LOW = AbsEnc_position2_int32_LOW-(AbsEnc_position2_int32_LOW_HIGH<<8);

  // SensorCan1.Data[0] = byte(AbsEnc_position1_int32_HIGH_HIGH);
  // SensorCan1.Data[1] = byte(AbsEnc_position1_int32_HIGH_LOW);
  // SensorCan1.Data[2] = byte(AbsEnc_position1_int32_LOW_HIGH);
  // SensorCan1.Data[3] = byte(AbsEnc_position1_int32_LOW_LOW);
  // SensorCan1.Data[4] = byte(AbsEnc_position2_int32_HIGH_HIGH);
  // SensorCan1.Data[5] = byte(AbsEnc_position2_int32_HIGH_LOW);
  // SensorCan1.Data[6] = byte(AbsEnc_position2_int32_LOW_HIGH);
  // SensorCan1.Data[7] = byte(AbsEnc_position2_int32_LOW_LOW);

  SensorCan1.Data[0] = byteRead(AbsEnc_position1_int32, 3);
  SensorCan1.Data[1] = byteRead(AbsEnc_position1_int32, 2);
  SensorCan1.Data[2] = byteRead(AbsEnc_position1_int32, 1);
  SensorCan1.Data[3] = byteRead(AbsEnc_position1_int32, 0);
  SensorCan1.Data[4] = byteRead(AbsEnc_position2_int32, 3);
  SensorCan1.Data[5] = byteRead(AbsEnc_position2_int32, 2);
  SensorCan1.Data[6] = byteRead(AbsEnc_position2_int32, 1);
  SensorCan1.Data[7] = byteRead(AbsEnc_position2_int32, 0);

  // Serial.println(" ");
  // for (int i = 0; i <= 7; i++) {
  //   Serial.print(SensorCanData.Data[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println(" ");


  CAN_EXTsend(SensorCan1);
  // Serial.println(":");

}
void RotEnc_CAN() {
  // Serial.print("RotEnc : ");
  // CanData SensorCanData; //Rotary_Encoder_pos
  // SensorCanData.Id = 0x2382;
  // SensorCanData.length = 8;
  
  // Rotary_Encoder_pos2 = 12345.67;   // HEX 00 12 D6 87
  // Rotary_Encoder_pos1 = -12345.67;  // HEX FF ED 29 79  
  
  RotEnc_position1 = Rotary_Encoder_pos1 * 100;
  RotEnc_position1_int32 = int(RotEnc_position1);
  // int RotEnc_position1_int32_HIGH = RotEnc_position1_int32 >> 16;
  // int RotEnc_position1_int32_LOW = RotEnc_position1_int32-(RotEnc_position1_int32_HIGH<<16);
  // int RotEnc_position1_int32_HIGH_HIGH = RotEnc_position1_int32_HIGH >> 8;
  // int RotEnc_position1_int32_HIGH_LOW = RotEnc_position1_int32_HIGH-(RotEnc_position1_int32_HIGH_HIGH<<8);
  // int RotEnc_position1_int32_LOW_HIGH = RotEnc_position1_int32_LOW >> 8;
  // int RotEnc_position1_int32_LOW_LOW = RotEnc_position1_int32_LOW-(RotEnc_position1_int32_LOW_HIGH<<8);

  RotEnc_position2 = Rotary_Encoder_pos2 * 100;
  RotEnc_position2_int32 = int(RotEnc_position2);
  // int RotEnc_position2_int32_HIGH = RotEnc_position2_int32 >> 16;
  // int RotEnc_position2_int32_LOW = RotEnc_position2_int32-(RotEnc_position2_int32_HIGH<<16);
  // int RotEnc_position2_int32_HIGH_HIGH = RotEnc_position2_int32_HIGH >> 8;
  // int RotEnc_position2_int32_HIGH_LOW = RotEnc_position2_int32_HIGH-(RotEnc_position2_int32_HIGH_HIGH<<8);
  // int RotEnc_position2_int32_LOW_HIGH = RotEnc_position2_int32_LOW >> 8;
  // int RotEnc_position2_int32_LOW_LOW = RotEnc_position2_int32_LOW-(RotEnc_position2_int32_LOW_HIGH<<8);

  // SensorCan2.Data[0] = byte(RotEnc_position1_int32_HIGH_HIGH);
  // SensorCan2.Data[1] = byte(RotEnc_position1_int32_HIGH_LOW);
  // SensorCan2.Data[2] = byte(RotEnc_position1_int32_LOW_HIGH);
  // SensorCan2.Data[3] = byte(RotEnc_position1_int32_LOW_LOW);
  // SensorCan2.Data[4] = byte(RotEnc_position2_int32_HIGH_HIGH);
  // SensorCan2.Data[5] = byte(RotEnc_position2_int32_HIGH_LOW);
  // SensorCan2.Data[6] = byte(RotEnc_position2_int32_LOW_HIGH);
  // SensorCan2.Data[7] = byte(RotEnc_position2_int32_LOW_LOW);

  SensorCan2.Data[0] = byteRead(RotEnc_position1_int32, 3);
  SensorCan2.Data[1] = byteRead(RotEnc_position1_int32, 2);
  SensorCan2.Data[2] = byteRead(RotEnc_position1_int32, 1);
  SensorCan2.Data[3] = byteRead(RotEnc_position1_int32, 0);
  SensorCan2.Data[4] = byteRead(RotEnc_position2_int32, 3);
  SensorCan2.Data[5] = byteRead(RotEnc_position2_int32, 2);
  SensorCan2.Data[6] = byteRead(RotEnc_position2_int32, 1);
  SensorCan2.Data[7] = byteRead(RotEnc_position2_int32, 0);

  // Serial.println(" ");
  // for (int i = 0; i <= 7; i++) {
  //   Serial.print(SensorCanData.Data[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println(" ");

  CAN_EXTsend(SensorCan2);
  // Serial.println(":");

}
void RS422Enc_CAN() {
  // Serial.print("422Enc : ");
  // CanData SensorCanData; //RS422_Encoder_pos
  // SensorCanData.Id = 0x2383;
  // SensorCanData.length = 8;
  
  // RS422_Encoder_pos1 = 12345.67;   // HEX 00 12 D6 87
  // RS422_Encoder_pos2 = -12345.67;  // HEX FF ED 29 79  

  float Enc_position1 = RS422_Encoder_pos1 * 100;
  int Enc_position1_int32 = int(Enc_position1);
  int Enc_position1_int32_HIGH = Enc_position1_int32 >> 16;
  int Enc_position1_int32_LOW = Enc_position1_int32-(Enc_position1_int32_HIGH<<16);
  int Enc_position1_int32_HIGH_HIGH = Enc_position1_int32_HIGH >> 8;
  int Enc_position1_int32_HIGH_LOW = Enc_position1_int32_HIGH-(Enc_position1_int32_HIGH_HIGH<<8);
  int Enc_position1_int32_LOW_HIGH = Enc_position1_int32_LOW >> 8;
  int Enc_position1_int32_LOW_LOW = Enc_position1_int32_LOW-(Enc_position1_int32_LOW_HIGH<<8);

  float Enc_position2 = RS422_Encoder_pos2 * 100;
  int Enc_position2_int32 = int(Enc_position2);
  int Enc_position2_int32_HIGH = Enc_position2_int32 >> 16;
  int Enc_position2_int32_LOW = Enc_position2_int32-(Enc_position2_int32_HIGH<<16);
  int Enc_position2_int32_HIGH_HIGH = Enc_position2_int32_HIGH >> 8;
  int Enc_position2_int32_HIGH_LOW = Enc_position2_int32_HIGH-(Enc_position2_int32_HIGH_HIGH<<8);
  int Enc_position2_int32_LOW_HIGH = Enc_position2_int32_LOW >> 8;
  int Enc_position2_int32_LOW_LOW = Enc_position2_int32_LOW-(Enc_position2_int32_LOW_HIGH<<8);

  SensorCan3.Data[0] = byte(Enc_position1_int32_HIGH_HIGH);
  SensorCan3.Data[1] = byte(Enc_position1_int32_HIGH_LOW);
  SensorCan3.Data[2] = byte(Enc_position1_int32_LOW_HIGH);
  SensorCan3.Data[3] = byte(Enc_position1_int32_LOW_LOW);
  SensorCan3.Data[4] = byte(Enc_position2_int32_HIGH_HIGH);
  SensorCan3.Data[5] = byte(Enc_position2_int32_HIGH_LOW);
  SensorCan3.Data[6] = byte(Enc_position2_int32_LOW_HIGH);
  SensorCan3.Data[7] = byte(Enc_position2_int32_LOW_LOW);

  // Serial.println(" ");
  // for (int i = 0; i <= 7; i++) {
  //   Serial.print(SensorCanData.Data[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println(" ");

  CAN_EXTsend(SensorCan3);
  // Serial.println(":");

}

void RS422Enc_CAN2() {
  // Serial.print("422Enc : ");
  // CanData SensorCanData; //RS422_Encoder_pos
  // SensorCanData.Id = 0x2383;
  // SensorCanData.length = 8;
  
  // RS422_Encoder_pos1 = 12345.67;   // HEX 00 12 D6 87
  // RS422_Encoder_pos2 = -12345.67;  // HEX FF ED 29 79  

  // float Enc_position1 = RS422_Encoder_pos1 * 100;
  // int Enc_position1_int32 = int(Enc_position1);
  // int Enc_position1_int32_HIGH = Enc_position1_int32 >> 16;
  // int Enc_position1_int32_LOW = Enc_position1_int32-(Enc_position1_int32_HIGH<<16);
  // int Enc_position1_int32_HIGH_HIGH = Enc_position1_int32_HIGH >> 8;
  // int Enc_position1_int32_HIGH_LOW = Enc_position1_int32_HIGH-(Enc_position1_int32_HIGH_HIGH<<8);
  // int Enc_position1_int32_LOW_HIGH = Enc_position1_int32_LOW >> 8;
  // int Enc_position1_int32_LOW_LOW = Enc_position1_int32_LOW-(Enc_position1_int32_LOW_HIGH<<8);

  // float Enc_position2 = RS422_Encoder_pos2 * 100;
  // int Enc_position2_int32 = int(Enc_position2);
  // int Enc_position2_int32_HIGH = Enc_position2_int32 >> 16;
  // int Enc_position2_int32_LOW = Enc_position2_int32-(Enc_position2_int32_HIGH<<16);
  // int Enc_position2_int32_HIGH_HIGH = Enc_position2_int32_HIGH >> 8;
  // int Enc_position2_int32_HIGH_LOW = Enc_position2_int32_HIGH-(Enc_position2_int32_HIGH_HIGH<<8);
  // int Enc_position2_int32_LOW_HIGH = Enc_position2_int32_LOW >> 8;
  // int Enc_position2_int32_LOW_LOW = Enc_position2_int32_LOW-(Enc_position2_int32_LOW_HIGH<<8);

  SensorCan3.Data[0] = 0x00;
  SensorCan3.Data[1] = data1.buffer1;
  SensorCan3.Data[2] = data1.buffer2;
  SensorCan3.Data[3] = data1.buffer3;
  SensorCan3.Data[4] = 0x00;
  SensorCan3.Data[5] = data2.buffer1;
  SensorCan3.Data[6] = data2.buffer2;
  SensorCan3.Data[7] = data2.buffer3;

  // Serial.println(" ");
  // for (int i = 0; i <= 7; i++) {
  //   Serial.print(SensorCanData.Data[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println(" ");

  CAN_EXTsend(SensorCan3);
  // Serial.println(":");

}

void LoadCell_CAN() {
  // Serial.print("LoadCell : ");
  // CanData SensorCanData; //LoadCell_value
  // SensorCanData.Id = 0x2384;
  // SensorCanData.length = 8;
  
  // LoadCell_value1 = 12345.67;   // HEX 00 12 D6 87
  // LoadCell_value2 = -12345.67;  // HEX FF ED 29 79  

  loadcell_degree1 = LoadCell_value1 * 100;
  loadcell_degree1_int32 = int(loadcell_degree1);
  // int loadcell_degree1_int32_HIGH = loadcell_degree1_int32 >> 16;
  // int loadcell_degree1_int32_LOW = loadcell_degree1_int32-(loadcell_degree1_int32_HIGH<<16);
  // int loadcell_degree1_int32_HIGH_HIGH = loadcell_degree1_int32_HIGH >> 8;
  // int loadcell_degree1_int32_HIGH_LOW = loadcell_degree1_int32_HIGH-(loadcell_degree1_int32_HIGH_HIGH<<8);
  // int loadcell_degree1_int32_LOW_HIGH = loadcell_degree1_int32_LOW >> 8;
  // int loadcell_degree1_int32_LOW_LOW = loadcell_degree1_int32_LOW-(loadcell_degree1_int32_LOW_HIGH<<8);

  loadcell_degree2 = LoadCell_value2 * 100;
  loadcell_degree2_int32 = int(loadcell_degree2);
  // int loadcell_degree2_int32_HIGH = loadcell_degree2_int32 >> 16;
  // int loadcell_degree2_int32_LOW = loadcell_degree2_int32-(loadcell_degree2_int32_HIGH<<16);
  // int loadcell_degree2_int32_HIGH_HIGH = loadcell_degree2_int32_HIGH >> 8;
  // int loadcell_degree2_int32_HIGH_LOW = loadcell_degree2_int32_HIGH-(loadcell_degree2_int32_HIGH_HIGH<<8);
  // int loadcell_degree2_int32_LOW_HIGH = loadcell_degree2_int32_LOW >> 8;
  // int loadcell_degree2_int32_LOW_LOW = loadcell_degree2_int32_LOW-(loadcell_degree2_int32_LOW_HIGH<<8);

  // SensorCan4.Data[0] = byte(loadcell_degree1_int32_HIGH_HIGH);
  // SensorCan4.Data[1] = byte(loadcell_degree1_int32_HIGH_LOW);
  // SensorCan4.Data[2] = byte(loadcell_degree1_int32_LOW_HIGH);
  // SensorCan4.Data[3] = byte(loadcell_degree1_int32_LOW_LOW);
  // SensorCan4.Data[4] = byte(loadcell_degree2_int32_HIGH_HIGH);
  // SensorCan4.Data[5] = byte(loadcell_degree2_int32_HIGH_LOW);
  // SensorCan4.Data[6] = byte(loadcell_degree2_int32_LOW_HIGH);
  // SensorCan4.Data[7] = byte(loadcell_degree2_int32_LOW_LOW);

  SensorCan4.Data[0] = byteRead(loadcell_degree1_int32, 3);
  SensorCan4.Data[1] = byteRead(loadcell_degree1_int32, 2);
  SensorCan4.Data[2] = byteRead(loadcell_degree1_int32, 1);
  SensorCan4.Data[3] = byteRead(loadcell_degree1_int32, 0);
  SensorCan4.Data[4] = byteRead(loadcell_degree2_int32, 3);
  SensorCan4.Data[5] = byteRead(loadcell_degree2_int32, 2);
  SensorCan4.Data[6] = byteRead(loadcell_degree2_int32, 1);
  SensorCan4.Data[7] = byteRead(loadcell_degree2_int32, 0);
  // Serial.println(" ");
  // for (int i = 0; i <= 7; i++) {
  //   Serial.print(SensorCanData.Data[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println(" ");

  CAN_EXTsend(SensorCan4);
  // Serial.println(":");

}
void EBImu_CAN() {
  // Serial.print("EBImu : ");
  CanData SensorCanData; //{euler[3](2byte) ,checksum(2byte)}  
  SensorCanData.Id = 0x2385;
  SensorCanData.length = 8;
  float euler_degree[3] = {0.00, 0.00, 0.00};

  for (int i = 0; i<3; i++) {
    euler_degree[i] = euler[i] * 100;
  }

  int16_t euler_degree1_int16 = int16_t(euler_degree[0]);
  int16_t euler_degree1_int16_HIGH = euler_degree1_int16 >> 8;
  int16_t euler_degree1_int16_LOW = euler_degree1_int16-(euler_degree1_int16_HIGH<<8);

  int16_t euler_degree2_int16 = int16_t(euler_degree[1]);
  int16_t euler_degree2_int16_HIGH = euler_degree2_int16 >> 8;
  int16_t euler_degree2_int16_LOW = euler_degree2_int16-(euler_degree2_int16_HIGH<<8);

  int16_t euler_degree3_int16 = int16_t(euler_degree[2]);
  int16_t euler_degree3_int16_HIGH = euler_degree3_int16 >> 8;
  int16_t euler_degree3_int16_LOW = euler_degree3_int16-(euler_degree3_int16_HIGH<<8);

  int16_t checksum_int16 = euler_degree1_int16+ euler_degree2_int16+ euler_degree3_int16;
  int16_t checksum_int16_HIGH = checksum_int16 >> 8;
  int16_t checksum_int16_LOW = checksum_int16-(checksum_int16_HIGH<<8);

  SensorCanData.Data[0] = byte(euler_degree1_int16_HIGH);
  SensorCanData.Data[1] = byte(euler_degree1_int16_LOW);
  SensorCanData.Data[2] = byte(euler_degree2_int16_HIGH);
  SensorCanData.Data[3] = byte(euler_degree2_int16_LOW);
  SensorCanData.Data[4] = byte(euler_degree3_int16_HIGH);
  SensorCanData.Data[5] = byte(euler_degree3_int16_LOW);
  SensorCanData.Data[6] = byte(checksum_int16_HIGH);
  SensorCanData.Data[7] = byte(checksum_int16_LOW);

  // Serial.println(" ");
  // for (int i = 0; i <= 7; i++) {
  //   Serial.print(SensorCanData.Data[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println(" ");


  CAN_EXTsend(SensorCanData);
  // Serial.println(":");

}
void GPS_CAN() {
  CanData SensorCanData; //GPS_value
  SensorCanData.Id = 0x2360;
  SensorCanData.length = 8;
  
  // LoadCell_value1 = 12345.67;   // HEX 00 12 D6 87
  // LoadCell_value2 = -12345.67;  // HEX FF ED 29 79  

  float loadcell_degree1 = LoadCell_value1 * 100;
  int loadcell_degree1_int32 = int(loadcell_degree1);
  int loadcell_degree1_int32_HIGH = loadcell_degree1_int32 >> 16;
  int loadcell_degree1_int32_LOW = loadcell_degree1_int32-(loadcell_degree1_int32_HIGH<<16);
  int loadcell_degree1_int32_HIGH_HIGH = loadcell_degree1_int32_HIGH >> 8;
  int loadcell_degree1_int32_HIGH_LOW = loadcell_degree1_int32_HIGH-(loadcell_degree1_int32_HIGH_HIGH<<8);
  int loadcell_degree1_int32_LOW_HIGH = loadcell_degree1_int32_LOW >> 8;
  int loadcell_degree1_int32_LOW_LOW = loadcell_degree1_int32_LOW-(loadcell_degree1_int32_LOW_HIGH<<8);

  float loadcell_degree2 = LoadCell_value2 * 100;
  int loadcell_degree2_int32 = int(loadcell_degree2);
  int loadcell_degree2_int32_HIGH = loadcell_degree2_int32 >> 16;
  int loadcell_degree2_int32_LOW = loadcell_degree2_int32-(loadcell_degree2_int32_HIGH<<16);
  int loadcell_degree2_int32_HIGH_HIGH = loadcell_degree2_int32_HIGH >> 8;
  int loadcell_degree2_int32_HIGH_LOW = loadcell_degree2_int32_HIGH-(loadcell_degree2_int32_HIGH_HIGH<<8);
  int loadcell_degree2_int32_LOW_HIGH = loadcell_degree2_int32_LOW >> 8;
  int loadcell_degree2_int32_LOW_LOW = loadcell_degree2_int32_LOW-(loadcell_degree2_int32_LOW_HIGH<<8);

  SensorCanData.Data[0] = byte(loadcell_degree1_int32_HIGH_HIGH);
  SensorCanData.Data[1] = byte(loadcell_degree1_int32_HIGH_LOW);
  SensorCanData.Data[2] = byte(loadcell_degree1_int32_LOW_HIGH);
  SensorCanData.Data[3] = byte(loadcell_degree1_int32_LOW_LOW);
  SensorCanData.Data[4] = byte(loadcell_degree2_int32_HIGH_HIGH);
  SensorCanData.Data[5] = byte(loadcell_degree2_int32_HIGH_LOW);
  SensorCanData.Data[6] = byte(loadcell_degree2_int32_LOW_HIGH);
  SensorCanData.Data[7] = byte(loadcell_degree2_int32_LOW_LOW);

  Serial.println(" ");
  for (int i = 0; i <= 7; i++) {
    Serial.print(SensorCanData.Data[i], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");


  Serial.print("....");
  CAN_EXTsend(SensorCanData);

}
// etc 
void Enc_CAN(float enc_pos[], int sensor_num) {
  CanData SensorCanData; //RS422_Encoder_pos
  SensorCanData.Id = int(strtol(String(300 + sensor_num).c_str(), NULL, 16));
  SensorCanData.length = 8;
  
  // enc_pos[0] = 12345.67;   // HEX 00 12 D6 87
  // enc_pos[1] = -12345.67;  // HEX FF ED 29 79  

  float Enc_position1 = enc_pos[0] * 100;
  int Enc_position1_int32 = int(Enc_position1);
  int Enc_position1_int32_HIGH = Enc_position1_int32 >> 16;
  int Enc_position1_int32_LOW = Enc_position1_int32-(Enc_position1_int32_HIGH<<16);
  int Enc_position1_int32_HIGH_HIGH = Enc_position1_int32_HIGH >> 8;
  int Enc_position1_int32_HIGH_LOW = Enc_position1_int32_HIGH-(Enc_position1_int32_HIGH_HIGH<<8);
  int Enc_position1_int32_LOW_HIGH = Enc_position1_int32_LOW >> 8;
  int Enc_position1_int32_LOW_LOW = Enc_position1_int32_LOW-(Enc_position1_int32_LOW_HIGH<<8);

  float Enc_position2 = enc_pos[1] * 100;
  int Enc_position2_int32 = int(Enc_position2);
  int Enc_position2_int32_HIGH = Enc_position2_int32 >> 16;
  int Enc_position2_int32_LOW = Enc_position2_int32-(Enc_position2_int32_HIGH<<16);
  int Enc_position2_int32_HIGH_HIGH = Enc_position2_int32_HIGH >> 8;
  int Enc_position2_int32_HIGH_LOW = Enc_position2_int32_HIGH-(Enc_position2_int32_HIGH_HIGH<<8);
  int Enc_position2_int32_LOW_HIGH = Enc_position2_int32_LOW >> 8;
  int Enc_position2_int32_LOW_LOW = Enc_position2_int32_LOW-(Enc_position2_int32_LOW_HIGH<<8);

  SensorCanData.Data[0] = byte(Enc_position1_int32_HIGH_HIGH);
  SensorCanData.Data[1] = byte(Enc_position1_int32_HIGH_LOW);
  SensorCanData.Data[2] = byte(Enc_position1_int32_LOW_HIGH);
  SensorCanData.Data[3] = byte(Enc_position1_int32_LOW_LOW);
  SensorCanData.Data[4] = byte(Enc_position2_int32_HIGH_HIGH);
  SensorCanData.Data[5] = byte(Enc_position2_int32_HIGH_LOW);
  SensorCanData.Data[6] = byte(Enc_position2_int32_LOW_HIGH);
  SensorCanData.Data[7] = byte(Enc_position2_int32_LOW_LOW);

  // Serial.println(" ");
  // for (int i = 0; i <= 7; i++) {
  //   Serial.print(SensorCanData.Data[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println(" ");

  CAN_send(SensorCanData);
  Serial.print(":");

}
 