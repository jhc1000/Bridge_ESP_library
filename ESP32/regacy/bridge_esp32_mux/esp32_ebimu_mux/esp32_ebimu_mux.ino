// [Serial2]
//define what pin rx2 tx2 is going to be
#define rx2 16                                          
#define tx2 17

// [MUX]
//esp32 pins to control pin S1 S2 S3
int s1 = 18;                                           
int s2 = 19;                                           
int s3 = 21;                                           
//what port to open
int port = 0;

// [EBIMU]
// parameter for EMimu
float euler[3];

void open_port();
void serial2Flush();
void rx_from_imu();
void rx_from_esp32();
void tx_to_esp32();

void setup() {
  //UART Expander S1 S2 S3. 000 -> Port1. 기본적으로 1인상태에서시작.
  pinMode(s1,OUTPUT);
  pinMode(s2,OUTPUT);
  pinMode(s3,OUTPUT);
  open_port();

  Serial2.begin(115200, SERIAL_8N1, rx2, tx2);
  Serial.begin(115200);
}
//===============================================================
 
void loop() {
  port = 1;         //port select
  open_port();      //port open
  Serial.println("Port : 1");
  serial2Flush();   //reset Serial buffer
  delay(100);        //delay for receiving new Serial data

  rx_from_imu();
  

  port = 2;
  open_port();
  Serial.println("Port : 2");
  serial2Flush();
  delay(100);

  tx_to_esp32();

  
  rx_from_esp32();

}

//====================================================================================
// Functions
void open_port() {                                  //this function controls what UART port is opened.

  if (port < 1 || port > 2)port = 1;                //if the value of the port is within range (1-8) then open that port. If it’s not in range set it to port 0

  digitalWrite(s1, bitRead(port-1, 0));               //Here we have two commands combined into one.
  digitalWrite(s2, bitRead(port-1, 1));               //The digitalWrite command sets a pin to 1/0 (high or low)
  digitalWrite(s3, bitRead(port-1, 2));               //The bitRead command tells us what the bit value is for a specific bit location of a number
                                          
  delay(2);                                         //this is needed to make sure the channel switching event has completed
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
  

      //Serial.print("\n\r");
      Serial.print("Pitch :"); Serial.print(euler[0]);   Serial.print(" ");
      Serial.print("Roll :"); Serial.print(euler[1]);   Serial.print(" ");
      Serial.print("Yaw :"); Serial.print(euler[2]);   Serial.println(" ");
    }
    else {
      Serial.println("====== not imu data ======");
    }
  }
  else {
    Serial.println("==== No data Received from imu ====");
  }

}

void rx_from_esp32() {
  if (Serial2.available()>0) {
      String msg_from_esp = Serial2.readStringUntil('D');
    if (msg_from_esp.startsWith("SRT")) {
      int term1 = msg_from_esp.indexOf(",");               // *
      int term2 = msg_from_esp.indexOf(",", term1 + 1);    // hello
      int term3 = msg_from_esp.indexOf(",", term2 + 1);    // port2
  
      String hello_string = msg_from_esp.substring(term1 + 1, term2);
      String port_string = msg_from_esp.substring(term2 + 1, term3);

      Serial.print("Received Data is :");
      Serial.print(hello_string);   Serial.print(" : ");
      Serial.print(port_string);    Serial.println(" ");
    }
    else {
      Serial.println("====== not esp data ======");
    }
  }
  else {
    Serial.println("==== No data Received from esp32 ====");
  }

}

void tx_to_esp32() {
  Serial2.print("SRT,Hello,port2,END");
  delay(2);
}
