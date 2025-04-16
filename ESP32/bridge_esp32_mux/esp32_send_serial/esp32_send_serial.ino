//define what pin rx2 tx2 is going to be
#define rx2 16                                          
#define tx2 17

void serial2Flush();
void rx_from_esp32();
void tx_to_esp32();

void setup() {
  // put your setup code here, to run once:
  Serial2.begin(115200, SERIAL_8N1, rx2, tx2);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Port : 2");
  
  tx_to_esp32();

  rx_from_esp32();

  delay(100);

}

void serial2Flush(){
  while(Serial2.available() > 0) {
    char t = Serial2.read();
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