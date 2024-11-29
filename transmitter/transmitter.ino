
#include <esp_now.h>
#include <WiFi.h>
const int switchPin1 = 18;  // Pin connected to one position of the switch
const int switchPin2 = 21;
#define THROTTLE_PIN 34
#define YAW_PIN 35
#define ROLL_PIN 32
#define PITCH_PIN 33
#define SWITCH_PIN 4
uint8_t broadcastAddress[] = {0x08, 0xF9, 0xE0, 0x78, 0x4D, 0xB4};//08:F9:E0:78:4D:B4


int pushButtonStateOut; 
int pushButtonStateIn; 
String success; 

const int pushDown = 2; // GPIO2

const int lightME = 4; //GPIO4

struct PacketData
{
  byte throttleValue;
  byte yawValue;
  byte rollValue;
  byte pitchValue;
  byte switchPressed;
};
PacketData data;
int mapJoystickValue(int value)
{
  return map(value, 0, 4095,0, 255);
}
// Create a struct_message to hold incoming sensor readings


esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}



  
  
 

 
void setup() {
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  // Init Serial Monitor
  Serial.begin(115200);
  pinMode(switchPin1, INPUT_PULLUP);
  pinMode(switchPin2, INPUT_PULLUP);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received

}
 
void loop() {
  getButtonReading();
  int switchState1 = digitalRead(switchPin1);
  int switchState2 = digitalRead(switchPin2);
  // Read joystick values and map them to 0-254 range
  data.throttleValue = mapJoystickValue(analogRead(THROTTLE_PIN));
  data.yawValue = mapJoystickValue(analogRead(YAW_PIN));
  data.rollValue = mapJoystickValue(analogRead(ROLL_PIN));
  data.pitchValue = mapJoystickValue(analogRead(PITCH_PIN));
  data.switchPressed = digitalRead(SWITCH_PIN) == LOW ? 1 : 0;

  


    if (switchState1 == LOW) {
  
  Serial.println("ARM"); // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &data, sizeof(data));
  delay(1000); // Add this delay
   Serial.println(data.throttleValue);
    Serial.println(analogRead(THROTTLE_PIN));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }



  switchONorOFFled();
    }
     else if (switchState2 == LOW) {
   esp_now_unregister_send_cb();
  
    Serial.println("DISARM");
  }
}



void getButtonReading(){
  pushButtonStateOut=digitalRead(pushDown);
}

void switchONorOFFled(){
 digitalWrite(lightME, pushButtonStateIn);
}
