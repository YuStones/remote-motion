//Declare Wi-Fi libraries
#include <ESP8266WiFi.h>
#include <espnow.h>

//Declare string-char library
#include <string.h>

//Declare receiving package
typedef struct message {
  char iden[17];
  float x;
  float y;
  float z;
}message;
message acceleration;

//Declare players
typedef struct player{
  String iden;
  float z;
  long t;
}player;
player A;
bool accel_A;
player B;
bool accel_B;

//Declare results package
typedef struct result{
  bool err;
  bool win;
}result;
result rA;
uint8_t a_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
result rB;
uint8_t b_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//Declare buzzer
const int buzzerPin = 14;

//Declare button
const int buttonPin = 5;

//Declare time variables
long checkpoint = 0;
long duration = 0;
long time_taken = 0;
long current_time;
const long wait_time = 10000; //in ms

const String null_iden = "00000000000000000";
const float accel_thres = 20.0;

//When data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&acceleration, incomingData, sizeof(acceleration)); //Copy received data to struct

  if(current_time - checkpoint <= duration){
    String identificator = acceleration.iden; //Pass temp MAC
    if(identificator != null_iden){ //If temp MAC is not null
      if(identificator == A.iden){ //If message is from player A
        Serial.println("Player A");

        //Record time
        Serial.print("Time: ");
        A.t = current_time - checkpoint; //Latency between timer start and message arrival
        Serial.println(A.t);
        strcpy(acceleration.iden, null_iden.c_str()); //Set temp MAC to null

        //Record acceleration
        Serial.print("Acceleration Z: ");
        A.z = acceleration.z;
        Serial.print(A.z);
        Serial.println(" m/s^2");
        Serial.println();

      }else if(identificator == B.iden){ //If message is from player B
        Serial.println("Player B");

        //Record time
        Serial.print("Time: ");
        B.t = current_time - checkpoint; //Latency between timer start and message arrival
        Serial.println(B.t);
        strcpy(acceleration.iden, null_iden.c_str()); //Set temp MAC to null

        //Record acceleration
        Serial.print("Acceleration Z: ");
        B.z = acceleration.z;
        Serial.print(B.z);
        Serial.println(" m/s^2");
        Serial.println();

      }
    }
  }

  //Check for acceleration threshold
  accel_A = A.z > accel_thres; //0 = fail, 1 = success
  accel_B = B.z > accel_thres; //0 = fail, 1 = success
  
  //End countdown early once both arrived
  if(A.t != 0 && B.t != 0){
    bool time_taken = B.t < A.t; //0 = A took less time than B, 1 = B took less time than A
    if(!accel_A && !accel_B){ 
      //F = A'B'
      sendResult(true, false);
    }else if(!(accel_B && (!accel_A || time_taken))){ 
      //If player A wins
      //F' = B (A'+C)
      sendResult(false, false);
    }else if((!accel_A && accel_B) || time_taken){ 
      //If player B win
      //F = A'B + C
      sendResult(false, true);
    }else{
      Serial.println("Error");
    }

    //Reset acceleration values & timer
    A.t = 0;
    B.t = 0;
    duration = 0;
  }
}

//When data is sent
void OnDataSent(uint8_t *mac, uint8_t sendStatus) {
  Serial.print("Result status: ");
  if (sendStatus == 0){
    Serial.println("delivery success");
  }
  else{
    Serial.println("delivery fail");
  }
}

void sendResult(bool err, bool whowon){
  if(err == true){
    //Error
    //1100
    Serial.println("No winner");
    rA.err = true;
    rB.err = true;
    rA.win = false;
    rB.win = false;
  }else if(err == false && whowon == false){
    //Player A wins
    //0010
    Serial.println("Player A wins");
    rA.err = false;
    rB.err = false;
    rA.win = true;
    rB.win = false;
  }else if(err == false && whowon == true){
    //Player B wins
    //0001
    Serial.println("Player B wins");
    rA.err = false;
    rB.err = false;
    rA.win = false;
    rB.win = true;
  }

  esp_now_send(a_MAC, (uint8_t *) &rA, sizeof(rA));
  esp_now_send(b_MAC, (uint8_t *) &rB, sizeof(rB));
}

void beep(){
  tone(buzzerPin, 1000); //Play sound at 1kHz
  delay(1000);
  noTone(buzzerPin);
  delay(50);
  tone(buzzerPin, 1000);
  delay(1000);
  noTone(buzzerPin);
  delay(50);
  tone(buzzerPin, 1000);
  delay(1000);
  noTone(buzzerPin);
  delay(50);
  tone(buzzerPin, 2000);
  delay(100);
  noTone(buzzerPin);
}

void setup() {
  //Declare player MAC
  A.iden = "A8:42:E3:91:0D:34";
  B.iden = "A8:42:E3:91:31:A4";

  //Declare serial monitor
  Serial.begin(115200);
  
  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Initialize ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  //Set receiver feedback
  esp_now_register_recv_cb(OnDataRecv);

  //Set send feedback
  esp_now_register_send_cb(OnDataSent);

  //Register role (dual role-slave priority)
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  //Register player A & player B
  esp_now_add_peer(a_MAC, ESP_NOW_ROLE_MAX, 1, NULL, 0);
  esp_now_add_peer(b_MAC, ESP_NOW_ROLE_MAX, 1, NULL, 0);

  pinMode(buzzerPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  current_time = millis(); //Update time counter for every loo[

  bool buttonState = digitalRead(buttonPin); //Read from button
  bool status = false; 
  if(buttonState == LOW){ //When button is pressed
    status = !status; //Flip boolean to true
    if(status){
      Serial.println("------------------------------");
      beep();
      checkpoint = current_time; //Set checkpoint time
      duration = wait_time; //Start timer
    }
  }

  if(current_time - checkpoint > duration && duration != 0){ //Once timer expires
    bool a_recv = A.t != 0; //If A was received
    bool b_recv = B.t != 0; //If B was received
    Serial.println("Timer expired");
    if(a_recv && accel_A){
      //A wins
      sendResult(false, false);
    }else if(b_recv && accel_B){
      //B wins
      sendResult(false, true);
    }else{
      //Error
      sendResult(true, false);
    }
    //Reset values & timer
    A.t = 0;
    B.t = 0;
    duration = 0;
  }
}
