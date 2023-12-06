//Sensor dependencies
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

//ESP-NOW dependencies
#include <esp_now.h>
#include <WiFi.h>

#include <string.h>

//Declare button
const int buttonPin = 23; //D23
int buttonState;

//Declare sensor
Adafruit_MPU6050 mpu;

//Declare acceleration variables
float accel_x = 0.0;
float accel_y = 0.0;
float accel_z = 0.0;

//Declare receiver
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//Store information about peer
esp_now_peer_info_t peerInfo;

//Declare message package
typedef struct struct_message {
  char iden[17];
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
} struct_message;
struct_message acceleration;

typedef struct result{
  bool err;
  bool win;
}result;
result r;

//Declare calibration variables
float cal_x = 0.0;
float cal_y = 0.0;
float cal_z = 0.0;
bool cal = true;

//Declare LED pins
const int bluePin = 19; //D19
const int greenPin = 18; //D18
const int redPin = 4; //D4

//Declare time variables
long current_time;
long checkpoint;
long duration;

int LED_colour; //0 = none, 1 = red, 2 = green, 3 = blue, 4 = yellow, 5 = white

//LED colour function
void LED(bool r, bool g, bool b){
  if(r == true){
    digitalWrite(redPin, HIGH); //Turn on red colour
  }else{
    digitalWrite(redPin, LOW); //Turn off red colour
  }

  if(g == true){
    digitalWrite(greenPin, HIGH);
  }else{
    digitalWrite(greenPin, LOW);
  }

  if(b == true){
    digitalWrite(bluePin, HIGH);
  }else{
    digitalWrite(bluePin, LOW);
  }
}

//Calibraton function
void calibrate(){
  //Sensor events handler
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  int i = 1;
  while (cal){ //While in calibration mode
    LED(true, true, true); //white light
    //Prompt once
    if (i == 1){
      Serial.println("Place on level surface and press button to calibrate");
      Serial.println("Calibration pending");
      i = 0;
    }
    
    //Capture when pressed
    buttonState = digitalRead(buttonPin);
    if (buttonState == HIGH){ //Store sensor data
      cal_x = a.acceleration.x;
      cal_y = a.acceleration.y;
      cal_z = a.acceleration.z;
    } else{
      if (cal_x || cal_y || cal_z){ //If calibration variable exist value
        //Feedback calibrated values
        Serial.println("\nCalibration success");
        Serial.print("Calibrated acceleration X: ");
        Serial.print(cal_x);
        Serial.print(", Y: ");
        Serial.print(cal_y);
        Serial.print(", Z: ");
        Serial.print(cal_z);
        Serial.println(" m/s^2 \n");
        cal = false; //Disable calibration mode
      }else{
        Serial.print(".");
        delay(100);
      }
    }
  }
}

//While data sent
void OnDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.print("Last packet send status: ");
  if(status == ESP_NOW_SEND_SUCCESS){
    Serial.println("delivery success");
    checkpoint = current_time; //Reset checkpoint
    LED_colour = 0; //No light
    duration = 1000;  //Start timer
  } else{
    Serial.println("delivery failed");
    checkpoint = current_time; //Reset checkpoint
    LED_colour = 4; //Yellow light
    duration = 10000;  //Start timer
  }
  Serial.println();
}

//While data received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){
  memcpy(&r, incomingData, sizeof(r));
  Serial.print("Error? ");
  if (r.err == true) {
    Serial.println("Yes");
    checkpoint = current_time;
    LED_colour = 4; //Yellow light
    duration = 10000; 
  }
  else if (r.err == false && r.win == true){
    Serial.println("No");
    Serial.println("Win");
    checkpoint = current_time;
    LED_colour = 3; //Blue light
    duration = 7500; 
  }
  else if (r.err == false && r.win == false){
    Serial.println("No");
    Serial.println("Lose");
    checkpoint = current_time;
    LED_colour = 1; //Red light
    duration = 7500; 
  }
  Serial.println();
}

void setup() {
  //Declare serial
  Serial.begin(115200);

  //Register identifier
  strcpy(acceleration.iden, (WiFi.macAddress()).c_str()); //Copy MAC address to identifier
  Serial.println(acceleration.iden);

  //Initialize sensor
  Serial.println("Adafruit MPU6050 initializing...");
  if(!mpu.begin()){
    Serial.println("Failed to find MPU6050");
    while (1){
      delay(10);
    }
  }
  Serial.println("MPU6050 initialized");

  //Setup sensor range
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  Serial.println("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()){ //Set sensor sensitivity
    case MPU6050_RANGE_2_G:
      Serial.print("+-2G \n"); 
      break;
    case MPU6050_RANGE_4_G:
      Serial.print("+-4G \n");
      break;
    case MPU6050_RANGE_8_G:
      Serial.print("+-8G \n");
      break;
    case MPU6050_RANGE_16_G:
      Serial.print("+-16G \n");
      break;
  }

  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  //Ask for feedback
  esp_now_register_send_cb(OnDataSent);

  //Display data upon receiving
  esp_now_register_recv_cb(OnDataRecv);
  
  //Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0; 
  peerInfo.encrypt = false;

  //Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("");
  //Declare pin as I/O
  pinMode(buttonPin, INPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  cal = true; //Enable calibration mode
}

void loop() {
  //Update time for every loop
  current_time = millis();

  //Sensor events
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //Check for calibration
  calibrate();
  
  //Print upon pressing
  buttonState = digitalRead(buttonPin);
  float temp_x;
  float temp_y;
  float temp_z;
  if (buttonState == HIGH) { //If button is pressed
    //Stores maximum acceleration when pressing
    temp_x = abs(a.acceleration.x - cal_x);
    temp_y = abs(a.acceleration.y - cal_y);
    temp_z = abs(a.acceleration.z - cal_z);
    if (temp_x > accel_x){ //Only record maximum values
      accel_x = temp_x;
    }

    if (temp_y > accel_y){
      accel_y = temp_y;
    }

    if (temp_z > accel_z){
      accel_z = temp_z;
    }
  } else { //If button is released
    //Send data once
    if (accel_x || accel_y || accel_z){
      Serial.print("Acceleration X: ");
      Serial.print(accel_x);
      Serial.print(", Y: ");
      Serial.print(accel_y);
      Serial.print(", Z: ");
      Serial.print(accel_z);
      Serial.println(" m/s^2");

      //Set values
      acceleration.x = accel_x;
      acceleration.y = accel_y;
      acceleration.z = accel_z;

      //Send message via ESP-NOW
      Serial.print("Sending data... ");
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &acceleration, sizeof(acceleration));

      if (result == ESP_OK) {
        Serial.println("Data sent");
      }
      else {
        Serial.println("Sending failed");
      }

      accel_x = 0.0;
      accel_y = 0.0;
      accel_z = 0.0;
    }
  }

  //LED colour handler
  if(current_time - checkpoint <= duration){ //If there is existing LED timer
    switch (LED_colour){
      case 0: LED(false, false, false); break; //none
      case 1: LED(true, false, false); break; //red
      case 2: LED(false, true, false); break; //green
      case 3: LED(false, false, true); break; //blue
      case 4: LED(true, true, false); break; //yellow
      case 5: LED(true, true, true); break; //white
    }
  }else{
    LED(false, true, false); //idle_green
  }
}
