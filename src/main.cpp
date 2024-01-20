#include <Arduino.h>
#include <networking.h>
#include <constants.h>
#include <WiFi.h>
#include <PID_v1.h>
#include <temperature-controller.h>
#include <HX711.h>
#include <soc/rtc.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <solenoid-controller.h>
#include <RBDdimmer.h>
#include <rancilio-functions.h>
#include <Smoothed.h>
#include <ZACwire.h>
#include <HampelFilter.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

enum slope {
  VERYSLOW,
  SLOW,
  MEDIUM,
  FAST,
  VERYFAST,
};

enum MODE {
  TIME_PROFILING,
  PRESSURE_PROFILING,
  CONSTANT_PRESSURE_PROFILING,
};

//JSON Object
StaticJsonDocument<256> doc;
StaticJsonDocument<256> sentJSONDoc;
StaticJsonDocument<256> settings;

//Server Variables.
WiFiServer wifiServer(80);
WebSocketsServer webSocket = WebSocketsServer(1337);

//wifi settings
//const char *ssid = "Rancilio";
const char *passphrase = "9886432663";

const char* ssid = "H11";
const char* password =  "9886432663";
const char* MyHostName = "ESP Silvia";

//Boiler PID Settings
#define PIN_OUTPUT 18
double Setpoint, Input, Output;
//double Kp=3, Ki=0.005 , Kd=10;
double Kp=20, Ki=0, Kd=0;
//double Kp=12, Ki=80, Kd=20;
PID boilerPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//Temperature Sensor Settings
float tempC;
//OneWire oneWire(ONE_WIRE_BUS);
//DallasTemperature sensors(&oneWire);

#define TEMPERATURE_SENSOR_PIN 5
ZACwire Sensor(TEMPERATURE_SENSOR_PIN,306);	

//ULKA Pump Settings
#define ULKA_PIN_OUTPUT 3
#define outputPin  22 
#define zerocross  23
#define ULKA_MIN_POWER 30
#define ULKA_MAX_POWER 97 
//dimmerLamp ulka(outputPin, zerocross);
double ulkaSetpoint, ulkaInput, ulkaOutput, ulkaPower;
Smoothed <int> ulkaPowerAverage;
double ulkaKp=30.0, ulkaKi=20 , ulkaKd=0;
PID pumpPID(&ulkaInput, &ulkaOutput, &ulkaSetpoint, ulkaKp, ulkaKi, ulkaKd, DIRECT);

//hw_timer_t *ulkaPulseTimer = NULL;
//int frequencyCounter = 0;
//int pulsesRequiredPerSecond = 1;


//Default Extraction Profile
double preInfusionTime              = 30.0; //seconds
double preInfusionTemperature       = 94.0; 
double preInfusionPressure          = 3;
double preInfusionPower             = 60;
bool targetPressureReached          = 0;
int preInfusionSlope                = MEDIUM;
int preInfustionType                = CONSTANT_PRESSURE_PROFILING;

double steamingPressure             = 1.2;
double steamingTemperature          = 130;


double coffeeExtractionTime         = 180.0; //seconds
double coffeeExtractionTemperature  = 93.0;
double coffeeExtractionPressure     = 9.0;
int coffeeExtractionSlope           = MEDIUM;
float coffeeExtractionTargetWeight  = 36.0;
float coffeeExtrationFlowRate       = 1.2;
#define DRIP_OFFSET 1
 
double rampDownTime                 = 5.0; //seconds
double rampDownTemperature          = 92.0; 
double rampDownPressure             = 5.0;
int rampDownSlope                   = MEDIUM;

#define FLUSH_TIME 3 //Seconds

//Machine Status
enum state {
    IDLE, 
    PREINFUSING,
    EXTRACTION,
    RAMPDOWN,
    FLUSHING,
    BREWREADY,
    STEAMREADY,
    ULKAPUMPTEST,
    SOLENOIDTEST,
    STEAMING,
};


int currentMachineState = IDLE;

//Pressure Sensor Settings
#define PRESSURE_SENSOR_INPUT_PIN 32
Smoothed <float> pressureOffset;
Smoothed <float> currentPressure; 


//Scale HX711 Settings
const int LOADCELL_DOUT_PIN = 26;
const int LOADCELL_SCK_PIN = 27;
HX711 scale;
//float currentWeight;
Smoothed <float> currentWeight;

// Define the dataBuffer variable which will carry the data that
// we would like to run the Hampel filter on. The buffer is
// defined like
//    HampelFilter <A> = HampelFilter(<B>,<C>,<D>);
// where
//    <A>:  Name of buffer variable
//    <B>:  Default value, like 0.00
//    <C>:  Buffer size (= window size of filter),
//             this should be an odd number, like 9, 27 or 289
//    <D>:  Threshold for outlier detection, this needs some
//             tweaking depending on the data. As a general
//             rule: The smaller the value, the more aggresive
//             the threshold. A good starting value is 3.5.
//             The value cannot be smaller than 0 and not
//             greater than 655.
HampelFilter filteredWeight = HampelFilter(0.00, 15, 3.5);

//Global Timer
unsigned long currentTime;
unsigned long preInfusionTimer;
unsigned long extractionTimer;
unsigned long RampDownTimer;
unsigned long previousFlowTime = 0;
unsigned long weighingScaleTimer;
unsigned long flushTimer;
unsigned long temperatureTimer;

//Flowrate 
float previousReading;
float flowRate;
int seconds = 0;


void setupTemperatureSensor(){
  //sensors.begin();
  temperatureTimer = 0;

  if (Sensor.begin() == true) {     //check if a sensor is connected to the pin
    Serial.println("TSIC 306 TEMP SENSOR FOUND");
  }
  else{
    Serial.println("TSIC 306 TEMP SENSOR NOT FOUND");
  }

  delay(5);
}

float getCurrentTemperature(){
  //sensors.requestTemperatures();
  //tempC = sensors.getTempCByIndex(0);
  //if (tempC != DEVICE_DISCONNECTED_C)
  //{
    //Serial.print("TEMP: ");
    //Serial.println(tempC);
  //}
  //else
  //{
  //  Serial.println("Error: Could not read temperature data");
  //}
  return tempC;
}

float updateTemperature(){
  if ((millis() - temperatureTimer) >= 100){
    temperatureTimer = millis();
    float temperature = Sensor.getTemp();
    if (temperature >= 221 ) {
      Serial.printf("TEMP SENSOR ERROR %f\n",temperature);
    }
    else{
      tempC = temperature;
    }
  }
  return tempC;
}

void setupServer(){
  wifiServer.begin();
}

//***********************************************************************//
//  TIMER INITILIZATION CODES
//***********************************************************************//

void updateTimer(){
  currentTime = millis();
}

void initializePreinfusionProfile(){
  preInfusionTimer = millis();
  ulkaPower = 0;
  targetPressureReached = 0;
}

void initializeExtrationProfile(){
  extractionTimer = millis();
}

void initializeRampDownProfile(){
  RampDownTimer = millis();
}

void initializeFlush(){
  flushTimer = millis();
}

void initializeJSONDocs(){
  Serial.println("Initializing JSON Objects");
  doc["EXTT"]=0;
}

void setupScale(){
  
  weighingScaleTimer = millis();
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  Serial.println(scale.read());
  Serial.println(scale.read_average(20)); 
  Serial.println(scale.get_value(5));  
  Serial.println(scale.get_units(5), 1); 
  scale.set_scale(911.495);
  //scale.set_scale(884.794);
  scale.tare(); 
  Serial.println("After setting up the scale:");
  Serial.print("read: \t\t");
  Serial.println(scale.read());                 // print a raw reading from the ADC
  Serial.print("read average: \t\t");
  Serial.println(scale.read_average(20));       // print the average of 20 readings from the ADC
  Serial.print("get value: \t\t");
  Serial.println(scale.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight, set with tare()
  Serial.print("get units: \t\t");
  Serial.println(scale.get_units(5), 1);        // print the average of 5 readings from the ADC minus tare weight, divided

  currentWeight.begin(SMOOTHED_AVERAGE, 30);
  currentWeight.clear();
}

void scaleCalibration(){
  if (scale.is_ready()) {
    scale.set_scale();
    delay(500);    
    Serial.println("Tare... remove any weights from the scale.");
    delay(7000);
    scale.tare();
    Serial.println("Tare done...");
    Serial.print("Place a known weight on the scale...");
    delay(7000);
    long reading = scale.get_units(10);
    Serial.print("Result: ");
    Serial.println(reading);
    delay (7000);
  } 
  else {
    Serial.println("HX711 not found.");
  }
  delay(1000);
}

float getCurrentWeight(){
  //return currentWeight;  
  return currentWeight.get();  
  //return filteredWeight.readMedian();
}

void tareScale(){
  scale.tare();

  //clear the weigheted average filter
  currentWeight.clear();
  
  //clear the hampel filter
  for(int i = 0; i < 16 ; i++){
    filteredWeight.write(0);
  }
}

void updateCurrentWeight(){
  
if ((millis() - weighingScaleTimer) >= 50){
    
    weighingScaleTimer = millis();
    if (scale.is_ready()){
      float tempWeight = scale.get_units();
      filteredWeight.write(tempWeight);
      //currentWeight = filteredWeight.readMedian();
    }
    else{
      //Serial.println("DATA NOT READY");
    } 
    
  }
  currentWeight.add(filteredWeight.readMedian());
}



void setupAp(){
  Serial.println(WiFi.softAP(ssid,passphrase) ? "Ready" : "Failed!");
  //WiFi.softAP(ssid);
  Serial.print("Soft-AP IP address = ");
  Serial.println(WiFi.softAPIP());
  
}

void checkWifiConnection(){
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
}

void setupWifiConnection(){
  Serial.println("Connecting to Wifi");
  WiFi.setHostname(MyHostName);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  checkWifiConnection();
  Serial.println(WiFi.localIP());
  //WiFi.begin(ssid, password);
  //while (WiFi.status() != WL_CONNECTED) {
  //  Serial.println("Connecting to WiFi..");
  //  delay(2000);
  //  WiFi.disconnect();
  //  delay(2000);
  //  WiFi.reconnect();
  //  delay(2000);
  //}
}



void loadDefaultProfile(){
  
}

void setupBoilerPID(){
  Input = getCurrentTemperature();
  Setpoint = preInfusionTemperature;
  boilerPID.SetMode(AUTOMATIC);
  boilerPID.SetSampleTime(1000);
}

void updateBoilerPID(){

  switch (currentMachineState) {
    case IDLE:
      Setpoint = preInfusionTemperature;
    break;
    
    case STEAMING:
      Setpoint = steamingTemperature;
    break;

    default:

      break;
  }


  Input = getCurrentTemperature();
  //boilerPID.SetTunings(Kp,Ki,Kd);
  boilerPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
}

void setupPressureController(){
  currentPressure.begin(SMOOTHED_AVERAGE, 20);
  currentPressure.clear();
}

float getCurrentPressure(){
   float rawPressureValue = analogRead(PRESSURE_SENSOR_INPUT_PIN)*3.3/4096;
   currentPressure.add(fmap(rawPressureValue,0.3,2.4,0,20.68));
   return (currentPressure.get()-pressureOffset.get());
   //return (rawPressureValue);
}

//***********************************************************************//
//  CALIBRATION CODE
//***********************************************************************//

void calibratePressureSensor(){
  pressureOffset.begin(SMOOTHED_AVERAGE, 100);
  pressureOffset.clear();
  for (int i=0; i<100; i++){
    float rawPressureValue = analogRead(PRESSURE_SENSOR_INPUT_PIN)*3.3/4096;
    pressureOffset.add(fmap(rawPressureValue,0.3,2.4,0,20.68));
    delay(20);
  }
  Serial.printf("PRESSURE OFFSET %f", pressureOffset.get());
}

void testUlkaPump(){
  currentMachineState=ULKAPUMPTEST;
  ulkaPower = ulkaPower + 5;
}

void flush(){
  if ((millis() - flushTimer) <= (FLUSH_TIME*1000)) {
    turnBrewSwitchOn();
    ulkaPower = 60;
  }
  else{
    turnBrewSwitchOff();
    ulkaPower = 0;
    currentMachineState = IDLE;
  }
}

void setupFlowRateCalculator(){
  previousFlowTime = millis();
}

float getCurrentFlowRate(){
  if (millis() - previousFlowTime >= 1000) {
    previousFlowTime = millis();
    float currentReading = getCurrentWeight();
    flowRate = (currentReading - previousReading);
    previousReading = currentReading;
  }
  //Serial.print("FLOWRATE: ");
  //Serial.print(flowRate);
  //Serial.println(" ml/g");
  return (flowRate);
}

//***********************************************************************//
//  EXTRACTION CODE
//***********************************************************************//

void rampDown (float seconds, float pressure, int setting, float temperature){
  if ((currentTime - RampDownTimer)<=(seconds*1000)){
    Serial.print("RAMP DOWN  ");
    Serial.println((currentTime - RampDownTimer)/1000);
  }
  else{
      currentMachineState = IDLE;
  }
}

void coffeeExtraction (float seconds, float pressure, int setting, float temperature, float targetWeight){
  if ((millis() - extractionTimer)<=(seconds*1000)){
    Serial.print("EXTRACTION ");
    Serial.println((millis() - extractionTimer)/1000);
    if (getCurrentWeight() <= (targetWeight - DRIP_OFFSET)){
      turnBrewSwitchOn();
      //ulkaPower = 80;
      doc["EXTT"]=(millis() - extractionTimer);
    }
    else{
      //turnBrewSwitchOff();
      //ulkaPower = 0;
      currentMachineState = IDLE;
    }
  }
  else{
    //initializeRampDownProfile();
    //rampDown(rampDownTime,rampDownPressure,rampDownSlope,rampDownTemperature);
    //currentMachineState = RAMPDOWN;
    //ulkaPower = 0;
    //turnBrewSwitchOff();
    currentMachineState = IDLE;
      
  }
}

void preInfuse (float seconds, float pressure, int setting, float temperature, int mode){
  if ((currentTime - preInfusionTimer)<=(seconds*1000)){
    Serial.print("PREINFUSION ");
    Serial.println((currentTime - preInfusionTimer)/1000);
    doc["EXTT"]=(currentTime - preInfusionTimer);
    turnBrewSwitchOn();
    
    switch (mode){
      case TIME_PROFILING:
        if ((currentTime - preInfusionTimer) < 5000){
          ulkaPower = 60;
        }
        else{
          ulkaPower = 0;
        }
        break;
      case PRESSURE_PROFILING:
        if(targetPressureReached == 0){
          ulkaPower = preInfusionPower;
          if (getCurrentPressure() >= preInfusionPressure){
            targetPressureReached = 1;
          }
        }
        else{
          ulkaPower = 0;
        }
        break;
        case CONSTANT_PRESSURE_PROFILING:
        break;
      default:
      break;
    }
    
  }
  else{
    
    initializeExtrationProfile();
    coffeeExtraction(coffeeExtractionTime,coffeeExtractionPressure,coffeeExtractionSlope,coffeeExtractionTemperature,coffeeExtractionTargetWeight);
    currentMachineState = EXTRACTION;
  }
  
}

//***********************************************************************//
//  ULKA PUMP FUNCTIONS
//***********************************************************************//

//void IRAM_ATTR zcTrigger(){
  //timerRestart(ulkaPulseTimer);
//}

//void IRAM_ATTR onUlkaPulseTimer(){
  //if (frequencyCounter < 50){
  //  detachInterrupt(zerocross);
  //  if (frequencyCounter < pulsesRequiredPerSecond){
  //    digitalWrite(outputPin, HIGH); 

  //  }
  //  else{
  //    digitalWrite(outputPin, LOW); 
  //  }
  //  frequencyCounter = frequencyCounter + 1;
  //}
  //else {
  //  frequencyCounter = 0;
    //timerStop(ulkaPulseTimer);
    //timerRestart(ulkaPulseTimer);
  //  attachInterrupt(zerocross, zcTrigger, RISING);
  //}
 
//}


void setupUlkaPump(){
  ulka.begin(NORMAL_MODE, ON);
  ulka.setPower(0);

  pumpPID.SetMode(AUTOMATIC);
  
  ulkaPowerAverage.begin(SMOOTHED_AVERAGE, 30);
  ulkaPowerAverage.clear();

  //ulkaPulseTimer = timerBegin(0,80,true);
  //timerAttachInterrupt(ulkaPulseTimer, &onUlkaPulseTimer, true); 
  //timerAlarmWrite(ulkaPulseTimer, 20000, true);
  //timerAlarmEnable(ulkaPulseTimer);

  //pinMode(zerocross, INPUT);
  //pinMode(outputPin, OUTPUT);
  //attachInterrupt(zerocross, zcTrigger, RISING);
}


void updatePump (double barsPerSecond, double targetPressure, double targetFlowRate){
  switch (currentMachineState) {
    case IDLE:
      ulkaInput = 0;
      ulkaSetpoint = 0;
      pumpPID.SetTunings(0,0,0);
      pumpPID.Compute();
      ulkaPower = 0;
      ulkaPowerAverage.add(ulkaPower);
      break;
    
    case PREINFUSING:
      ulkaInput = getCurrentPressure();
      ulkaSetpoint = preInfusionPressure;
      pumpPID.SetTunings(4,5,10);

      pumpPID.Compute();
      ulkaPower = map(ulkaOutput,0,255,ULKA_MIN_POWER,ULKA_MAX_POWER);
      ulkaPowerAverage.add(ulkaPower);
      break;

    case EXTRACTION:
      //ulkaInput = flowRate;
      //ulkaSetpoint = coffeeExtrationFlowRate;
      //pumpPID.SetTunings(ulkaKp,ulkaKi,ulkaKd);

      ulkaInput = getCurrentPressure();
      ulkaSetpoint = coffeeExtractionPressure;
      pumpPID.SetTunings(4,7,6);

      pumpPID.Compute();
      ulkaPower = map(ulkaOutput,0,255,ULKA_MIN_POWER,ULKA_MAX_POWER);
      ulkaPowerAverage.add(ulkaPower);
      break;

    case STEAMING:
      ulkaInput = getCurrentPressure();
      ulkaSetpoint = steamingPressure;
      pumpPID.SetTunings(4,7,6);

      pumpPID.Compute();
      ulkaPower = map(ulkaOutput,0,255,ULKA_MIN_POWER,ULKA_MAX_POWER);
      ulkaPowerAverage.add(ulkaPower);

      break;

    default:

      break;
  }
  //ulka.setPower(ulkaPower);
  ulka.setPower(ulkaPowerAverage.get());
  
}

void stopPump(){
  ulkaPower = 0;
}

//***********************************************************************//
//  JSON DOC 
//***********************************************************************//

void updateJSONData(){
  doc["fdx"]=flowRate;
  doc["P"]=getCurrentPressure();
  doc["SSRP"]=map(Output,0,255,0,100);
  doc["W"]=getCurrentWeight();
  doc["T"]=tempC;
  doc["S"]=currentMachineState;
  //doc["UP"]=ulka.getPower();
  doc["UP"]=ulkaPowerAverage.get();
  doc["SOL"]=getSolenoidStatus();
  doc["UPID"]=ulkaOutput;

  doc["BREWTEMP"]=preInfusionTemperature;
  doc["BREWPRE"]=coffeeExtractionPressure;
  doc["FLOWRATE"]=coffeeExtrationFlowRate;
  doc["TARGETWEIGHT"]=coffeeExtractionTargetWeight;
  doc["PIT"]=preInfusionTime;
  doc["PIP"]=preInfusionPressure;
  doc["PIPWR"]=preInfusionPower;


  char msg_buf[256];
  serializeJson(doc,msg_buf);
  webSocket.broadcastTXT(msg_buf);
}

void updateSettings(){
  settings["BREWTEMP"]=preInfusionTemperature;
}

void brew(){
  
  switch (currentMachineState) {
    case IDLE:
      turnBrewSwitchOff();
      break;

    case PREINFUSING: 
      preInfuse(preInfusionTime,preInfusionPressure,preInfusionSlope,preInfusionTemperature,preInfustionType);
      break;

    case EXTRACTION:
      coffeeExtraction(coffeeExtractionTime,coffeeExtractionPressure,coffeeExtractionSlope,coffeeExtractionTemperature,coffeeExtractionTargetWeight);
      break;

    case RAMPDOWN:
      rampDown(rampDownTime,rampDownPressure,rampDownSlope,rampDownTemperature);
      break;

    case FLUSHING:
      flush();
      break;

    case SOLENOIDTEST:
      turnBrewSwitchOn();
      break;

    case STEAMING:
    break;

    default:
      break;
  }
}

void onWebSocketEvent(uint8_t client_num,
                      WStype_t type,
                      uint8_t * payload,
                      size_t length) {

  // Figure out the type of WebSocket event
  switch(type) {

    // Client has disconnected
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", client_num);
      break;

    // New client has connected
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(client_num);
        Serial.printf("[%u] Connection from ", client_num);
        Serial.println(ip.toString());
      }
      break;

    // Handle text messages from client
    case WStype_TEXT:

      // Print out raw message
      Serial.printf("[%u] Received text: %s\n", client_num, payload);
      deserializeJson(sentJSONDoc,payload);

      //if ( strcmp(sentJSONDoc["message"], "DATA") == 0 ) {
      //  char msg_buf[256];
      //  serializeJson(doc,msg_buf);
      //  webSocket.sendTXT(client_num, msg_buf);
      //}

      //if ( strcmp((char *)payload, "TARE") == 0 ) {
      //  scale.tare();
      //}

      if ( strcmp(sentJSONDoc["message"], "TARE") == 0 ) {
        tareScale();
      }


      if ( strcmp(sentJSONDoc["message"], "CALIBRATEPRESSURE") == 0 ) {
         calibratePressureSensor();
      }


      if ( strcmp(sentJSONDoc["message"], "SOLENOIDON") == 0 ) {
         currentMachineState = SOLENOIDTEST;
      }

      
      if ( strcmp(sentJSONDoc["message"], "SOLENOIDOFF") == 0 ) {
         currentMachineState = IDLE;
      }


      if ( strcmp(sentJSONDoc["message"], "BREW") == 0 ) {
        tareScale();
        initializePreinfusionProfile();
        currentMachineState = PREINFUSING;
      }


      if ( strcmp(sentJSONDoc["message"], "ULKAPUMPTEST") == 0 ) {
        testUlkaPump();
      }


      if ( strcmp(sentJSONDoc["message"], "FLUSH") == 0 ) {
        initializeFlush();
        currentMachineState = FLUSHING;
      }

      if ( strcmp(sentJSONDoc["message"], "IDLE") == 0 ) {
        currentMachineState = IDLE;
      }

      if ( strcmp(sentJSONDoc["message"], "STEAMING") == 0 ) {
        currentMachineState = STEAMING;
      }

      if ( strcmp(sentJSONDoc["message"], "CALIBRATESCALE") == 0 ) {
        scaleCalibration();
      }

      if ( strcmp(sentJSONDoc["message"], "SETTINGS") == 0 ) {
        preInfusionTemperature = sentJSONDoc["setBrewTemperature"];
        coffeeExtrationFlowRate = sentJSONDoc["setFlowRate"];
        coffeeExtractionTargetWeight = sentJSONDoc["setExtrationWeight"];

        preInfusionTime = sentJSONDoc["setPIT"];
        preInfusionPressure = sentJSONDoc["setPIP"];
        preInfusionPower = sentJSONDoc["setPIPWR"];

        coffeeExtractionPressure = sentJSONDoc["setBREWP"];

        setupBoilerPID();
      }


      break;

    // For everything else: do nothing
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
      
      break;
  }
}

void setupWebSocket(){
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}

void setupOta(){
   // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("Rancilio");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
}

void setup()
{
  Serial.begin(115200);
  //calibratePressureSensor();
  setupWifiConnection();
  //setupAp();
  //setupServer();
  setupWebSocket();
  setupOta();
  setupTemperatureSensor();
  setupBoilerPID();
  setupScale();
  initializePreinfusionProfile();
  setupFlowRateCalculator();
  setupSolenidController();
  setupUlkaPump();
  setupPressureController();
  initializeJSONDocs();
}

void loop() {
  
  //scaleCalibration();
  updateTimer();
  updateCurrentWeight();
  updateTemperature();
  updateJSONData();
  getCurrentFlowRate();
  getCurrentPressure();
  updateBoilerPID();  
  brew(); 
  updatePump(3,6,1.5);
  webSocket.loop();
  ArduinoOTA.handle();
  checkWifiConnection();
}