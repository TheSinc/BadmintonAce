#include <WiFi.h>                                     // Wifi access point configuration
#include <SPIFFS.h>                                   // SPIFFS library to allow web page to be uploaded as seperate document
#include <ESPAsyncWebServer.h>                        // Web server
#include <WebSocketsServer.h>                         // Library for web-sockets to msg with front-end
#include <ArduinoJson.h>                              // Needed for JSON encapsulation (send multiple variables with one string)
#include <ESP32Servo.h>                               // Servo library to control ESC's and servos
#include <Preferences.h>                              // Library to allow saving/loading settings data
#include <Math.h>                                     // Library for maths functions
#include <NewPing.h>                                  // Sonar libary for obstruction sensor
#include <ESPmDNS.h>                                  // Library to allow .local instead of IP address to access page.


// Published values for SG90 servos; adjust if needed.
// QY3225 Digital Servo - 50-333hz 500-2500 usec
#define servoMinPulseLength 500 // Minimum pulse length in µs
#define servoMaxPulseLength 2500  // Maximum pulse length in µs

// ESC min max
#define escMinPulseLength 1000 // Minimum pulse length in µs
#define escMaxPulseLength 2500 // Maximum pulse length in µs

#define motorKV 750 //Motor KV
#define batteryVoltage 11.1 //Battery voltage for 3 cell lipo

// Recommended pins include 2,4,12-19,21-23,25-27,32-33  //not pin 2 as that's the LED on the DEV BOARD
#define startPin                0 
#define stopPin                 4 
#define throughBeamPin          13 
#define throughBeamIndicatorPin 2 //Built in LED
#define servoTrajectoryPin      16
#define servoRotationPin        17
#define servoPusherPin          18
#define servoGripperPin         19
#define servoLeftESCPin         22
#define servoRightESCPin        23
#define ultrasonicTriggerPin    15 //White Wire
#define ultrasonicEchoPin       34 //Grey Wire
#define ultrasonicMaxDistance   100 // Maximum distance we want to measure (in centimeters). Badminton Serice line is 198cm from net.

 //Create a state engine variables for the feeding sequence.
#define ST_IDLE               0
#define ST_START              1
#define ST_GRIPPER_OPEN       2
#define ST_GRIPPER_CLOSED     3
#define ST_SHUTTLE_CHECK      4
#define ST_PREP_LAUNCH        5
#define ST_LAUNCH             6
#define ST_RESET              7
static byte system_state = ST_IDLE; 

// Constants
const char *ssid = "ACE";
const char *password =  "Launcher"; //Password must be >=8 characters
const int dns_port = 53;
const int http_port = 80;
const int ws_port = 81;

//global variables
Preferences preferences;

byte sequence_length = 0;





struct shotDetails
{
  float court_x_pos;
  float court_y_pos;
  float trajectory;
  float rotation;
  float v0;
  int left_motor_rpm;
  int right_motor_rpm;
};
shotDetails active_shots[10];
int active_shot_array_length = 10;
float default_delay = 3;

struct shotEntry
{
  int ID;
  float delay;
};
shotEntry shot_sequence[50];



enum PLAY_TYPE {PLAY_ONCE, REPEAT_SHOTS, RANDOM_ORDER, RANDOM_SHOTS};
PLAY_TYPE ACTIVE_PLAY_TYPE = PLAY_ONCE;

int gripper_open_pos = 90;
int gripper_closed_pos = 90;
int pusher_catch_pos = 90;
int pusher_push_pos = 90;
int maxSpeed = 20000;//motorKV * batteryVoltage; 750kV Motor * 11.1V 3S Battery = 8,325. However that's not the ESC max...
//ESC max for a 12 pole motor is 40,000 RPM.
//If there is a linear response and 0 RPM is low, and 40K is high... & i've been using 7500K as max... then i'd expext
int escLowPulseSetPoint=map(2500, 0 , maxSpeed, escMinPulseLength, escMaxPulseLength);
int escHighPulseSetPoint=map(5000, 0 , maxSpeed, escMinPulseLength, escMaxPulseLength);
int rpmLowSetPoint = 2500;
int rpmHighSetPoint = 5000;
float traj_servo_correction = 0;
float traj_correction = 0;
static bool stop_launcher = true;
bool shuttle_check = true;
bool obstacle_check = true;


//Setup Servo Objects
Servo servoPusher;
Servo servoGripper;
Servo servoTrajectory;
Servo servoRotation;
Servo servoLeftESC;
Servo servoRightESC;

//Setup Sonar Object
NewPing sonar(ultrasonicTriggerPin,ultrasonicEchoPin,ultrasonicMaxDistance); // Setup the ultrasonic sensor pins and max distance.

// The JSON library uses static memory, so this will need to be allocated:
StaticJsonDocument<1000> doc_tx;                       // provision memory for about 1000 characters
StaticJsonDocument<2000> doc_rx;                       // provision memory for about 2000 characters

// Setup the web server
AsyncWebServer server(http_port);
WebSocketsServer webSocket = WebSocketsServer(ws_port);
char msg_buf[10];

/***********************************************************
   Functions
*/

// Callback: receiving any WebSocket message
void onWebSocketEvent(uint8_t client_num, WStype_t type, uint8_t * payload, size_t length) {
  // Figure out the type of WebSocket event
  switch (type) {
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
        sendStatus("Ready", "green");
        sendParameters();
      }
      break;

    // Handle messages from client
    case WStype_TEXT:
    {
      //Stop current feeding.
      //stopAll();
            
      // Print out raw message
      Serial.printf("[%u] Received text: %s\n", client_num, payload);
     
      DeserializationError error = deserializeJson(doc_rx, payload);
      
      if (error) {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        return;
      }
      
      const char* command = doc_rx["command"]; // "update_shots", "start", "stop"

       if (strcmp(command,"start")== 0)
       {
         //Example JSON {"command":"start","play_type":"once","shot_pattern":[{"shotID":3,"delay":"0.5"},{"shotID":4,"delay":"0.5"}]}

         //Process the play type
          const char* play_type = doc_rx["play_type"]; 
          if (strcmp(play_type,"once")== 0) ACTIVE_PLAY_TYPE = PLAY_ONCE;
          if (strcmp(play_type,"repeat")== 0) ACTIVE_PLAY_TYPE = REPEAT_SHOTS;
          if (strcmp(play_type,"random_order")== 0) ACTIVE_PLAY_TYPE = RANDOM_ORDER;
          if (strcmp(play_type,"random_shots")== 0) ACTIVE_PLAY_TYPE = RANDOM_SHOTS;
          

          //Process the shot pattern array
          sequence_length = doc_rx["shot_pattern"].as<JsonArray>().size(); 
          int array_index = 0;
          for (JsonObject shot_pattern : doc_rx["shot_pattern"].as<JsonArray>()) {      
            int pattern_shot_id = shot_pattern["shotID"].as<int>(); // 1, 2, 3, 4, 5, 6, 7, 8, 9, 10
            int pattern_shot_delay = shot_pattern["delay"].as<int>(); // 1, 2, 3, 4, 5, 6, 7, 8, 9, 10

            shot_sequence [array_index].ID = pattern_shot_id;
            shot_sequence [array_index].delay = pattern_shot_delay;              
            array_index = array_index +1;
          }

          system_state = ST_START;          

       }
        else if (strcmp(command,"save_globals")== 0)
       {
         //Example JSON: {"command":"save_globals","height":"1.5","velocity_correction":"0","flight_correction":"0"}
          

          Serial.println("Save globals.");
          sendStatus("Saving Globals" , "orange");
          preferences.begin("Globals", false);
          preferences.putFloat("velocity_adj", doc_rx["velocity_correction"].as<float>());
          preferences.putFloat("flight_adj", doc_rx["flight_correction"].as<float>());
          preferences.putFloat("launcher_y", doc_rx["height"].as<float>());
          preferences.end();
          sendStatus("Ready" , "green");

       }
       else if (strcmp(command,"update_shots")== 0)
       {
         //Example JSON: {"command":"update_shots","height":"1.5","velocity_correction":"0","shots":[{"x":-2.5,"y":1.5,"rotation":-19,"trajectory":25},{"x":-0.937876256651103,"y":0.9950842255448553,"rotation":0,"trajectory":25},{"x":2.5,"y":1.5,"rotation":19,"trajectory":25},{"x":-2.5,"y":4,"rotation":-19,"trajectory":30},{"x":-1.0410451154325955,"y":3.5397617551513334,"rotation":0,"trajectory":30},{"x":2.5,"y":4,"rotation":19,"trajectory":30},{"x":-1.913775113193595,"y":5.877384048675795,"rotation":-15,"trajectory":40},{"x":0,"y":5.5,"rotation":0,"trajectory":40},{"x":2.5,"y":5.5,"rotation":15,"trajectory":40},{"x":0,"y":3,"rotation":0,"trajectory":35}]}
          default_delay = doc_rx["default_delay"].as<float>();
          
          int array_index = 0;
          for (JsonObject settings_shot : doc_rx["shots"].as<JsonArray>()) {      
            active_shots[array_index].court_x_pos = settings_shot["x"].as<float>();
            active_shots[array_index].court_y_pos = settings_shot["y"].as<float>();
            active_shots[array_index].trajectory = settings_shot["trajectory"].as<float>();
            active_shots[array_index].rotation = settings_shot["rotation"].as<float>();
            active_shots[array_index].left_motor_rpm = settings_shot["lm_speed"].as<int>();
            active_shots[array_index].right_motor_rpm = settings_shot["rm_speed"].as<int>();
            
            Serial.printf("index:%i   x: %f   y: %f   traj: %f   rot: %f   lm: %i   rm: %i" ,array_index,active_shots[array_index].court_x_pos,active_shots[array_index].court_y_pos,active_shots[array_index].trajectory,active_shots[array_index].rotation,active_shots[array_index].left_motor_rpm,active_shots[array_index].right_motor_rpm);
            Serial.println("");
            array_index ++;
          }
          active_shot_array_length = array_index;


       }
       else if (strcmp(command,"update_setup_values")== 0){
         //Example JSON 
          gripper_open_pos = doc_rx["gripper_open_pos"].as<int>(); // "20"
          gripper_closed_pos = doc_rx["gripper_closed_pos"].as<int>(); // "30"
          pusher_catch_pos = doc_rx["pusher_catch_pos"].as<int>(); // "40"
          pusher_push_pos = doc_rx["pusher_push_pos"].as<int>(); // "50"
       }       
       else if (strcmp(command,"stop")== 0){
         //Example JSON 
          stopAll();
       }        
        else if (strcmp(command,"setup_gripper_open")== 0){
          //Example JSON {"command":"setup_gripper_open"}
          Serial.println("Gripper Opening");
          sendStatus("Opening Gripper" , "orange");
          servoGripper.write(gripper_open_pos);
        }
        else if (strcmp(command,"setup_gripper_close")== 0){
          //Example JSON {"command":"setup_gripper_close"}
          Serial.println("Gripper Closing");
          sendStatus("Closing Gripper" , "orange");
          servoGripper.write(gripper_closed_pos);
        }
        else if (strcmp(command,"setup_pusher_catch")== 0){
          //Example JSON {"command":"setup_pusher_catch"}
          Serial.println("Moving to Catch Position");
          sendStatus("Moving Pusher to Catch Position" , "orange");
          servoPusher.write(pusher_catch_pos);
        }
        else if (strcmp(command,"setup_pusher_push")== 0){
          //Example JSON {"command":"setup_pusher_push"}
          Serial.println("Moving to Push Position");
          sendStatus("Moving Pusher to Push Position" , "orange");
          servoPusher.write(pusher_push_pos);
        }
        else if (strcmp(command,"setup_save_servo")== 0){
         //Example JSON  {"command":"setup_save"}
         Serial.println("Save calibration.");
         sendStatus("Saving Settings" , "orange");
         preferences.begin("Servo_Settings", false);
         preferences.putInt("gripper_open", gripper_open_pos);
         preferences.putInt("gripper_closed", gripper_closed_pos);
         preferences.putInt("pusher_catch", pusher_catch_pos);
         preferences.putInt("pusher_push", pusher_push_pos);
         preferences.end();
         sendStatus("Ready" , "green");
        }
        else if (strcmp(command,"setup_esc_max")== 0){
          //Example JSON {"command":"setup_esc_max"}
          Serial.println("Sending max throttle");
          sendStatus("Calibration - Max Throttle" , "orange");
          servoLeftESC.writeMicroseconds(escMaxPulseLength);
          servoRightESC.writeMicroseconds(escMaxPulseLength);
        }
        else if (strcmp(command,"setup_esc_min")== 0){
         //Example JSON {"command":"setup_esc_min"}
         Serial.println("Sending min throttle");
         sendStatus("Calibration - Min Throttle" , "orange");
         servoLeftESC.writeMicroseconds(escMinPulseLength);
         servoRightESC.writeMicroseconds(escMinPulseLength);
          
        }        
        else if (strcmp(command,"setup_rpm_run")== 0){
           //Example JSON {"command":"setup_run","setup_rpm":"500"}
           Serial.println("Calibration Run Motor.");
           sendStatus("Continuous Run" , "orange");
           int setup_rpm = doc_rx["setup_rpm"].as<int>(); // "50"

          //At lower speeds first start with a higher value to get the motor spinning.
          if(setup_rpm<2500){
            servoLeftESC.writeMicroseconds(map(3000, rpmLowSetPoint, rpmHighSetPoint, escLowPulseSetPoint , escHighPulseSetPoint));
            servoRightESC.writeMicroseconds(map(3000, rpmLowSetPoint, rpmHighSetPoint, escLowPulseSetPoint , escHighPulseSetPoint));
            delay(100);
          }
          
          //Run the motor based on the current set points as mapped.
           servoLeftESC.writeMicroseconds(map(setup_rpm, rpmLowSetPoint, rpmHighSetPoint, escLowPulseSetPoint , escHighPulseSetPoint));
           servoRightESC.writeMicroseconds(map(setup_rpm, rpmLowSetPoint, rpmHighSetPoint, escLowPulseSetPoint , escHighPulseSetPoint));
        }
        else if (strcmp(command,"setup_stop")== 0){
           //Example JSON {"command":"setup_stop"}
           Serial.println("Calibration Stop Motor.");
           sendStatus("Stop Motor" , "orange");
           servoLeftESC.writeMicroseconds(escMinPulseLength);
           servoRightESC.writeMicroseconds(escMinPulseLength);
           sendStatus("Ready" , "green");
        }
        else if (strcmp(command,"setup_pos")== 0){
          //Example JSON {"command":"setup_pos"}
           Serial.println("Moving to setup pose.");
           sendStatus("Calibration - Setup Pose" , "orange");

           servoPusher.write(90);
           servoGripper.write(90);
           servoTrajectory.write(0);
           servoRotation.write(90);
          
        }
        else if (strcmp(command,"setup_high_speed")== 0){
          //Example JSON {"command":"setup_high_speed"}
          Serial.println("Going to high speed.");
          sendStatus("Going to high speed." , "orange");
          //Go to 5000RPM based on original scaling
          servoLeftESC.writeMicroseconds(map(5000, 0 , maxSpeed, escMinPulseLength, escMaxPulseLength));
          servoRightESC.writeMicroseconds(map(5000, 0 , maxSpeed, escMinPulseLength, escMaxPulseLength));
        }
        else if (strcmp(command,"setup_low_speed")== 0){
          //Example JSON {"command":"setup_low_speed"}
          Serial.println("Going to low speed.");
          sendStatus("Going to low speed." , "orange");

          //Get the motors spinning, then after we'll slow to 2500
          servoLeftESC.writeMicroseconds(map(3000, 0 , maxSpeed, escMinPulseLength, escMaxPulseLength));
          servoRightESC.writeMicroseconds(map(3000, 0 , maxSpeed, escMinPulseLength, escMaxPulseLength));
          delay(100);


          //Go to 2500RPM based on original scaling
          servoLeftESC.writeMicroseconds(map(2500, 0 , maxSpeed, escMinPulseLength, escMaxPulseLength));
          servoRightESC.writeMicroseconds(map(2500, 0 , maxSpeed, escMinPulseLength, escMaxPulseLength));
        }
        else if (strcmp(command,"setup_adjust_rpm")== 0){
          //Example JSON {"command":"setup_adjust_rpm","setup_rpm_low":"2500","setup_rpm_high":"5000"}
          Serial.println("Adjusting RPM Values");
          sendStatus("Adjusting RPM Values" , "orange");

          //Use the pulse values at 2500 & 5000 along with the measured speeds at those pulse lengths to set speed maping during feeding.
          rpmLowSetPoint = doc_rx["setup_rpm_low"].as<int>(); // "2500"
          rpmHighSetPoint = doc_rx["setup_rpm_high"].as<int>(); // "5000"

          //Prit out the maping points. Perhaps knowing these we can improve the base values.
          Serial.printf("Low Point - Pulse Width:%i   RPM: %i",escLowPulseSetPoint,rpmLowSetPoint);
          Serial.println("");
          Serial.printf("High Point - Pulse Width:%i   RPM: %i",escHighPulseSetPoint,rpmHighSetPoint);
          Serial.println("");

          //Save
          preferences.begin("Servo_Settings", false);
          preferences.putInt("rpmLowSetPoint", rpmLowSetPoint);
          preferences.putInt("rpmHighSetPoint", rpmHighSetPoint);
          preferences.end();

        }
        else if (strcmp(command,"setup_traj_zero")== 0){
          //Example JSON {"command":"setup_traj_zero","setup_traj_correction":"2"}
          Serial.println("Adjusting Trajectory Zero");
          sendStatus("Adjusting Trajectory Zero" , "orange");

          traj_correction = doc_rx["setup_traj_correction"].as<float>(); // "0"

          //zero out the servo correction so that it's not affecting itself then find the correction in servo degrees.
          traj_servo_correction = 0;
          traj_servo_correction = getTrajectoryServoPos (traj_correction)-getTrajectoryServoPos (0); // "500"
          Serial.println(traj_servo_correction);
          //Go to the zero position with the correction factored in.
          servoTrajectory.write(getTrajectoryServoPos(0));

          //Save
          preferences.begin("Servo_Settings", false);
          preferences.putFloat("traj_servo_adj", traj_servo_correction);
          preferences.putFloat("traj_correction", traj_correction);
          preferences.end();

        }
        else if (strcmp(command,"setup_update_settings")== 0){
          //Example JSON {"command":"setup_update_settings","shuttle_check":true,"obstacle_check":true}
          Serial.println("updating settings");
          sendStatus("Updating Settings" , "orange");
          shuttle_check = doc_rx["shuttle_check"].as<bool>(); // "0"
          obstacle_check = doc_rx["obstacle_check"].as<bool>(); // "0"
        }        
        else if (strcmp(command,"get_preset")== 0){
          //Example JSON {"command":"get_preset","preset":"Doubles - Service Return"}
          Serial.println("Request Preset Load.");
          sendStatus("Loading Preset." , "orange");
          //sendShots();
          
        }
        else{
          Serial.println("Command not recognized.");
          sendStatus("Command Not Recognized" , "red");
        }       
      }
      Serial.println("");

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

// Callback: send homepage
void onIndexRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                 "] HTTP GET request (index) of " + request->url() );
  request->send(SPIFFS, "/ace.html", "text/html");
}

// Callback: send style sheet
void onCSSRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                 "] HTTP GET request (css) of " + request->url());
  request->send(SPIFFS, "/simple.css", "text/css");
}

// Callback: send logo image
void onLogoRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                 "] HTTP GET request (image) of " + request->url());
  request->send(SPIFFS, "/ace_logo.png", "image/png");
}

// Callback: send 404 if requested file does not exist
void onPageNotFound(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                 "] HTTP GET request (404) of " + request->url());
  request->send(404, "text/plain", "Not found");
}

// Send status update
void sendStatus(String status_text, String status_color){
  String jsonString = "";
  JsonObject object = doc_tx.to<JsonObject>();
  object["command"] = "status";
  object["statusText"] = status_text;
  object["statusColor"] = status_color;
  serializeJson(doc_tx, jsonString);
  //Serial.printf("Sending: %s\n", jsonString);
  //webSocket.sendTXT(client_num, msg_buf);
  webSocket.broadcastTXT(jsonString);               // send JSON string to all clients
}
// Send parameters
void sendParameters(){
  String jsonString = "";
  JsonObject object = doc_tx.to<JsonObject>();

  object["command"] = "load_setup_values";

  //send servo positions
  object["gripper_open"] = gripper_open_pos;
  object["gripper_closed"] = gripper_closed_pos;
  object["pusher_catch"] = pusher_catch_pos;
  object["pusher_push"] = pusher_push_pos;
  object["rmpLowSetPoint"]=rpmLowSetPoint;
  object["rmpHighSetPoint"]=rpmHighSetPoint;
  object["traj_correction"]=traj_correction;

  //send global values
  Serial.println("Load Globals.");
  preferences.begin("Globals", false);   
  object["velocity_correction"] = preferences.getFloat("velocity_adj", 0);
  object["flight_correction"] = preferences.getFloat("flight_adj", 0);
  object["height"] = roundf(preferences.getFloat("launcher_y", 1.5)*100)/100;
  preferences.end();  

  serializeJson(doc_tx, jsonString);
  Serial.printf("Sending: %s\n", jsonString);
  //webSocket.sendTXT(client_num, msg_buf);
  webSocket.broadcastTXT(jsonString);               // send JSON string to all clients
}

// Send Shots
void sendShots(){
  String jsonString = "";
  JsonObject object = doc_tx.to<JsonObject>();

  object["command"] = "load_shot_values";
  JsonObject Shots = object.createNestedObject("shots");

  for(int i = 0; i<10;i++){
            
            JsonObject Shot = Shots.createNestedObject(String(i));
            Shot["x"] = active_shots[i].court_x_pos;
            Shot["y"] = active_shots[i].court_y_pos;
            Shot["rotation"] = active_shots[i].trajectory;
            Shot["trajectory"] = active_shots[i].rotation;
  }



  //object["shots"] = active_shots;
  //myJSON = '["Ford", "BMW", "Fiat"]';

  //JsonArray 


  /* FROM RECEIPT OF SHotS
   for (JsonObject settings_shot : doc_rx["shots"].as<JsonArray>()) {      
            active_shots[array_index].court_x_pos = settings_shot["x"].as<float>();
            active_shots[array_index].court_y_pos = settings_shot["y"].as<float>();
            active_shots[array_index].trajectory = settings_shot["trajectory"].as<float>();
            active_shots[array_index].rotation = settings_shot["rotation"].as<float>();
            active_shots[array_index].left_motor_rpm = settings_shot["lm_speed"].as<int>();
            active_shots[array_index].right_motor_rpm = settings_shot["rm_speed"].as<int>();*/

//DynamicJsonDocument doc(1024);
//JsonObject wifi  = doc.createNestedObject("wifi");
//wifi["SSID"] = "TheBatCave";
  
  serializeJson(doc_tx, jsonString);
  Serial.printf("Sending: %s\n", jsonString);
  //webSocket.sendTXT(client_num, msg_buf);
  webSocket.broadcastTXT(jsonString);               // send JSON string to all clients
}


   


/***********************************************************
   Main
*/

void setup() {
  
  //Setup the servo pins
  pinMode(servoPusherPin, OUTPUT);
  pinMode(servoGripperPin, OUTPUT);
  pinMode(servoTrajectoryPin, OUTPUT);
  pinMode(servoRotationPin, OUTPUT);
  pinMode(servoLeftESCPin, OUTPUT);
  pinMode(servoRightESCPin, OUTPUT);

  //Setup button IO
  pinMode(startPin, INPUT_PULLUP);
  pinMode(stopPin, INPUT_PULLUP);  

  //Setup Through Beam Sensor
  pinMode(throughBeamPin, INPUT_PULLUP);

  //Setup onboard LED for indication
  pinMode(throughBeamIndicatorPin, OUTPUT);


  //Setup Ultrasonic Sesnor
  pinMode(ultrasonicTriggerPin, OUTPUT);
  pinMode(ultrasonicEchoPin, INPUT_PULLUP);
  
  // Start Serial port
  Serial.begin(115200);

  // Make sure we can read the file system
  if ( !SPIFFS.begin()) {
    Serial.println("Error mounting SPIFFS");
    while (1);
  }
/**/
  //STAND-ALONE ACCESS POINT FOR DIRECT CONNECTION
  // Start access point
  WiFi.softAP(ssid, password);
  // Print our IP address
  Serial.println();
  Serial.println("AP running");
  Serial.print("My IP address: ");
  Serial.println(WiFi.softAPIP());
/**/
/*
  //CONNECT TO EXISTING NETWORK FOR TESTING
  //Connect to Network
  ssid = "";
  password = "";
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  */


  if(!MDNS.begin("ace")) {
     Serial.println("Error starting mDNS");
      while(1) {
         delay(1000);
      }
  }
  Serial.println("mDNS Responder Started");

  // Add service to MDNS-SD
  MDNS.addService("http", "tcp", 80); //this catches http://ace.local
  MDNS.addService("", "tcp", 80); //this catches ace.local

   // On HTTP request for root, provide index.html file
  server.on("/", HTTP_GET, onIndexRequest);

  // On HTTP request for style sheet, provide style.css
  server.on("/simple.css", HTTP_GET, onCSSRequest);

  // On Logo request provide logo
  server.on("/ace_logo.png", HTTP_GET, onLogoRequest);

  // Start web server
  server.begin();

  // Handle requests for pages that do not exist
  server.onNotFound(onPageNotFound);

  // Start WebSocket server and assign callback
  webSocket.begin();
  
  webSocket.onEvent(onWebSocketEvent);

  //Load calibration settings.
   Serial.println("Load calibration.");
   preferences.begin("Servo_Settings", false);
   gripper_open_pos=preferences.getInt("gripper_open", 90);
   gripper_closed_pos=preferences.getInt("gripper_closed", 90);
   pusher_catch_pos=preferences.getInt("pusher_catch", 90);
   pusher_push_pos=preferences.getInt("pusher_push", 90);

   rpmLowSetPoint=preferences.getInt("rpmLowSetPoint", 2500);
   rpmHighSetPoint=preferences.getInt("rpmHighSetPoint", 5000);
   traj_servo_correction=preferences.getFloat("traj_servo_adj", 0);
   traj_correction =preferences.getFloat("traj_correction", 0);
   preferences.end();

  //Setup servos
  servoPusher.attach(servoPusherPin, servoMinPulseLength, servoMaxPulseLength);
  servoGripper.attach(servoGripperPin, servoMinPulseLength, servoMaxPulseLength);
  servoTrajectory.attach(servoTrajectoryPin, servoMinPulseLength, servoMaxPulseLength);
  servoRotation.attach(servoRotationPin, servoMinPulseLength, servoMaxPulseLength);
  servoLeftESC.attach(servoLeftESCPin, escMinPulseLength, escMaxPulseLength);
  servoRightESC.attach(servoRightESCPin, escMinPulseLength, escMaxPulseLength);  

  //Go to default servo positions
  servoGripper.write(gripper_closed_pos);
  servoPusher.write(pusher_catch_pos);
  servoTrajectory.write(0);
  servoRotation.write(90);

  //Stop the esc's.
  servoLeftESC.writeMicroseconds(escMinPulseLength);
  servoRightESC.writeMicroseconds(escMinPulseLength);

}



void loop() {
  static int last_time = millis();
  static int last_sonar_time = millis();
  static int last_through_beam_time = millis();
  static int last_state_time = millis();
  static int start_time = millis();
  static int blocked_counts = 0;
  static int count_down = 5;
  static bool countdown_required = true;
 
  int current_time = millis();

  static int startStatus = 0;
  static int stopStatus = 0;
  startStatus = digitalRead(startPin);
  stopStatus = digitalRead(stopPin);

  //If the physical start button is pressed then start.
  if (startStatus == 0){
    
  }
  //If the physical stop button is pressed then stop.
  if (stopStatus == 0){
    stopAll();  
  }

  //Serial.print("Start Button Value:" );
  //Serial.println(startStatus);

  //Serial.print("Stop Button Value:" );
  //Serial.println(stopStatus);
  
  //Always start by checking the sonar sensor for any obstructions & stop imediately if detected.
  static bool sonar_long_blocked = false;
  
  if(current_time - last_sonar_time >= 100){
    int distance = sonar.ping_cm();
    //Serial.print("Distance:" );
    //Serial.print(distance);
    //Serial.println("cm");
    last_sonar_time = current_time;
    //sendStatus("Distance: " + String(distance, 6) + "cm", "red"); //String(distance, 6)
    
    //Sensor library returns 0 for any distances greater than the max detection window set when initialized.
    //So in this case a value of zero means free from obstructions.
    if(distance>0){ 
      //If the sensor is blocked for 4 sonar pings (1s) then stop all.
      if (blocked_counts >= 4 ){
         
           if(system_state>0 && obstacle_check == true && sonar_long_blocked == false) {
              //Stop all
             sendStatus("Obstruction Detected", "red");   
             stopAll();
             sonar_long_blocked = true;
           }       
                  
      }
      else{
        blocked_counts++;
      }      
    }
    else{
      blocked_counts=0;   
      if (sonar_long_blocked==true){
        sonar_long_blocked = false;
        sendStatus("Ready", "green");
      }      
    }    
  }

  //Update shuttle through-beam status.
  static int throughBeamClear = 0;
  if(current_time - last_through_beam_time >= 50){
    throughBeamClear = digitalRead(throughBeamPin);
    digitalWrite(throughBeamIndicatorPin,throughBeamClear); 
    //Serial.print("Beam Clear:" );
    //Serial.println(throughBeamClear);
    last_through_beam_time = current_time;
  }

  // Look for and handle WebSocket data
  webSocket.loop();  

  static bool skip_open_delay = false;
  static int shuttle_drop_attempts = 0;
  static int current_sequence_index = 0;
  static int current_shot = 0;


  static int current_shot_delay=0;
  static int current_lm_speed=0;
  static int current_rm_speed=0;
  static float current_trajectory=0;
  static float current_rotation=0;

  static float traj_servo_pos = 0;
  static float rot_servo_pos = 0;
  static int last_shot_delay=0;
  
  //Do a countdown before starting feeding when first activated to allow player time to get to position.
  if(system_state>0 && countdown_required == true &&  system_state != ST_RESET) //
  {    
    if(current_time - last_time >= 1000){
      
      String status_string = "Starting In ";
      status_string = status_string + count_down + "s";
      Serial.println(status_string);
      sendStatus(status_string , "green");
      last_time = millis();

      if (count_down <=1){ 
        countdown_required=false;        
      }
      else count_down = count_down - 1;        
    }
    return;
  }

  int open_delay = 500;
  int close_delay = 500;
  
  switch(system_state)
  {
    case ST_IDLE:
      last_state_time = current_time;
      countdown_required = true;
      count_down = 5;
      current_sequence_index = 0;
           
      break;
    case ST_START:      
      skip_open_delay = false;
      shuttle_drop_attempts = 0;
      stop_launcher = false;     
      servoPusher.write(pusher_catch_pos); 

      //If a countdown is required do not add additional delay before the shot.
      if(countdown_required==true) last_shot_delay = 0;

      //If a countdown is not required, or if we've counted down to 2s remaining (feed prep length)
      if(countdown_required == false || current_time - last_state_time>(5000-2000)){  
        Serial.print("Starting");   
         
        if (ACTIVE_PLAY_TYPE == RANDOM_SHOTS){ 
          current_shot = random(0,active_shot_array_length); //lower bound inclusive, upper bound exclusive.
          current_shot_delay = (int)(default_delay*1000); //This should be based on the default shot delay.
        }        
        if (ACTIVE_PLAY_TYPE == RANDOM_ORDER){
          current_sequence_index = random(0,sequence_length); //lower bound inclusive, upper bound exclusive.
          current_shot = shot_sequence[current_sequence_index].ID;
          current_shot_delay = (int)(shot_sequence[current_sequence_index].delay*1000);
      
        }
        if (ACTIVE_PLAY_TYPE == PLAY_ONCE || ACTIVE_PLAY_TYPE == REPEAT_SHOTS){
          current_shot = shot_sequence[current_sequence_index].ID;
          current_shot_delay = (int)(shot_sequence[current_sequence_index].delay*1000); 
        }
        //Subtract the delay built into the shot sequence. Approximately 2 second delay plus the time it takes to push into the feeders.
        //current_shot_delay -= 2100;
        //current_shot_delay = max(0,current_shot_delay);
          
        current_trajectory = active_shots[current_shot].trajectory;
        current_rotation = active_shots[current_shot].rotation;
        current_lm_speed = active_shots[current_shot].left_motor_rpm;
        current_rm_speed = active_shots[current_shot].right_motor_rpm;

        //Serial.printf("shot index:%i   x: %f   y: %f   traj: %f   rot: %f   lm: %i   rm: %i" ,current_shot,active_shots[current_shot].court_x_pos,active_shots[current_shot].court_y_pos,active_shots[current_shot].trajectory,active_shots[current_shot].rotation,active_shots[current_shot].left_motor_rpm,active_shots[current_shot].right_motor_rpm);
        //Serial.println("");
        
        last_state_time = current_time;
        start_time = current_time;
        system_state = ST_GRIPPER_OPEN;
      }
      break;
    case ST_GRIPPER_OPEN:
      if( current_time - last_state_time >= 500 || skip_open_delay)
      {
        Serial.print(" -> Gripper Opening");
        servoGripper.write(gripper_open_pos);
        skip_open_delay = false;
        shuttle_drop_attempts++;
        last_state_time = current_time;
        system_state = ST_GRIPPER_CLOSED;
      }          
      break;
    case ST_GRIPPER_CLOSED:
      if( current_time - last_state_time >= 500)
      {
        Serial.print(" -> Gripper Closing");
        servoGripper.write(gripper_closed_pos);
        last_state_time = current_time;
        system_state = ST_SHUTTLE_CHECK;
      }          
      break;
    case ST_SHUTTLE_CHECK:
      Serial.print(" -> Shuttle Check");
      if( throughBeamClear == true && shuttle_check == true) //If the through beam sensor is not blocked and shuttle checking is enabled the either try again or stop.
      {
        //If there is no shuttle loaded try cycling the gripper.
        if(shuttle_drop_attempts<2){
          skip_open_delay = true;
          system_state = ST_GRIPPER_OPEN;
          //current_shot_delay -= 500; //Subtract the extra time to open the gripper from the current requested shot delay.
          //current_shot_delay = max(0,current_shot_delay);
        }
        else{
          sendStatus("Shuttle Feed Failure", "red");
          stop_launcher=true;
          system_state = ST_RESET;
        }         
      } 
      else
      {        
        system_state = ST_PREP_LAUNCH;
      }  
      last_state_time = current_time;       
      break;
    case ST_PREP_LAUNCH:
      if( current_time - last_state_time >= 500 && current_time - start_time >=  last_shot_delay-1000) //wait at least 500ms, but if the delay is longer wait until 500ms before the shot.
      {
        Serial.print(" -> Launch Prep");
        sendStatus("Feeding" , "green");  
        //Move to the desired position & start the motors.
        //Serial.printf("Feed Shuttle   LM: %i   RM: %i   Trajectory: %f   Rotation: %f", current_lm_speed, current_rm_speed, current_trajectory, current_rotation);
        //Serial.println("");
         
        //At lower speeds first start with a higher value to get the motor spinning.
        if(current_lm_speed<2500 || current_rm_speed<2500){
            servoLeftESC.writeMicroseconds(map(3000, rpmLowSetPoint, rpmHighSetPoint, escLowPulseSetPoint , escHighPulseSetPoint));
            servoRightESC.writeMicroseconds(map(3000, rpmLowSetPoint, rpmHighSetPoint, escLowPulseSetPoint , escHighPulseSetPoint));
            delay(100);
        }

        servoLeftESC.writeMicroseconds(map(current_lm_speed, rpmLowSetPoint, rpmHighSetPoint, escLowPulseSetPoint , escHighPulseSetPoint));
        servoRightESC.writeMicroseconds(map(current_rm_speed, rpmLowSetPoint, rpmHighSetPoint, escLowPulseSetPoint , escHighPulseSetPoint));
        traj_servo_pos = getTrajectoryServoPos(current_trajectory);
        rot_servo_pos = getRotationServoPos(current_rotation);                

        //Serial.printf("Trajectory Servo: %f   Rotation Servo: %f", traj_servo_pos,rot_servo_pos);
        //Serial.println("");
        servoTrajectory.write(traj_servo_pos); 
        servoRotation.write(rot_servo_pos); 
        last_state_time = current_time;
        system_state = ST_LAUNCH;
      }          
      break;
    case ST_LAUNCH:
      if( current_time - last_state_time >= 1000 && current_time - start_time >=  last_shot_delay) //Minimum of 500 to allow motors to reach speed.
      {
        Serial.print(" -> Launching");
        servoPusher.write(pusher_push_pos);
        last_state_time = current_time;
        system_state = ST_RESET;
      }          
      break;
    case ST_RESET:

      if( current_time - last_state_time >= 1000 )
      { 
        Serial.print(" -> Resetting");
        Serial.printf(" | Details | index:%i   x: %f   y: %f   traj: %f   traj_servo: %f   rot: %f   rot_servo: %f   lm: %i   rm: %i" ,current_shot,active_shots[current_shot].court_x_pos,active_shots[current_shot].court_y_pos,active_shots[current_shot].trajectory, traj_servo_pos,active_shots[current_shot].rotation,rot_servo_pos,active_shots[current_shot].left_motor_rpm,active_shots[current_shot].right_motor_rpm);
        Serial.println("");
        servoLeftESC.writeMicroseconds(escMinPulseLength);
        servoRightESC.writeMicroseconds(escMinPulseLength);
        servoPusher.write(pusher_catch_pos); 
        servoGripper.write(gripper_closed_pos);

        last_state_time = current_time;

        //Store the delay attached to the current shot to time the next shot.
        last_shot_delay = current_shot_delay;
        
        if(stop_launcher==true){                    
          sendStatus("Stopping" , "orange");
          system_state = ST_IDLE;         
        }
        else{
          //plan to start the next shot
          system_state = ST_START;
          current_sequence_index = current_sequence_index + 1;

          if (current_sequence_index>=sequence_length){
            current_sequence_index = 0;
            //If the pattern is only to be played once then stop.            
            if (ACTIVE_PLAY_TYPE == PLAY_ONCE){
              sendStatus("Stopping" , "orange");
              system_state = ST_IDLE; 
            }              
          }
        }
      }          
      break;
  }
}

//Prevent any further shuttle feeds being initialized then stop the esc's.
void stopAll(){
  Serial.println("Stop All");
  sendStatus("Stopping" , "orange");
  stop_launcher=true;
  if (system_state!=ST_RESET && system_state!=ST_IDLE){
    system_state = ST_RESET;
  }  
}

//Get the required wheel speeds for a given court position
void getWheelSpeeds (float rotation, float x, float y, float v0, float &leftWheelRPM, float &rightWheelRPM){
  float launcherX = 0;
  float launcherY= -2;
  float requiredRotation = atan((x -launcherX)/(y-launcherY))/PI*180; //In degrees
  //float shotFinalRotation = parseFloat(shots[i].rotation)+ min(max(requiredRotation-shots[i].rotation,-maxWheelCorrection),maxWheelCorrection);
  float wheelImpartAngle = requiredRotation-rotation; //In degrees
  float wheelRadius =104/1000/2; //In m

  //float wheelCenterToCenter = 125;
  //float shuttleRadius = 26.5/1000/2; //25-28mm diameter
  float releaseAngle = 16.7;//Solved graphically.

  releaseAngle = releaseAngle/180*PI; //Convert to radians
	wheelImpartAngle = wheelImpartAngle/180*PI; //Convert to radians

  //When we consider the shuttle center we must consider that a speed mismatch results in a rolling situation.
  //This gives the expected result at matched speeds as equal roller speeds will have the shuttle traveling at almost the wheel velocity.
  //float velYComponent = v1*cos(releaseAngle)/2 + v2*cos(releaseAngle)/2 //Component going directly away from the launcher.
  //float velXComponent = v1*sin(releaseAngle)/2 - v2*sin(releaseAngle)/2 //Component going awway from the launcher center-line.
  //float angle = atan(velXComponent/velYComponent);

  // Rearange
  //float velYComponent*2/cos(releaseAngle) = v1 + v2 //Component going directly away from the launcher.
  //float velXComponent*2/sin(releaseAngle) = v1 - v2 //Component going awway from the launcher center-line.

  // Rearange & Substitute
  //v1 = velYComponent*2/cos(releaseAngle) - v2
  //v2 =  v1 - velXComponent*2/sin(releaseAngle)
  //Substitute
  //v1 = velYComponent*2/cos(releaseAngle)  + velXComponent*2/sin(releaseAngle)
  //v1 = (velYComponent*2/cos(releaseAngle)  + velXComponent*2/sin(releaseAngle))/2

  //The velYComponent component must be V0 to achieve the required shuttle distance.
  float velYComponent = v0;
  float velXComponent= tan(wheelImpartAngle)*v0;
  float v1 = (velYComponent*2/cos(releaseAngle)  + velXComponent*2/sin(releaseAngle))/2;
  float v2 = v1 - velXComponent*2/sin(releaseAngle);

  //This will update the variables passed to the function.
  rightWheelRPM = v1 / wheelRadius / (2*PI) * 60;
  leftWheelRPM = v2 / wheelRadius / (2*PI) * 60;	
}

//Get the required servo angle to set the motor mount plate at the desired trajectory angle.
int getTrajectoryServoPos (float inputAngle){
  //Angle limits based on linkage and geometry.
  if (inputAngle<-4) inputAngle = -4;
  if (inputAngle>40) inputAngle = 40;  
  //Serial.printf("Trajectory Input: %f", inputAngle);
  //Serial.println("");

  float pivotLinkDistance = 20; //Distance from motor plate pivot to driving linkage pin.
  float pivotLinkAngle = -11.5; //Angle between motor plate horizontal and line between pivot and linkage pin. D3 was -14.5.
  float linkLength = 51; //D3 was 45
  float servoPosX = 54; //D3 was 43
  float servoPosY = 25; //D3 was 30
  float servoHornLength = 7.2; //D3 was 7.5

  float effectiveAngle = radians( pivotLinkAngle-inputAngle);
  float x1 = pivotLinkDistance*cos(effectiveAngle);
  float y1 = pivotLinkDistance * sin(effectiveAngle);
  float c = sqrt(pow(servoPosX - x1, 2) + pow(servoPosY - y1, 2)); //the distance between the motor mount linkage pin to the servo center.
  float beta = atan((servoPosX-x1)/(servoPosY-y1)); //The angle between vertical and the line C
  float alpha  = acos((pow(linkLength,2)-pow(c,2)-pow(servoHornLength,2))/(-2*c*servoHornLength));
  //float servoAngle = PI - (alpha + beta); //Servo zero is top dead center.
  float servoAngle = PI/4 - beta+alpha; //Servo zero is 45 degrees off of vertical pointing towards the front. Approx. in line with linkage.
  //Serial.printf("x1: %f   y1: %f   c: %f   beta: %f   alpha: %f   servoAngle: %f   ", x1, y1, c, beta, alpha, servoAngle);
  //Serial.println("");
  servoAngle = degrees(servoAngle); //convert to degrees

  servoAngle+=traj_servo_correction;

  //Set hard limits based on servo values.
  if (servoAngle<0) servoAngle = 0;
  if (servoAngle>180) servoAngle = 180;
  if (isnan(servoAngle)) servoAngle = 0;      
  //Serial.printf("Trajectory Servo: %f", servoAngle);
  //Serial.println("");
  return servoAngle;
}

//Get the required servo angle to set the chute at the desired rotation angle.
int getRotationServoPos (float inputAngle){

  //Angle limits based on linkage and geometry.
  if (inputAngle<-19) inputAngle = -19;
  if (inputAngle>19) inputAngle = 19;  
  //Serial.printf("Rotation Input: %f", inputAngle);
  //Serial.println("");
  
  inputAngle = radians(inputAngle); //calculate angle in radians
  float a1 = 72.82618; //Chute rotation center to servo center
  float a2 = 24; //Servo horn arm controlling rotation
  float servoAngle = PI - inputAngle - asin((a1*sin(inputAngle))/a2) - PI/2;
  
  servoAngle = PI-servoAngle;

  servoAngle = degrees(servoAngle); //convert to degrees

 

  //Set hard limits based on servo values.
  if (servoAngle<0) servoAngle = 0;
  if (servoAngle>180) servoAngle = 180;   
  if (isnan(servoAngle)) servoAngle = 0;      
  //Serial.printf("Rotation Servo: %f", servoAngle);
  //Serial.println("");
  return servoAngle;
}
