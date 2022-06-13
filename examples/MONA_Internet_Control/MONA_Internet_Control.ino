/* 
 *  Alex Wende SparkFun Electronics
 *  ESP32 Web Controlled Motor Test
 *  
 *  To use this code, download the ESP32WebServer library from:
 *  https://github.com/Pedroalbuquerque/ESP32WebServer
 *  
 *  In this Example we'll use the arrow keys from our keyboard to send commands to the Serial 
 *  Controlled Motor Driver. When the ESP32 connects to the WiFi network, the ESP32 sends the
 *  IP address over the Serial to the terminal window at 9600 baud. Copy and paste the IP 
 *  address into your brower's window to go to the ESp32's web page. From there, use the arrow keys
 *  to control the motors.
 *  
 *  UP Arrow - Drive Forward
 *  DOWN Arrow - Drive in Reverse
 *  LEFT Arrow - Turn Left
 *  RIGHT Arrow - Turn Righ
 *  
 *  If the motors aren't spinning in the correct direction, you'll need to to change the motor
 *  number and/or motor direction in the handleMotors() function.
 */

#include <WiFiClient.h>
#include <ESP32WebServer.h>
#include "Mona_ESP_lib.h"
#include <WiFi.h>
#include <SPI.h>
#include "index_video.h"  //change this to represent the required html code inside the relevant .h file

const char* ssid = "*******"; //ssid for your router
const char* password = "********"; //password for your router

ESP32WebServer server(80);  //Choose required port number
bool IR_values[5] = {false, false, false, false, false};
int threshold = 35;

void handleRoot()
{  
  String s = MAIN_page; //Read HTML contents
  server.send(200, "text/html", s); //Send web page
}

//XML page to listen for motor commands
void handleMotors() 
{ 
  String motorState = "OFF";
  String t_state = server.arg("motorState"); //Refer  xhttp.open("GET", "setMotors?motorState="+motorData, true);
 
  Motors_stop(); //Disable motors
  delay(50);
  //Serial.println("F");
  Serial.println(t_state);

  if(t_state.startsWith("D"))  //Drive Forward (UP Arrow)
  {
    Motors_forward(100);
    //delay(100);
    //Serial.println("A");
  }
  else if(t_state.startsWith("U")) //Reverse (DOWN Arrow)
  {
    Motors_backward(100);
    //delay(50);
   // Serial.println("B");
  }
  else if(t_state.startsWith("L")) //Turn Right (Right Arrow)
  {
    Motors_spin_right(100);
    //delay(50);
   // Serial.println("C");
  }
  else if(t_state.startsWith("R")) //Turn Left (LEFT Arrow)
  {
    Motors_spin_left(100);
    //delay(50);
    //Serial.println("D");
  }
   //Motors_stop(); //Disable motors
   server.send(200, "text/plain", motorState); //Send web page
}

// cannot handle request so return 404
void handleNotFound()
{
  server.send(404, "text/plain", "File Not Found\n\n");
}

void setup(){
  Mona_ESP_init();
  //Turn on Blue light on LEDS to show initialization process
  Set_LED(1,0,0,20);
  Set_LED(2,0,0,20);
  Serial.begin(9600); //SCMD + debug messages
  WiFi.begin(ssid, password); //WiFi network to connect to

  Serial.println();

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  /* register the callbacks to process client request */
  /* root request we will read the memory card to get 
  the content of index.html and respond that content to client */
  server.on("/", handleRoot);
  server.on("/setMotors", handleMotors);
  server.onNotFound(handleNotFound);
  server.begin(); //Start the web server

  Serial.println("HTTP server started");
}
void loop(){
  server.handleClient();
  //Serial.println("E");
}
