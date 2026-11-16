/*
  Example_C_style.ino - Usage of the libraries Example
  Using the MonaV2 library in C style.
  Created by Bart Garcia, November 2020.
  bart.garcia.nathan@gmail.com
  Released into the public domain.
*/
//Include the Mona_ESP library
#include <Wire.h>
#include "Mona_ESP_lib.h"

//Testing to get a chip ID
#include <WiFi.h>


//Variables
int IR_values1,IR_values2,IR_values3,IR_values4,IR_values5;
int threshold = 50;
int old_point=0;
int counter = 0;

//State Machine Variable
// 0 -move forward , 1 - forward obstacle , 2 - right proximity , 3 - left proximity
int state, old_state;

bool led_state = 1;

void setup()
{
	//Initialize the MonaV2 robot
	Mona_ESP_init();
	//Initialize Serial comms
	Serial.begin(115200);
  //Initialize variables
  state=0;
  old_state=0;
	//Set Breakout pins as outputs to test
	pinMode(breakout_14, OUTPUT);
	pinMode(breakout_27, OUTPUT);
	pinMode(breakout_26, OUTPUT);
	pinMode(breakout_25, OUTPUT);
	Serial.println("Initialization finished, start testing...");
}

void loop(){
	int read;
	//Read control command
	while (Serial.available() > 0) {
    state = Serial.parseInt();
		//Serial.println("Something was read ");
	}
	if(state ==0){
		Motors_stop();
		Serial.println("This is state 0, testing RGN leds");
		Set_LED(1,20,0,0);
		Set_LED(2,0,20,0);
		delay(500);
		Set_LED(1,0,20,0);
		Set_LED(2,0,0,20);
		delay(500);
		Set_LED(1,0,0,20);
		Set_LED(2,20,0,0);
		delay(500);
		Set_LED(1,0,20,0);
		Set_LED(2,0,20,0);
		uint64_t chipid;
		chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
		Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipid>>32));//print High 2 bytes
		Serial.printf("%08X\n",(uint32_t)chipid);//print Low 4bytes.

		byte mac[6];
		WiFi.macAddress(mac);
		String uniq =  String(mac[0],HEX) +String(mac[1],HEX) +String(mac[2],HEX) +String(mac[3],HEX) + String(mac[4],HEX) + String(mac[5],HEX);
		Serial.println(uniq);
	}
	if(state ==1){
		Motors_stop();
		Serial.println("This is state 1, testing Breakout pins");
		digitalWrite(breakout_14,LOW);
		digitalWrite(breakout_27,LOW);
		digitalWrite(breakout_26,LOW);
		digitalWrite(breakout_25,LOW);
		delay(500);
		digitalWrite(breakout_14,HIGH);
		digitalWrite(breakout_27,HIGH);
		digitalWrite(breakout_26,HIGH);
		digitalWrite(breakout_25,HIGH);
		delay(500);
	}
	if(state ==2){
		Motors_stop();
		Serial.println("This is state 2, testing the IR sensors");
		Enable_IR(1);
		Enable_IR(2);
		Enable_IR(3);
		Enable_IR(4);
		Enable_IR(5);
		IR_values1=Read_IR(1);
		IR_values2=Read_IR(2);
		IR_values3=Read_IR(3);
		IR_values4=Read_IR(4);
		IR_values5=Read_IR(5);
		delay(500);
		Disable_IR(1);
		Disable_IR(2);
		Disable_IR(3);
		Disable_IR(4);
		Disable_IR(5);
		char buffer[100];
		sprintf(buffer,"%d\t %d\t %d\t %d\t %d\t",IR_values1,IR_values2,IR_values3,IR_values4,IR_values5);
		Serial.println(buffer);
		delay(500);
	}
	if(state==3){
		Serial.println("This is state 3, testing the motors");
		Motors_spin_left(100);
		delay(500);
		Motors_spin_right(100);
		delay(500);
		Motors_stop();
	}
	if(state == 4){
		int current_point = 0;
		if(counter ==0){
			Serial.println("This is state 4, testing the motor encoders");
		}
		//Motors_spin_left(100);
		Right_mot_forward(100);
		if(counter<=200){
      Set_LED(1,0,20,0);
			current_point = digitalRead(Mot_right_feedback);
		}
		if(counter<=400 && counter >200){
      Set_LED(1,20,20,0);
			current_point = digitalRead(Mot_right_feedback_2);
		}
		if(counter<=600 && counter >400){
      Set_LED(1,0,0,0);
      Set_LED(2,0,20,0);
			Left_mot_forward(100);
			Right_mot_forward(0);
			current_point = digitalRead(Mot_left_feedback);
		}
		if(counter<=800 && counter >600){
      Set_LED(2,20,20,0);
			current_point = digitalRead(Mot_left_feedback_2);
		}
		if( current_point != old_point){
			counter++;
			old_point = current_point;
		}
		if(counter==200){
			Serial.println("Counter works on righ 1");
		}
		if(counter==400){
			Serial.println("Counter works on righ 2");
		}
		if(counter==600){
			Serial.println("Counter works on left 1");
		}
		if(counter>=800){
			Serial.println("Counter works on left 2");
			counter =0;
		}
	}
	if(state==5){
		Serial.println("This is state 5, testing the IMU");
		//Create variables for sensors data

		sensors_event_t a, m, g, temp;
		IMU_read_sensors(&a, &m, &g, &temp);
		//IMU_read_sensors();

		Serial.print("Accel X: "); Serial.print(a.acceleration.x);
	  Serial.print("\tY: "); Serial.print(a.acceleration.y);
	  Serial.print("\tZ: "); Serial.print(a.acceleration.z);

	  Serial.print(" Mag X: "); Serial.print(m.magnetic.x);
	  Serial.print("\tY: "); Serial.print(m.magnetic.y);
	  Serial.print("\tZ: "); Serial.print(m.magnetic.z);

	  Serial.print(" Gyro X: "); Serial.print(g.gyro.x);
	  Serial.print("\tY: "); Serial.print(g.gyro.y);
	  Serial.print("\tZ: "); Serial.print(g.gyro.z);

	  Serial.println();
	  delay(200);

	}

}
