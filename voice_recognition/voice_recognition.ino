//include required libraries
#include <SoftwareSerial.h>
#include "VoiceRecognitionV3.h"

//declare used objects
SoftwareSerial serial(5,4);
VR myVR(3,2);

//declare global variables
uint8_t buf[255];
uint8_t records[7]={0,1,2,3,4,5,6};
char c[7][10]={"x","two","pizza","fish","beef","tea","coffe"};

void setup() {
	//declare pin as output
	pinMode(13,OUTPUT);
	digitalWrite(13,0);
	//begin erial comunication
	Serial.begin(9600);
	serial.begin(9600);
	//begin vr comunication
	myVR.begin(9600);
	//laod the recordings
	myVR.load(records,7,buf);
}

void loop() {
	//if voice is recognized
	if(myVR.recognize(buf,50)>0){
		//blink the led
		digitalWrite(13,1);
		//send the data to the other arduino
		serial.print(c[buf[1]]);
		if(buf[1]!=1)
			serial.println();
		else
			serial.print(' ');
		//print the data to the seral(for debugging)
		Serial.print(c[buf[1]]);
		if(buf[1]!=1)
			Serial.println();
		else
			Serial.print(' ');
		delay(300);
		digitalWrite(13,0);
	}
	//used for debugging
	if(Serial.available()){
		//read a character from sthe serial
		int x=Serial.read();
		if(x=='c'){
			//clear the vr
			if(myVR.clear())
				Serial.println("error");
			else
				Serial.println("cleared");
		}
		else if(x=='l'){
			//load recoring to the vr
			if(myVR.load(records,7,buf))
				Serial.println("loaded");
			else
				Serial.println("error");
		}
		else if(x>='0'&&x<='6'){
			//train the vr chip
			uint8_t vect[7]={0};
			vect[0]=x-'0';
			Serial.print("training ");
			Serial.print(vect[0]);
			Serial.print(' ');
			Serial.println(c[vect[0]]);
			myVR.train(vect,1,buf);
		}
	}
}
