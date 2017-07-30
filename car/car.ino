//include required libraries
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <SPI.h>
#include <RF24.h>
#include <ISD1700.h>

#define serial Serial2

//sensors and chips used
RF24 radio(49,48);
MPU6050 mpu;
ISD1700 chip(53);

//general variables
byte addr[][6]={"1Node","2Node"};
byte task=0,taskdone=1,secondary=0;
float p[4][2]={{100,0},{0,100},{200,100},{100,200}};
float ct=0,orientation=90;
unsigned long long Time=0;
float coordx=100,coordy=0;

//gyroscope variables
bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;    
uint16_t packetSize;   
uint16_t fifoCount;     
uint8_t fifoBuffer[64];
Quaternion q;           
VectorInt16 aa;       
VectorInt16 aaReal;     
VectorInt16 aaWorld;  
VectorFloat gravity;    
float ypr[3];          
volatile bool mpuInterrupt = false;

//gyroscope intrerrupt function
void dmpDataReady() {
	mpuInterrupt = true;
}

//stop the car
void Stop(){
	for(int i=2;i<=7;i++)
		digitalWrite(i,0);
}

//isd chip move to next recording
void fwd(){
	Serial.println("fwd...");
	chip.pu();
	while(!chip.PU())delay(1);
	while(!chip.RDY())delay(1);
	chip.fwd();
	while(!chip.RDY())delay(1);
}

//isd chip play recording
void play(){
	chip.pu();
	while(!chip.PU())delay(1);
	while(!chip.RDY())delay(1);
	chip.play();
	while(!chip.RDY())delay(1);
}

//get gyroscope yaw
int getx(){
	float X;
	bool modified=0;
	//read until valid gyroscope data
	while(!modified){ 
		while (!mpuInterrupt && fifoCount < packetSize);
		mpuInterrupt = false;
		mpuIntStatus = mpu.getIntStatus();
		fifoCount = mpu.getFIFOCount(); 
		if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
			//reset fifo buffer if it overflows
			mpu.resetFIFO();
		}else if (mpuIntStatus & 0x02) {
			//got valid data
			while (fifoCount < packetSize)
				fifoCount = mpu.getFIFOCount();
			mpu.getFIFOBytes(fifoBuffer, packetSize);
			fifoCount -= packetSize;
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			X=ypr[0]*180/M_PI;
			modified=1;
		}
	}
	//use calibration result to get real angle
	X+=ct;
	if(X>360)
		X-=360;
	if(X<0)
		X=X+360;
	return X;
}

//move the car forward
void forward(){
	analogWrite(2,96);
	analogWrite(7,88);
	digitalWrite(3,1);
	digitalWrite(4,0);
	digitalWrite(5,1);
	digitalWrite(6,0);  
}

//move the car function
void go(int dist){
	dist*=25;
	forward();
	long long Times=millis();
	while(millis()-Times<dist){
		//check for obstacles
		if(!digitalRead(8)){
			long long T=millis();
			Stop();
			//wait for obstacle to pass and start beeping
			while(!digitalRead(8))
				tone(9,1000);  
			noTone(9);
			forward();
			Times+=(millis()-T-20);
		}
	}
	Stop();
}

//rotate the car to the required angle function
void rotate(int d){
	Stop();
	delay(1000);
	if(orientation==d)
		return;
	if(orientation==90)
		coordy-=5;
	else if(orientation==0||orientation==360)
		coordx+=5;
	else if(orientation==180)
		coordx-=5;
	else if(orientation==270)
		coordy+=5;
	//get current rotation
	float curr_angle=getx();
	analogWrite(2,125);
	analogWrite(7,125);
	//keep turning until the car is aligned
	while(fabs(curr_angle-d)>5&&fabs(curr_angle-d)<355){
		//get rotation angle
		curr_angle=getx();
		//rotate the car right
		digitalWrite(3,1);
		digitalWrite(4,0);
		digitalWrite(5,0);
		digitalWrite(6,1);
	}
	orientation=d;
	Stop();
	delay(1000);
}

//read the order from the voice recognition chip
String getOrder(){
	String raw;
	char ch;
	int len=-1;
	//wait for data
	while(!serial.available());
	while(1){
		if(serial.available()){ 
			ch=(char)serial.read();
			//break if the order is done
			if(ch=='x')
				break;
			len++;
			Serial.print(ch);
			raw+=ch;
		}
	}
	return raw;
}

//radio data struct
struct sTr{
	byte done;
	String str; 
}radioPack;

//setup function
void setup(){
	//begin serial comunication
	serial.begin(9600);
	Serial.begin(9600);
	//begin the radio transciever
	radio.begin();
	radio.setChannel(85);
	radio.setDataRate(RF24_250KBPS); 
	radio.setPALevel(RF24_PA_MAX);
	radio.openWritingPipe(addr[1]);
	radio.openReadingPipe(1,addr[0]);
	//declare the pins used for moving the car
	for(int i=2;i<=7;i++)
		pinMode(i,OUTPUT);
	//declare other pins
	pinMode(8,INPUT);
	pinMode(A0,INPUT);
	pinMode(9,OUTPUT);
	//begin the mpu gyroscope
	Wire.begin();
	Wire.setClock(400000);
	mpu.initialize();
	pinMode(18, INPUT);
	devStatus = mpu.dmpInitialize();
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788);
	mpu.setDMPEnabled(true);
	attachInterrupt(digitalPinToInterrupt(18), dmpDataReady, RISING);
	mpuIntStatus = mpu.getIntStatus();
	dmpReady = true;
	packetSize = mpu.dmpGetFIFOPacketSize();
	long long timer=millis();
	bool mod=0;
	Serial.println("\nstarted");
	//wait for the gyroscope to stop calibrating
	while(millis()-timer<60000){
		//play beep sound until the calibration is done
		mod=((millis()-timer)/500)%2;
		if(mod)
			tone(9,120);
		else
			noTone(9);
		Serial.println(millis()/1000);
		getx();
	}
	//stop beep
	noTone(9);
	//get the calibration result
	ct=90-getx();
	rotate(90);
	coordy=0;
	Serial.println("done");
	radio.stopListening();
	Stop();
	delay(100);
}

//loop function
void loop() {
	//if the robot arrived at the required location
	if(taskdone){
		check:
		//start listening for radio data
		radio.startListening();
		Serial.print("waiting for a task");
		//wait until a new task is receieved
		while(!radio.available());
		Serial.println();
		//read the task
		radio.read(&task,sizeof(task));
		taskdone=0;
		secondary=0;
		//get the secondary task if available
		if(task==1||task==2||task==3)
			secondary=1;
		else if(task==10)
			task=0,secondary=2;
		else if(task==4||task==5||task==6)
			task-=3,secondary=3;
		Serial.print("got a task= ");
		Serial.print(task);
		Serial.print(" with secondary= ");
		Serial.println(secondary);
		delay(1000);
		//stop lsitening for radio data
		radio.stopListening();
	}else{
		Serial.print(coordx);
		Serial.print("->");
		Serial.println(p[task][0]);
		Serial.print(coordy);
		Serial.print("->");
		Serial.println(p[task][1]);
		//if the robot arrived at the location
		if(fabs(p[task][0]-coordx)<10&&fabs(p[task][1]-coordy)<10){
			taskdone=1;
			Stop();
			//if the robot has to get a new order
			if(secondary==1){
				fwd();
				Serial.println(secondary);
				delay(1000);
				//play "hello may i take your order" recording
				play();
				String order;
				delay(1000);
				//get the order
				order=getOrder();
				fwd();
				delay(1000);
				//play "thank you" recording
				play();
				delay(1000);
				radioPack.str=order;
				radioPack.done=taskdone;
				Serial.print("order is: ");
				Serial.println(order);
				//send the order
				radio.write(&radioPack,sizeof(radioPack));
			}
			//if the robot has to go back to the kitchen
			if(secondary==2){
				Serial.println(secondary);
				radio.write(&taskdone,sizeof(taskdone));
			}
			//if the robot has to deliver the order
			if(secondary==3){
				Serial.println(secondary);
				fwd();
				delay(1000);
				//play "here is yout order"
				play();
				delay(2000);
				radio.write(&taskdone,sizeof(taskdone));
			}
			Serial.println("going to start");
			//go back to the begining
			goto check;
		}
		//check current position and rotate according to the task
		if(coordy>p[task][1]){//go back
			if(orientation!=270)
				rotate(270);
			go(coordy-p[task][1]);
			coordy=p[task][1];
		}else if(p[task][1]>coordy){//go forward
			if(orientation!=90)
				rotate(90);
			go(p[task][1]-coordy);
			coordy=p[task][1];
		}
		if(coordx>p[task][0]){//go left
			if(orientation!=0)
				rotate(0);
			go(coordx-p[task][0]);
			coordx=p[task][0];
		}else if(coordx<p[task][0]){//go right
			if(orientation!=180)
				rotate(180);
			go(p[task][0]-coordx);
			coordx=p[task][0];
		}
	}
}
