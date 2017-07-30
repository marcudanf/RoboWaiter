//include the required libraries
#include <SPI.h>
#include <LiquidCrystal.h>
#include <RF24.h>

//declared the objects
RF24 radio(49,53);
LiquidCrystal lcd(8,9,4,5,6,7);

//declare required variables
byte addr[][6]={"1Node","2Node"};
bool bt[7]={0},btp[7]={0};
byte v[100]={0}; 
byte q=1;
bool taskdone=1;
int x=0,y=0;
int preval=1023;
int MessLen;
int val=1023;
int task,table,changed=0;
byte val_0=0;
byte val_10=10;

//data structure used for the radio
struct p{
	byte done;
	String str;  
}radioPack;

//function used to print an order of only 2 lines to the lcd
void Print(String string){
	int END=string.length();
	int i;
	for(i=0;i<END;i++)
		if(string[i]=='\n')
			break;
	lcd.print(string.substring(0,i));
	lcd.setCursor(0,1);
	lcd.print(string.substring(i+1,END));
}

//function used to print an order of any size to te lcd
void Print2(String string,int linenum){
	//clear lcd
	lcd.clear();
	Serial.println(linenum);
	Serial.println("decomposing");
	Serial.println(string);
	Serial.println("all");
	int END=string.length();
	int i=0,num=-1,pos=0,lastpos=0;
	//check for first line
	while(num<linenum){
		if(string[i]=='\n'){
			num++;
			lastpos=pos;
			pos=i;
		}
		i++;
	}
	Serial.println(i);
	if(linenum)
		lastpos++;
	//print first line
	lcd.print(string.substring(lastpos,pos));
	Serial.println(string.substring(lastpos,pos));
	lcd.setCursor(0,1);
	String to_print;
	//check for second line
	for(int j=i;j<END;j++){
		if(string[j]=='\n')
			break;
		to_print+=string[j];
	}
	//print the second line
	lcd.print(to_print);
	Serial.println(to_print);
}

//check for lcd keypad buttons pressed
int btpressed(){
	preval=val;
	val=analogRead(A0);
	if(val>1000)
		return 0;
	if(val>710&&val<730&&preval>1000)
		return 1;
	else if(val>100&&val<150&&preval>1000)
		return 2;
	else if(val>300&&val<320&&preval>1000)
		return 3;
	else
	return 0;
}

//setup function
void setup() {
	//begin the lcd
	lcd.begin(16,2);
	//declare button pins
	for(int i=0;i<=2;i++)
		pinMode(i+46,2);
	//begin serial communication
	Serial.begin(9600);
	pinMode(A0,INPUT);
	Serial.println("started");
	//begin the radio communication
	radio.begin();
	radio.setChannel(85);
	radio.setDataRate(RF24_250KBPS); 
	radio.setPALevel(RF24_PA_MAX);
	radio.openWritingPipe(addr[0]);
	radio.openReadingPipe(1,addr[1]);
	//clear the lcd and print a message
	lcd.clear();
	lcd.print("waiting for an ");
	lcd.setCursor(0,1);
	lcd.print("order...");
	lcd.setCursor(0,0);
}

//loop function
void loop() {
	//read the button values
	for(int i=1;i<=3;i++){
		bt[i]=!digitalRead(i+45);
		if(bt[i]==1&&btp[i]==0){
			//v[q++]=0;
			v[q++]=i;
			changed=1;
		}
	}
	if(taskdone&&q>1){//the robot arrived
		//stop listening for radio data
		radio.stopListening();
		//send the next task
		radio.write(&v[q-1],sizeof(v[q-1]));
		task=v[q-1];
		Serial.print("sent");
		Serial.println(v[q-1]);
		q--;
		taskdone=0;
		//start listening for radio data
		radio.startListening();
	}else if(q>1){//check if the robot arrived
		if(radio.available()){//check for radio data
			if(task<=3){//if the robot has to return an order
				//read the order
				radio.read(&radioPack,sizeof(radioPack));
				radio.stopListening();
				table=task;
				radioPack.str+="\n................";
				//print to lcd
				lcd.clear();
				lcd.print("new order from  ");
				lcd.setCursor(0,1);
				lcd.print("table no. ");
				lcd.print(table);
				lcd.setCursor(0,0);
				//wait for the order to be accepted
				while(btpressed()!=1);
				delay(100);
				lcd.clear();
				int MaxValLine=-1;
				int STRLEN=radioPack.str.length();
				//get how many lines are in the order
				for(int i=0;i<STRLEN;i++)
					if(radioPack.str[i]=='\n')
						MaxValLine++;
				if(MaxValLine<0)
					MaxValLine=0;
				Serial.print("max=");
				Serial.println(MaxValLine);
				int line_number=0;
				int bt_val=btpressed();
				//print the first 2 lines
				Print2(radioPack.str,line_number);
				Serial.println("entered whuile");
				//wait until the food is done
				while(bt_val!=1){
					if(bt_val==2)//if up btn is pressed scroll up
						line_number--;
					else if(bt_val==3)//if down btn is pressed scroll down
						line_number++;
					if(line_number<0)
						line_number=0;
					if(line_number>MaxValLine)
						line_number=MaxValLine;
					//if line_number has changed print the change
					if(bt_val==2||bt_val==3)
						Print2(radioPack.str,line_number);
					bt_val=btpressed();
				}
				//call the robot to the kitchen
				radio.write(&val_10,sizeof(byte));
				radio.startListening();
				delay(100);
				lcd.clear();
				lcd.print("waiting for a");
				lcd.setCursor(0,1);
				lcd.print(" new order...");
				lcd.setCursor(0,0);
				taskdone=0;
				table+=3;
				//wait for the robot to arrive
				while(!radio.available());
				radio.read(&taskdone,sizeof(taskdone));
				radio.stopListening();
				radio.write(&table,sizeof(table));
				radio.startListening();
			}else{//if it doesn't have to return an order
				radio.read(&taskdone, sizeof(taskdone));
			}
			Serial.println("task done");
		}
	}else if(taskdone&&q<=1&&changed){//send the robot to the kitchen
		//stop listenign for radio data
		radio.stopListening();
		//write the task to the radio
		radio.write(&val_0,sizeof(byte));
		task=0;
		Serial.print("sent ");
		Serial.println(val_0);
		taskdone=0;
		//start listening for radio data
		radio.startListening();
	}
	//keep old button data
	for(int i=1;i<=3;i++)
		btp[i]=bt[i];
}
