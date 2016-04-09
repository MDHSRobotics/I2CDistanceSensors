// ---------------------------------------------------------------------------
// Example NewPing library sketch that does a ping about 20 times per second.
// I2C slave
// ---------------------------------------------------------------------------

#include <NewPing.h>
#include <Wire.h>


#define SLAVE_ADDRESS 0x41
#define REGISTER_SIZE 19
#define MAX_COMMAND_SIZE 12
#define SLAVE_STATUS_LED 13
#define STATUS_I2C_DATA_READY 0x4141
#define SONAR_NUM 2
#define MODE_SIZE 3
#define CONFIG_SIZE 3


//define register map addresses
#define STATUS_ADDRESS      0x00
#define DISTANCE0_ADDRESS   0x02
#define DISTANCE1_ADDRESS   0x04
#define MODE_ADDRESS        0x06
#define CONFIG_ADDRESS      0x0C
#define ID_ADDRESS          0x12


//define commands
#define LED_COMMAND 0x4C00

#define MAX_DISTANCE 380 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define LOOP_DELAY_DEFAULT 100 

#define TRIGGER_PIN1   2  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN1      3  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN2  8  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN2     9  // Arduino pin tied to echo pin on the ultrasonic sensor

/********* Global  Variables  ***********/

byte registerMap[REGISTER_SIZE];
byte registerMapTemp[REGISTER_SIZE - 1];
byte receivedCommands[MAX_COMMAND_SIZE];
 // NewPing setup of pins and maximum distance.
NewPing sonar[SONAR_NUM] = { // Sensor object array.
  NewPing(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE),
  NewPing(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE)
};
unsigned int distance[2];  
unsigned int deviceStatus = STATUS_I2C_DATA_READY;
bool newDataAvailable = false; 
bool configModeUpdated = false;

// mode structure
// 0 - command , 
// 1 - data 1 ,
// 2 - data 2
unsigned int mode[MODE_SIZE];

// config structure
// 0 - to configure the max distance of sensor 0 , 
// 1 - to configure the max distance of sensor 0 ,
// 2 - to configure loop delay 
unsigned int configuration[3];  

void setup() {
  setSerialMonitor();
  Serial.println("Initializing subsystems ...");
  setupI2C();
  setupPins();
}

void loop() {
//  Serial.println("looping...");
  getDistanceData();
  if(configModeUpdated){
    configModeUpdate();
  }
  delay(LOOP_DELAY_DEFAULT);                      // Wait between pings. 29ms should be the shortest delay between pings.
}
void configModeUpdate(){
    configModeUpdated = false;
    switch(mode[0]){
      case LED_COMMAND:
        switch(mode[1]){
          case 0x4800:
          digitalWrite(SLAVE_STATUS_LED,HIGH);
          break;
          case 0x4C00:
          digitalWrite(SLAVE_STATUS_LED,LOW);
          break;
        }
        break;
    }

//    printMode();
}
void printMode(){
  char tmp[85];
  sprintf(tmp,"mode: [%.4X %.4X %.4X]",mode[0],mode[1],mode[2]);
  Serial.print(tmp);
}
void requestEvent(){
  if(newDataAvailable){
    for(uint8_t c=0;c<REGISTER_SIZE-1;c++){
        registerMap[c] = registerMapTemp[c];
    }
  }
  newDataAvailable = false;
  Wire.write(registerMap+receivedCommands[0], REGISTER_SIZE);  //Set the buffer up to send all 17 bytes of data
}
/* adding a comment */

void receiveEvent(int bytesReceived){
  for (uint8_t a = 0; a < bytesReceived; a++){
    if ( a < MAX_COMMAND_SIZE){
         receivedCommands[a] = Wire.read();
    }
    else{
         Wire.read();  // if we receive more data then allowed just throw it away
    }
  }
  if(bytesReceived == 1 && (receivedCommands[0] < REGISTER_SIZE))
  {
      return; 
  }
  if(bytesReceived == 1 && (receivedCommands[0] >= REGISTER_SIZE))
  {
      receivedCommands[0] = 0x00;
      return;
  }

  byte address = receivedCommands[0];
//  Serial.print("received more than 1 byte"); //meaning that the master is writing to the slave.
//  Serial.print(", address writing to is: 0x");
//  Serial.print(address,HEX);
//  Serial.println();
  if(address < MODE_ADDRESS || address >= ID_ADDRESS){
//    Serial.println('address is not a valid write address');
    return;
  }
  configModeUpdated = true;
  short i=0;
  byte * bytePointer;  //we declare a pointer as type byte

  if(address == MODE_ADDRESS){
    if(bytesReceived > (1+2*MODE_SIZE)){
      bytePointer = (byte*)&mode; 
      while(bytesReceived>1 && i<2*MODE_SIZE)
      {
        bytePointer[i] = receivedCommands[i+1];
        bytesReceived--;i++;
      }
      i=0;
      bytePointer = (byte*)&configuration; 
      while(bytesReceived>1 && i<2*CONFIG_SIZE)
      {
        bytePointer[i] = receivedCommands[i+1];
        bytesReceived--;i++;
      }
    }
    else{
      bytePointer = (byte*)&mode; 
      while(bytesReceived>1 && i<2*MODE_SIZE)
      {
        bytePointer[i] = receivedCommands[i+1];
        bytesReceived--;i++;
      }
    }
  }
  else if(address == CONFIG_ADDRESS){
      bytePointer = (byte*)&configuration; 
      while(bytesReceived>1 && i<2*CONFIG_SIZE)
      {
        bytePointer[i] = receivedCommands[i+1];
        bytesReceived--;i++;
      }
  }
}

void getDistanceData(){
  distance[0] = sonar[0].ping()/ US_ROUNDTRIP_CM;
  distance[1] = sonar[1].ping()/ US_ROUNDTRIP_CM;
  storeData();
  printData();
}

void printData(){
  for(uint8_t  i=0;i<SONAR_NUM;i++){
    Serial.print("Ping: sonar");
    Serial.print(i);
    Serial.print("=");
    Serial.print(distance[i]); // Convert ping time to distance in cm and print result (0 = outside set distance range)
    Serial.print(" cm\t");  
  }
  Serial.println();
}
void storeData()
{
    uint8_t arrayIndex = 0;  //we need to keep track of where we are storing data in the array
    uint8_t datasize = sizeof(unsigned int)-1;
    byte * bytePointer;  //we declare a pointer as type byte


    //all of our data is unsigned int.  Why - easier to just deal with 1 type;
    //code below converts from little endian to bin endian - why?  don't know, but it is recommended so I'm doing it
    
    bytePointer = (byte*)&deviceStatus; 
    for (short b = datasize; b >= 0; b--)
    {
       registerMapTemp[arrayIndex] = bytePointer[b];  //increment pointer to store each byte
       arrayIndex++;
    }      
    
     
    //write our data variables into our data register
    for(short  i=0;i<SONAR_NUM;i++){
      bytePointer = (byte*)&distance[i]; //distance is datasize bytes
      for (short b = datasize; b >= 0; b--)
      {
          registerMapTemp[arrayIndex] = bytePointer[b];  //increment pointer to store each byte
          arrayIndex++;
      }      
    }     


    //copy over data from our mode and config registers
    for(short  i=0;i<MODE_SIZE;i++){
      bytePointer = (byte*)&mode[i]; 
      for (short b = datasize; b >= 0; b--)
      {
          registerMapTemp[arrayIndex] = bytePointer[b];  //increment pointer to store each byte
          arrayIndex++;
      }      
    }  

    for(short  i=0;i<CONFIG_SIZE;i++){
      bytePointer = (byte*)&configuration[i]; 
      for (short b = datasize; b >= 0; b--)
      {
          registerMapTemp[arrayIndex] = bytePointer[b];  //increment pointer to store each byte
          arrayIndex++;
      }      
    }  
    newDataAvailable=true;
}
void setSerialMonitor(){
  Serial.begin(9600); // Open serial monitor at 9600 baud to see ping results.
}
void setupI2C(){
  Serial.println("setting up I2C");
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
}
void  setupPins(){
  Serial.println("Setting up pins");
  pinMode(SLAVE_STATUS_LED,OUTPUT);
  digitalWrite(SLAVE_STATUS_LED,LOW);
}

