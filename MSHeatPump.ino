/**
 *  Amitime heatpump <-> mysensors integrator
 *  
 *  @Version: v0.6
 *  @date:    3.3.2017
 *  @author:  PJR
 *  
 *  
 *  Methods:
 *  -- initializations --
 *  @setup:                    init mysensors and modbus
 *  @initModbusMessages:       init all modbus messages  
 *  @initModbusMessage:        init one modbus message
 *  @populateQueueWithPolling: initializes new polling round
 *  
 *  -- state machine --
 *  @loop:                     state machine for modbus polling
 *  
 *  -- received message processing --
 *  @onModbusProcessed:        modbus reponse handler
 *  @onMysensorsMessage:       handles messages coming from home automation controller
 *  
 *  -- message list/send order handling --
 *  @getFirstMessageIndex:     get first message from poll list
 *  @queueMaxOrderNumber:      get biggest order number(not index) of any message
 *  @addToEndOfSendQueue:      adds message to end of queue
 *  @addToBeginingOfSendQueue: adds message to begining of queue
 *  
 */

//
#define MY_PARENT_NODE_ID 0
#define MY_NODE_ID 1
#define MY_TRANSPORT_WAIT_READY_MS 1000ul

#define MY_DEFAULT_ERR_LED_PIN 13 // Display errors on led pin 13
#define MY_DEBUG // Enable debug prints to serial monitor
#define MY_RADIO_NRF24 // Enable radio transport layer
#define MY_RF24_PA_LEVEL RF24_PA_LOW // set radio level to low since we dont have enough juice provide

#define UBRR1H       // Tell modbus library it can use Serial1. Not if this this is really needed, havent tested.
#define TXEN       8 // assign the Arduino pin that must be connected to RE-DE RS485 transceiver
#define SERIALPORT 1 // 0 for RS-232 and USB-FTDI or any pin number > 1 for RS-485
#define MBREAD     3 // modbus read command
#define MBWRITE    6 // modbus write command
#define SLAVEADDR  1 // modbus slave address

#define MAX_TIME_BETWEEN_SEND 1000*60*2 // Time between message send even if no change to value. 2minutes

#include <avr/wdt.h>
#include <SPI.h>
#include <stdlib.h>           // used for atoi in error codes.. perhaps could be replaces with some other functionality
#include <ModbusRtu.h>
#include <MySensors.h>

// Modbus message indexes
enum message {
 READ_RUNMODE = 0,
 WRITE_RUNMODE = 1,
 READ_SETPOINT_COLD_BY_WATER = 2,
 WRITE_SETPOINT_COLD_BY_WATER = 3,
 READ_SETPOINT_COLD_BY_ROOM = 4,
 WRITE_SETPOINT_COLD_BY_ROOM = 5,
 READ_SETPOINT_HEAT_BY_WATER = 6,
 WRITE_SETPOINT_HEAT_BY_WATER = 7,
 READ_SETPOINT_HEAT_BY_ROOM = 8,
 WRITE_SETPOINT_HEAT_BY_ROOM = 9,
 READ_SETPOINT_WATERHEAT = 10,
 WRITE_SETPOINT_WATERHEAT = 11,
 READ_TANK_TEMP = 12,
 READ_POWER = 13,
 READ_FREQUENCY = 14,
 READ_EXPVALV = 15,
 READ_OUTTEMP = 16, 
 READ_INDOOR_INLET_TEMP = 17,
 READ_OUTDOOR_COIL_TEMP = 18,
 READ_OUTDOOR_EXHAUSTING_TEMP = 19,
 READ_ERRORS_0_7 = 20,
 READ_ERRORS_8_14 = 21,
 BUFFER_COUNT = 22, 
};

// Modbus data 
// Data arrays for modbus network sharing
uint16_t au16runmode[1];
uint16_t au16runmodeWrite[1];
uint16_t au16setPointCold[1];
uint16_t au16setPointColdWrite[1];
uint16_t au16setPointHeat[1];
uint16_t au16setPointHeatWrite[1];
uint16_t au16tankTemp[1];
uint16_t au16tankPower[2];
uint16_t au16frequency[1];
uint16_t au16expValve[1];
uint16_t au16outTemp[1];
uint16_t au16indoorInletTemp[1];
uint16_t au16outdoorCoilTemp[1];
uint16_t au16outdoorExhaustTemp[1];
uint16_t au16errors_0[8];
uint16_t au16errors_8[7];

uint16_t lastValue[16]; // store for previous value
 
uint8_t u8state; // Modbus machine state
uint8_t u8retryCount;

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  u8serno : serial port (use 0 for Serial)
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Modbus master(0,SERIALPORT,TXEN); // this is master and RS-232 or USB-FTDI

/**
 * This is an structe which contains a query to an slave device
 */
modbus_t modbusMessage[BUFFER_COUNT];
unsigned long lastSentTime[BUFFER_COUNT];
int messageOrder[BUFFER_COUNT];
int messageSize[BUFFER_COUNT];

// wait "timers" for state machine
unsigned long u32waitSerialFill;
unsigned long u32waitPollResult;
unsigned long u32waitFullData;
unsigned long u32waitBetweenMessages;
unsigned long u32lastMemSend;

// index of current message polling
int sendIndex;
short pollCount;

// Init mysensors messages
//MyMessage msgOnOffState(1, V_LIGHT); // temporary message for on/off hack for old domoticz
MyMessage msgFlowState(1, V_DIMMER /*V_HVAC_FLOW_STATE*/);   // 1 S_HVAC default functionality
MyMessage msgSetPntHeat(2, V_HVAC_SETPOINT_HEAT); // 2...
MyMessage msgSetPntCold(2, V_HVAC_SETPOINT_COOL); // 2...
MyMessage msgTankTemp(2, V_TEMP);                 // 2...
MyMessage msgError(3, V_TEXT);                    // 3 S_INFO error codes
char errorMsgBuffer[7];                           // .... message buffer for above
MyMessage msgACCurrent(4, V_CURRENT);             // 4 S_MULTIMETER power info
MyMessage msgACVoltage(4, V_VOLTAGE);             // ....
MyMessage msgFreqComp(5, V_LEVEL);                // 5 S_VIBRATION Compressor frequency
MyMessage msgOpenExpValve(6, V_LEVEL);            // 6 S_VIBRATION Expansion Valve Openness
MyMessage msgOutTemp(7, V_TEMP);                  // 7 S_TEMP outdoor temp
MyMessage msgIndoorInletTemp(8, V_TEMP);          // 8 S_TEMP Indoor inlet water temp
MyMessage msgOutdoorCoilTemp(9, V_TEMP);          // 9 S_TEMP Outdoor coil temp
MyMessage msgOutdoorExhaustTemp(10, V_TEMP);      // 10 S_TEMP Outdoor exhausting temp
MyMessage msgArduMemory(11, V_LEVEL);             // 11 S_VIBRATION Amount of memory free in arduino

char *error_values[] = 
{ 
  "3124 Communication error",
  "3125 Indoor temperature sensor fault",
  "3126 Input voltage or current sensor fault",
  "3127 Drive failure protection or drive fault",
  "3128 Indoor unit EEPROM fault",
  "3129 Overload protection",
  "3130 Input under voltage protection",
  "3131 High pressure sensor fault",
  "3132 Outdoor unit EEPROM fault",
  "3133 Outdoor water flow protection",
  "3134 Outdoor temperature sensor fault",
  "3135 High-pressure protection",
  "3136 Outdoor temperature protection",
  "3137 Indoor coil temperature protection",
  "3138 Indoor water flow protection"
};

// initializes mysensors and presents all child devices
// initializes modbus and messages
void setup() {
  // turn off the watch dog (to make sure no bootloop after first wdt reboot)
  wdt_disable();
  //Serial.println("Setup");
  // make sure there wont be any messages going
  for(int i = 0; i < BUFFER_COUNT; i++) {
    messageOrder[i] = 0;
    lastSentTime[i] = 0;
  }
  
  // init modbus messges
  initModbusMessages();
  
  // init serial line
  master.begin( 1200,  SERIAL_8E2 ); // 1200 baud-rate, 8 data bits, EVEN parity control, 2 stop bits
  master.setTimeOut( 4000 ); // if there is no answer in 4000 ms, roll over
  
  // init state machine
  u32waitFullData = millis() + 5000; // 30 sek between polls
  u32waitBetweenMessages = millis();
  u8state = sendIndex = 0;
  pollCount = 0;
  u8retryCount = 0;
  u32lastMemSend = 0;

  // turn on the watch dog
  wdt_enable(WDTO_8S);
}

void presentation()  
{ 
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("HeatPump", "2.0");

  // 3000 Run Mode 0: Standby, 2: Cooling Mode by Water, 4: Heating Mode by Water, 5: Water Heater Mode, 6: Level I of freeze, 7: Level II of freeze
  present(1, S_LIGHT, "Mode");
  
  // 3002 Run mode 2 temp: 7℃~25℃ --> V_HVAC_SETPOINT_COLD
  // 3004 Run mode 4 temp: 7℃~52℃ --> V_HVAC_SETPOINT_HEAT
  // 3103 Indoor outlet water temp: (temperature+60℃)*2
  present(2, S_HVAC, "PumpThermostat");

  // read all and check if some of these has some error in it
  // 3124 Communication error
  // 3125 Indoor temperature sensor fault 
  // 3126 Input voltage or current sensor fault
  // 3127 Drive failure protection or drive fault
  // 3128 Indoor unit EEPROM fault
  // 3129 Overload protection
  // 3130 Input under voltage protection
  // 3131 High pressure sensor fault
  // 3132 Outdoor unit EEPROM fault
  // 3133 Outdoor water flow protection
  // 3134 Outdoor temperature sensor fault
  // 3135 High-pressure protection
  // 3136 Outdoor temperature protection
  // 3137 Indoor coil temperature protection
  // 3138 Indoor water flow protection
  present(3, S_INFO, "Errors");

  // 3120 Input current: 0-225=0.0-22.5 Arms --> V_CURRENT
  // 3121 Input AC voltage: 0-365=0-365V --> V_VOLTAGE
  present(4, S_MULTIMETER, "Power");

  // 3100 Compressor frequency: 0-120hz
  present(5, S_VIBRATION, "Compressor");

  // 3122 Expansion Valve Openness: 0-500
  present(6, S_VIBRATION, "ExpValve");

  // 3107 Outdoor air temp
  present(7, S_TEMP, "OutAirTemp");

  // 3104 Indoor inlet water temp
  present(8, S_TEMP, "InInletWaterTemp");
  
  // 3108 Outdoor coil temp
  present(9, S_TEMP, "OutCoilTemp");

  // 3109 Outdoor exhausting temp
  present(10, S_TEMP, "OutExhaustTemp");

  // amount of free memory in arduino
  present(11, S_VIBRATION, "ArduMemory");
}

void loop() {
  processModbus();
  //checkFreeRam();
}

//void checkFreeRam() {
//  if(u32lastMemSend + 30000 < millis())
//  {
//    u32lastMemSend = millis();
//    Serial.println("Sending free mem amount");
//    send(msgArduMemory.set(freeRam()));
//  }
//}

int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

// init modbus messages
void initModbusMessages()
{
  initModbusMessage(READ_RUNMODE, MBREAD, 3000, 1, au16runmode, 7);
  initModbusMessage(WRITE_RUNMODE, MBWRITE, 3000, 1, au16runmodeWrite, 8);

  initModbusMessage(READ_SETPOINT_COLD_BY_ROOM, MBREAD, 3001, 1, au16setPointCold, 7);
  initModbusMessage(WRITE_SETPOINT_COLD_BY_ROOM, MBWRITE, 3001, 1, au16setPointColdWrite, 8);
  
  initModbusMessage(READ_SETPOINT_COLD_BY_WATER, MBREAD, 3002, 1, au16setPointCold, 7);
  initModbusMessage(WRITE_SETPOINT_COLD_BY_WATER, MBWRITE, 3002, 1, au16setPointColdWrite, 8);

  initModbusMessage(READ_SETPOINT_HEAT_BY_ROOM, MBREAD, 3003, 1, au16setPointHeat, 7);
  initModbusMessage(WRITE_SETPOINT_HEAT_BY_ROOM, MBWRITE, 3003, 1, au16setPointHeatWrite, 8);
  
  initModbusMessage(READ_SETPOINT_HEAT_BY_WATER, MBREAD, 3004, 1, au16setPointHeat, 7);
  initModbusMessage(WRITE_SETPOINT_HEAT_BY_WATER, MBWRITE, 3004, 1, au16setPointHeatWrite, 8);
  
  initModbusMessage(READ_SETPOINT_WATERHEAT, MBREAD, 3005, 1, au16setPointHeat, 7);
  initModbusMessage(WRITE_SETPOINT_WATERHEAT, MBWRITE, 3005, 1, au16setPointHeatWrite, 8);
  
  initModbusMessage(READ_TANK_TEMP, MBREAD, 3103, 1, au16tankTemp, 7);
  initModbusMessage(READ_POWER, MBREAD, 3120, 2, au16tankPower, 9);
  
  initModbusMessage(READ_FREQUENCY, MBREAD, 3100, 1, au16frequency, 7);
  initModbusMessage(READ_EXPVALV, MBREAD, 3122, 1, au16expValve, 7);
  
  initModbusMessage(READ_OUTTEMP, MBREAD, 3107, 1, au16outTemp, 7);
  initModbusMessage(READ_ERRORS_0_7, MBREAD, 3124, 8, au16errors_0, 21); // 3124, 3125, 3126, 3127, 3128, 3129, 3130, 3131
  initModbusMessage(READ_ERRORS_8_14, MBREAD, 3132, 7, au16errors_8, 19); // 3132, 3133, 3134, 3135, 3136, 3137, 3138

  initModbusMessage(READ_INDOOR_INLET_TEMP, MBREAD, 3104, 1, au16indoorInletTemp, 7);
  initModbusMessage(READ_OUTDOOR_COIL_TEMP, MBREAD, 3108, 1, au16outdoorCoilTemp, 7);
  initModbusMessage(READ_OUTDOOR_EXHAUSTING_TEMP, MBREAD, 3109, 1, au16outdoorExhaustTemp, 7);
}

// init one message
void initModbusMessage(int index, uint8_t functionCode, uint16_t regAdd, uint16_t coilsNo, uint16_t *dataBuffer, int resultSize) {
  modbusMessage[index].u8id = SLAVEADDR; // slave address
  modbusMessage[index].u8fct = functionCode; // function code (this one is write a single register)
  
  // for some reason there is +1 offset when using modbus so we need to shift register by -1
  modbusMessage[index].u16RegAdd = (regAdd-1); // start address in slave
  modbusMessage[index].u16CoilsNo = coilsNo; // number of elements (coils or registers) to read
  modbusMessage[index].au16reg = dataBuffer; // pointer to a memory array in the Arduino
  messageSize[index] = resultSize; // how many bytes we are expecting from slave
}

// adds all query message to queue
void populateQueueWithPolling()
{
  Serial.println("START NEW POLLING");
  
  // on/off switch hack for older domoticz
  int runmode = modbusMessage[READ_RUNMODE].au16reg[0];
  addToEndOfSendQueue(READ_RUNMODE);

  // 1: Cooling Mode by Room, 
  // 2: Cooling Mode by Water, 
  // 3: Heating Mode by Room, 
  // 4: Heating Mode by Water, 
  // 5: Water Heater Mode
  switch(runmode){
    case 1:
      addToEndOfSendQueue(READ_SETPOINT_COLD_BY_ROOM);
      addToEndOfSendQueue(READ_SETPOINT_HEAT_BY_ROOM);
      break;
    case 2:
      addToEndOfSendQueue(READ_SETPOINT_COLD_BY_WATER);
      addToEndOfSendQueue(READ_SETPOINT_HEAT_BY_WATER);
      break;
    case 3:
      addToEndOfSendQueue(READ_SETPOINT_COLD_BY_ROOM);
      addToEndOfSendQueue(READ_SETPOINT_HEAT_BY_ROOM);
      break;
    case 4:
      addToEndOfSendQueue(READ_SETPOINT_COLD_BY_WATER);
      addToEndOfSendQueue(READ_SETPOINT_HEAT_BY_WATER);
      break;
    case 5:
      addToEndOfSendQueue(READ_SETPOINT_COLD_BY_WATER);
      addToEndOfSendQueue(READ_SETPOINT_WATERHEAT);
      break;
  }
  
  addToEndOfSendQueue(READ_TANK_TEMP);
  addToEndOfSendQueue(READ_POWER);
  addToEndOfSendQueue(READ_FREQUENCY);
  addToEndOfSendQueue(READ_EXPVALV);
  addToEndOfSendQueue(READ_OUTTEMP);
  addToEndOfSendQueue(READ_INDOOR_INLET_TEMP);
  addToEndOfSendQueue(READ_OUTDOOR_COIL_TEMP);
  addToEndOfSendQueue(READ_OUTDOOR_EXHAUSTING_TEMP);
  addToEndOfSendQueue(READ_ERRORS_0_7);
  addToEndOfSendQueue(READ_ERRORS_8_14);
}

// state machine implementation for modbus polling
void processModbus() {
    // modbus state machine
  switch( u8state ) {
    
    case 0: 
      {
        //Serial.print ("StateMachine:0 ");
        int queueMax = queueMaxOrderNumber();
        
        //Serial.println (queueMax);
        if(queueMax > 0) {
          u32waitFullData = millis() + 15000;
          if(u32waitBetweenMessages < millis())
            u8state++;
        } else if (u32waitFullData < millis()) {
          populateQueueWithPolling();
          u8state++;
        } else {
          wdt_reset();
          //Serial.println("WDT:Reset0");
        }
        break;  
      } 
    case 1:
      //Serial.print ("StateMachine:1 ");
      // store last value to lastValue
      
      if(pollCount == 0 && sendIndex != READ_ERRORS_0_7 && sendIndex != READ_ERRORS_8_14) {
        //Serial.println ("Copy old value");
        memcpy(&lastValue, modbusMessage[sendIndex].au16reg, modbusMessage[sendIndex].u16CoilsNo * sizeof(uint16_t) );
        //Serial.println ("Copy done");
      }
      
      sendIndex = getFirstMessageIndex();
      master.query( modbusMessage[sendIndex] ); // send query
      //Serial.print (millis());
      //Serial.println (" Modbus query sent");
      u32waitPollResult = millis() + 50;
      u8state++;
      break;
      
    case 2:
      {
        //Serial.print ("StateMachine:2 ");
        // wait a while to serial line
        if(u32waitPollResult > millis())
          break;
          
        // check poll result. tells if correct response is received
        int8_t pollResult = master.poll(); // check incoming messages

        // Just some debug message to help understading how the bus behaves
        if(pollResult != 0) {
          //Serial.print("Poll result: ");
          //Serial.print(pollResult);
          //Serial.print(" count: ");
          //Serial.println(pollCount);
          u8retryCount++;
        }

        // COM_IDLE -> no more data to receive
        if (master.getState() == COM_IDLE) {
          u8state = 0;
          u32waitBetweenMessages = millis() + 600; // reset between messages counter
          u32waitFullData = millis() + 3000;       // reset full scan counter
          if(pollResult == messageSize[sendIndex]) {
            if(pollCount < 1 && modbusMessage[sendIndex].u8fct == MBREAD) {
              wdt_reset();
//              Serial.println("WDT:Reset2-1");
//              Serial.print("Idle read skip ");
//              Serial.println(pollCount);
              u8state = 3;
              u32waitSerialFill = millis() + 200;
              break;
            }
            
            //pollCount = 0; zeroed at state 4
            wdt_reset();
            //Serial.println("WDT:Reset2-2");
            messageOrder[sendIndex] = 0;
            onModbusProcessed(sendIndex);
            u8retryCount = 0;
            u8state = 3;
            // for next state wait some time if there is some rubbish comting from serial line
            u32waitSerialFill = millis() + 200;
          }
        }

        if(u8retryCount > 15) {
          u32waitSerialFill = millis() + 200;
          //pollCount = 0; zeroed at state 4
          messageOrder[sendIndex] = 0;
          u8state = 4;
          u8retryCount = 0;
        }
        break;
      }
    case 3:
      //Serial.print ("StateMachine:3 ");
      if(u32waitSerialFill < millis()) {
        // clear the serial buffer and continue to first state
        while(Serial1.available() > 0) {
          //Serial.print(Serial1.read());
          Serial1.read();
          //Serial.println(" ");
        }
        u8state++;
        // for next state wait some time if there is some rubbish comting from serial line
        u32waitSerialFill = millis() + 100;
        wdt_reset();
        //Serial.println("WDT:Reset3");
      }
      break;
    case 4:
      //Serial.print ("StateMachine:4 ");
      if(u32waitSerialFill < millis()) {
        // clear the serial buffer and continue to first state
        //Serial.println("State 4 wait finished");
        while(Serial1.available() > 0) {
          //Serial.print(Serial1.read());
          Serial1.read();
          //Serial.print(" ");
        }

        // make reads twice since there is 
        // some times unusable shit as result at first time
        if(pollCount < 1 && modbusMessage[sendIndex].u8fct == MBREAD) {
          //Serial.println("Go to re-read: ");
          pollCount++;
          u8state = 1;
          break;
        }
        pollCount = 0;
        u8state = 0;
        wdt_reset();
        //Serial.println("WDT:Reset4");
      }
      break;
  }
}

// called when modbus message is succesfully send.
void onModbusProcessed(int messageIndex) {

  // check for exception
  //Serial1.flush();
  unsigned long currentTime = millis();
  for(int i = 0; i < BUFFER_COUNT; i++)
    if(lastSentTime[i] > currentTime)
      lastSentTime[i] = currentTime > MAX_TIME_BETWEEN_SEND ? currentTime - MAX_TIME_BETWEEN_SEND : 0;
      
  Serial.print("MODBUS MESSAGE PROCESSED: ");
  Serial.println(messageIndex);
  switch( messageIndex ) {
    case READ_RUNMODE: 
      {
        // Run Mode 
        // 0: Standby, 
        // 1: Cooling Mode by Room, 2: Cooling Mode by Water, 
        // 3: Heating Mode by Room, 4: Heating Mode by Water, 
        // 5: Water Heater Mode, 
        // 6: Level I of freeze, 7: Level II of freeze
        int flowState = modbusMessage[messageIndex].au16reg[0];
        Serial.print("READ_RUNMODE: ");
        Serial.print(flowState);
        Serial.print(" ");
        Serial.println(modbusMessage[messageIndex].au16reg[0]);
        if(lastValue[0] == modbusMessage[messageIndex].au16reg[0] && 
           currentTime - lastSentTime[messageIndex] > MAX_TIME_BETWEEN_SEND) {
          Serial.println("Skip runmode send");
          break;
        }
        switch( flowState ) {
          case 0:
            // Off
            send(msgFlowState.set(0));
            break;
          case 1:
            // CoolByRoom
            send(msgFlowState.set(10));
            break;
          case 2:
            // CoolByWater
            send(msgFlowState.set(20));
            break;
          case 3:
            // HeatByRoom
            send(msgFlowState.set(30));
            break;
          case 4:
            // HeatByWater
            send(msgFlowState.set(40));
            break;
          case 5:
            // WaterHeaterMode
            send(msgFlowState.set(50));
            break;
          case 6:
            // Level I of freeze
            send(msgFlowState.set(60));
            //gw.send(msgError.set("Level I of freeze"));
          case 7:
            // Level II of freeze
            send(msgFlowState.set(70));
            //gw.send(msgError.set("Level II of freeze"));
            break;
        }
      }
      break;
    case READ_SETPOINT_COLD_BY_WATER:
      Serial.print("READ_SETPOINT_COLD_BY_WATER: ");
      Serial.println(modbusMessage[messageIndex].au16reg[0]);
      Serial.print("Last setpoint cold value: ");
      Serial.println(lastValue[0]);
      Serial.print("New setpoint cold value: ");
      Serial.println(modbusMessage[messageIndex].au16reg[0]);
      if(lastValue[0] == modbusMessage[messageIndex].au16reg[0] && 
         currentTime - lastSentTime[messageIndex] > MAX_TIME_BETWEEN_SEND) {
        Serial.println("Skip setpoint cold send");
        break;
      }
      send(msgSetPntCold.set(modbusMessage[messageIndex].au16reg[0]));
      break;
    case READ_SETPOINT_COLD_BY_ROOM:
      Serial.print("READ_SETPOINT_COLD_BY_ROOM: ");
      Serial.println(modbusMessage[messageIndex].au16reg[0]);
      Serial.print("Last setpoint cold value: ");
      Serial.println(lastValue[0]);
      Serial.print("New setpoint cold value: ");
      Serial.println(modbusMessage[messageIndex].au16reg[0]);
      if(lastValue[0] == modbusMessage[messageIndex].au16reg[0] &&
         currentTime - lastSentTime[messageIndex] > MAX_TIME_BETWEEN_SEND) {
        Serial.println("Skip setpoint cold send");
        break;
      }
      send(msgSetPntCold.set(modbusMessage[messageIndex].au16reg[0]));
      break;
    case READ_SETPOINT_HEAT_BY_WATER:
      Serial.print("READ_SETPOINT_HEAT_BY_WATER: ");
      Serial.println(modbusMessage[messageIndex].au16reg[0]);
      if(lastValue[0] == modbusMessage[messageIndex].au16reg[0] && 
         currentTime - lastSentTime[messageIndex] > MAX_TIME_BETWEEN_SEND) {
        Serial.println("Skip setpoint heat send");
        break;
      }
      send(msgSetPntHeat.set(modbusMessage[messageIndex].au16reg[0]));
      break;
    case READ_SETPOINT_HEAT_BY_ROOM:
      Serial.print("READ_SETPOINT_HEAT_BY_ROOM: ");
      Serial.println(modbusMessage[messageIndex].au16reg[0]);
      if(lastValue[0] == modbusMessage[messageIndex].au16reg[0] && 
         currentTime - lastSentTime[messageIndex] > MAX_TIME_BETWEEN_SEND) {
        Serial.println("Skip setpoint heat send");
        break;
      }
      send(msgSetPntHeat.set(modbusMessage[messageIndex].au16reg[0]));
      break;
    case READ_SETPOINT_WATERHEAT:
      Serial.print("READ_SETPOINT_WATERHEAT: ");
      Serial.println(modbusMessage[messageIndex].au16reg[0]);
      if(lastValue[0] == modbusMessage[messageIndex].au16reg[0] && 
         currentTime - lastSentTime[messageIndex] > MAX_TIME_BETWEEN_SEND) {
        Serial.println("Skip setpoint waterheat send");
        break;
      }
      send(msgSetPntHeat.set(modbusMessage[messageIndex].au16reg[0]));
      break;
    case READ_TANK_TEMP:
      {
        // (temperature+60â„ƒ)*2
        float tankTemp = ((float)modbusMessage[messageIndex].au16reg[0]/2.0)-60.0;
        Serial.print("READ_TANK_TEMP: ");
        Serial.print(modbusMessage[messageIndex].au16reg[0]);
        Serial.print(" -> ");
        Serial.print(tankTemp);
        Serial.println("C");
        if(lastValue[0] == modbusMessage[messageIndex].au16reg[0] && 
           currentTime - lastSentTime[messageIndex] > MAX_TIME_BETWEEN_SEND) {
          Serial.println("Skip tank temp send");
          break;
        }
        
        // for some reason there was values of 168 in domoticz logs
        if(tankTemp > -5.0) {
          send(msgTankTemp.set(tankTemp,2));
        }
        break;
      };
    case READ_POWER:
      {
        Serial.print("READ POWER: ");
        float acCurrent = ((float)modbusMessage[messageIndex].au16reg[0])*0.1;
        float acVoltage = ((float)modbusMessage[messageIndex].au16reg[1]);
        Serial.print(acCurrent);
        Serial.print("A ");
        Serial.print(acVoltage);
        Serial.println("V");
        if(lastValue[0] == modbusMessage[messageIndex].au16reg[0] &&
           lastValue[1] == modbusMessage[messageIndex].au16reg[1] && 
           currentTime - lastSentTime[messageIndex] > MAX_TIME_BETWEEN_SEND) {
          Serial.println("Skip power send");
          break;
        }
        
        send(msgACCurrent.set(acCurrent,2));
        send(msgACVoltage.set(acVoltage,2));
        break;
      };
    case READ_FREQUENCY:
      Serial.print("READ FREQUENCY: ");
      Serial.println(modbusMessage[messageIndex].au16reg[0]);
      if(lastValue[0] == modbusMessage[messageIndex].au16reg[0] && 
         currentTime - lastSentTime[messageIndex] > MAX_TIME_BETWEEN_SEND) {
        Serial.println("Skip frequency send");
        break;
      }
      send(msgFreqComp.set(modbusMessage[messageIndex].au16reg[0]));
      break;
    case READ_EXPVALV:
      Serial.print("READ EXPVALV: ");
      Serial.println(modbusMessage[messageIndex].au16reg[0]);
      if(lastValue[0] == modbusMessage[messageIndex].au16reg[0] &&
         currentTime - lastSentTime[messageIndex] > MAX_TIME_BETWEEN_SEND) {
        Serial.println("Skip expvalve send");
        break;
      }
      send(msgOpenExpValve.set(modbusMessage[messageIndex].au16reg[0]));
      break;
    case READ_OUTTEMP:
      {
        Serial.print("READ OUTTEMP: ");
        Serial.print(modbusMessage[messageIndex].au16reg[0]);
        float outTemp = ((float)modbusMessage[messageIndex].au16reg[0]/2.0)-60.0;
        Serial.print(" -> ");
        Serial.print(outTemp);
        Serial.println("C");
        if(lastValue[0] == modbusMessage[messageIndex].au16reg[0] &&
           currentTime - lastSentTime[messageIndex] > MAX_TIME_BETWEEN_SEND) {
          Serial.println("Skip out temo send");
          break;
        }
        // for some reason there was values of 168 in domoticz logs
        if(outTemp < 45.0) {
          send(msgOutTemp.set(outTemp,2));
        }
        break;
      };
    case READ_INDOOR_INLET_TEMP:
      {
        Serial.print("READ INDOOR_INLET_TEMP: ");
        Serial.print(modbusMessage[messageIndex].au16reg[0]);
        float indoorInletTemp = ((float)modbusMessage[messageIndex].au16reg[0]/2.0)-60.0;
        Serial.print(" -> ");
        Serial.print(indoorInletTemp);
        Serial.println("C");
        if(lastValue[0] == modbusMessage[messageIndex].au16reg[0] &&
           currentTime - lastSentTime[messageIndex] > MAX_TIME_BETWEEN_SEND) {
          Serial.println("Skip indoor inlet temp send");
          break;
        }
        // for some reason there was values of 168 in domoticz logs
        //if(indoorInletTemp < 100.0) {
          send(msgIndoorInletTemp.set(indoorInletTemp,2));
        //}
        break;
      };
    case READ_OUTDOOR_COIL_TEMP:
      {
        Serial.print("READ READ_OUTDOOR_COIL_TEMP: ");
        Serial.print(modbusMessage[messageIndex].au16reg[0]);
        float outdoorCoilTemp = ((float)modbusMessage[messageIndex].au16reg[0]/2.0)-60.0;
        Serial.print(" -> ");
        Serial.print(outdoorCoilTemp);
        Serial.println("C");
        if(lastValue[0] == modbusMessage[messageIndex].au16reg[0] &&
           currentTime - lastSentTime[messageIndex] > MAX_TIME_BETWEEN_SEND) {
          Serial.println("Skip out coil temp send");
          break;
        }
        // for some reason there was values of 168 in domoticz logs
        //if(outTemp < 100.0) {
          send(msgOutdoorCoilTemp.set(outdoorCoilTemp,2));
        //}
        break;
      };
    case READ_OUTDOOR_EXHAUSTING_TEMP:
      {
        Serial.print("READ_OUTDOOR_EXHAUSTING_TEMP: ");
        Serial.print(modbusMessage[messageIndex].au16reg[0]);
        float outExhaustTemp = ((float)modbusMessage[messageIndex].au16reg[0]/2.0)-60.0;
        Serial.print(" -> ");
        Serial.print(outExhaustTemp);
        Serial.println("C");
        if(lastValue[0] == modbusMessage[messageIndex].au16reg[0] &&
           currentTime - lastSentTime[messageIndex] > MAX_TIME_BETWEEN_SEND) {
          Serial.println("Skip out ext temp send");
          break;
        }
        // for some reason there was values of 168 in domoticz logs
        //if(outTemp < 100.0) {
          send(msgOutdoorExhaustTemp.set(outExhaustTemp,2));
        //}
        break;
      };
    case READ_ERRORS_0_7:
      {
        Serial.println("READ_ERRORS_0_7: ");
        for(int i = 0; i < 8; i++) {
          if(modbusMessage[messageIndex].au16reg[i] != 0) {
            Serial.print(i);
            Serial.print(":");
            Serial.println(error_values[i]);
            send(msgError.set(error_values[i]));
            wdt_reset();
            wait(1000);
          }
        }
        break;
      };
    case READ_ERRORS_8_14:
      {
        Serial.println("READ_ERRORS_8_14: ");
        for(int i = 0; i < 7; i++) {
          if(modbusMessage[messageIndex].au16reg[i] != 0) {
            Serial.print(i+8);
            Serial.print(":");
            Serial.println(error_values[i+8]);
            send(msgError.set(error_values[i + 8]));
            wdt_reset();
            wait(1000);
          }
        }
        break;
      };


  }
}

// handling incoming messages from HA-controller
void receive(const MyMessage &message) {
  Serial.print("Incoming change for sensor:");
  Serial.println(message.sensor);
  
  if (message.type==V_HVAC_FLOW_STATE) {

    Serial.print("FLOW STATE: ");
    
    // write new flow state
    const char* state = message.getString();
    Serial.println(state);
    if(state == "Off") {
      modbusMessage[WRITE_RUNMODE].au16reg[0] = 0; // Standby
    } else if(state == "HeatOn") {
      modbusMessage[WRITE_RUNMODE].au16reg[0] = 4; // Heating Mode by Water
    } else if(state == "CoolOn") {
      modbusMessage[WRITE_RUNMODE].au16reg[0] = 2; // Cooling Mode by Water
    } else {
      modbusMessage[WRITE_RUNMODE].au16reg[0] = 5; // Water Heater Mode
    }
    addToBeginingOfSendQueue(WRITE_RUNMODE);
    addToEndOfSendQueue(READ_RUNMODE);
    
  } else if (message.type==V_DIMMER) {
    Serial.print("V_DIMMER: ");
    int val = message.getInt();
    Serial.println(val);
    bool setVal = true;
    if(val < 10) // 0: StandBy
      modbusMessage[WRITE_RUNMODE].au16reg[0] = 0;
    else if(val < 20) // 10: CoolByRoom
      modbusMessage[WRITE_RUNMODE].au16reg[0] = 1;
    else if(val < 30) // 20: CoolByWater
      modbusMessage[WRITE_RUNMODE].au16reg[0] = 2;
    else if(val < 40) // 30: HeatByRoom
      modbusMessage[WRITE_RUNMODE].au16reg[0] = 3;
    else if(val < 50) // 40: HeatByWater
      modbusMessage[WRITE_RUNMODE].au16reg[0] = 4;
    else if(val < 60) // 50: WaterHeaterMode
      modbusMessage[WRITE_RUNMODE].au16reg[0] = 5;
    else
      setVal = false; // Cannot set mode Level I of freeze(6) or Level II of freeze(7)   

    if(setVal) {
      addToBeginingOfSendQueue(WRITE_RUNMODE);
    }
    addToEndOfSendQueue(READ_RUNMODE);
    
  } else if (message.type==V_LIGHT) {
  
    // on/off switch hack for older domoticz
    
    Serial.print("ON/OFF: ");
    Serial.println(message.getBool());
    if(message.getBool()) {
      modbusMessage[WRITE_RUNMODE].au16reg[0] = 4;
    } else {
      modbusMessage[WRITE_RUNMODE].au16reg[0] = 0;
    }

    addToBeginingOfSendQueue(WRITE_RUNMODE);
    addToEndOfSendQueue(READ_RUNMODE);
  
  } else if (message.type==V_HVAC_SETPOINT_HEAT) {
    
    Serial.print("SETPOINT HEAT: ");
    Serial.println(message.getInt());
    modbusMessage[WRITE_SETPOINT_HEAT_BY_WATER].au16reg[0] = message.getInt();
    
    // this is becouse of on/off switch hack for older domoticz
    if(modbusMessage[READ_RUNMODE].au16reg[0] == 5) {
       addToBeginingOfSendQueue(WRITE_SETPOINT_WATERHEAT);
       addToEndOfSendQueue(READ_SETPOINT_WATERHEAT); 
    } else {
      addToBeginingOfSendQueue(WRITE_SETPOINT_HEAT_BY_WATER);
      addToEndOfSendQueue(READ_SETPOINT_HEAT_BY_WATER);
    }
    
  } else if (message.type==V_HVAC_SETPOINT_COOL) {
    
    Serial.print("SETPOINT COOL: ");
    Serial.println(message.getInt());
    modbusMessage[WRITE_SETPOINT_COLD_BY_WATER].au16reg[0] = message.getInt();
    addToBeginingOfSendQueue(WRITE_SETPOINT_COLD_BY_WATER);
    addToEndOfSendQueue(READ_SETPOINT_COLD_BY_WATER);
    
  }
}

// returns -1 when there is no messages to send. Otherwise index of the next pending message.
int getFirstMessageIndex() {
  
  int smallest = 32767; // int max
  int messageIndex = -1;
  for(int i = 0; i < BUFFER_COUNT; i++)
    if(messageOrder[i] > 0 && messageOrder[i] < smallest) {
      smallest = messageOrder[i];
      messageIndex = i;
    }
  return messageIndex;
}

// check the biggest number in queue
int queueMaxOrderNumber() {
  
  int biggest = 0;
  for(int i = 0; i < BUFFER_COUNT; i++)
    if(messageOrder[i] > 0 && messageOrder[i] > biggest) {
      biggest = messageOrder[i];
    }

  return biggest;
}

// add message(index) to end of queue
void addToEndOfSendQueue(int index) {
  
  int maxOrder = queueMaxOrderNumber(); // check biggest order number in queue
  
  // todo: handle int max.. 
  // reorganize the whole queue.. 
  // should not happend never.. hopefully :D
  
  messageOrder[index] = maxOrder + 1; // add one
}

// add message(index) to begining of queue
void addToBeginingOfSendQueue(int index) {
  int firstQueueMessageIndex = getFirstMessageIndex(); // get first message from queue. should be never zero.

  // check if queue is empty or number one is not set -> set message to first and return
  // most of the time this should be true..
  if(firstQueueMessageIndex == -1 || messageOrder[firstQueueMessageIndex] > 1) {
    messageOrder[index] = 1;
    return;
  }

  // number one is set.. we must shift all order definitions
  for(int i = 0; i < BUFFER_COUNT; i++) {
    messageOrder[i]++;
  }
  sendIndex++;
  messageOrder[index] = 1;
}

/*
3000 Run Mode 0: Standby, 1: Cooling Mode by Room, 2: Cooling Mode by Water, 
              3: Heating Mode by Room, 4: Heating Mode by Water, 5: Water Heater Mode
              6: Level I of freeze, 7: Level II of freeze
3001 Run mode 1 setpoint: 10℃~31℃
3002 Run mode 2 setpoint: 7℃~25℃
3003 Run mode 3 setpoint: 10℃~31℃
3004 Run mode 4 setpoint: 7℃~52℃
3005 Run mode 5 setpoint: 20℃~52℃
---
3100 Compressor frequency: 0-120
3101 Wired controller temp: 0-80
3102 Outdoor outlet water temp: (temperature+60℃)*2 NOT USED IN AIR TO WATER
3103 Indoor outlet water temp: (temperature+60℃)*2 (Tank center)
3104 Indoor inlet water temp: (temperature+60℃)*2 (Tank bottom)
3105 Heat sink temp: NOT USED
3106 Indoor coil temp: (temperature+60℃)*2 (Outdoor inlet water temp in water out systems)
3107 Outdoor air temp: (temperature+60℃)*2 (Out air)
3108 Outdoor coil temp: (temperature+60℃)*2 (Evaporator temp)
3109 Outdoor exhausting temp: (temperature+60℃)*2 (Compresor exhaust temp)
---
3120 Input current: 0-225=0.0-22.5 Arms
3121 Input AC voltage: 0-365=0-365V
3122 Expansion Valve Openness: 0-500
3123 DC Bus Voltage: NOT IN USE CURRENTLY
---
3124 Communication error
3125 Indoor temperature sensor fault 
3126 Input voltage or current sensor fault
3127 Drive failure protection or drive fault
3128 Indoor unit EEPROM fault
3129 Overload protection
3130 Input under voltage protection
3131 High pressure sensor fault
3132 Outdoor unit EEPROM fault
3133 Outdoor water flow protection
3134 Outdoor temperature sensor fault
3135 High-pressure protection
3136 Outdoor temperature protection
3137 Indoor coil temperature protection
3138 Indoor water flow protection
 */
