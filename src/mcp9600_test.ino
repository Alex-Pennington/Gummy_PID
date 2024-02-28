#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include "Adafruit_MCP9600.h"
#include "PID_v1.h"

#define MCP_I2C_ADDRESS (0x67)

Adafruit_MCP9600 mcp;

//PID

int gap = 0, gap_2 = 0;
double Input = 0, Output = 0;
double Input_2 = 0, Output_2 = 0;

//Calibration Values
bool PIDMode = false;
double Setpoint = 70; //PID1
double Kp = 2, Ki = .2,  Kd = 1;
double aggKp = 1, aggKi = 0, aggKd = 0;
byte aggSP = 10;
bool AdaptiveMode = false;

bool PIDMode_2 = true;
double Setpoint_2 = 125; //PID2
double Kp_2 = .5, Ki_2 = .0005,  Kd_2 = 0;
double aggKp_2 = 2, aggKi_2 = 6, aggKd_2 = 0.1;
byte aggSP_2 = 10;
bool AdaptiveMode_2 = false;
bool SSRArmed = true;

//Duty Cycle

float dC = 0.0;
bool ELEMENT = false;
int dCLoopTime = 1; // Duty cycle total loop time in seconds.
unsigned long ElementONTime = 0;
unsigned long ElementOFFTime = 0;
#define ElementPowerPin 5


//Duty Cycle 2

float dC2 = 0.0;
bool ELEMENT2 = false;
int dCLoopTime2 = 1; // Duty cycle total loop time in seconds.
unsigned long ElementONTime2 = 0;
unsigned long ElementOFFTime2 = 0;
#define ElementPowerPin2 6

// Specify the links and initial tuning parameters
// PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID( &Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID_2( &Input_2, &Output_2, &Setpoint_2, Kp_2, Ki_2, Kd_2, DIRECT );


//Timer variables
unsigned long last_loop_time = 0;
unsigned long last_loop_time_3 = 0;

//===================
// Using I2C to send and receive structs between two Arduinos
//   SDA is the data connection and SCL is the clock connection
//   On an Uno  SDA is A4 and SCL is A5
//   On an Mega SDA is 20 and SCL is 21
//   GNDs must also be connected
//===================


        // data to be sent and received
struct I2cTxStruct {
    char textA[16];        // 16 bytes
    double valA;           //  4
    unsigned long valB;    //  4
    byte padding[8];       //  8
                           //------
                           // 32
};

struct I2cRxStruct {
    char textB[16];        // 16 bytes
    double valC;           //  4
    unsigned long valD;    //  4
    byte padding[8];       //  8
                           //------
                           // 32
};

I2cTxStruct txData = {"xxx", 236, 0};
I2cRxStruct rxData;

bool newTxData = false;
bool newRxData = false;
bool rqSent = false;

const byte thisAddress = 9; // these need to be swapped for the other Arduino
const byte otherAddress = 8;

void setup()
{

  pinMode(ElementPowerPin, OUTPUT);
  pinMode(ElementPowerPin2, OUTPUT);

  //turn the PID on
  myPID.SetOutputLimits(0, 99);
  myPID.SetSampleTime(1000);
  myPID.SetMode(MANUAL);

  myPID_2.SetOutputLimits(0, 5);
  myPID_2.SetSampleTime(1000);
  myPID_2.SetMode(MANUAL);

  Serial.begin(9600);
  Serial.println("PID v1.0.0.1 akp");

  /* Initialise the driver with I2C_ADDRESS and the default I2C bus. */
  if (! mcp.begin(MCP_I2C_ADDRESS)) {
      Serial.println("Sensor not found. Check wiring!");
      while (1);
    }

  Serial.println("Found MCP9600!");

  mcp.setADCresolution(MCP9600_ADCRESOLUTION_18);
  Serial.print("ADC resolution set to ");
  switch (mcp.getADCresolution()) {
    case MCP9600_ADCRESOLUTION_18:   Serial.print("18"); break;
    case MCP9600_ADCRESOLUTION_16:   Serial.print("16"); break;
    case MCP9600_ADCRESOLUTION_14:   Serial.print("14"); break;
    case MCP9600_ADCRESOLUTION_12:   Serial.print("12"); break;
  }
  Serial.println(" bits");

  mcp.setThermocoupleType(MCP9600_TYPE_K);

  mcp.setFilterCoefficient(3);
  //Serial.print("Filter coefficient value set to: ");
  //Serial.println(mcp.getFilterCoefficient());

  mcp.setAlertTemperature(1, 30);
  //Serial.print("Alert #1 temperature set to ");
  //Serial.println(mcp.getAlertTemperature(1));
  mcp.configureAlert(1, true, true);  // alert 1 enabled, rising temp

  mcp.enable(true);


  myPID_2.SetMode(AUTOMATIC);

  Wire.begin(thisAddress); // join i2c bus
  Wire.onReceive(receiveEvent); // register function to be called when a message arrives
  Wire.onRequest(requestEvent); // register function to be called when a request arrives

}

void loop()
{
  // this bit checks if a message has been received
  if (newRxData == true) {
    showNewData();
    newRxData = false;
  }
  // this function updates the data in txData
  // this function sends the data if one is ready to be sent
  updateDataToSend();


  // get PID inputs, set agg constants, send LCD vars
  if ((millis() - last_loop_time) > 1000)  {
    Input_2 = ((mcp.readThermocouple()*9)/5)+32; // TESTI
    Input = ((mcp.readThermocouple()*9)/5)+32;
    Serial.print("Input PID2 : ");
    Serial.println(Input_2);

    gap = abs(Setpoint - Input); //distance away from setpoint
    if ((gap < aggSP && AdaptiveMode == true) || AdaptiveMode == false)
    { //we're close to setpoint, use conservative tuning parameters
      myPID.SetTunings(Kp, Ki, Kp);
    }
    else if (gap > aggSP && AdaptiveMode == true)
    {
      //we're far from setpoint, use aggressive tuning parameters
      myPID.SetTunings(aggKp, aggKi, aggKd);
    }
    gap_2 = abs(Setpoint_2 - Input_2); //distance away from setpoint
    if ((gap_2 < aggSP_2 && AdaptiveMode_2 == true) || AdaptiveMode_2 == false)
    { //we're close to setpoint, use conservative tuning parameters
      myPID_2.SetTunings(Kp_2, Ki_2, Kp_2);
    }
    else if (gap_2 > aggSP_2 && AdaptiveMode_2 == true )
    {
      //we're far from setpoint, use aggressive tuning parameters
      myPID_2.SetTunings(aggKp_2, aggKi_2, aggKd_2);
    }
    last_loop_time = millis();
    // end of old_time local
  }

  myPID.Compute();
  myPID_2.Compute();

  if ( (millis() - last_loop_time_3) > 1000) { // dC Calc
    if (PIDMode == true) {
      Serial.print("Input PID1 : ");
      Serial.println(Input);
      Serial.print("Output : ");
      Serial.println(Output);
      dC = (Output / 100.0);
    }
    if ((PIDMode_2 == true)) {
      Serial.print("Output_2 : ");
      Serial.println(Output_2);
      dC2 = (Output_2 / 100.0);
    }
    last_loop_time_3 = millis();
  }

  DutyCycleLoop();
}

void DutyCycleLoop() {

  if ((dC > 0 && SSRArmed == true)  && (PIDMode == true)) {
    if (!ELEMENT) {
      if ((millis() - ElementONTime) > (dC * dCLoopTime * 1000)) {
        ElementOFFTime = millis();
        ElementONTime = 0;
        digitalWrite(ElementPowerPin, HIGH);
        ELEMENT = HIGH;
      }
    } else {
      if ((millis() - ElementOFFTime) > ((1 - dC) * dCLoopTime * 1000)) {
        ElementONTime = millis();
        ElementOFFTime = 0;
        digitalWrite(ElementPowerPin, LOW);
        ELEMENT = LOW;
      }
    }
  } else {
    digitalWrite(ElementPowerPin, HIGH);
    ElementONTime = 0;
    ELEMENT = HIGH;
  }

  if ((dC2 > 0 && SSRArmed == true) && ( PIDMode_2 == true )) {
    if (!ELEMENT2) {
      if ((millis() - ElementONTime2) > (dC2 * dCLoopTime2 * 1000)) {
        ElementOFFTime2 = millis();
        ElementONTime2 = 0;
        digitalWrite(ElementPowerPin2, !HIGH);
        digitalWrite(LED_BUILTIN, !HIGH);
        ELEMENT2 = HIGH;
      }
    } else {
      if ((millis() - ElementOFFTime2) > ((1 - dC2) * dCLoopTime2 * 1000)) {
        ElementONTime2 = millis();
        ElementOFFTime2 = 0;
        digitalWrite(ElementPowerPin2, !LOW);
        digitalWrite(LED_BUILTIN, !LOW);
        ELEMENT2 = LOW;
      }
    }
  } else {
    digitalWrite(ElementPowerPin2, !HIGH);
    digitalWrite(LED_BUILTIN, !HIGH);
    ElementONTime2 = 0;
    ELEMENT2 = HIGH;
  }
}

void updateDataToSend() {
  // update the data after the previous message has been
  //    sent in response to the request
  // this ensures the new data will ready when the next request arrives
  if (rqSent == true) {
    rqSent = false;
    char sText[] = "yyy";
    strcpy(txData.textA, sText);
    txData.valA = Input_2;
    txData.valB = millis();
  }
}

void showTxData() {
  // for demo show the data that as been sent
  Serial.print("Sent ");
  Serial.print(txData.textA);
  Serial.print(' ');
  Serial.print(txData.valA);
  Serial.print(' ');
  Serial.println(txData.valB);
}

void showNewData() {
  Serial.print("This just in    ");
  Serial.print(rxData.textB);
  Serial.print(' ');
  Serial.print(rxData.valC);
  Serial.print(' ');
  Serial.println(rxData.valD);
}

// this function is called by the Wire library when a message is received
void receiveEvent(int numBytesReceived) {
  if (newRxData == false) {
    // copy the data to rxData
    Wire.readBytes( (byte*) &rxData, numBytesReceived);
    newRxData = true;
  }
  else {
    // dump the data if we havent seen the last message
    while(Wire.available() > 0) {
      byte c = Wire.read();
    }
  }
}

void requestEvent() {
  Wire.write((byte*) &txData, sizeof(txData));
  rqSent = true;
}