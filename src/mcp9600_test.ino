#define pinNozzel 4

#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include "Adafruit_MCP9600.h"
#include "PID_v1.h"

#define I2C_ADDRESS (0x67)

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
double Setpoint_2 = 100; //PID2
double Kp_2 = 2.219, Ki_2 = 0.0005,  Kd_2 = 0;
double aggKp_2 = 1, aggKi_2 = 0, aggKd_2 = 0;
byte aggSP_2 = 10;
bool AdaptiveMode_2 = false;
bool SSRArmed = false;

//Duty Cycle

float dC = 0.0;
bool ELEMENT = false;
int dCLoopTime = 10; // Duty cycle total loop time in seconds.
unsigned long ElementONTime = 0;
unsigned long ElementOFFTime = 0;
#define ElementPowerPin 5


//Duty Cycle 2

float dC2 = 0.0;
bool ELEMENT2 = false;
int dCLoopTime2 = 10; // Duty cycle total loop time in seconds.
unsigned long ElementONTime2 = 0;
unsigned long ElementOFFTime2 = 0;
#define ElementPowerPin2 4

// Specify the links and initial tuning parameters
// PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID( &Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID_2( &Input_2, &Output_2, &Setpoint_2, Kp_2, Ki_2, Kd_2, DIRECT);


//Timer variables
unsigned long last_loop_time = 0;
unsigned long last_loop_time_3 = 0;


void setup()
{

  pinMode(ElementPowerPin, OUTPUT);
  pinMode(ElementPowerPin2, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {
      delay(10);
    }
  Serial.println("PID v1.0.0.1 akp");

  /* Initialise the driver with I2C_ADDRESS and the default I2C bus. */
  if (! mcp.begin(I2C_ADDRESS)) {
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
  // Serial.print("Thermocouple type set to ");
  // switch (mcp.getThermocoupleType()) {
  //   case MCP9600_TYPE_K:  Serial.print("K"); break;
  //   case MCP9600_TYPE_J:  Serial.print("J"); break;
  //   case MCP9600_TYPE_T:  Serial.print("T"); break;
  //   case MCP9600_TYPE_N:  Serial.print("N"); break;
  //   case MCP9600_TYPE_S:  Serial.print("S"); break;
  //   case MCP9600_TYPE_E:  Serial.print("E"); break;
  //   case MCP9600_TYPE_B:  Serial.print("B"); break;
  //   case MCP9600_TYPE_R:  Serial.print("R"); break;
  // }
  //Serial.println(" type");

  mcp.setFilterCoefficient(3);
  //Serial.print("Filter coefficient value set to: ");
  //Serial.println(mcp.getFilterCoefficient());

  mcp.setAlertTemperature(1, 30);
  //Serial.print("Alert #1 temperature set to ");
  //Serial.println(mcp.getAlertTemperature(1));
  mcp.configureAlert(1, true, true);  // alert 1 enabled, rising temp

  mcp.enable(true);

  //Serial.println(F("------------------------------"));
}

void loop()
{
  Serial.print("Element 1 - Nozzel "); Serial.println(((mcp.readThermocouple()*9)/5)+32);
  //Serial.print("Cold Junction: "); Serial.println(((mcp.readAmbient()*9)/5)+32);
  //Serial.print("ADC: "); Serial.print(mcp.readADC() * 2); Serial.println(" uV");
  //delay(1000);
  //digitalWrite(pinNozzel, HIGH); // sets the digital pin 13 on
  //delay(10000);            // waits for a second
  //digitalWrite(pinNozzel, LOW);  // sets the digital pin 13 off
  //delay(1000); 

  // get PID inputs, set agg constants, send LCD vars
  if ((millis() - last_loop_time) > 10000)  {
    Input_2 = 100; // TESTI
    Serial.print("Input PID2 :");
    Serial.println(Input_2);
    Input = ((mcp.readThermocouple()*9)/5)+32;
    Serial.print("Lower Spool Input PID1:");
    Serial.println(Input);

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


  if ( (millis() - last_loop_time_3) > 10000) { // dC Calc
    if (PIDMode == true) {
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
}