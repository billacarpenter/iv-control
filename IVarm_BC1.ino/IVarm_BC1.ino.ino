
//#define DEBUG 1
// IVArm MCU 
// control of:
// A) valve driver
// B) flow switch
// C) IV puck initialization switch
// D) pulse motor driver
// E) pulse palpation sensor
//
// communication with AMM core via serial bridge
//
#define BAUDRATE 115200 //Open the serial port at 115200 baud

boolean tagged = false;
String key = "";
String _name = "";
String value = "";
String current_capability = "";
boolean current_enabled = false;

// capability enabled flags
boolean pulse_capability = false;
boolean iv_detection_capability = false;
boolean pulse_palpation_capability = false;
boolean flowRateDetected = false;         // TODO: deprecated; replace with ivState == 3

float heartRate = 0;
float systolic = 0;

// setting pin numbers for valve and flow switch
const int valvePin = 9;          // valve pin number 
const int flowSwitchPin = 8;     // flow switch pin number
const int ivInitButtonPin = 7;   // discreet switch to initialize the IV arm puck
const int fsrPin = A0;           // FSR - force sensitive resistor
const int pulseMotorPin = 3;     // pulse motor output to driver
const int pulseLED = 13;         // LED for visual indication of pulse         

unsigned int pulsePalCnt = 0;

// placeholder for operation without AMM core
const int pulseCheckLEDPin = 6;
const int ivFlowLEDPin = 5;

/* IV placement control states 
    0 - startup, waiting for puck fill command
    1 - fill puck
    2 - IV placement
    3 - IV on
    4 - IV stopped
*/
int ivState = 0;  
    
unsigned long currentTime = millis(); //to keep track of how much time there has been flow
unsigned long flowTimerStart = currentTime; // reset timer 

int fillTime = 5000;  // 5 seconds flow without any bubbles to consider iv filled
int ivFlowTime = 3000; // minimum time of continuous flow before IV is considered placed 
int ivFlowToggleTime = 2000;

/* This is the standard setup() function. It begins serial communication with
 * the AMM DDS Serial Bridge. It then publishes this module's custom
 * configuration data.
 *
 * All modules must implement the 3 steps of the handshake process with AMM:
 * 1) Publish their configuration data including:
 *   a) module info
 *   b) capabilities
 *   c) published data by capability
 *   d) subscribed data by capability
 *   e) custom configuration fields by capability
 * 2) Process configuration data and enable capabilities
 * 3) Publish initial operation status (and subsequent change to status)
 */

///////////////// Begin SETUP
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(BAUDRATE);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  printConfig();

  // set up IO
  pinMode(valvePin, OUTPUT);
  pinMode(flowSwitchPin, INPUT);
  pinMode(ivInitButtonPin, INPUT);
  pinMode(fsrPin, INPUT);
  pinMode(pulseCheckLEDPin, OUTPUT);
  pinMode(ivFlowLEDPin, OUTPUT);  
  pinMode(pulseMotorPin, OUTPUT);

  digitalWrite(ivFlowLEDPin, LOW);
  digitalWrite(pulseCheckLEDPin, LOW);

  cli();//stop interrupts
  //set timer1 interrupt at 1Hz; interrupt handler ISR()
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;   // Set for 60 bpm = 15624; 50 bpm = 18751; OCR1A = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  //allow interrupts
  sei();
  
  Serial.println("IVarm MCU ready");
  delay(1000); 
  printConfig();  
}
/////////////////// End SETUP

/* This is the standard loop() function. It process one character of serial
 * input from the AMM DDS Serial Bridge. It also processes any signal input
 * from pins or grove sensors and publishes messages to the AMM DDS Serial
 * Bridge. Messages have a key, a name, and an optional value and take the form
 * of [KEY]NAME or [KEY]NAME=VALUE. Incoming messages are handled in another
 * function, but this function will need to be modified to handle sensor input
 * and publish messages to the AMM DDS Serial Bridge.
 */

/////////////////// Begin MAIN LOOP 
void loop() {
  int initButton = digitalRead(ivInitButtonPin);
  int flowStatus = digitalRead(flowSwitchPin);

  // process one character of serial input at a time
  if (Serial.available()) processCharacter(Serial.read());

  currentTime = millis();
  
  if (ivState != 1 && initButton) {                 
    ivState = 1;                          // transition to ivState 1: fill puck
    digitalWrite(valvePin, HIGH);         //open valve
    flowTimerStart = currentTime;  // reset timer
    Serial.println("IVarm initialization: filling puck");   
    digitalWrite(ivFlowLEDPin, LOW);
    digitalWrite(pulseCheckLEDPin, LOW);
  }
  
  if (ivState == 1) {
    if (flowStatus == 0) {     // if no flow or bubbles reset flow timer
      flowTimerStart = currentTime;  // reset timer
    } else if (currentTime - flowTimerStart > fillTime) {
        ivState = 2;                      // transition to ivState 2: IV placement
        digitalWrite(valvePin, LOW);
        Serial.print("IV puck filled sucessfully");
        flowTimerStart = currentTime;  // reset timer
    }
//    Serial.print("Flow detected: ");
//    Serial.println(flowStatus);   
  } 

  if (ivState == 2) {
    if (flowStatus == 0) {     // if no flow or bubbles reset flow timer
      flowTimerStart = currentTime;  // reset timer
    } else if (currentTime - flowTimerStart > ivFlowTime) {
        ivState = 3;                      // transition to ivState 3: IV on
        digitalWrite(ivFlowLEDPin, HIGH);
        flowTimerStart = currentTime;  // reset timer
        /*
         * send message to core that IV has been placed
         */
        Serial.println("IV placed successfully");  
        digitalWrite(ivFlowLEDPin, HIGH);
        ivPlacementDetected();
   }
  }

  if (ivState == 3) {               // IV is on
    if (flowStatus == 1) {     // if flow is on reset flow timer; off delay
      flowTimerStart = currentTime;  // reset timer
    } else if (currentTime - flowTimerStart > ivFlowToggleTime) {
      ivState = 4;                         // transition to ivState 4: IV stopped
        digitalWrite(ivFlowLEDPin, LOW);
        flowTimerStart = currentTime;  // reset timer
        Serial.println("IV flow stopped");
    }
  }

  if (ivState == 4) {               // IV is off
    if (flowStatus == 0) {     // if no flow or bubbles reset flow timer
      flowTimerStart = currentTime;  // reset timer
    } else if (currentTime - flowTimerStart > ivFlowToggleTime) {
      ivState = 3;                         // transition to ivState 3: IV on
        digitalWrite(ivFlowLEDPin, HIGH);
        flowTimerStart = currentTime;  // reset timer
        Serial.println("IV flow is on");
    }
    
  }

}
/////////////// End MAIN LOOP

/////////////// Begin INTERRUPT SERVICE ROUTINE
ISR(TIMER1_COMPA_vect){                 //timer1 interrupt at pulse rate (1Hz - default)
//  Serial.println(" Interrupt handler ");
  //  int i = 2048;
  int i = 1080;
// TODO: the pulse train output in the while loop is blocking; replace with PWM(?)
    while (i > 0) {
      digitalWrite(pulseMotorPin,HIGH);
      digitalWrite(pulseMotorPin,HIGH);
      digitalWrite(pulseMotorPin,LOW);
      digitalWrite(pulseMotorPin,LOW);
      digitalWrite(pulseMotorPin,LOW);
      digitalWrite(pulseMotorPin,LOW);
      digitalWrite(pulseMotorPin,LOW);
      digitalWrite(pulseMotorPin,LOW);
      digitalWrite(pulseMotorPin,LOW);
      digitalWrite(pulseMotorPin,LOW);
      digitalWrite(pulseMotorPin,LOW);
      digitalWrite(pulseMotorPin,LOW);
      digitalWrite(pulseMotorPin,LOW);
      digitalWrite(pulseMotorPin,LOW);
      digitalWrite(pulseMotorPin,LOW);
      digitalWrite(pulseMotorPin,LOW);   
      i--;
    }
    
    Serial.print("Heart Rate ");
    Serial.println(heartRate);
    //Serial.println(OCR1A);
    Serial.print("Pressure Systolic ");
    Serial.println(systolic);

// pulse palpation check after pulse train is off
// polling at pulse rate only    
    int fsrADC = analogRead(fsrPin);
    Serial.print("FSR Reading");
    Serial.println(fsrADC);
    if (fsrADC > 200) {
        Serial.println("Pulse palpation detected");
        pulsePalCnt++;
    }
    if (fsrADC <= 200) {
        Serial.println( "Pulse palpation not detected");
        //count is reset
        pulsePalCnt =  0;
    }
    if (pulsePalCnt > 15) {
        Serial.println("Successful pulse palpation");
        pulsePalpationDetected();
        digitalWrite(pulseCheckLEDPin, HIGH);
        pulsePalCnt = 0;
    }
}
/////////////// End INTERRUPT SERVICE ROUTINE
  
/////////////// Begin COMMUNICATION AMM CORE
/* This function is called as part of publishing configuration and reporting
 * status.
 *
 * You will need to modify the module name, manufacture, model, serial number,
 * and module version data for your specific module.
 */

void printModuleInfo() {
  Serial.print(F("  <module name=\"ivarm\" manufacturer=\"Vcom3D_CREST\" model=\"ivarm_mule_2\" serial_number=\"903745\" module_version=\"0.0.1\">"));
}
/* This function is called by processCharacter() when a key/name/value set has
 * been parsed. You will need to modify this function to process incoming
 * serial messages.
 *
 * Examples of key/name and key/name/value sets are below:
 *
 * [AMM_Command]DO_THIS_THING
 * [AMM_Node_Data]Cardiovascular_Heart_Rate=72
 * [AMM_Pain_Response]TBD
 * [Scenario]scenario_name=id
 * [Capability]cap_name=true/false
 * [Config_Data]field_name=field_value
 * [Config_Data]sound_alarm=false
*/
void processKeyNameValue(String k, String n, String v) {
#ifdef DEBUG
  Serial.println("");
  Serial.print(F("key("));
  Serial.print(k);
  Serial.print(F(") name("));
  Serial.print(n);
  Serial.print(F(") value("));
  Serial.print(v);
  Serial.println(F(")"));
  Serial.flush();
#endif // DEBUG

  if (k.equals(F("AMM_Command"))) {

  }
  else if (k.equals(F("Scenario"))) {
    // clear old capabilities
    pulse_capability = false;
    iv_detection_capability = false;
    pulse_palpation_capability = false;
    flowRateDetected = false;
  }
  else if (k.equals(F("AMM_Node_Data"))) {
    if (n.equals(F("Cardiovascular_HeartRate"))) {      
      heartRate = v.toFloat();
      OCR1A = (16000000/((heartRate/60)*1024) - 1);
    }
    else if (n.equals(F("Cardiovascular_Arterial_Systolic_Pressure"))) {      
      systolic = v.toFloat();
    }
  }
  else if (k.equals(F("Capability"))) {
    current_capability = n;
    current_enabled = !v.equalsIgnoreCase(F("FALSE"));
  }

  else if (k.equals(F("/Capability"))) {
    if (current_capability.equals(F("iv_detection"))) {
      iv_detection_capability = current_enabled;
      if (iv_detection_capability) {
          printStatus(F("iv_detection"), F("OPERATIONAL"), F(""));
      }
    }


    else if (current_capability.equals(F("pulse"))) {
      pulse_capability = current_enabled;
      if (pulse_capability) {
          printStatus(F("pulse"), F("OPERATIONAL"), F(""));
      }
    }

    else if (current_capability.equals(F("pulse_palpation"))) {
      pulse_palpation_capability = current_enabled;
      if (pulse_palpation_capability)  {
         printStatus(F("pulse_palpation"), F("OPERATIONAL"), F(""));
      }
      
    }

  }

  else if (k.equals(F("Config_Data"))) {

  }
}
/* This function processes a single character if serial input.
 *
 * <There should be no need to edit this function.>
 */
void processCharacter(char c) {
  if (c == '\r' || c == '\n') {
    /*int bracket = value.lastIndexOf('[');
    String echo = value.substring(bracket);
    Serial.println("");
    Serial.println("Got: " + echo);*/

    if (tagged) {
      value = '[' + key;
      key = "";
    }
    if (key.length() + value.length() > 0) {
      if (value.length() > 0) {
        int eq = value.indexOf('=');
        if (eq > -1) {
          _name = value.substring(0, eq);
          value = value.substring(eq+1, value.length());
        }
      }
      processKeyNameValue(key, _name, value);
    }
    tagged = false;
    _name = "";
    key = "";
    value = "";
  }
  else if (tagged) {
    if (c == ']') tagged = false;
    else key = key + c;
  }
  else {
    if (c == '[' && key.length() + value.length() == 0) tagged = true;
    else {
      value = value + c;
    }
  }
}
/* This function will print the AMM operational status over serial.
 * This needs to be called for each enabled capability after setup is complete
 * and whenever the operational status of that capability changes.
 * params:
 *   capability - the name of the capability whose status you are reporting
 *   status - either "OPERATIONAL", "HALTING_ERROR", or "IMPENDING_ERROR"
 *   message - for informational purposes
 *
 * <There should be no need to edit this function.>
 */
void printStatus(String capability, String status, String message) {
  Serial.print(F("<?xml version=\"1.0\" encoding=\"utf-8\"?>"));
  Serial.print(F("<AMMModuleStatus>"));
  printModuleInfo();
  Serial.print(F("    <capabilities>"));
  Serial.print(F("<capability name=\""));
  Serial.print(capability);
  Serial.print(F("\" status=\""));
  Serial.print(status);
  if (message.length() > 0) {
    Serial.print(F("\" message=\""));
    Serial.print(message);
  }
  Serial.print(F("\"/>"));
  Serial.print(F("    </capabilities>"));
  Serial.print(F("  </module>"));
  Serial.println(F("</AMMModuleStatus>"));
  Serial.flush();
}
/* This function will print the configuration data for this module over serial.
 * Modify this section to define each capability of this module.
 * Each capability should declare which topics are published, which topics are
 * subscribed to, and what configuration data is required.
 */

void printConfig() {
  Serial.print(F("<?xml version=\"1.0\" encoding=\"utf-8\"?>"));
  Serial.print(F("<AMMModuleConfiguration>"));
  printModuleInfo();
  Serial.print(F("    <versions>"));
  Serial.print(F("      <data name=\"amm_core_version\" minimumVersion=\"0.0.1\"/>"));
  Serial.print(F("      <data name=\"amm_specification_version\" minimumVersion=\"0.0.1\"/>"));
  Serial.print(F("    </versions>"));
  Serial.print(F("    <capabilities>"));

  Serial.print(F("      <capability name=\"iv_detection\">"));
  Serial.print(F("        <published_topics>"));
  Serial.print(F("          <topic name=\"AMM_Render_Modification\" type=\"ARM_R_IV_CATH\"/>"));
  Serial.print(F("        </published_topics>"));
  //Serial.print(F("        <configuration_data>"));
  //Serial.print(F("          <data name=\"sound_alarm\" type=\"boolean\"/>"));
  //Serial.print(F("        </configuration_data>"));
  Serial.print(F("      </capability>"));

  Serial.print(F("      <capability name=\"pulse_palpation\">"));
  Serial.print(F("        <published_topics>"));
  Serial.print(F("          <topic name=\"AMM_Render_Modification\" type=\"ARM_R_IV_PULSE_PALP\"/>"));
  Serial.print(F("        </published_topics>"));
  Serial.print(F("      </capability>"));

  Serial.print(F("      <capability name=\"pulse\">"));
  Serial.print(F("        <subscribed_topics>"));
  Serial.print(F("          <topic name=\"AMM_Node_Data\" nodepath=\"Cardiovascular_HeartRate\"/>"));
  Serial.print(F("          <topic name=\"AMM_Node_Data\" nodepath=\"Cardiovascular_Arterial_Systolic_Pressure\"/>"));
  Serial.print(F("        </subscribed_topics>"));
  //Serial.print(F("        <configuration_data>"));
  //Serial.print(F("          <data name=\"pressure_data\" type=\"analog\"/>"));
  //Serial.print(F("        </configuration_data>"));
  Serial.print(F("      </capability>"));
  
  Serial.print(F("    </capabilities>"));
  Serial.print(F("  </module>"));
  Serial.println(F("</AMMModuleConfiguration>"));
  Serial.flush();
}

void ivPlacementDetected() {
  if (iv_detection_capability) {
    Serial.println(F("[AMM_Render_Modification]type=ARM_R_IV_CATH;location=RightArm"));
  }
}

void pulsePalpationDetected() {
  if (pulse_palpation_capability) {
  Serial.println(F("[AMM_Render_Modification]type=ARM_R_IV_PULSE_PALP;location=RightArm"));
  }
}
