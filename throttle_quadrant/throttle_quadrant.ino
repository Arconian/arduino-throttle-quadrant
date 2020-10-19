/////////////////////
//// Version 1.4 ////
/////////////////////

#include <Joystick.h>

//// Debouncing
#define DEBOUNCE_ITERATIONS 5

//// Shift reg data
#define NUMBER_OF_SHIFT_CHIPS   4
#define DATA_WIDTH   NUMBER_OF_SHIFT_CHIPS * 8
#define PULSE_WIDTH_USEC   5
#define BYTES_VAL_T unsigned long

//// Encoder pause
#define ENCODER_BUTTON_PRESS_TIME 27
#define SWITCH_BUTTON_PRESS_TIME 50

//// Pins
#define PLOAD_PIN 13  // Connects to Parallel load pin the 165
#define CLOCK_ENABLE_PIN 10  // Connects to Clock Enable pin the 165
#define DATA_PIN 11  // Connects to the Q7 pin the 165
#define CLOCK_PIN 12  // Connects to the Clock pin the 165

#define LED1_PIN 16
#define LED2_PIN 15

#define POT1_PIN A0
#define POT2_PIN A1
#define POT3_PIN A2
#define POT4_PIN A3
#define POT5_PIN A4
#define POT6_PIN A5

#define ENC1_PIN_A 0
#define ENC1_PIN_B 4
#define ENC2_PIN_A 1
#define ENC2_PIN_B 5
#define ENC3_PIN_A 2
#define ENC3_PIN_B 6
#define ENC4_PIN_A 3
#define ENC4_PIN_B 8
#define ENC5_PIN_A 7
#define ENC5_PIN_B 9
//External Interrupts:
//3 (interrupt 0),
//2 (interrupt 1),
//0 (interrupt 2),
//1 (interrupt 3)
//7 (interrupt 4)

//// Modes
#define BUTTON_MODE_1 1
#define BUTTON_MODE_2 2
#define BUTTON_MODE_BOTH 3

#define ENCODER_BUTTON_MODE_JOY_1_ONLY 0
#define ENCODER_BUTTON_MODE_SEPARATE 1

#define POT_MODE_ALL 0
#define POT_MODE_3 1

#define BUTTON_MODE_BIT 22
#define POT_MODE_BIT 21

BYTES_VAL_T pinValues;
BYTES_VAL_T oldPinValues;

Joystick_ Joystick1 = Joystick_(0x04, JOYSTICK_TYPE_JOYSTICK, 31, 0, true, true, true, true, true, true, false, false, false, false, false);
Joystick_ Joystick2 = Joystick_(0x05, JOYSTICK_TYPE_JOYSTICK, 20, 0, false, false, false, false, false, false, false, false, false, false, false);

short currentButtonMode = BUTTON_MODE_1;
short currentPotMode = POT_MODE_ALL;

struct Button {
  int bitNum;
  int logicalButtonNum;
  short buttonMode;
};

struct TwoWaySwitchButton {
  int bitNum;
  int logicalButtonNum;
  short buttonMode;
  bool active;
  unsigned int activation;
  bool lastState;
};

#define JOY1_BUTTONS_NUM 19

Button joy1Buttons[JOY1_BUTTONS_NUM] = {
  {2, 10, BUTTON_MODE_BOTH},
  {3, 11, BUTTON_MODE_BOTH},
  {0, 12, BUTTON_MODE_BOTH},
  {1, 13, BUTTON_MODE_BOTH},
  {4, 14, BUTTON_MODE_BOTH},
  {5, 15, BUTTON_MODE_BOTH},
  {6, 16, BUTTON_MODE_BOTH},
  {7, 17, BUTTON_MODE_1},
  {16, 18, BUTTON_MODE_1},
  {12, 19, BUTTON_MODE_1},
  {13, 20, BUTTON_MODE_1},
  {14, 21, BUTTON_MODE_BOTH},
  {15, 22, BUTTON_MODE_BOTH},
  //  {25, 23, BUTTON_MODE_BOTH},
  //  {24, 24, BUTTON_MODE_BOTH},
  {27, 25, BUTTON_MODE_1},
  {26, 26, BUTTON_MODE_1},
  {19, 27, BUTTON_MODE_1},
  {18, 28, BUTTON_MODE_1},
  {23, 29, BUTTON_MODE_BOTH},
  {17, 30, BUTTON_MODE_BOTH},
};

#define JOY1_SWITCH_BUTTONS_NUM 2

TwoWaySwitchButton joy1SwitchButtons[JOY1_SWITCH_BUTTONS_NUM] = {
  {25, 23, BUTTON_MODE_BOTH},
  {24, 24, BUTTON_MODE_BOTH},
};

#define JOY2_BUTTONS_NUM 8

Button joy2Buttons[JOY2_BUTTONS_NUM] = {
  //  {10, 8, BUTTON_MODE_BOTH},
  //  {11, 9, BUTTON_MODE_BOTH},
  //  {8, 10, BUTTON_MODE_BOTH},
  //  {9, 11, BUTTON_MODE_BOTH},
  {7, 12, BUTTON_MODE_2},
  {16, 13, BUTTON_MODE_2},
  {12, 14, BUTTON_MODE_2},
  {13, 15, BUTTON_MODE_2},
  {27, 16, BUTTON_MODE_2},
  {26, 17, BUTTON_MODE_2},
  {19, 18, BUTTON_MODE_2},
  {18, 19, BUTTON_MODE_2},
};

#define JOY2_SWITCH_BUTTONS_NUM 4

TwoWaySwitchButton joy2SwitchButtons[JOY2_SWITCH_BUTTONS_NUM] = {
  {10, 8, BUTTON_MODE_BOTH},
  {11, 9, BUTTON_MODE_BOTH},
  {8, 10, BUTTON_MODE_BOTH},
  {9, 11, BUTTON_MODE_BOTH},
};

volatile boolean state0;

struct EncoderButton {
  int joy1Button;
  int joy2Button;
  short buttonMode;
  volatile bool joy1Active;
  volatile unsigned int joy1Activation;
  volatile bool joy2Active;
  volatile unsigned int joy2Activation;
};

struct Encoder {
  int pinA;
  int pinB;
  int leftButtonIndex;
  int rightButtonIndex;
  volatile int encCounter;
  volatile boolean lastState;
  volatile int lastCounter;
};

EncoderButton encoderButtons[10] = {
  {0, 0, ENCODER_BUTTON_MODE_JOY_1_ONLY},
  {1, 0, ENCODER_BUTTON_MODE_JOY_1_ONLY},
  {2, 0, ENCODER_BUTTON_MODE_SEPARATE},
  {3, 1, ENCODER_BUTTON_MODE_SEPARATE},
  {4, 2, ENCODER_BUTTON_MODE_SEPARATE},
  {5, 3, ENCODER_BUTTON_MODE_SEPARATE},
  {6, 4, ENCODER_BUTTON_MODE_SEPARATE},
  {7, 5, ENCODER_BUTTON_MODE_SEPARATE},
  {8, 6, ENCODER_BUTTON_MODE_SEPARATE},
  {9, 7, ENCODER_BUTTON_MODE_SEPARATE},
};

Encoder encoders[5] = {
  {ENC1_PIN_A, ENC1_PIN_B, 0, 1},
  {ENC2_PIN_A, ENC2_PIN_B, 2, 3},
  {ENC3_PIN_A, ENC3_PIN_B, 4, 5},
  {ENC4_PIN_A, ENC4_PIN_B, 6, 7},
  {ENC5_PIN_A, ENC5_PIN_B, 8, 9},
};

int pot1value;
int pot2value;
int pot3value;
int pot4value;
int pot5value;
int pot6value;
int axisXValue;
int axisYValue;
int axisZValue;
int axisRxValue;
int axisRyValue;
int axisRzValue;
int oldAxisXValue = -1;
int oldAxisYValue = -1;
int oldAxisZValue = -1;
int oldAxisRxValue = -1;
int oldAxisRyValue = -1;
int oldAxisRzValue = -1;

int axisDebounceCounter = 0;
int axisXLastValues[DEBOUNCE_ITERATIONS];
int axisYLastValues[DEBOUNCE_ITERATIONS];
int axisZLastValues[DEBOUNCE_ITERATIONS];
int axisRxLastValues[DEBOUNCE_ITERATIONS];
int axisRyLastValues[DEBOUNCE_ITERATIONS];
int axisRzLastValues[DEBOUNCE_ITERATIONS];

BYTES_VAL_T readShiftRegs()
{
  long bitVal;
  BYTES_VAL_T bytesVal = 0;

  /* Trigger a parallel Load to latch the state of the data lines,
  */
  digitalWrite(CLOCK_ENABLE_PIN, HIGH);
  digitalWrite(PLOAD_PIN, LOW);
  delayMicroseconds(PULSE_WIDTH_USEC);
  digitalWrite(PLOAD_PIN, HIGH);
  digitalWrite(CLOCK_ENABLE_PIN, LOW);

  /* Loop to read each bit value from the serial out line
     of the SN74HC165N.
  */
  for (int i = 0; i < DATA_WIDTH; i++)
  {
    bitVal = digitalRead(DATA_PIN);

    /* Set the corresponding bit in bytesVal.
    */
    bytesVal |= (bitVal << ((DATA_WIDTH - 1) - i));

    /* Pulse the Clock (rising edge shifts the next bit).
    */
    digitalWrite(CLOCK_PIN, HIGH);
    delayMicroseconds(PULSE_WIDTH_USEC);
    digitalWrite(CLOCK_PIN, LOW);
  }

  return (bytesVal);
}


void displayPinValues()
{
  return;
  Serial.print("Pin States:\r\n");

  for (int i = 0; i < DATA_WIDTH; i++)
  {
    Serial.print("  Pin-");
    Serial.print(i);
    Serial.print(": ");

    if ((pinValues >> i) & 1)
      Serial.print("HIGH");
    else
      Serial.print("LOW");

    Serial.print("\r\n");
  }

  Serial.print("\r\n");
}

void setup()
{
  Serial.begin(9600);

  pinMode(ENC1_PIN_A, INPUT_PULLUP);
  pinMode(ENC1_PIN_B, INPUT_PULLUP);
  pinMode(ENC2_PIN_A, INPUT_PULLUP);
  pinMode(ENC2_PIN_B, INPUT_PULLUP);
  pinMode(ENC3_PIN_A, INPUT_PULLUP);
  pinMode(ENC3_PIN_B, INPUT_PULLUP);
  pinMode(ENC4_PIN_A, INPUT_PULLUP);
  pinMode(ENC4_PIN_B, INPUT_PULLUP);
  pinMode(ENC5_PIN_A, INPUT_PULLUP);
  pinMode(ENC5_PIN_B, INPUT_PULLUP);

  pinMode(POT1_PIN, INPUT);
  pinMode(POT2_PIN, INPUT);
  pinMode(POT3_PIN, INPUT);
  pinMode(POT4_PIN, INPUT);
  pinMode(POT5_PIN, INPUT);
  pinMode(POT6_PIN, INPUT);

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  pinMode(PLOAD_PIN, OUTPUT);
  pinMode(CLOCK_ENABLE_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, INPUT);

  digitalWrite(CLOCK_PIN, LOW);
  digitalWrite(PLOAD_PIN, HIGH);

  /* Read in and display the pin states at startup.
  */
  pinValues = readShiftRegs();
  displayPinValues();
  oldPinValues = pinValues;

  Joystick1.begin();
  Joystick2.begin();

  processButtonMode();
  processPotMode();
  processLeds();
  processButtons();

  attachInterrupt(2, processEncoder1, CHANGE);
  attachInterrupt(3, processEncoder2, CHANGE);
  attachInterrupt(1, processEncoder3, CHANGE);
  attachInterrupt(0, processEncoder4, CHANGE);
  attachInterrupt(4, processEncoder5, CHANGE);
}

void loop()
{
  /* Read the state of all zones.
  */
  pinValues = readShiftRegs();

  /* If there was a change in state, display which ones changed.
  */
  if (pinValues != oldPinValues)
  {
    displayPinValues();

    processButtonMode();
    processPotMode();
    processLeds();
    processButtons();

    oldPinValues = pinValues;
  }

  //  processButtonMode();
  //  processPotMode();
  //  processLeds();
  // processButtons();
  processPots();
  deactivateEncoderButtons();
  deactivateEncoderSwitchButtons();

  // Joystick1.sendState();
  // Joystick2.sendState();
}

void processButtons() {
  for (int i = 0; i < JOY1_BUTTONS_NUM; i++) {
    int state = ((currentButtonMode & joy1Buttons[i].buttonMode) == currentButtonMode) && ((pinValues >> joy1Buttons[i].bitNum) & 1);
    Joystick1.setButton(joy1Buttons[i].logicalButtonNum, state);
  }
  for (int i = 0; i < JOY2_BUTTONS_NUM; i++) {
    int state = ((currentButtonMode & joy2Buttons[i].buttonMode) == currentButtonMode) && ((pinValues >> joy2Buttons[i].bitNum) & 1);
    Joystick2.setButton(joy2Buttons[i].logicalButtonNum, state);
  }

  for (int i = 0; i < JOY1_SWITCH_BUTTONS_NUM; i++) {
    int state = ((currentButtonMode & joy1SwitchButtons[i].buttonMode) == currentButtonMode) && ((pinValues >> joy1SwitchButtons[i].bitNum) & 1);
    if (state) {
      if (!joy1SwitchButtons[i].lastState) {
        Joystick1.setButton(joy1SwitchButtons[i].logicalButtonNum, state);
        joy1SwitchButtons[i].active = true;
        joy1SwitchButtons[i].activation = (unsigned int) millis();
        joy1SwitchButtons[i].lastState = true;
      }
    } else {
      joy1SwitchButtons[i].lastState = false;
    }
  }
  for (int i = 0; i < JOY2_SWITCH_BUTTONS_NUM; i++) {
    int state = ((currentButtonMode & joy2SwitchButtons[i].buttonMode) == currentButtonMode) && ((pinValues >> joy2SwitchButtons[i].bitNum) & 1);
    if (state) {
      if (!joy2SwitchButtons[i].lastState) {
        Joystick2.setButton(joy2SwitchButtons[i].logicalButtonNum, state);
        joy2SwitchButtons[i].active = true;
        joy2SwitchButtons[i].activation = (unsigned int) millis();
        joy2SwitchButtons[i].lastState = true;
      }
    } else {
      joy2SwitchButtons[i].lastState = false;
    }
  }
}

void processButtonMode() {
  if ((pinValues >> BUTTON_MODE_BIT) & 1) {
    currentButtonMode = BUTTON_MODE_2;
  } else {
    currentButtonMode = BUTTON_MODE_1;
  }
}

void processPotMode() {
  if ((pinValues >> POT_MODE_BIT) & 1) {
    currentPotMode = POT_MODE_3;
  } else {
    currentPotMode = POT_MODE_ALL;
  }
}

void processLeds() {
  if (currentButtonMode == BUTTON_MODE_1) {
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, LOW);
  } else {
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, HIGH);
  }
}

void processPots() {
  pot1value = analogRead(POT1_PIN);
  pot2value = analogRead(POT2_PIN);
  pot3value = analogRead(POT3_PIN);
  pot4value = analogRead(POT4_PIN);
  pot5value = analogRead(POT5_PIN);
  pot6value =  analogRead(POT6_PIN);

  axisXValue = pot1value;
  axisYValue = currentPotMode == POT_MODE_ALL ? pot2value : pot4value;
  axisZValue = currentPotMode == POT_MODE_ALL ? pot3value : pot2value;
  axisRxValue = currentPotMode == POT_MODE_ALL ? pot4value : pot5value;
  axisRyValue = currentPotMode == POT_MODE_ALL ? pot5value : pot3value;
  axisRzValue = currentPotMode == POT_MODE_ALL ? pot6value : pot6value;


  axisXLastValues[axisDebounceCounter] = axisXValue;
  axisYLastValues[axisDebounceCounter] = axisYValue;
  axisZLastValues[axisDebounceCounter] = axisZValue;
  axisRxLastValues[axisDebounceCounter] = axisRxValue ;
  axisRyLastValues[axisDebounceCounter] = axisRyValue;
  axisRzLastValues[axisDebounceCounter] = axisRzValue ;

  axisDebounceCounter++;

  if (axisDebounceCounter == DEBOUNCE_ITERATIONS) {
    axisXValue = getAverage(axisXLastValues, DEBOUNCE_ITERATIONS);
    if (axisXValue != oldAxisXValue) {
      Joystick1.setXAxis(axisXValue);

      oldAxisXValue = axisXValue;

      //Serial.println(axisXValue);
    }
    axisYValue = getAverage(axisYLastValues, DEBOUNCE_ITERATIONS);
    if (axisYValue != oldAxisYValue) {
      Joystick1.setYAxis(axisYValue);

      oldAxisYValue = axisYValue;
    }
    axisZValue = getAverage(axisZLastValues, DEBOUNCE_ITERATIONS);
    if (axisZValue != oldAxisZValue) {
      Joystick1.setZAxis(axisZValue);

      oldAxisZValue = axisZValue;
    }
    axisRxValue = getAverage(axisRxLastValues, DEBOUNCE_ITERATIONS);
    if (axisRxValue != oldAxisRxValue) {
      Joystick1.setRxAxis(axisRxValue);

      oldAxisRxValue = axisRxValue;
    }
    axisRyValue = getAverage(axisRyLastValues, DEBOUNCE_ITERATIONS);
    if (axisRyValue != oldAxisRyValue) {
      Joystick1.setRyAxis(axisRyValue);

      oldAxisRyValue = axisRyValue;
    }
    axisRzValue = getAverage(axisRzLastValues, DEBOUNCE_ITERATIONS);
    if (axisRzValue != oldAxisRzValue) {
      Joystick1.setRzAxis(axisRzValue);

      oldAxisRzValue = axisRzValue;
    }

    axisDebounceCounter = 0;
  }
}

int getAverage(int arr[], int size) {
  int i, sum = 0;
  float avg;

  for (i = 0; i < size; ++i) {
    sum += arr[i];
  }
  avg = float(sum) / size;

  return round(avg);
}

void processEncoder1() {
  processEncoderInterrupt(0);
}
void processEncoder2() {
  processEncoderInterrupt(1);
}
void processEncoder3() {
  processEncoderInterrupt(2);
}
void processEncoder4() {
  processEncoderInterrupt(3);
}
void processEncoder5() {
  processEncoderInterrupt(4);
}

void processEncoderInterrupt(int index) {
  state0 = digitalRead(encoders[index].pinA);
  if (state0 != encoders[index].lastState) {
    encoders[index].encCounter += (digitalRead(encoders[index].pinB) != encoders[index].lastState) ? -1 : 1;
  }

  encoders[index].lastState = state0;

  int diff = encoders[index].encCounter - encoders[index].lastCounter;
  if (diff == 2) {
    encoders[index].lastCounter = encoders[index].encCounter = 0;
    activateEncoderButton(encoders[index].rightButtonIndex);
  } else if (diff == -2 ) {
    encoders[index].lastCounter = encoders[index].encCounter = 0;
    activateEncoderButton(encoders[index].leftButtonIndex);
  }
}

void activateEncoderButton(int index) {
  if (encoderButtons[index].buttonMode == ENCODER_BUTTON_MODE_JOY_1_ONLY || currentButtonMode == BUTTON_MODE_1) {
    if (!encoderButtons[index].joy1Active) {
      Joystick1.setButton(encoderButtons[index].joy1Button, 1);
      encoderButtons[index].joy1Active = true;
      encoderButtons[index].joy1Activation = (unsigned int) millis();
    }
  } else if (currentButtonMode == BUTTON_MODE_2) {
    if (!encoderButtons[index].joy2Active) {
      Joystick2.setButton(encoderButtons[index].joy2Button, 1);
      encoderButtons[index].joy2Active = true;
      encoderButtons[index].joy2Activation = (unsigned int) millis();
    }
  }
}

void deactivateEncoderButtons() {
  for (int i = 0; i < 10; i++) {
    if (encoderButtons[i].joy1Active && (unsigned int) millis() - encoderButtons[i].joy1Activation >= ENCODER_BUTTON_PRESS_TIME) {
      Joystick1.setButton(encoderButtons[i].joy1Button, 0);
      encoderButtons[i].joy1Active = false;
    }
    if (encoderButtons[i].joy2Active && (unsigned int) millis() - encoderButtons[i].joy2Activation >= ENCODER_BUTTON_PRESS_TIME) {
      Joystick2.setButton(encoderButtons[i].joy2Button, 0);
      encoderButtons[i].joy2Active = false;
    }
  }
}

void deactivateEncoderSwitchButtons() {
  for (int i = 0; i < JOY1_SWITCH_BUTTONS_NUM; i++) {
    if (joy1SwitchButtons[i].active && (unsigned int) millis() - joy1SwitchButtons[i].activation >= SWITCH_BUTTON_PRESS_TIME) {
      Joystick1.setButton(joy1SwitchButtons[i].logicalButtonNum, 0);
      joy1SwitchButtons[i].active = false;
    }
  }
  for (int i = 0; i < JOY2_SWITCH_BUTTONS_NUM; i++) {
    if (joy2SwitchButtons[i].active && (unsigned int) millis() - joy2SwitchButtons[i].activation >= SWITCH_BUTTON_PRESS_TIME) {
      Joystick2.setButton(joy2SwitchButtons[i].logicalButtonNum, 0);
      joy2SwitchButtons[i].active = false;
    }
  }
}
