/////////////////////
//// Version 2.0-alpha ////
/////////////////////

#include <Joystick.h>
#include <MIDIUSB.h>

//// Debouncing
#define DEBOUNCE_ITERATIONS 3

//// Shift reg data
#define NUMBER_OF_SHIFT_CHIPS 4
#define DATA_WIDTH NUMBER_OF_SHIFT_CHIPS * 8
#define PULSE_WIDTH_USEC 5
#define BYTES_VAL_T unsigned long

#define ENCODERS_NUM 5
#define TWO_POSITION_SWITCHES_NUM 3

//// Encoder pause
#define ENCODER_BUTTON_PRESS_TIME 27
#define SWITCH_BUTTON_PRESS_TIME 50

#define ENCODER_PUSH_BUTTON_TIME_FOR_OUTPUT_SWITCH 350

//// Pins
#define PLOAD_PIN 13         // Connects to Parallel load pin the 165
#define CLOCK_ENABLE_PIN 10  // Connects to Clock Enable pin the 165
#define DATA_PIN 11          // Connects to the Q7 pin the 165
#define CLOCK_PIN 12         // Connects to the Clock pin the 165

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

#define ROTATION_LEFT 0
#define ROTATION_RIGHT 1

#define ENCODER_BUTTON_MODE_JOY_1_ONLY 0
#define ENCODER_BUTTON_MODE_SEPARATE 1

#define POT_MODE_ALL 0
#define POT_MODE_3 1

#define BUTTON_MODE_BIT 22
#define POT_MODE_BIT 21

#define ENCODERS_TO_JOY
#define ENCODERS_TO_MIDI

const short JOY_0 = 0;
const short JOY_1 = 1;

BYTES_VAL_T pinValues;
BYTES_VAL_T oldPinValues;

volatile boolean pinAState;

short currentButtonMode = BUTTON_MODE_1;
short currentPotMode = POT_MODE_ALL;

struct EncoderPushButton {
  short bitNum;
  short encoderIndex;
  bool active;
  unsigned int activation;
};

EncoderPushButton encoderPushButtons[ENCODERS_NUM] = {
  { 6, 0 },
  { 7, 1 },
  { 16, 2 },
  { 27, 3 },
  { 26, 4 }
};

struct EncoderPushButtonEvent {
  short pushButtonIndex;
  short buttonMode;
  short joyIndex;
  short buttonNumber;
  byte midiChannel;
  byte midiPitch;
  bool active;
  unsigned int activation;
};

#define JOY0_ENCODER_PUSH_BUTTONS_NUM 9
#define JOY1_ENCODER_PUSH_BUTTONS_NUM 0
EncoderPushButtonEvent encoderPushButtonsEvents[JOY0_ENCODER_PUSH_BUTTONS_NUM + JOY1_ENCODER_PUSH_BUTTONS_NUM] = {
  { 0, BUTTON_MODE_BOTH, JOY_0, 24, 10, 24 },
  { 1, BUTTON_MODE_1, JOY_0, 25, 10, 25 },
  { 2, BUTTON_MODE_1, JOY_0, 26, 10, 26 },
  { 3, BUTTON_MODE_1, JOY_0, 27, 10, 27 },
  { 4, BUTTON_MODE_1, JOY_0, 28, 10, 28 },
  { 1, BUTTON_MODE_2, JOY_0, 29, 10, 29 },
  { 2, BUTTON_MODE_2, JOY_0, 30, 10, 30 },
  { 3, BUTTON_MODE_2, JOY_0, 31, 10, 31 },
  { 4, BUTTON_MODE_2, JOY_0, 32, 10, 32 },
};

struct Button {
  short bitNum;
  short buttonMode;
  short joyIndex;
  short buttonNumber;
  byte midiChannel;
  byte midiPitch;
  bool previousState;
};

#define JOY0_BUTTONS_NUM 18
#define JOY1_BUTTONS_NUM 0
Button buttonEvents[JOY0_BUTTONS_NUM + JOY1_BUTTONS_NUM] = {
  { 2, BUTTON_MODE_BOTH, JOY_0, 6, 10, 6 },
  { 3, BUTTON_MODE_BOTH, JOY_0, 7, 10, 7 },
  { 0, BUTTON_MODE_BOTH, JOY_0, 8, 10, 8 },
  { 1, BUTTON_MODE_BOTH, JOY_0, 9, 10, 9 },
  { 4, BUTTON_MODE_BOTH, JOY_0, 10, 10, 10 },
  { 5, BUTTON_MODE_BOTH, JOY_0, 11, 10, 11 },
  { 12, BUTTON_MODE_1, JOY_0, 12, 10, 12 },
  { 14, BUTTON_MODE_BOTH, JOY_0, 13, 10, 13 },
  { 13, BUTTON_MODE_1, JOY_0, 14, 10, 14 },
  { 15, BUTTON_MODE_BOTH, JOY_0, 15, 10, 15 },
  { 19, BUTTON_MODE_1, JOY_0, 16, 10, 16 },
  { 18, BUTTON_MODE_1, JOY_0, 17, 10, 17 },
  { 23, BUTTON_MODE_BOTH, JOY_0, 18, 10, 18 },
  { 17, BUTTON_MODE_BOTH, JOY_0, 19, 10, 19 },
  { 12, BUTTON_MODE_2, JOY_0, 20, 10, 20 },
  { 13, BUTTON_MODE_2, JOY_0, 21, 10, 21 },
  { 19, BUTTON_MODE_2, JOY_0, 22, 10, 22 },
  { 18, BUTTON_MODE_2, JOY_0, 23, 10, 23 },
};

struct TwoWaySwitchButton {
  int bitNum;
  short buttonMode;
  short joyIndex;
  short buttonNumber;
  byte midiChannel;
  byte midiPitch;
  bool active;
  unsigned int activation;
  bool lastState;
};

#define JOY0_SWITCH_BUTTONS_NUM 6
#define JOY1_SWITCH_BUTTONS_NUM 0
TwoWaySwitchButton twoWaySwitchEvents[JOY0_SWITCH_BUTTONS_NUM + JOY1_SWITCH_BUTTONS_NUM] = {
  { 25, BUTTON_MODE_BOTH, JOY_0, 0, 10, 0 },
  { 24, BUTTON_MODE_BOTH, JOY_0, 1, 10, 1 },
  { 10, BUTTON_MODE_BOTH, JOY_0, 2, 10, 2 },
  { 11, BUTTON_MODE_BOTH, JOY_0, 3, 10, 3 },
  { 8, BUTTON_MODE_BOTH, JOY_0, 4, 10, 4 },
  { 9, BUTTON_MODE_BOTH, JOY_0, 5, 10, 5 },
};

struct Encoder {
  short pinA;
  short pinB;
  volatile int encCounter;
  volatile boolean lastState;
  volatile int lastCounter;
};

Encoder encoders[ENCODERS_NUM] = {
  { ENC1_PIN_A, ENC1_PIN_B },
  { ENC2_PIN_A, ENC2_PIN_B },
  { ENC3_PIN_A, ENC3_PIN_B },
  { ENC4_PIN_A, ENC4_PIN_B },
  { ENC5_PIN_A, ENC5_PIN_B },
};

struct EncoderRotationToJoy {
  short encoderIndex;
  short direction;
  bool pushButtonValue;
  short buttonMode;
  short joyIndex;
  short buttonNumber;
  volatile bool active;
  volatile unsigned int activation;
};

#define ENCODER_ROTATION_JOY_0_BUTTONS_NUM 0
#define ENCODER_ROTATION_JOY_1_BUTTONS_NUM 36
EncoderRotationToJoy encoderRotationToJoyButtons[ENCODER_ROTATION_JOY_0_BUTTONS_NUM + ENCODER_ROTATION_JOY_1_BUTTONS_NUM] = {
  { 0, ROTATION_LEFT, false, BUTTON_MODE_BOTH, JOY_1, 0 },
  { 0, ROTATION_RIGHT, false, BUTTON_MODE_BOTH, JOY_1, 1 },
  { 0, ROTATION_LEFT, true, BUTTON_MODE_BOTH, JOY_1, 2 },
  { 0, ROTATION_RIGHT, true, BUTTON_MODE_BOTH, JOY_1, 3 },
  { 1, ROTATION_LEFT, false, BUTTON_MODE_1, JOY_1, 4 },
  { 1, ROTATION_RIGHT, false, BUTTON_MODE_1, JOY_1, 5 },
  { 1, ROTATION_LEFT, true, BUTTON_MODE_1, JOY_1, 6 },
  { 1, ROTATION_RIGHT, true, BUTTON_MODE_1, JOY_1, 7 },
  { 2, ROTATION_LEFT, false, BUTTON_MODE_1, JOY_1, 8 },
  { 2, ROTATION_RIGHT, false, BUTTON_MODE_1, JOY_1, 9 },
  { 2, ROTATION_LEFT, true, BUTTON_MODE_1, JOY_1, 10 },
  { 2, ROTATION_RIGHT, true, BUTTON_MODE_1, JOY_1, 11 },
  { 3, ROTATION_LEFT, false, BUTTON_MODE_1, JOY_1, 12 },
  { 3, ROTATION_RIGHT, false, BUTTON_MODE_1, JOY_1, 13 },
  { 3, ROTATION_LEFT, true, BUTTON_MODE_1, JOY_1, 14 },
  { 3, ROTATION_RIGHT, true, BUTTON_MODE_1, JOY_1, 15 },
  { 4, ROTATION_LEFT, false, BUTTON_MODE_1, JOY_1, 16 },
  { 4, ROTATION_RIGHT, false, BUTTON_MODE_1, JOY_1, 17 },
  { 4, ROTATION_LEFT, true, BUTTON_MODE_1, JOY_1, 18 },
  { 4, ROTATION_RIGHT, true, BUTTON_MODE_1, JOY_1, 19 },
  { 1, ROTATION_LEFT, false, BUTTON_MODE_2, JOY_1, 20 },
  { 1, ROTATION_RIGHT, false, BUTTON_MODE_2, JOY_1, 21 },
  { 1, ROTATION_LEFT, true, BUTTON_MODE_2, JOY_1, 22 },
  { 1, ROTATION_RIGHT, true, BUTTON_MODE_2, JOY_1, 23 },
  { 2, ROTATION_LEFT, false, BUTTON_MODE_2, JOY_1, 24 },
  { 2, ROTATION_RIGHT, false, BUTTON_MODE_2, JOY_1, 25 },
  { 2, ROTATION_LEFT, true, BUTTON_MODE_2, JOY_1, 26 },
  { 2, ROTATION_RIGHT, true, BUTTON_MODE_2, JOY_1, 27 },
  { 3, ROTATION_LEFT, false, BUTTON_MODE_2, JOY_1, 28 },
  { 3, ROTATION_RIGHT, false, BUTTON_MODE_2, JOY_1, 29 },
  { 3, ROTATION_LEFT, true, BUTTON_MODE_2, JOY_1, 30 },
  { 3, ROTATION_RIGHT, true, BUTTON_MODE_2, JOY_1, 31 },
  { 4, ROTATION_LEFT, false, BUTTON_MODE_2, JOY_1, 32 },
  { 4, ROTATION_RIGHT, false, BUTTON_MODE_2, JOY_1, 33 },
  { 4, ROTATION_LEFT, true, BUTTON_MODE_2, JOY_1, 34 },
  { 4, ROTATION_RIGHT, true, BUTTON_MODE_2, JOY_1, 35 },
};

struct EncoderRotationToMidi {
  short encoderIndex;
  bool pushButtonValue;
  short buttonMode;
  short channel;
  short control;
};

#define ENCODER_ROTATION_MIDI_COMMANDS_NUM 18
EncoderRotationToMidi encoderRotationToMidiCommands[ENCODER_ROTATION_MIDI_COMMANDS_NUM] = {
  { 0, false, BUTTON_MODE_BOTH, 10, 0 },
  { 0, true, BUTTON_MODE_BOTH, 10, 1 },
  { 1, false, BUTTON_MODE_1, 10, 2 },
  { 1, true, BUTTON_MODE_1, 10, 3 },
  { 2, false, BUTTON_MODE_1, 10, 4 },
  { 2, true, BUTTON_MODE_1, 10, 5 },
  { 3, false, BUTTON_MODE_1, 10, 6 },
  { 3, true, BUTTON_MODE_1, 10, 7 },
  { 4, false, BUTTON_MODE_1, 10, 8 },
  { 4, true, BUTTON_MODE_1, 10, 9 },
  { 1, false, BUTTON_MODE_2, 10, 10 },
  { 1, true, BUTTON_MODE_2, 10, 11 },
  { 2, false, BUTTON_MODE_2, 10, 12 },
  { 2, true, BUTTON_MODE_2, 10, 13 },
  { 3, false, BUTTON_MODE_2, 10, 14 },
  { 3, true, BUTTON_MODE_2, 10, 15 },
  { 4, false, BUTTON_MODE_2, 10, 16 },
  { 4, true, BUTTON_MODE_2, 10, 17 },
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

Joystick_ Joysticks[2]{
  Joystick_(0x04, JOYSTICK_TYPE_JOYSTICK, JOY0_BUTTONS_NUM + JOY0_SWITCH_BUTTONS_NUM + JOY0_ENCODER_PUSH_BUTTONS_NUM + ENCODER_ROTATION_JOY_0_BUTTONS_NUM, 0, true, true, true, true, true, true, false, false, false, false, false),
  Joystick_(0x05, JOYSTICK_TYPE_JOYSTICK, JOY1_BUTTONS_NUM + JOY1_SWITCH_BUTTONS_NUM + JOY1_ENCODER_PUSH_BUTTONS_NUM + ENCODER_ROTATION_JOY_1_BUTTONS_NUM, 0, false, false, false, false, false, false, false, false, false, false, false),
};

BYTES_VAL_T readShiftRegs() {
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
  for (int i = 0; i < DATA_WIDTH; i++) {
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


void displayPinValues() {
  return;
  Serial.print("Pin States:\r\n");

  for (int i = 0; i < DATA_WIDTH; i++) {
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

void setup() {
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

  Joysticks[0].begin();
  Joysticks[1].begin();

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

void loop() {
  /* Read the state of all zones.
  */
  pinValues = readShiftRegs();

  /* If there was a change in state, display which ones changed.
  */
  if (pinValues != oldPinValues) {
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
  deactivateSwitchButtons();
  deactivateEncoderPushButtons();

  MidiUSB.flush();

  // Joystick1.sendState();
  // Joystick2.sendState();
}

void processButtons() {
  for (int i = 0; i < JOY0_BUTTONS_NUM + JOY1_BUTTONS_NUM; i++) {
    bool state = ((currentButtonMode & buttonEvents[i].buttonMode) == currentButtonMode) && ((pinValues >> buttonEvents[i].bitNum) & 1);

    if (state != buttonEvents[i].previousState) {
      Joysticks[buttonEvents[i].joyIndex].setButton(buttonEvents[i].buttonNumber, state);

      if (state) {
        noteOn(buttonEvents[i].midiChannel, buttonEvents[i].midiPitch, 127);
      } else {
        noteOff(buttonEvents[i].midiChannel, buttonEvents[i].midiPitch, 0);
      }

      buttonEvents[i].previousState = state;
    }
  }

  for (int i = 0; i < JOY0_SWITCH_BUTTONS_NUM + JOY1_SWITCH_BUTTONS_NUM; i++) {
    bool state = ((currentButtonMode & twoWaySwitchEvents[i].buttonMode) == currentButtonMode) && ((pinValues >> twoWaySwitchEvents[i].bitNum) & 1);
    if (state) {
      if (!twoWaySwitchEvents[i].lastState) {
        Joysticks[twoWaySwitchEvents[i].joyIndex].setButton(twoWaySwitchEvents[i].buttonNumber, state);

        noteOn(twoWaySwitchEvents[i].midiChannel, twoWaySwitchEvents[i].midiPitch, 127);

        twoWaySwitchEvents[i].active = true;
        twoWaySwitchEvents[i].activation = (unsigned int)millis();
        twoWaySwitchEvents[i].lastState = true;
      }
    } else {
      twoWaySwitchEvents[i].lastState = false;
    }
  }

  for (short i = 0; i < ENCODERS_NUM; i++) {
    bool isActive = (pinValues >> encoderPushButtons[i].bitNum) & 1;
    if (isActive) {
      if (!encoderPushButtons[i].active) {
        encoderPushButtons[i].active = true;
        encoderPushButtons[i].activation = (unsigned int)millis();
      }
    } else {
      if (encoderPushButtons[i].active) {
        if ((unsigned int)millis() - encoderPushButtons[i].activation <= ENCODER_PUSH_BUTTON_TIME_FOR_OUTPUT_SWITCH) {
          activateEncoderPushButton(i);
        }

        encoderPushButtons[i].active = false;
      }
    }
  }
}

void activateEncoderPushButton(short pushButtonIndex) {
  for (short i = 0; i < JOY0_ENCODER_PUSH_BUTTONS_NUM + JOY1_ENCODER_PUSH_BUTTONS_NUM; i++) {
    if (encoderPushButtonsEvents[i].pushButtonIndex == pushButtonIndex && (currentButtonMode & encoderPushButtonsEvents[i].buttonMode) == currentButtonMode) {
      // Joy
      Joysticks[encoderPushButtonsEvents[i].joyIndex].setButton(encoderPushButtonsEvents[i].buttonNumber, true);

      //Midi
      noteOn(encoderPushButtonsEvents[i].midiChannel, encoderPushButtonsEvents[i].midiPitch, 127);

      encoderPushButtonsEvents[i].active = true;
      encoderPushButtonsEvents[i].activation = (unsigned int)millis();
    }
  }

  // midi
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
  pot6value = analogRead(POT6_PIN);

  axisXValue = pot1value;
  axisYValue = currentPotMode == POT_MODE_ALL ? pot2value : pot4value;
  axisZValue = currentPotMode == POT_MODE_ALL ? pot3value : pot2value;
  axisRxValue = currentPotMode == POT_MODE_ALL ? pot4value : pot5value;
  axisRyValue = currentPotMode == POT_MODE_ALL ? pot5value : pot3value;
  axisRzValue = currentPotMode == POT_MODE_ALL ? pot6value : pot6value;


  axisXLastValues[axisDebounceCounter] = axisXValue;
  axisYLastValues[axisDebounceCounter] = axisYValue;
  axisZLastValues[axisDebounceCounter] = axisZValue;
  axisRxLastValues[axisDebounceCounter] = axisRxValue;
  axisRyLastValues[axisDebounceCounter] = axisRyValue;
  axisRzLastValues[axisDebounceCounter] = axisRzValue;

  axisDebounceCounter++;

  if (axisDebounceCounter == DEBOUNCE_ITERATIONS) {
    axisXValue = getAverage(axisXLastValues, DEBOUNCE_ITERATIONS);
    if (axisXValue != oldAxisXValue) {
      Joysticks[0].setXAxis(axisXValue);

      oldAxisXValue = axisXValue;

      //Serial.println(axisXValue);
    }
    axisYValue = getAverage(axisYLastValues, DEBOUNCE_ITERATIONS);
    if (axisYValue != oldAxisYValue) {
      Joysticks[0].setYAxis(axisYValue);

      oldAxisYValue = axisYValue;
    }
    axisZValue = getAverage(axisZLastValues, DEBOUNCE_ITERATIONS);
    if (axisZValue != oldAxisZValue) {
      Joysticks[0].setZAxis(axisZValue);

      oldAxisZValue = axisZValue;
    }
    axisRxValue = getAverage(axisRxLastValues, DEBOUNCE_ITERATIONS);
    if (axisRxValue != oldAxisRxValue) {
      Joysticks[0].setRxAxis(axisRxValue);

      oldAxisRxValue = axisRxValue;
    }
    axisRyValue = getAverage(axisRyLastValues, DEBOUNCE_ITERATIONS);
    if (axisRyValue != oldAxisRyValue) {
      Joysticks[0].setRyAxis(axisRyValue);

      oldAxisRyValue = axisRyValue;
    }
    axisRzValue = getAverage(axisRzLastValues, DEBOUNCE_ITERATIONS);
    if (axisRzValue != oldAxisRzValue) {
      Joysticks[0].setRzAxis(axisRzValue);

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
  pinAState = digitalRead(encoders[index].pinA);
  if (pinAState != encoders[index].lastState) {
    bool pinBState = digitalRead(encoders[index].pinB);
    encoders[index].encCounter += (pinBState != pinAState) ? 1 : -1;
  }

  encoders[index].lastState = pinAState;

  int diff = encoders[index].encCounter - encoders[index].lastCounter;

  if (diff == 2) {
    processEncoderRotation(index, ROTATION_RIGHT);
  } else if (diff == -2) {
    processEncoderRotation(index, ROTATION_LEFT);
  }

  if (diff == 2 || diff == -2 || pinAState == HIGH) {
    encoders[index].lastCounter = encoders[index].encCounter = 0;
  }
}

void processEncoderRotation(short encoderIndex, short rotation) {
  for (int i = 0; i < ENCODER_ROTATION_JOY_0_BUTTONS_NUM + ENCODER_ROTATION_JOY_1_BUTTONS_NUM; i++) {
    if (encoderRotationToJoyButtons[i].encoderIndex == encoderIndex && encoderRotationToJoyButtons[i].direction == rotation && encoderRotationToJoyButtons[i].pushButtonValue == encoderPushButtons[encoderIndex].active && (currentButtonMode & encoderRotationToJoyButtons[i].buttonMode) == currentButtonMode) {
      if (!encoderRotationToJoyButtons[i].active) {
        Joysticks[encoderRotationToJoyButtons[i].joyIndex].setButton(encoderRotationToJoyButtons[i].buttonNumber, 1);
        encoderRotationToJoyButtons[i].active = true;
        encoderRotationToJoyButtons[i].activation = (unsigned int)millis();
      }
    }
  }

  for (int i = 0; i < ENCODER_ROTATION_MIDI_COMMANDS_NUM; i++) {
    if (encoderRotationToMidiCommands[i].encoderIndex == encoderIndex && encoderRotationToMidiCommands[i].pushButtonValue == encoderPushButtons[encoderIndex].active && (currentButtonMode & encoderRotationToMidiCommands[i].buttonMode) == currentButtonMode) {
      controlChange(encoderRotationToMidiCommands[i].channel, encoderRotationToMidiCommands[i].control, rotation ? 65 : 63);
      MidiUSB.flush();
    }
  }
}

void deactivateEncoderButtons() {
  for (int i = 0; i < ENCODER_ROTATION_JOY_0_BUTTONS_NUM + ENCODER_ROTATION_JOY_1_BUTTONS_NUM; i++) {
    if (encoderRotationToJoyButtons[i].active && (unsigned int)millis() - encoderRotationToJoyButtons[i].activation >= ENCODER_BUTTON_PRESS_TIME) {
      Joysticks[encoderRotationToJoyButtons[i].joyIndex].setButton(encoderRotationToJoyButtons[i].buttonNumber, 0);
      encoderRotationToJoyButtons[i].active = false;
    }
  }
}

void deactivateSwitchButtons() {
  for (int i = 0; i < JOY0_SWITCH_BUTTONS_NUM + JOY1_SWITCH_BUTTONS_NUM; i++) {
    if (twoWaySwitchEvents[i].active && (unsigned int)millis() - twoWaySwitchEvents[i].activation >= SWITCH_BUTTON_PRESS_TIME) {
      Joysticks[twoWaySwitchEvents[i].joyIndex].setButton(twoWaySwitchEvents[i].buttonNumber, 0);

      noteOff(twoWaySwitchEvents[i].midiChannel, twoWaySwitchEvents[i].midiPitch, 0);

      twoWaySwitchEvents[i].active = false;
    }
  }
}

void deactivateEncoderPushButtons() {
  for (int i = 0; i < JOY0_ENCODER_PUSH_BUTTONS_NUM + JOY1_ENCODER_PUSH_BUTTONS_NUM; i++) {
    if (encoderPushButtonsEvents[i].active && (unsigned int)millis() - encoderPushButtonsEvents[i].activation >= SWITCH_BUTTON_PRESS_TIME) {
      Joysticks[encoderPushButtonsEvents[i].joyIndex].setButton(encoderPushButtonsEvents[i].buttonNumber, 0);

      //Midi
      noteOff(encoderPushButtonsEvents[i].midiChannel, encoderPushButtonsEvents[i].midiPitch, 0);

      encoderPushButtonsEvents[i].active = false;
    }
  }
}

#ifdef ENCODERS_TO_MIDI
// Arduino MIDI functions MIDIUSB Library
void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = { 0x09, 0x90 | channel, pitch, velocity };
  MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = { 0x08, 0x80 | channel, pitch, velocity };
  MidiUSB.sendMIDI(noteOff);
}

void controlChange(byte channel, byte control, byte value) {
  midiEventPacket_t event = { 0x0B, 0xB0 | channel, control, value };
  MidiUSB.sendMIDI(event);
}
#endif
