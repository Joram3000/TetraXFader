#include <Arduino.h>
#line 1 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
#include <Encoder.h>
#include <LiquidCrystal_I2C.h>

// Constants
const int NUM_PAIRS = 4;          // Number of pairs of channels
const int NUM_CHANNELS = 8;       // Total number of channels
const int MIDI_MAX_VALUE = 127;   // Maximum MIDI value
const int PWM_MAX_VALUE = 255;    // Maximum PWM value
const int DEBOUNCE_DELAY = 20;    // Debounce delay for button and encoder
const int MIDI_BAUD_RATE = 31250; // MIDI baud rate
const int THRESHOLD = 2;          // Threshold for MIDI value change
const int A = 2;                  // Multiplexer control pin A
const int B = 3;                  // Multiplexer control pin B
const int C = 4;                  // Multiplexer control pin C
const int buttonPin = 7;          // Button pin
const int pwmPin = 10;            // PWM output pin
const int xfaderLedPin = 11;      // Crossfader LED pin

byte barLevels[8][8] = {
    {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111},
    {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b00000},
    {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b00000, 0b00000},
    {0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b00000, 0b00000, 0b00000},
    {0b00000, 0b00000, 0b00000, 0b11111, 0b00000, 0b00000, 0b00000, 0b00000},
    {0b00000, 0b00000, 0b11111, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000},
    {0b00000, 0b11111, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000},
    {0b11111, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000}};

LiquidCrystal_I2C lcd(0x27, 16, 2);
Encoder myEnc(5, 6);

bool lastButtonState = false;
unsigned long lastDebounceTime = 0;

int XFaderValue = 0;
int oldXfaderValue = 0;

int lastPrintedPairValues[NUM_PAIRS] = {-1, -1, -1, -1};
int lastPrintedValues[NUM_CHANNELS] = {-1, -1, -1, -1, -1, -1, -1, -1};

int midiSettings[NUM_CHANNELS][2] = {
    {1, 2},
    {1, 4},
    {2, 6},
    {2, 8},
    {1, 10},
    {1, 22},
    {2, 33},
    {2, 44}};

// Variables for encoder position
int oldEncPosition = 0;
int debouncedEncPosition = 0;
unsigned long lastEncoderDebounceTime = 0;
int initialEncoderPosition = 0;

// Selected channel
int selectedChannel = 0;

#line 60 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void setup();
#line 80 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void loop();
#line 149 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void handleButtonPress();
#line 163 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void handleEncoder();
#line 185 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void selectChannel(int channel);
#line 192 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void sendMIDIControlChange(int midiChannel, int ccNumber, int value);
#line 60 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void setup()
{
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(pwmPin, OUTPUT);
  pinMode(xfaderLedPin, OUTPUT);

  lcd.init();
  lcd.backlight();

  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    lcd.createChar(i, barLevels[i]);
  }

  Serial.begin(MIDI_BAUD_RATE);
}

void loop()
{
  int analogValues[NUM_CHANNELS];                                                  // Array to store analog values
  int XfaderReading = 500;                                                         // analogRead(A2);                                              // Read crossfader value
  float morphFactor = 1.0 - XFaderValue / 1023.0;                                  // Calculate morph factor
  static int lastDisplayedValues[NUM_CHANNELS] = {-1, -1, -1, -1, -1, -1, -1, -1}; // Last displayed values
  static int lastSelectedChannel = -1;                                             // Last selected channel
  int mappedXFaderValue = map(XFaderValue, 0, 1023, 7, 0);                         // Map crossfader value to 0-7
  analogWrite(pwmPin, mappedXFaderValue);                                          // Write PWM value
  analogWrite(xfaderLedPin, max(0, 4 - mappedXFaderValue));                        // Write inverted LED value

  if (XfaderReading != oldXfaderValue) // Check if crossfader value changed
  {
    XFaderValue = XfaderReading; // Update crossfader value
  }

  // Read analog values for each channel
  for (int channel = 0; channel < NUM_CHANNELS; channel++)
  {
    selectChannel(channel);                                                 // Select channel
    analogValues[channel] = analogRead(A0);                                 // Read analog value
    int midiValue = map(analogValues[channel], 0, 1023, 0, MIDI_MAX_VALUE); // Map to MIDI value
    lastPrintedValues[channel] = midiValue;                                 // Update last printed value

    int barLevel = map(lastPrintedValues[channel], 0, MIDI_MAX_VALUE, 0, 7);
    if (lastDisplayedValues[channel] != barLevel || lastSelectedChannel != selectedChannel)
    {
      int col = 8 + channel;
      lcd.setCursor(col, 0);
      lcd.write((byte)barLevel);

      lcd.setCursor(col, 1);
      lcd.print(channel == selectedChannel ? "*" : " ");

      lastDisplayedValues[channel] = barLevel;
    }
  }

  // Calculate and send MIDI values for pairs of channels
  for (int pair = 0; pair < NUM_CHANNELS; pair += 2)
  {
    int morphedValue = analogValues[pair] * (1 - morphFactor) + analogValues[pair + 1] * morphFactor; // Calculate morphed value
    int midiValue = map(morphedValue, 0, 1023, 0, MIDI_MAX_VALUE);                                    // Map to MIDI value

    if (abs(midiValue - lastPrintedPairValues[pair]) >= THRESHOLD) // Check if value changed significantly
    {
      lastPrintedPairValues[pair] = midiValue; // Update last printed pair value

      sendMIDIControlChange(midiSettings[pair][0], midiSettings[pair][1], midiValue); // Send MIDI control change
    }
  }

  // Update LCD display
  lcd.setCursor(0, 0);
  lcd.print("CC");
  lcd.print(midiSettings[selectedChannel][1]);
  lcd.print("X");
  lcd.write((byte)mappedXFaderValue);
  lcd.setCursor(0, 1);
  lcd.print("MIDI:");
  lcd.print(midiSettings[selectedChannel][0]);

  oldXfaderValue = XfaderReading;        // Update old crossfader value
  lastSelectedChannel = selectedChannel; // Update last selected channel

  handleButtonPress();
  handleEncoder();
}

void handleButtonPress()
{
  bool currentButtonState = digitalRead(buttonPin);   // Read button state
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) // Check debounce delay
  {
    if (currentButtonState == LOW && lastButtonState == HIGH) // Button press detected
    {
      selectedChannel = (selectedChannel + 1) % NUM_PAIRS; // Cycle through channels
      lcd.clear();                                         // Clear LCD display
    }
  }
  lastButtonState = currentButtonState; // Update last button state
}

void handleEncoder()
{
  int newPosition = myEnc.read() / 4; // Read encoder position
  if (newPosition != oldEncPosition)  // Check if position changed
  {
    lastEncoderDebounceTime = millis(); // Update debounce time
    oldEncPosition = newPosition;       // Update old position
  }

  if ((millis() - lastEncoderDebounceTime) > DEBOUNCE_DELAY) // Check debounce delay
  {
    if (debouncedEncPosition != oldEncPosition) // Check if debounced position changed
    {
      debouncedEncPosition = oldEncPosition;                                                                              // Update debounced position
      int relativeChange = debouncedEncPosition - initialEncoderPosition;                                                 // Calculate relative change
      midiSettings[selectedChannel][1] = constrain(midiSettings[selectedChannel][1] + relativeChange, 0, MIDI_MAX_VALUE); // Update MIDI value
      initialEncoderPosition = debouncedEncPosition;                                                                      // Update initial position
    }
  }
}

// Select multiplexer channel
void selectChannel(int channel)
{
  digitalWrite(A, channel & 0x01);
  digitalWrite(B, (channel >> 1) & 0x01);
  digitalWrite(C, (channel >> 2) & 0x01);
}

void sendMIDIControlChange(int midiChannel, int ccNumber, int value)
{
  Serial.write(0xB0 | (midiChannel - 1));
  Serial.write(ccNumber);
  Serial.write(value);
}

