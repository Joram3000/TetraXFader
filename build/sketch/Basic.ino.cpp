#include <Arduino.h>
#line 1 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
#include <Encoder.h>
#include <LiquidCrystal_I2C.h>

const int NUM_CHANNELS = 8;
const int MIDI_MAX_VALUE = 127;
const int PWM_MAX_VALUE = 255;
const int DEBOUNCE_DELAY = 10;
const int MIDI_BAUD_RATE = 31250;
const int THRESHOLD = 2;
const int A = 2;
const int B = 3;
const int C = 4;
const int buttonPin = 7;
const int pwmPin = 10;
const int xfaderLedPin = 11;
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

int selectedChannel = 0;
bool lastButtonState = false;
unsigned long lastDebounceTime = 0;
int XFaderValue = 0;
int oldXfaderValue = 0;
int lastPrintedValues[NUM_CHANNELS] = {-1, -1, -1, -1, -1, -1, -1, -1};
int midiSettings[NUM_CHANNELS][2] = {
    {1, 2},
    {2, 4},
    {3, 6},
    {4, 48},
    {5, 55},
    {6, 66},
    {7, 77},
    {8, 88},
};

int oldEncPosition = 0;
int debouncedEncPosition = 0;
unsigned long lastEncoderDebounceTime = 0;
int initialEncoderPosition = 0;

#line 51 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void setup();
#line 71 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void loop();
#line 132 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void handleButtonPress();
#line 146 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void handleEncoder();
#line 167 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void selectChannel(int channel);
#line 174 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void sendMIDIControlChange(int midiChannel, int ccNumber, int value);
#line 51 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
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
  int analogValues[NUM_CHANNELS];
  int XfaderReading = analogRead(A2);
  float morphFactor = XFaderValue / 1023.0;
  static int lastDisplayedValues[NUM_CHANNELS] = {-1, -1, -1, -1, -1, -1, -1, -1};
  static int lastSelectedChannel = -1;
  int mappedXFaderValue = map(XFaderValue, 0, 1023, 7, 0);

  if (XfaderReading != oldXfaderValue)
  {
    XFaderValue = XfaderReading;
  }

  for (int channel = 0; channel < NUM_CHANNELS; channel++)
  {
    selectChannel(channel);
    analogValues[channel] = analogRead(A0);
    int midiValue = map(analogValues[channel], 0, 1023, 0, MIDI_MAX_VALUE);

    if (abs(midiValue - lastPrintedValues[channel]) >= THRESHOLD)
    {
      sendMIDIControlChange(midiSettings[channel][0], midiSettings[channel][1], midiValue); // Send MIDI control change
      lastPrintedValues[channel] = midiValue;
    }

    lastPrintedValues[channel] = midiValue; // Update last printed value

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

  analogWrite(pwmPin, mappedXFaderValue);                   // Write PWM value
  analogWrite(xfaderLedPin, max(0, 4 - mappedXFaderValue)); // Write inverted LED value

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
      selectedChannel = (selectedChannel + 1) % 4; // Cycle through channels
      lcd.clear();                                 // Clear LCD display
    }
  }
  lastButtonState = currentButtonState; // Update last button state
}

void handleEncoder()
{
  int newPosition = myEnc.read() / 4;
  if (newPosition != oldEncPosition)
  {
    lastEncoderDebounceTime = millis();
    oldEncPosition = newPosition;
  }

  if ((millis() - lastEncoderDebounceTime) > DEBOUNCE_DELAY)
  {
    if (debouncedEncPosition != oldEncPosition)
    {
      debouncedEncPosition = oldEncPosition;                                                                              // Update debounced position
      int relativeChange = debouncedEncPosition - initialEncoderPosition;                                                 // Calculate relative change
      midiSettings[selectedChannel][1] = constrain(midiSettings[selectedChannel][1] + relativeChange, 0, MIDI_MAX_VALUE); // Update MIDI value
      initialEncoderPosition = debouncedEncPosition;                                                                      // Update initial position
    }
  }
}

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

