#include <Arduino.h>
#line 1 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
#include <Encoder.h>
#include <LiquidCrystal_I2C.h>

const int NUM_CHANNELS = 8;
const int MIDI_MAX_VALUE = 127;
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
int lastPrintedValues[NUM_CHANNELS] = {0};
int midiSettings[NUM_CHANNELS][2] = {
    {1, 2}, {2, 4}, {3, 6}, {4, 48}, {5, 55}, {6, 66}, {7, 77}, {8, 88}};
int oldEncPosition = 0;

#line 37 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void setup();
#line 55 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void loop();
#line 92 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void handleButtonPress();
#line 106 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void handleEncoder();
#line 117 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void selectChannel(int channel);
#line 124 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void sendMIDIControlChange(int midiChannel, int ccNumber, int value);
#line 37 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void setup()
{
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

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

  // Read analog values and send MIDI if needed
  for (int channel = 0; channel < NUM_CHANNELS; channel++)
  {
    selectChannel(channel);
    analogValues[channel] = analogRead(A0);
    int midiValue = map(analogValues[channel], 0, 1023, 0, MIDI_MAX_VALUE);

    if (abs(midiValue - lastPrintedValues[channel]) >= THRESHOLD)
    {
      sendMIDIControlChange(midiSettings[channel][0], midiSettings[channel][1], midiValue);
      lastPrintedValues[channel] = midiValue;
    }

    // Update LCD display
    int barLevel = map(midiValue, 0, MIDI_MAX_VALUE, 0, 7);
    lcd.setCursor(8 + channel, 0);
    lcd.write((byte)barLevel);
    lcd.setCursor(8 + channel, 1);
    lcd.print(channel == selectedChannel ? "*" : " ");
  }

  // Display selected channel
  lcd.setCursor(0, 0);
  lcd.print("Ch:");
  lcd.print(selectedChannel + 1);
  lcd.setCursor(0, 1);
  lcd.print("CC:");
  lcd.print(midiSettings[selectedChannel][1]);

  handleButtonPress();
  handleEncoder();
}

void handleButtonPress()
{
  bool currentButtonState = digitalRead(buttonPin);
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY)
  {
    if (currentButtonState == LOW && lastButtonState == HIGH)
    {
      selectedChannel = (selectedChannel + 1) % NUM_CHANNELS;
      lcd.clear();
    }
  }
  lastButtonState = currentButtonState;
}

void handleEncoder()
{
  int newPosition = myEnc.read() / 4;
  if (newPosition != oldEncPosition)
  {
    int relativeChange = newPosition - oldEncPosition;
    midiSettings[selectedChannel][1] = constrain(midiSettings[selectedChannel][1] + relativeChange, 0, MIDI_MAX_VALUE);
    oldEncPosition = newPosition;
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
  Serial.write(0xB0 | (midiChannel - 1)); // Control Change message
  Serial.write(ccNumber);                 // Control Number
  Serial.write(value);                    // Value
}

