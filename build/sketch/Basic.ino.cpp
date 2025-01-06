#include <Arduino.h>
#line 1 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
#include <Encoder.h>
#include <LiquidCrystal_I2C.h>

// Constants
const int NUM_CHANNELS = 8;
const int MIDI_MAX_VALUE = 127;
const int PWM_MAX_VALUE = 240; // FOR SAFETY
const int DEBOUNCE_DELAY = 20;
const int MIDI_BAUD_RATE = 31250;
const int THRESHOLD = 2;

unsigned long previousMillis = 0;
const long interval = 10; // Adjust for desired fade speed
int brightness = 0;
int fadeAmount = 5; // Adjust for desired fade step

// Bar levels for LCD
byte barLevels[8][8] = {
    {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111},
    {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111},
    {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111},
    {0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111},
    {0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111},
    {0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111},
    {0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111},
    {0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111}};

// Pin definitions
const int A = 2;
const int B = 3;
const int C = 4;
Encoder myEnc(5, 6);
const int buttonPin = 7;
const int pwmPin = 10;
LiquidCrystal_I2C lcd(0x27, 16, 2);
bool lastButtonState = false;
unsigned long lastDebounceTime = 0;
int XFaderValue = 0;
int oldXfaderValue = 0;
int lastPrintedValues[NUM_CHANNELS] = {-1, -1, -1, -1, -1, -1, -1, -1};
int settingsMidiCC[NUM_CHANNELS][2] = {
    {1, 2},
    {1, 4},
    {2, 6},
    {2, 8},
    {1, 10},
    {1, 22},
    {2, 33},
    {2, 44}};
int oldPosition = 0;
int debouncedPosition = 0;
unsigned long lastEncoderDebounceTime = 0;
int selectedChannel = 0;
int initialEncoderPosition = 0;

#line 56 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void setup();
#line 75 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void loop();
#line 85 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void handleButtonPress();
#line 99 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void handleEncoder();
#line 120 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void updateDisplay();
#line 154 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void selectChannel(int channel);
#line 161 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void sendMIDIControlChange(int midiChannel, int ccNumber, int value);
#line 168 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void readXfader();
#line 179 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void readAndSendMidiValues();
#line 200 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void fadeLEDs();
#line 56 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/Basic.ino"
void setup()
{
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(pwmPin, OUTPUT);

  TCCR1B = TCCR1B & B11111000 | B00000001;

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
  handleButtonPress();
  handleEncoder();
  updateDisplay();
  readAndSendMidiValues();
  readXfader();
  fadeLEDs();
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
  if (newPosition != oldPosition)
  {
    lastEncoderDebounceTime = millis();
    oldPosition = newPosition;
  }

  if ((millis() - lastEncoderDebounceTime) > DEBOUNCE_DELAY)
  {
    if (debouncedPosition != oldPosition)
    {
      debouncedPosition = oldPosition;
      int relativeChange = debouncedPosition - initialEncoderPosition;
      settingsMidiCC[selectedChannel][1] = constrain(settingsMidiCC[selectedChannel][1] + relativeChange, 0, MIDI_MAX_VALUE);
      initialEncoderPosition = debouncedPosition;
    }
  }
}

void updateDisplay()
{
  static int lastDisplayedValues[NUM_CHANNELS] = {-1, -1, -1, -1, -1, -1, -1, -1};
  static int lastSelectedChannel = -1;

  lcd.setCursor(0, 0);
  lcd.print("CC:");
  lcd.print(settingsMidiCC[selectedChannel][1]);
  lcd.print("X:");
  lcd.print(XFaderValue);
  lcd.setCursor(0, 1);
  lcd.print("MIDI:");
  lcd.print(settingsMidiCC[selectedChannel][0]);

  for (int channel = 0; channel < NUM_CHANNELS; channel++)
  {
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

  lastSelectedChannel = selectedChannel;
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

void readXfader()
{
  int XfaderReading = analogRead(A2);

  if (XfaderReading != oldXfaderValue)
  {
    XFaderValue = map(XfaderReading, 0, 1023, 0, 30);
  }
  oldXfaderValue = XfaderReading;
}

void readAndSendMidiValues()
{
  int analogValues[NUM_CHANNELS];

  for (int channel = 0; channel < NUM_CHANNELS; channel++)
  {
    selectChannel(channel);
    analogValues[channel] = analogRead(A0);

    int midiValue = map(analogValues[channel], 0, 1023, 0, MIDI_MAX_VALUE);
    // Add XFaderValue to midiValue
    midiValue += XFaderValue;
    midiValue = constrain(midiValue, 0, MIDI_MAX_VALUE);
    if (abs(midiValue - lastPrintedValues[channel]) >= THRESHOLD)
    {
      lastPrintedValues[channel] = midiValue;
      sendMIDIControlChange(settingsMidiCC[channel][0], settingsMidiCC[channel][1], midiValue);
    }
  }
}

void fadeLEDs()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;

    analogWrite(pwmPin, brightness);

    brightness = brightness + fadeAmount;

    if (brightness <= 0 || brightness >= 255)
    {
      fadeAmount = -fadeAmount;
    }
  }
}
