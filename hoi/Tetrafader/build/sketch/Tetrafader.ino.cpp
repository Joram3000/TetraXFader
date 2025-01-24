#include <Arduino.h>
#line 1 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/hoi/Tetrafader/Tetrafader.ino"
#include <Encoder.h>
#include <LiquidCrystal_I2C.h>

const int NUM_PAIRS = 4;
const int NUM_CHANNELS = 8;
const int MIDI_MAX_VALUE = 127;
const int PWM_MAX_VALUE = 255;
const int DEBOUNCE_DELAY = 20;
const int MIDI_BAUD_RATE = 31250;
const int THRESHOLD = 2;
const int A = 2;
const int B = 3;
const int C = 4;
const int buttonPin = 7;
const int pwmPin = 10;
const int xfaderLedPin = 11;
const int XFADER_DEADZONE = 3;

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
float smoothedXFaderValue = 0;
const float XFADER_SMOOTHING = 0.3;

int lastPrintedCHANNELValues[NUM_CHANNELS] = {-1, -1, -1, -1, -1, -1, -1, -1};
int lastPrintedPAIRValues[NUM_PAIRS] = {-1, -1, -1, -1};

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

#line 62 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/hoi/Tetrafader/Tetrafader.ino"
void setup();
#line 82 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/hoi/Tetrafader/Tetrafader.ino"
void loop();
#line 202 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/hoi/Tetrafader/Tetrafader.ino"
void selectChannel(int channel);
#line 209 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/hoi/Tetrafader/Tetrafader.ino"
void sendMIDIControlChange(int midiChannel, int ccNumber, int value);
#line 62 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/hoi/Tetrafader/Tetrafader.ino"
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
  int morphedValues[NUM_PAIRS];
  int XfaderReading = analogRead(A2);

  if (abs(XfaderReading - oldXfaderValue) <= XFADER_DEADZONE)
  {
    XfaderReading = oldXfaderValue;
  }
  smoothedXFaderValue = smoothedXFaderValue * (1 - XFADER_SMOOTHING) + XfaderReading * XFADER_SMOOTHING;
  float morphFactor = smoothedXFaderValue / 1023.0;
  int mappedXFaderValue = map(round(smoothedXFaderValue), 0, 1023, 0, 7);

  static int lastDisplayedValues[NUM_CHANNELS] = {-1, -1, -1, -1, -1, -1, -1, -1};
  static int lastSelectedChannel = -1;

  analogWrite(pwmPin, mappedXFaderValue);
  analogWrite(xfaderLedPin, max(0, 4 - mappedXFaderValue));

  // Read analog values for each channel and calculate morphed values
  for (int channel = 0; channel < NUM_CHANNELS; channel++)
  {
    selectChannel(channel);
    analogValues[channel] = analogRead(A0);
    int midiValue = map(analogValues[channel], 0, 1023, 0, MIDI_MAX_VALUE);

    if (abs(midiValue - lastPrintedCHANNELValues[channel]) >= THRESHOLD)
    {
      lastPrintedCHANNELValues[channel] = midiValue;
    }

    // Calculate morphed values for each pair
    if (channel % 2 == 1) // For every odd channel (1, 3, 5, 7)
    {
      int pairIndex = channel / 2;
      int valueA = lastPrintedCHANNELValues[channel - 1];
      int valueB = lastPrintedCHANNELValues[channel];
      int morphedValue = (int)(valueA * (1 - morphFactor) + valueB * morphFactor);

      if (abs(morphedValue - morphedValues[pairIndex]) >= THRESHOLD)
      {
        morphedValues[pairIndex] = morphedValue;

        // Send MIDI for the morphed value of the pair
        sendMIDIControlChange(midiSettings[channel][0], midiSettings[channel][1], morphedValue);
      }
    }

    // LCD SCREEN update (existing code)
    int barLevel = map(lastPrintedCHANNELValues[channel], 0, MIDI_MAX_VALUE, 0, 7);
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

  // Update LCD for morphed values
  lcd.setCursor(0, 0);
  lcd.print("CC");
  lcd.print(midiSettings[selectedChannel][1]);
  lcd.print("X");
  lcd.write((byte)mappedXFaderValue);

  lcd.setCursor(0, 1);
  for (int i = 0; i < NUM_PAIRS; i++)
  {
    lcd.print(morphedValues[i]);
    lcd.print(" ");
  }

  oldXfaderValue = XfaderReading;
  lastSelectedChannel = selectedChannel;

  // handleButtonPress();
  // handleEncoder();
}

// void handleButtonPress()
// {
//   bool currentButtonState = digitalRead(buttonPin);
//   if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY)
//   {
//     if (currentButtonState == LOW && lastButtonState == HIGH)
//     {
//       selectedChannel = (selectedChannel + 1) % NUM_PAIRS;
//       lcd.clear();
//     }
//   }
//   lastButtonState = currentButtonState;
// }

// void handleEncoder()
// {
//   int newPosition = myEnc.read() / 4;
//   if (newPosition != oldEncPosition)
//   {
//     lastEncoderDebounceTime = millis();
//     oldEncPosition = newPosition;
//   }

//   if ((millis() - lastEncoderDebounceTime) > DEBOUNCE_DELAY)
//   {
//     if (debouncedEncPosition != oldEncPosition)
//     {
//       debouncedEncPosition = oldEncPosition;
//       int relativeChange = debouncedEncPosition - initialEncoderPosition;
//       midiSettings[selectedChannel][1] = constrain(midiSettings[selectedChannel][1] + relativeChange, 0, MIDI_MAX_VALUE);
//       initialEncoderPosition = debouncedEncPosition;
//     }
//   }
// }

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

