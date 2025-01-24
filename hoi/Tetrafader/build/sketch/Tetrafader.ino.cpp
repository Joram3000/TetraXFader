#include <Arduino.h>
#line 1 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/hoi/Tetrafader/Tetrafader.ino"
#include <Encoder.h>
#include <LiquidCrystal_I2C.h>

const uint8_t NUM_PAIRS = 4;
const uint8_t NUM_CHANNELS = 8;
const uint8_t MIDI_MAX_VALUE = 127;
const uint8_t PWM_MAX_VALUE = 255;
const uint8_t DEBOUNCE_DELAY = 20;
const uint16_t MIDI_BAUD_RATE = 31250;
const uint8_t THRESHOLD = 2;
const uint8_t A = 2;
const uint8_t B = 3;
const uint8_t C = 4;
const uint8_t buttonPin = 7;
const uint8_t pwmPin = 10;
const uint8_t xfaderLedPin = 11;
const uint8_t XFADER_DEADZONE = 3;

const uint8_t barLevels[8][8] PROGMEM = {
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
uint32_t lastDebounceTime = 0;

int16_t XFaderValue = 0;
int16_t oldXfaderValue = 0;
float smoothedXFaderValue = 0;
const float XFADER_SMOOTHING = 0.3;

int8_t lastPrintedCHANNELValues[NUM_CHANNELS] = {-1, -1, -1, -1, -1, -1, -1, -1};
int8_t lastPrintedPAIRValues[NUM_PAIRS] = {-1, -1, -1, -1};

uint8_t midiSettings[NUM_CHANNELS][2] = {
    {1, 2},
    {1, 4},
    {2, 6},
    {2, 8},
    {3, 10},
    {3, 22},
    {4, 33},
    {4, 44}};

int32_t oldEncPosition = 0;
int32_t debouncedEncPosition = 0;
uint32_t lastEncoderDebounceTime = 0;
int32_t initialEncoderPosition = 0;

uint8_t selectedChannel = 0;

#line 60 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/hoi/Tetrafader/Tetrafader.ino"
void setup();
#line 84 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/hoi/Tetrafader/Tetrafader.ino"
void loop();
#line 165 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/hoi/Tetrafader/Tetrafader.ino"
void handleButtonPress();
#line 179 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/hoi/Tetrafader/Tetrafader.ino"
void handleEncoder();
#line 205 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/hoi/Tetrafader/Tetrafader.ino"
void selectChannel(uint8_t channel);
#line 212 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/hoi/Tetrafader/Tetrafader.ino"
void sendMIDIControlChange(uint8_t midiChannel, uint8_t ccNumber, uint8_t value);
#line 60 "/Users/joram/Documents/Arduino/libraries/Encoder/examples/Basic/hoi/Tetrafader/Tetrafader.ino"
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

  for (uint8_t i = 0; i < NUM_CHANNELS; i++)
  {
    uint8_t customChar[8];
    memcpy_P(customChar, barLevels[i], 8);
    lcd.createChar(i, customChar);
  }

  Serial.begin(MIDI_BAUD_RATE);

  TCCR1B = TCCR1B & B11111000 | B00000001;
}

void loop()
{
  static int8_t lastDisplayedValues[NUM_CHANNELS] = {-1, -1, -1, -1, -1, -1, -1, -1};
  static int8_t lastDisplayedMorphedValues[NUM_PAIRS] = {-1, -1, -1, -1};
  static int8_t lastSelectedChannel = -1;

  int16_t XfaderReading = analogRead(A2);

  if (abs(XfaderReading - oldXfaderValue) <= XFADER_DEADZONE)
  {
    XfaderReading = oldXfaderValue;
  }
  smoothedXFaderValue = smoothedXFaderValue * (1 - XFADER_SMOOTHING) + XfaderReading * XFADER_SMOOTHING;
  float morphFactor = 1.0 - smoothedXFaderValue / 1023.0;
  uint8_t mappedXFaderValue = map(round(smoothedXFaderValue), 0, 1023, 0, 7);

  analogWrite(xfaderLedPin, max(0, 4 - mappedXFaderValue));

  for (uint8_t channel = 0; channel < NUM_CHANNELS; channel++)
  {
    selectChannel(channel);
    int16_t analogValue = analogRead(A0);
    uint8_t midiValue = map(analogValue, 0, 1023, 0, MIDI_MAX_VALUE);
    analogWrite(pwmPin, midiValue * 2);

    if (abs(midiValue - lastPrintedCHANNELValues[channel]) >= THRESHOLD)
    {
      lastPrintedCHANNELValues[channel] = midiValue;
    }

    if (channel % 2 == 1)
    {
      uint8_t pairIndex = channel / 2;
      int16_t valueA = lastPrintedCHANNELValues[channel - 1];
      int16_t valueB = lastPrintedCHANNELValues[channel];
      int16_t morphedValue = (int16_t)(valueA * (1 - morphFactor) + valueB * morphFactor);

      if (abs(morphedValue - lastPrintedPAIRValues[pairIndex]) >= THRESHOLD)
      {
        lastPrintedPAIRValues[pairIndex] = morphedValue;
        sendMIDIControlChange(midiSettings[channel][0], midiSettings[channel][1], morphedValue);
      }
    }

    uint8_t barLevel = map(lastPrintedCHANNELValues[channel], 0, MIDI_MAX_VALUE, 0, 7);
    if (lastDisplayedValues[channel] != barLevel || lastSelectedChannel != selectedChannel)
    {
      uint8_t col = 8 + channel;
      lcd.setCursor(col, 0);
      lcd.write((uint8_t)barLevel);

      lastDisplayedValues[channel] = barLevel;
    }
  }

  lcd.setCursor(0, 0);
  lcd.print("[");
  lcd.print(selectedChannel + 1);
  lcd.print("]");
  lcd.print("M");
  lcd.print(midiSettings[selectedChannel][0]);
  lcd.print("C");
  lcd.print(midiSettings[selectedChannel][1]);
  lcd.setCursor(0, 1);
  lcd.print("X");
  lcd.write((uint8_t)mappedXFaderValue);
  lcd.setCursor(8, 1);
  for (uint8_t i = 0; i < NUM_PAIRS; i++)
  {
    int barLevel = map(lastPrintedPAIRValues[i], 0, MIDI_MAX_VALUE, 0, 7);
    lcd.write((byte)barLevel);
    lcd.write((byte)barLevel);
  }

  oldXfaderValue = XfaderReading;
  lastSelectedChannel = selectedChannel;

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
      selectedChannel = (selectedChannel + 1) % NUM_PAIRS;
      lcd.clear();
    }
  }
  lastButtonState = currentButtonState;
}

void handleEncoder()
{
  static int32_t lastEncoderValue = 0;
  int32_t encoderValue = myEnc.read();

  if (encoderValue != lastEncoderValue)
  {
    if ((millis() - lastEncoderDebounceTime) > DEBOUNCE_DELAY)
    {
      int32_t change = encoderValue - lastEncoderValue;

      // Adjust sensitivity: change by 1 for every 4 steps
      int8_t adjustedChange = change / 4;

      if (adjustedChange != 0)
      {
        int16_t newValue = midiSettings[selectedChannel][1] + adjustedChange;
        midiSettings[selectedChannel][1] = constrain(newValue, 0, MIDI_MAX_VALUE);

        lastEncoderDebounceTime = millis();
        lastEncoderValue = encoderValue;
      }
    }
  }
}

inline void selectChannel(uint8_t channel)
{
  digitalWrite(A, channel & 0x01);
  digitalWrite(B, (channel >> 1) & 0x01);
  digitalWrite(C, (channel >> 2) & 0x01);
}

inline void sendMIDIControlChange(uint8_t midiChannel, uint8_t ccNumber, uint8_t value)
{
  Serial.write(0xB0 | (midiChannel - 1));
  Serial.write(ccNumber);
  Serial.write(value);
}
