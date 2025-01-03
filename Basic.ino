#include <Encoder.h>
#include <LiquidCrystal_I2C.h>

// Constants
const int NUM_CHANNELS = 8;
const int MIDI_MAX_VALUE = 127;
const int DEBOUNCE_DELAY = 20;
const int MIDI_BAUD_RATE = 31250;
const int THRESHOLD = 2;

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
LiquidCrystal_I2C lcd(0x27, 16, 2);
bool lastButtonState = false;
unsigned long lastDebounceTime = 0;
int settingsMidiChannels[NUM_CHANNELS][2] = {{1, 0}, {1, 1}, {1, 2}, {1, 3}, {1, 4}, {2, 5}, {2, 4}, {2, 3}};

int lastPrintedValues[NUM_CHANNELS] = {-1, -1, -1, -1, -1, -1, -1, -1};
int oldPosition = 0;
int debouncedPosition = 0;
unsigned long lastEncoderDebounceTime = 0;
int selectedChannel = 0;
int initialEncoderPosition = 0;

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
  handleButtonPress();
  handleEncoder();
  updateDisplay();
  readAndSendMidiValues();
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
  int newPosition = myEnc.read() / 3;
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
      settingsMidiChannels[selectedChannel][1] = constrain(settingsMidiChannels[selectedChannel][1] + relativeChange, 0, MIDI_MAX_VALUE);
      initialEncoderPosition = debouncedPosition;
    }
  }
}

void updateDisplay()
{
  static int lastDisplayedValues[NUM_CHANNELS] = {-1, -1, -1, -1, -1, -1, -1, -1};
  static int lastSelectedChannel = -1;

  // Display MIDI channel in the top left corner
  lcd.setCursor(1, 0);
  lcd.print("MIDI");
  lcd.print(settingsMidiChannels[selectedChannel][0]);

  // Display MIDI channel on the left side of the second row
  lcd.setCursor(1, 1);
  lcd.print("CC");
  lcd.print(settingsMidiChannels[selectedChannel][1] + 1);
  lcd.print("   "); // Clear any leftover characters

  for (int channel = 0; channel < NUM_CHANNELS; channel++)
  {
    int barLevel = map(lastPrintedValues[channel], 0, MIDI_MAX_VALUE, 0, 7);

    if (lastDisplayedValues[channel] != barLevel || lastSelectedChannel != selectedChannel)
    {
      int col = 8 + channel; // Start from the 9th column for bars

      // Display bar levels on the first row
      lcd.setCursor(col, 0);
      lcd.write((byte)barLevel);

      // Display asterisk under the selected channel
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

void sendMIDIControlChange(int controller, int value)
{
  Serial.write(0xB0 | (settingsMidiChannels[controller][0] - 1));
  Serial.write(settingsMidiChannels[controller][1]);
  Serial.write(value);
}

void readAndSendMidiValues()
{
  int analogValues[NUM_CHANNELS];

  for (int channel = 0; channel < NUM_CHANNELS; channel++)
  {
    selectChannel(channel);
    analogValues[channel] = analogRead(A0);

    int midiValue = map(analogValues[channel], 0, 1023, 0, MIDI_MAX_VALUE);

    if (abs(midiValue - lastPrintedValues[channel]) >= THRESHOLD)
    {
      lastPrintedValues[channel] = midiValue;
      sendMIDIControlChange(channel, midiValue);
    }
  }
}