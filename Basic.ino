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

int lastPrintedValues[NUM_CHANNELS] = {-1, -1, -1, -1, -1, -1, -1, -1};
int settingsMidiCC[NUM_CHANNELS][2] = {
    {1, 2},  // Channel 1, CC 0
    {1, 4},  // Channel 1, CC 1
    {2, 6},  // Channel 2, CC 2
    {2, 8},  // Channel 2, CC 3
    {1, 10}, // Channel 1, CC 4
    {1, 22}, // Channel 1, CC 5
    {2, 33}, // Channel 2, CC 6
    {2, 44}  // Channel 2, CC 7
};
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
  sendMidiValues();
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

  // Display selected channel and CC number
  lcd.setCursor(0, 0);
  lcd.print("CC:");
  lcd.print(settingsMidiCC[selectedChannel][1]);
  lcd.print(" "); // Clear any leftover characters
  lcd.setCursor(0, 1);
  lcd.print("Ch:");
  lcd.print(settingsMidiCC[selectedChannel][0]);

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

// Function to send a MIDI Control Change (CC) message over Serial
void sendMIDIControlChange(int controller, int value)
{
  Serial.write(0xB0 | (settingsMidiCC[controller][0] - 1)); // Control Change message on the specified channel
  Serial.write(settingsMidiCC[controller][1]);              // Controller number (0-7 for channels 0-7)
  Serial.write(value);                                      // Value (0-127)
}

void sendMidiValues()
{
  // Default mode: Read and display MIDI values
  int values[8]; // Array to store the analog values of all channels

  for (int channel = 0; channel < 8; channel++)
  {
    selectChannel(channel);           // Select the active channel
    values[channel] = analogRead(A0); // Read and store the analog value

    // Map the analog value (0–1023) to MIDI range (0–127)
    int midiValue = map(values[channel], 0, 1023, 0, 127);

    // Update only if the value has changed significantly (by the threshold)
    if (abs(midiValue - lastPrintedValues[channel]) >= THRESHOLD)
    {
      lastPrintedValues[channel] = midiValue; // Update the last printed value

      // Send MIDI CC message
      sendMIDIControlChange(channel, midiValue);
    }
  }
}

void selectChannel(int channel)
{
  digitalWrite(A, channel & 0x01);
  digitalWrite(B, (channel >> 1) & 0x01);
  digitalWrite(C, (channel >> 2) & 0x01);
}