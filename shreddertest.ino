/*
 * Test code for the UBC Rapid Filament Recycler Shredder
 */
const int buttonPin = 2; // The digital pin connected to the button
const int ledPin = 13; // The digital pin connected to an LED
unsigned long previousMillis = 0;
const unsigned long analogReadInterval = 1000; // 1-second interval
unsigned long lastButtonPressTime = 0;
const unsigned long debounceDelay = 50; // 50ms debounce time

float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // set the ADC attenuation to 11 dB (up to ~3.3V input)
  analogSetAttenuation(ADC_11db);
  pinMode(buttonPin, INPUT_PULLDOWN); // Set the button pin as an input with internal pull-down resistor
  pinMode(ledPin, OUTPUT); // Set the LED pin as an output (optional)
}

// the loop routine runs over and over again forever:
void loop() {
  unsigned long currentMillis = millis();

  // Non-blocking Analog Read from potentiometer
  if (currentMillis - previousMillis >= analogReadInterval) {
    previousMillis = currentMillis;

    int analogValue = analogRead(36);
    float voltage = floatMap(analogValue, 0, 4095, 0, 3.3);

    Serial.print("Analog: ");
    Serial.print(analogValue);
    Serial.print(", Voltage: ");
    Serial.println(voltage);
  }

  // Button debounce logic
  if (digitalRead(buttonPin) == HIGH) {
    if (currentMillis - lastButtonPressTime > debounceDelay) {
      digitalWrite(ledPin, LOW); // Turn on LED immediately
      lastButtonPressTime = currentMillis;
    }
  } else {
    digitalWrite(ledPin, HIGH); // Turn off LED when button is released
  }
}