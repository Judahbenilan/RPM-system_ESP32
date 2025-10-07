#define IR_PIN 32       // IR sensor pin for RPM
#define VIB_PIN 33      // Vibration sensor pin (digital)
#define DHT_PIN 4       // DHT11 data pin

// PZEM004T UART pins
#define PZEM_RX 16
#define PZEM_TX 17

volatile unsigned long pulseCount = 0;  
unsigned long lastTime = 0;
float rpm = 0;

// --- Vibration counting ---
unsigned int vibCount = 0;  

// --- DHT11 data ---
int temperature = 0;
int humidity = 0;

// --- PZEM004T data ---
float voltage = 0;
float current = 0;
float power = 0;
float energy = 0;

void IRAM_ATTR countPulse() {
  pulseCount++;   // Increment pulse count for RPM
}

bool readDHT11(int pin, int *temperature, int *humidity) {
  uint8_t data[5] = {0, 0, 0, 0, 0};

  // Start signal
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(20); // 20ms
  digitalWrite(pin, HIGH);
  delayMicroseconds(40);
  pinMode(pin, INPUT);

  // Acknowledge from DHT11
  unsigned long t = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - t > 90) return false;
  }
  t = micros();
  while (digitalRead(pin) == LOW) {
    if (micros() - t > 90) return false;
  }
  t = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - t > 90) return false;
  }

  // Read 40 bits (5 bytes)
  for (int i = 0; i < 40; i++) {
    // Wait for LOW
    t = micros();
    while (digitalRead(pin) == LOW) {
      if (micros() - t > 100) return false;
    }
    // Measure HIGH time
    t = micros();
    while (digitalRead(pin) == HIGH) {
      if (micros() - t > 100) return false;
    }
    int pulseLength = micros() - t;

    // If high pulse > 40µs → it's a 1
    data[i / 8] <<= 1;
    if (pulseLength > 40) data[i / 8] |= 1;
  }

  // Verify checksum
  if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4]) return false;

  *humidity = data[0];
  *temperature = data[2];
  return true;
}

uint16_t crc16(const uint8_t *data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc = crc >> 1;
    }
  }
  return crc;
}

void sendPZEMRead() {
  uint8_t frame[8] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00};
  uint16_t crc = crc16(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = crc >> 8;
  for (int i = 0; i < 8; i++) {
    Serial2.write(frame[i]);
  }
}

bool readPZEM(float *voltage, float *current, float *power, float *energy) {
  uint8_t buffer[13];
  unsigned long start = millis();
  int idx = 0;
  while (millis() - start < 1000 && idx < 13) {
    if (Serial2.available()) {
      buffer[idx++] = Serial2.read();
    }
  }
  if (idx < 13) return false; // Incomplete data

  // Verify CRC
  uint16_t crcCalc = crc16(buffer, 11);
  uint16_t crcRecv = buffer[11] | (buffer[12] << 8);
  if (crcCalc != crcRecv) return false;

  if (buffer[0] != 0x01 || buffer[1] != 0x04) return false;

  *voltage = (buffer[3] << 8 | buffer[4]) / 10.0f;
  *current = (buffer[5] << 8 | buffer[6]) / 1000.0f;
  *power = (buffer[7] << 8 | buffer[8]) / 10.0f;
  *energy = (buffer[9] << 8 | buffer[10]) / 100.0f;

  return true;
}

void setup() {
  Serial.begin(115200);

  // --- IR Sensor for RPM ---
  pinMode(IR_PIN, INPUT_PULLUP);    
  attachInterrupt(digitalPinToInterrupt(IR_PIN), countPulse, FALLING);

  // --- Vibration sensor ---
  pinMode(VIB_PIN, INPUT);

  // --- PZEM004T UART ---
  Serial2.begin(9600, SERIAL_8N1, PZEM_RX, PZEM_TX);

  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();

  // Count vibration pulses continuously
  if (digitalRead(VIB_PIN) == HIGH) {
    vibCount++;
    delay(5); // debounce
  }

  // --- Update every 1 sec ---
  if (currentTime - lastTime >= 10000) {
    // --- RPM ---
    rpm = (pulseCount * 60.0);   // pulses per revolution = 1
    Serial.print("RPM: ");
    Serial.println(rpm);

    // --- Vibration ---
    Serial.print("Vibration Count (per sec): ");
    Serial.println(vibCount);

    // --- Temperature & Humidity (DHT11) ---
    if (readDHT11(DHT_PIN, &temperature, &humidity)) {
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.println(" °C");
      Serial.print("Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");
    } else {
      Serial.println("DHT11 read failed!");
    }

    // --- PZEM004T Reading every 2 seconds ---
    static unsigned long lastPZEMRequest = 0;
    if (currentTime - lastPZEMRequest >= 2000) {
      sendPZEMRead();
      if (readPZEM(&voltage, &current, &power, &energy)) {
        Serial.print("Voltage: ");
        Serial.print(voltage);
        Serial.println(" V");

        Serial.print("Current: ");
        Serial.print(current);
        Serial.println(" A");

        Serial.print("Power: ");
        Serial.print(power);
        Serial.println(" W");

        Serial.print("Energy: ");
        Serial.print(energy);
        Serial.println(" Wh");
      } else {
        Serial.println("PZEM004T read failed!");
      }
      lastPZEMRequest = currentTime;
    }

    Serial.println("---------------------");

    // Reset counters
    pulseCount = 0;
    vibCount = 0;
    lastTime = currentTime;
  }
}
