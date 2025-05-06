#define BLYNK_TEMPLATE_ID "TMPL3s_Bc2-PR"
#define BLYNK_TEMPLATE_NAME "intrusion"
#define BLYNK_AUTH_TOKEN "vsDXg9Gdl7QzYftkLq_45VhQW52Oe2YL"

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// **Sensor and Output Pin Definitions**
#define TRIG_PIN 5       // HC-SR04 Trig pin
#define ECHO_PIN 18      // HC-SR04 Echo pin
#define BUZZER_PIN 19    // Buzzer
#define PIR_PIN 21       // PIR Sensor Output pin

#define ULTRASONIC_LED 4 // LED for ultrasonic detection
#define PIR_LED 27       // LED for PIR motion detection
#define BLINK_LED 2      // General blinking LED

// **WiFi Credentials**
char ssid[] = "Tp-link";   // Change to your WiFi name
char pass[] = "12345678800"; // Change to your WiFi password
char auth[] = BLYNK_AUTH_TOKEN;

// **Constants**
const int ULTRASONIC_THRESHOLD = 20;  // Object detection distance (cm)
bool blinkState = LOW;
unsigned long previousMillis = 0;
const long blinkInterval = 500;  // 500ms blink interval
BlynkTimer timer;

void setup() {
    Serial.begin(115200);

    // **Pin Setup**
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(PIR_PIN, INPUT);
    pinMode(ULTRASONIC_LED, OUTPUT);
    pinMode(PIR_LED, OUTPUT);
    pinMode(BLINK_LED, OUTPUT);

    // **LED Test at Startup**
    digitalWrite(ULTRASONIC_LED, HIGH);
    digitalWrite(PIR_LED, HIGH);
    digitalWrite(BLINK_LED, HIGH);
    delay(500);
    digitalWrite(ULTRASONIC_LED, LOW);
    digitalWrite(PIR_LED, LOW);
    digitalWrite(BLINK_LED, LOW);

    // **Connect to WiFi and Blynk**
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println("\nWiFi Connected!");

    Blynk.begin(auth, ssid, pass);
    Serial.println("Connected to Blynk!");

    // **Send data to Blynk every second**
    timer.setInterval(1000L, sendDataToBlynk);
}

void loop() {
    Blynk.run();
    timer.run();

    unsigned long currentMillis = millis();
    
    // **Read Sensor Values**
    float distance = getUltrasonicDistance();
    bool motion = digitalRead(PIR_PIN);

    // **Object Detected by Ultrasonic Sensor**
    if (distance > 0 && distance < ULTRASONIC_THRESHOLD) {
        triggerUltrasonicAlarm();
        Serial.println("Ultrasonic Sensor: Object Detected!");
    } else {
        digitalWrite(ULTRASONIC_LED, LOW);
    }

    // **Motion Detected by PIR Sensor**
    if (motion) {
        triggerPirAlarm();
        Serial.println("PIR Sensor: Motion Detected!");
    } else {
        digitalWrite(PIR_LED, LOW);
    }

    // **Non-blocking LED Blinking**
    if (currentMillis - previousMillis >= blinkInterval) {
        previousMillis = currentMillis;
        blinkState = !blinkState;
        digitalWrite(BLINK_LED, blinkState);
    }
}

// **Trigger Alarm for Ultrasonic Sensor**
void triggerUltrasonicAlarm() {
    digitalWrite(ULTRASONIC_LED, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
}

// **Trigger Alarm for PIR Sensor**
void triggerPirAlarm() {
    digitalWrite(PIR_LED, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
}

// **Send Data to Blynk**
void sendDataToBlynk() {
    float distance = getUltrasonicDistance();
    bool motionDetected = digitalRead(PIR_PIN);
    
    Blynk.virtualWrite(V0, distance);
    Blynk.virtualWrite(V1, motionDetected);
}

// **Measure Distance using Ultrasonic Sensor**
float getUltrasonicDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // Timeout of 30ms
    if (duration == 0) {
        Serial.println("Ultrasonic sensor timeout!");
        return -1;  // Invalid reading
    }
    return (duration * 0.0343) / 2;
}
