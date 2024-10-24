#include <MFRC522.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define RST_PIN 22 // Pin de reinicio (RST) para el módulo RFID
#define SS_PIN 4   // Pin de selección (SDA/SS) para el módulo RFID

const int laserPin = 36;            // Pin del sensor láser
const int lightSensorPin = 33;      // Pin del sensor de luz
const int ledPin = 0;               // Pin del LED indicador de alarma
const int buzzerPin2 = 15;          // Pin del buzzer 2
const int buzzerPin3 = 2;           // Pin del buzzer 3
const int pirPin = 12;              // Pin del sensor PIR
const int pinLED = 26;              // Pin del LED

bool isAlarmOn = false;             // Estado de la alarma
bool isLaserActive = true;          // Estado del sensor láser
bool isMovementActive = true;       // Estado del sensor de movimiento
bool lastPIRState = LOW;            // Estado anterior del PIR
bool lastLaserState = LOW;          // Estado anterior del láser
unsigned long lastDebounceTime = 0; // Último tiempo de cambio
const long debounceDelay = 1000;    // Tiempo de estabilización después de encender

MFRC522 mfrc522(SS_PIN, RST_PIN);   // Instancia del objeto MFRC522

// UID de las tarjetas RFID válidas
byte validUID1[] = {0x3A, 0xE2, 0x45, 0x17};
byte validUID2[] = {0xAB, 0xCD, 0xEF, 0x01};

const char* ssid = "RE04";        // SSID de WiFi
const char* password = "lamismadesiempre"; // Contraseña de WiFi

const char* mqtt_server = "54.198.221.238"; // IP del servidor MQTT
const int mqtt_port = 1883;               // Puerto del servidor MQTT

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
    Serial.begin(115200);
    SPI.begin();
    mfrc522.PCD_Init();
    pinMode(laserPin, OUTPUT);
    pinMode(lightSensorPin, INPUT);
    pinMode(ledPin, OUTPUT);
    pinMode(buzzerPin2, OUTPUT);
    pinMode(buzzerPin3, OUTPUT);
    pinMode(pirPin, INPUT);
    pinMode(pinLED, OUTPUT);

    connectToWiFi();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(mqttCallback);
    
    delay(1000); // Retardo inicial para estabilización de sensores
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
        checkCard();
    }

    unsigned long currentTime = millis();
    if ((currentTime - lastDebounceTime) > debounceDelay) {
        activateAlarm(currentTime);
    }
    if (!isAlarmOn && isLaserActive) {
        digitalWrite(laserPin, HIGH);
    } else {
        digitalWrite(laserPin, LOW);
    }

    if (isAlarmOn) {
        digitalWrite(ledPin, (millis() / 500) % 2); // Hacer titilar el LED
    } else {
        digitalWrite(ledPin, LOW); // Apagar el LED
    }
}

void connectToWiFi() {
    int retryCount = 0;
    Serial.print("Connecting to WiFi...");
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED && retryCount < 20) {
        delay(500);
        Serial.print(".");
        retryCount++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Connected");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("Failed to connect to WiFi");
    }
}

void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP32Client")) {
            Serial.println("connected");
            client.subscribe("device/command");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();

    StaticJsonDocument<200> doc;
    deserializeJson(doc, payload, length);

    if (doc["alarm1"] != nullptr) {
        if (doc["alarm1"]) {
            activateAlarm(millis());
        } else {
            deactivateAlarm();
        }
    }
    if (doc["laser"] != nullptr) {
        isLaserActive = doc["laser"];
    }
    if (doc["movement"] != nullptr) {
        isMovementActive = doc["movement"];
    }
}

void checkCard() {
    byte currentUID[4];
    for (byte i = 0; i < 4; i++) {
        currentUID[i] = mfrc522.uid.uidByte[i];
    }
    if (checkUID(currentUID)) {
        Serial.println("Access granted");
        digitalWrite(pinLED, LOW);
        isAlarmOn = false;  // Correcto, estás cambiando el estado de la alarma aquí
        deactivateAlarm(); // Debería llamar a esta función para asegurar que todos los procesos de desactivación se manejen
        publishStatus();   // Confirma el nuevo estado al sistema
    } else {
        Serial.println("Access denied");
        digitalWrite(pinLED, HIGH);
    }
}

bool checkUID(byte* uid) {
    return memcmp(uid, validUID1, sizeof(validUID1)) == 0 || memcmp(uid, validUID2, sizeof(validUID2)) == 0;
}

void activateAlarm(unsigned long currentTime) {
   byte currentUID[4];
    for (byte i = 0; i < 4; i++) {
        currentUID[i] = mfrc522.uid.uidByte[i];
    }
    bool move = digitalRead(pirPin) == HIGH;
    bool laser = digitalRead(lightSensorPin) == HIGH;

    if (move != lastPIRState || laser != lastLaserState) {
        lastDebounceTime = currentTime;
        if (move && isMovementActive && !isAlarmOn && !checkUID(currentUID) ) {
            Serial.println("Alarm activated due to movement");
            isAlarmOn = true;
            publishStatus();
        }

        if (laser && isLaserActive && !isAlarmOn && !checkUID(currentUID)) {
            Serial.println("Alarm activated due to laser sensor");
            isAlarmOn = true;
            publishStatus();
        }
    }

    lastPIRState = move;
    lastLaserState = laser;
}

void publishStatus() {
    StaticJsonDocument<200> doc;
    doc["alarm1"] = isAlarmOn;
    doc["alarm2"] = false;
    doc["laser"] = isLaserActive;
    doc["movement"] = isMovementActive;
    char buffer[256];
    size_t n = serializeJson(doc, buffer);
    client.publish("device/status", buffer, n);
}

void deactivateAlarm() {
    // Remueve la comprobación del UID aquí, ya que se maneja en checkCard
    Serial.println("Alarm deactivated");
    isAlarmOn = false;
    publishStatus();
}
