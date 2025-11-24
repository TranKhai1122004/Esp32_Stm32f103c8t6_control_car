/*************************************************************
 * ESP32 MQTT Interface (KET NOI VOI HIVEMQ CLOUD BAO MAT)
 * - Chuc nang: Nhan lenh tu Node-RED (MQTT qua TLS/SSL), kiem tra va cham HC-SR04,
 * va gui lenh cuoi cung (bao gom lenh Dung khan cap) qua Serial2 den STM32.
 *************************************************************/

// THU VIEN CAN THIET
#include <WiFi.h>
#include <WiFiClientSecure.h> // Thu vien BAO MAT cho TLS/SSL
#include <PubSubClient.h>     // Thu vien MQTT Client
#include <HardwareSerial.h>
#include <NewPing.h>          // Thu vien NewPing

// =================================================================
// --- THONG TIN CAU HINH CUA BAN (CAN THAY THE) ---
// =================================================================
// Thong tin WiFi
char ssid[] = "Ipc"; 
char pass[] = "2212202444"; 

// CAU HINH HIVEMQ CLOUD (TLS/SSL)
const char* mqtt_server = "0b9f6841ad1048afb1f4200aa7284c1e.s1.eu.hivemq.cloud"; 
const int mqtt_port = 8883; // Cong SSL/TLS
const char* mqtt_user = "trankhai112204"; // <-- USERNAME CUA BAN
const char* mqtt_password = "Loveghita89@"; // <-- PASSWORD CUA BAN

// *** CLIENT ID CỐ ĐỊNH ĐÃ THÊM VÀO ***
const char* CLIENT_ID = "RemoteCar_ESP32_Control"; 
// =================================================================


// Cac Topic ma ESP32 SE DANG KY (Subscribe) de nhan lenh tu Node-RED
const char* TOPIC_DIRECTION = "remote_car/direction"; // Nhan lenh huong: 'F', 'B', 'L', 'R', 'S'
const char* TOPIC_SPEED = "remote_car/speed";     // Nhan lenh toc do: 'sXXX' (vi du: s170)

// Topic ma ESP32 SE GUI (Publish) trang thai (optional)
const char* TOPIC_STATUS = "remote_car/status";

// Khoi tao cac doi tuong
WiFiClientSecure espClient; // SU DUNG WiFiClientSecure
PubSubClient client(espClient);

// --- CAU HINH UART GIAO TIEP VOI STM32 ---
#define STM32_TX_PIN 17          // Chan TX (Truyen) cua ESP32 (Noi toi PA10/RX cua STM32)
#define STM32_BAUD_RATE 9600 

// --- CAU HINH HC-SR04 ---
#define US_TRIG_PIN 5          // Chan TRIG HC-SR04
#define US_ECHO_PIN 18         // Chan ECHO HC-SR04
#define MAX_DISTANCE 400       // Khoang cach toi da (cm)
#define COLLISION_THRESHOLD 15 // Nguong dung khan cap (15 cm)

NewPing sonar(US_TRIG_PIN, US_ECHO_PIN, MAX_DISTANCE); 

// --- KHAI BAO BIEN ---
int currentSpeed = 170; 
char currentCarDirection = 'S'; 
bool isOverridden = false; 
unsigned long lastMsg = 0;
long distance_cm = MAX_DISTANCE; // Bien luu khoang cach

// =================================================================
// HAM HO TRO WIFI VA MQTT
// =================================================================

/**
 * @brief Ket noi lai WiFi.
 */
void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, pass);

    int max_attempts = 20;
    while (WiFi.status() != WL_CONNECTED && max_attempts-- > 0) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("");
        Serial.println("Failed to connect to WiFi. Please check credentials.");
    }
}

/**
 * @brief Gui lenh dieu khien duoi dang chuoi qua Serial2 den STM32.
 */
void sendSTM32Command(String cmd) {
    // Chi gui lenh neu KHONG dang bi ghi de, HOAC neu lenh la DUNG khan cap
    // Lenh Dung khan cap ('S') luon duoc gui.
    if (!isOverridden || cmd == "S") {
        Serial2.print(cmd); 
        Serial.println("STM32 Command: " + cmd);
        // Gui trang thai nguoc lai cho Node-RED
        client.publish(TOPIC_STATUS, ("SENT: "+cmd).c_str());
    }
}

/**
 * @brief Gui lenh huong di chuyen va cap nhat trang thai.
 */
void sendDirectionCommand(char direction) {
    currentCarDirection = direction;
    
    // Gui lenh qua UART (Ham nay se kiem tra isOverridden)
    sendSTM32Command(String(direction));
    client.publish(TOPIC_STATUS, ("Direction set to: " + String(direction)).c_str());
}

/**
 * @brief Xu ly lenh MQTT nhan duoc tu Node-RED (Topic: remote_car/direction)
 */
void handleDirectionCommand(char* message) {
    if (strlen(message) == 1) {
        char dir = message[0];
        
        // Chi xu ly cac lenh hop le
        if (dir == 'F' || dir == 'B' || dir == 'L' || dir == 'R' || dir == 'S') {
            sendDirectionCommand(dir);
        } else {
            Serial.println("Invalid direction command received: " + String(dir));
        }
    }
}

/**
 * @brief Xu ly lenh MQTT nhan duoc tu Node-RED (Topic: remote_car/speed)
 */
void handleSpeedCommand(char* message) {
    // Lenh toc do phai co dang "sXXX" (vi du: s170)
    if (strlen(message) == 4 && message[0] == 's') {
        int newSpeed = atoi(message + 1); // Chuyen "XXX" thanh so
        
        if (newSpeed >= 0 && newSpeed <= 255) {
            currentSpeed = newSpeed;
            sendSTM32Command(String(message)); // Gui thang chuoi 'sXXX' den STM32
            client.publish(TOPIC_STATUS, ("Speed set to: " + String(currentSpeed)).c_str());
        } else {
            Serial.println("Invalid speed value received: " + String(newSpeed));
        }
    }
}


/**
 * @brief Ham callback duoc goi khi nhan duoc tin nhan MQTT.
 */
void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message received [");
    Serial.print(topic);
    Serial.print("] ");
    
    // Chuyen payload thanh chuoi ky tu
    char message[length + 1];
    for (int i = 0; i < length; i++) {
        message[i] = (char)payload[i];
    }
    message[length] = '\0'; // Ket thuc chuoi
    Serial.println(message);

    // Xu ly theo Topic
    if (String(topic) == String(TOPIC_DIRECTION)) {
        handleDirectionCommand(message);
    } else if (String(topic) == String(TOPIC_SPEED)) {
        handleSpeedCommand(message);
    }
}

/**
 * @brief Ket noi lai MQTT Broker.
 */
void reconnect() {
    // Lap lai cho den khi ket noi
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection with Client ID: ");
        Serial.print(CLIENT_ID); // In ra Client ID cố định
        Serial.print("...");
        
        // *** SỬ DỤNG CLIENT ID CỐ ĐỊNH ĐÃ KHAI BÁO ***
        if (client.connect(CLIENT_ID, mqtt_user, mqtt_password)) {
            Serial.println("connected");
            // Dang ky nhan tin tu cac topic dieu khien
            client.subscribe(TOPIC_DIRECTION);
            client.subscribe(TOPIC_SPEED);
            
            client.publish(TOPIC_STATUS, "ESP32 Connected and Subscribed!");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Cho 5 giay truoc khi thu lai
            delay(5000);
        }
    }
}

// =================================================================
// LOGIC VA CHAM (Collision Logic)
// =================================================================

/**
 * @brief Kiem tra khoang cach va Ghi de lenh dieu khien neu co vat can.
 * Ham nay chay dinh ky.
 */
void checkCollisionAndSendOverride() {
    
    // 1. CHI KIEM TRA KHI DANG TIEN ('F')
    if (currentCarDirection != 'F') { 
        isOverridden = false; // Reset co neu khong phai Tien
        return;
    }
    
    delay(50); // Delay can thiet giua cac lan ping HC-SR04
    unsigned int raw_distance_cm = sonar.ping_cm(); 
    
    // Neu timeout hoac khoang cach do duoc = 0, gan gia tri an toan (Out of Range)
    if (raw_distance_cm == 0) {
        distance_cm = MAX_DISTANCE; 
    } else {
        distance_cm = raw_distance_cm;
    }
    
    // 2. LOGIC GHI DE (Override)
    if (distance_cm <= COLLISION_THRESHOLD) {
        if (!isOverridden) {
            // Gui lenh DUNG khan cap ('S') chi 1 lan
            Serial2.print("S"); // Gui lenh 'S' qua UART den STM32
            isOverridden = true;
            
            // Bao trang thai len Node-RED
            client.publish(TOPIC_STATUS, ("DANGER: STOP at " + String(distance_cm) + "cm").c_str());
            Serial.println("!!! COLLISION OVERRIDE: STOP !!!");
        }
    } 
    // 3. LOGIC HOI PHUC (Reset Override)
    else if (isOverridden) {
        // Neu xe dang bi dung khan cap VA vat can da di chuyen ra xa:
        isOverridden = false;
        // KHONG GUI LENH 'F'. Chi thong bao va cho nguoi dung gui lenh Tien lai.
        client.publish(TOPIC_STATUS, "SAFE: Override reset. Awaiting command.");
        Serial.println("Override reset. Ready for next command.");
    }
}

// =================================================================
// SETUP VA LOOP
// =================================================================

void setup() {
    Serial.begin(9600);

    // Khoi tao Serial2 de giao tiep voi STM32
    Serial2.begin(STM32_BAUD_RATE, SERIAL_8N1, -1, STM32_TX_PIN); 

    // THIET LAP KET NOI BAO MAT (TLS)
    // Bo qua kiem tra chung chi cho muc dich test (INSECURE)
    espClient.setInsecure(); 

    // Ket noi WiFi
    setup_wifi();

    // Thiet lap MQTT Broker va ham Callback
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
}

void loop() {
    // Dam bao ket noi MQTT luon duoc duy tri
    if (!client.connected()) {
        reconnect();
    }
    client.loop(); // Xu ly cac goi tin den
    
    // Chay logic va cham dinh ky (200ms)
    unsigned long now = millis();
    if (now - lastMsg > 200) {
        lastMsg = now;
        checkCollisionAndSendOverride(); 
    }
}