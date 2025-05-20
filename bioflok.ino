#include "esp_camera.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "FS.h"
#include "SD_MMC.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"
#include <Wire.h>
#include "RTClib.h"
#include <esp_now.h>

// WiFi and MQTT settings
const char* ssid = "AMANDA";
const char* password = "aurora08";
const char* mqtt_server = "broker.emqx.io";
const char* mqttTopic = "esp32cam/bioflok";

// ESP-NOW settings
uint8_t peerAddress[] = {0x20, 0x43, 0xA8, 0x65, 0x71, 0x78};
#define MSG_BIOFLOC 1  // Message type for biofloc data

WiFiClient espClient;
PubSubClient client(espClient);
RTC_DS3231 rtc;

// Hardware settings
#define FLASH_LED_PIN 4
unsigned long lastMsg = 0;
const long interval = 15000; // 15 seconds between captures
const int MAX_FILES = 1000;  // Maximum number of files to keep

// Camera pin definitions
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

camera_config_t camera_config;

// ESP-NOW callback function
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
  
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Sent to: ");
  Serial.println(macStr);
}

// BMP header structure
#pragma pack(push, 1)
typedef struct {
  uint16_t signature;
  uint32_t fileSize;
  uint32_t reserved;
  uint32_t dataOffset;
  uint32_t headerSize;
  uint32_t width;
  uint32_t height;
  uint16_t planes;
  uint16_t bitsPerPixel;
  uint32_t compression;
  uint32_t imageSize;
  uint32_t xPixelsPerMeter;
  uint32_t yPixelsPerMeter;
  uint32_t colorsUsed;
  uint32_t importantColors;
} BMPHeader;
#pragma pack(pop)

void setupCamera(pixformat_t format, framesize_t size) {
  esp_camera_deinit();

  camera_config.ledc_channel = LEDC_CHANNEL_0;
  camera_config.ledc_timer = LEDC_TIMER_0;
  camera_config.pin_d0 = Y2_GPIO_NUM;
  camera_config.pin_d1 = Y3_GPIO_NUM;
  camera_config.pin_d2 = Y4_GPIO_NUM;
  camera_config.pin_d3 = Y5_GPIO_NUM;
  camera_config.pin_d4 = Y6_GPIO_NUM;
  camera_config.pin_d5 = Y7_GPIO_NUM;
  camera_config.pin_d6 = Y8_GPIO_NUM;
  camera_config.pin_d7 = Y9_GPIO_NUM;
  camera_config.pin_xclk = XCLK_GPIO_NUM;
  camera_config.pin_pclk = PCLK_GPIO_NUM;
  camera_config.pin_vsync = VSYNC_GPIO_NUM;
  camera_config.pin_href = HREF_GPIO_NUM;
  camera_config.pin_sscb_sda = SIOD_GPIO_NUM;
  camera_config.pin_sscb_scl = SIOC_GPIO_NUM;
  camera_config.pin_pwdn = PWDN_GPIO_NUM;
  camera_config.pin_reset = RESET_GPIO_NUM;
  camera_config.xclk_freq_hz = 20000000;
  camera_config.pixel_format = format;
  camera_config.frame_size = size;
  camera_config.jpeg_quality = 12;
  camera_config.fb_count = 1;

  if (esp_camera_init(&camera_config) != ESP_OK) {
    Serial.println("Camera init failed");
    while (true);
  }
}

// Initialize ESP-NOW
void setupESPNow() {
  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }
  
  // Register send callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  Serial.println("ESP-NOW initialized");
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  // Initialize RTC
  Wire.begin(14, 15);
  if (!rtc.begin()) {
    Serial.println("RTC not found!");
    while (1);
  }

  // Set system time from RTC
  DateTime now = rtc.now();
  struct timeval tv;
  struct tm timeinfo;
  timeinfo.tm_year = now.year() - 1900;
  timeinfo.tm_mon = now.month() - 1;
  timeinfo.tm_mday = now.day();
  timeinfo.tm_hour = now.hour();
  timeinfo.tm_min = now.minute();
  timeinfo.tm_sec = now.second();        
  timeinfo.tm_isdst = 0;
  tv.tv_sec = mktime(&timeinfo);
  tv.tv_usec = 0;
  settimeofday(&tv, NULL);

  // Setup ESP-NOW first (before connecting to WiFi)
  WiFi.mode(WIFI_AP_STA);
  setupESPNow();
  
  // Setup camera in RGB565 mode
  setupCamera(PIXFORMAT_RGB565, FRAMESIZE_VGA);

  // Configure camera settings
  sensor_t *s = esp_camera_sensor_get();
  s->set_whitebal(s, 0);       // Disable auto white balance
  s->set_awb_gain(s, 0);       // Disable auto WB gain
  s->set_exposure_ctrl(s, 0);  // Disable auto exposure
  s->set_gain_ctrl(s, 1);      // Disable auto gain
  s->set_bpc(s, 0);            // Disable bad pixel correction
  s->set_wpc(s, 0);
  s->set_aec_value(s, 100);    // Manual exposure (100-1200)
  s->set_agc_gain(s, 0);       // Manual gain (0-30)
  s->set_saturation(s, 0);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  
  // Setup MQTT
  client.setServer(mqtt_server, 1883);

  // Initialize SD card
  if (!SD_MMC.begin()) {
    Serial.println("SD card failed");
    while (true);
  }

  if (!SD_MMC.exists("/bioflok")) {
    SD_MMC.mkdir("/bioflok");
  }

  Serial.println("System ready");
}

void loop() {
  if (!client.connected()) reconnectMQTT();
  client.loop();

  if (millis() - lastMsg > interval) {
    lastMsg = millis();
    captureAndPublish();
  }
}

void manageStorage() {
  File root = SD_MMC.open("/bioflok");
  if (!root || !root.isDirectory()) {
    Serial.println("Failed to open directory");
    return;
  }

  String oldestFile = "";
  uint32_t oldestTime = 0xFFFFFFFF;
  
  File file = root.openNextFile();
  int fileCount = 0;
  
  while (file) {
    fileCount++;
    time_t fileTime = file.getLastWrite();
    if (fileTime < oldestTime) {
      oldestTime = fileTime;
      oldestFile = String("/bioflok/") + file.name();
    }
    file = root.openNextFile();
  }
  
  root.close();
  
  if (fileCount >= MAX_FILES && oldestFile != "") {
    Serial.print("Storage full. Deleting oldest file: ");
    Serial.println(oldestFile);
    SD_MMC.remove(oldestFile);
  }
}

void saveAsBMP(camera_fb_t *fb) {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)) {
    Serial.println("Failed to get time");
    return;
  }

  char filename[40];
  strftime(filename, sizeof(filename), "/bioflok/%Y-%m-%d_%H-%M-%S.bmp", &timeinfo);
  
  File file = SD_MMC.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to create BMP file");
    return;
  }

  // Prepare BMP header
  BMPHeader header;
  header.signature = 0x4D42; // 'BM'
  header.fileSize = sizeof(BMPHeader) + (fb->width * fb->height * 3);
  header.reserved = 0;
  header.dataOffset = sizeof(BMPHeader);
  header.headerSize = 40;
  header.width = fb->width;
  header.height = -fb->height; // Negative for top-down image
  header.planes = 1;
  header.bitsPerPixel = 24;
  header.compression = 0;
  header.imageSize = fb->width * fb->height * 3;
  header.xPixelsPerMeter = 0;
  header.yPixelsPerMeter = 0;
  header.colorsUsed = 0;
  header.importantColors = 0;

  // Write header
  file.write((uint8_t*)&header, sizeof(BMPHeader));

  // Convert and write image data (RGB565 to RGB888)
  for (int y = 0; y < fb->height; y++) {
    for (int x = 0; x < fb->width; x++) {
      int pos = (y * fb->width + x) * 2;
      uint16_t pixel = (fb->buf[pos] << 8) | fb->buf[pos + 1];
      
      // Convert RGB565 to RGB888
      uint8_t bmpPixel[3];
      bmpPixel[2] = ((pixel >> 11) & 0x1F) << 3; // R
      bmpPixel[1] = ((pixel >> 5) & 0x3F) << 2;  // G
      bmpPixel[0] = (pixel & 0x1F) << 3;         // B
      
      file.write(bmpPixel, 3);
    }
  }

  file.close();
  Serial.print("Saved BMP: ");
  Serial.println(filename);
}

// Function to send data via ESP-NOW
void kirimDataESPNow(uint8_t r, uint8_t g, uint8_t b, float mcci) {
  struct tm timeinfo;
  char waktu[20];
  
  if (!getLocalTime(&timeinfo)) {
    strcpy(waktu, "unknown");
  } else {
    strftime(waktu, sizeof(waktu), "%Y-%m-%d %H:%M:%S", &timeinfo);
  }
  
  // Format: farmID;type;timestamp;R;G;B;MCCI
  char payload[100];
  sprintf(payload, "11;A;%s;%d;%d;%d;%.2f", 
          waktu, r, g, b, mcci);
  
  // Print to serial monitor in the required format
  Serial.println(payload);
  
  // Send the message via ESP-NOW
  esp_err_t result = esp_now_send(peerAddress, (uint8_t *)payload, strlen(payload));
  
  if (result != ESP_OK) {
    Serial.println("Error sending the data");
  }
}

void captureAndPublish() {
  // Capture image with flash
  digitalWrite(FLASH_LED_PIN, HIGH);
  delay(200);
  camera_fb_t *fb = esp_camera_fb_get();
  digitalWrite(FLASH_LED_PIN, LOW);

  if (!fb || fb->format != PIXFORMAT_RGB565) {
    Serial.println("Failed to capture RGB565 image");
    if (fb) esp_camera_fb_return(fb);
    return;
  }

  // Analyze center portion of image (100x100 pixels in the center)
  const int sampleSize = 100;
  int startX = (fb->width - sampleSize) / 2;
  int startY = (fb->height - sampleSize) / 2;
  
  // Ensure we don't go out of bounds
  if (startX < 0) startX = 0;
  if (startY < 0) startY = 0;
  int endX = startX + sampleSize;
  int endY = startY + sampleSize;
  if (endX > fb->width) endX = fb->width;
  if (endY > fb->height) endY = fb->height;

  uint32_t sumR = 0, sumG = 0, sumB = 0;
  uint32_t pixelCount = 0;

  // Sample every 4th pixel for speed
  for (int y = startY; y < endY; y += 2) {
    for (int x = startX; x < endX; x += 2) {
      int pos = (y * fb->width + x) * 2;
      if (pos + 1 >= fb->len) continue;
      
      uint16_t pixel = (fb->buf[pos] << 8) | fb->buf[pos + 1];
      
      // Extract RGB components (5-6-5 format)
      uint8_t r5 = (pixel >> 11) & 0x1F;
      uint8_t g6 = (pixel >> 5) & 0x3F;
      uint8_t b5 = pixel & 0x1F;
      
      // Convert to 8-bit values
      uint8_t r = (r5 * 527 + 23) >> 6;  // More accurate conversion
      uint8_t g = (g6 * 259 + 33) >> 6;  // than simple bit shifting
      uint8_t b = (b5 * 527 + 23) >> 6;
      
      sumR += r;
      sumG += g;
      sumB += b;
      pixelCount++;
    }
  }

  if (pixelCount == 0) {
    Serial.println("Error: No pixels processed");
    esp_camera_fb_return(fb);
    return;
  }

  // Calculate averages
  uint8_t avgR = sumR / pixelCount;
  uint8_t avgG = sumG / pixelCount;
  uint8_t avgB = sumB / pixelCount;
  
  // Calculate MCCI with improved formula
  float mcci = 0.0;
  if (avgG > avgB) {  // Only calculate if G > B
    mcci = (float)(avgR - avgB) / (float)(avgG - avgB);
    // Clamp the value between 0 and 15
    mcci = constrain(mcci, 0.0, 15.0);
  }

  // Print results
  Serial.println("--- COLOR ANALYSIS ---");
  Serial.printf("RGB: %d, %d, %d\n", avgR, avgG, avgB);
  Serial.printf("MCCI: %.3f\n", mcci);

  // Publish to MQTT
  StaticJsonDocument<200> doc;
  doc["R"] = avgR;
  doc["G"] = avgG;
  doc["B"] = avgB;
  doc["MCCI"] = mcci;
  char output[200];
  serializeJson(doc, output);
  client.publish(mqttTopic, output);
  
  // Send data via ESP-NOW
  kirimDataESPNow(avgR, avgG, avgB, mcci);

  // Manage storage and save image
  manageStorage();
  saveAsBMP(fb);
  esp_camera_fb_return(fb);
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Reconnecting MQTT...");
    if (client.connect("ESP32CAMClient")) {
      Serial.println("Connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.println(client.state());
      delay(5000);
    }
  }
}