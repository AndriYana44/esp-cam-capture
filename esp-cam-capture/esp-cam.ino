#include <WiFi.h>
#include <HTTPClient.h>
#include <DHT.h>
#include <ESPAsyncWebServer.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"

const char* ssid[2] = {"BP", "vivo 2019"};
const char* pass[2] = {"1sampai8", "12345678an"};
//const char* ssid = "BP";
//const char* password = "1sampai8";

String serverName = "192.168.43.150";   

String serverPath = "/upload";     // The default serverPath should be upload.php

WiFiServer server(80);
const int serverPort = 80;

WiFiClient client;

// CAMERA_MODEL_AI_THINKER
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

#define RELAYPIN_1        14
#define RELAYPIN_2        15

// DHT INITIALIZE
DHT dht(13, DHT22);

const int timerInterval = 30000;    // time between each HTTP POST image
unsigned long previousMillis = 0;   // last time image was sent

TaskHandle_t Task1;

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  Serial.begin(115200);

  // DHT Initialize
  dht.begin(); 

  // RELAY
  pinMode(RELAYPIN_1, OUTPUT);
  pinMode(RELAYPIN_2, OUTPUT);
  digitalWrite(RELAYPIN_1, LOW);
  digitalWrite(RELAYPIN_2, HIGH);

  for(short i=0; i<2; i++) {
    Serial.println();
    Serial.print("Connecting to ");
    Serial.print("SSID: ");
    Serial.println(ssid[i]);
    Serial.print("Password: ");
    Serial.println(pass[i]);

    WiFi.begin(ssid[i], pass[i]);
    int WiFi_timeout_count = 0;
    while (WiFi.status() != WL_CONNECTED && WiFi_timeout_count<30) { //waiting 10 sec
      delay(500);
      Serial.print(".");
      ++WiFi_timeout_count;
    }
    
    if(WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi Connected!");
      Serial.println(WiFi.localIP());
      break;
    }
  }
  
  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }
  
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */         /* pin task to core 1 */
}

//Task1code: blinks an LED every 700 ms
void Task1code( void * pvParameters ){
  for(;;){
    sendDHT();
    delay(500);
  }
}


void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= timerInterval) {
    // capture
    sendPhoto();
    previousMillis = currentMillis;
  }
//
//  sendDHT();
}

String sendPhoto() {
  String getAll;
  String getBody;

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
  }
  
  if (client.connect(serverName.c_str(), serverPort)) {
    Serial.println("Connection successful!");   
    String head = "--DeteksiKematanganPisang\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--DeteksiKematanganPisang--\r\n";

    uint32_t imageLen = fb->len;
    uint32_t extraLen = head.length() + tail.length();
    uint32_t totalLen = imageLen + extraLen;
  
    client.println("POST " + serverPath + " HTTP/1.1");
    client.println("Host: " + serverName);
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=DeteksiKematanganPisang");
    client.println();
    client.print(head);
  
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0; n<fbLen; n=n+1024) {
      if (n+1024 < fbLen) {
        client.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        client.write(fbBuf, remainder);
      }
    }
    client.print(tail);
    
    esp_camera_fb_return(fb);
    
    int timoutTimer = 10000;
    long startTimer = millis();
    boolean state = false;
    
    while ((startTimer + timoutTimer) > millis()) {
      Serial.print(".");
      delay(100);      
      while (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (getAll.length()==0) { state=true; }
          getAll = "";
        }
        else if (c != '\r') { getAll += String(c); }
        if (state==true) { getBody += String(c); }
        startTimer = millis();
      }
      if (getBody.length()>0) { break; }
    }
    Serial.println();
    client.stop();
    Serial.println(getBody);
  }else {
    getBody = "Connection to " + serverName +  " failed.";
    Serial.println(getBody);
  }
  return getBody;
}

void sendDHT() {
  float kelembaban = dht.readHumidity();
  float suhu = dht.readTemperature();
  Serial.print("kelembaban: ");
  Serial.print(kelembaban);
  Serial.print(" ");
  Serial.print("suhu: ");
  Serial.println(suhu);
  if(!isnan(kelembaban) && !isnan(suhu)) {
    
    String url = "http://192.168.43.150/suhu?suhu=";
    url += suhu;
    
    if(WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      http.begin(url.c_str());
      // send HTTP GET request
      int httpResponseCode = http.GET();
      
      if (httpResponseCode>0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        Serial.println(payload);
      }
      http.end();
       
      Serial.println(suhu > 30);
      Serial.println(url);
      if(suhu > 30) {
        digitalWrite(RELAYPIN_1, HIGH);
        digitalWrite(RELAYPIN_2, LOW);
      }else {
        digitalWrite(RELAYPIN_1, LOW); 
        digitalWrite(RELAYPIN_2, HIGH);
      }
    }
  }
}
