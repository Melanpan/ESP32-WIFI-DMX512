#include <Arduino.h>
#include <ESPAsyncE131.h>
#include <ArtnetWiFi.h>
#include <esp_dmx.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <config.h>
#include <PubSubClient.h>
#include <FastLED.h>

const dmx_port_t dmx_num = DMX_NUM_1;
unsigned long previousMillis = 0;

#if ENABLE_LED
CRGB led[1];
#endif

#if ENABLE_E131
ESPAsyncE131 e131(UNIVERSE_COUNT);
#endif
#if ENABLE_ARTNET
ArtnetWiFiReceiver artnet;
#endif
#if ENABLE_MQTT
WiFiClient espClient;
PubSubClient mqttClient(espClient);
#endif

void writeDMXData(const uint8_t* data) {
   dmx_write_packet(dmx_num, data, DMX_MAX_PACKET_SIZE);
   dmx_send_packet(dmx_num, DMX_MAX_PACKET_SIZE);
   dmx_wait_send_done(dmx_num, DMX_PACKET_TIMEOUT_TICK);
}

void resetDMX() {
   /* Send out 512 bytes of zeros, resetting all devices on the chain. */
   byte clearBuffer[512];
   digitalWrite(2, HIGH);
   writeDMXData(clearBuffer);
   digitalWrite(2, LOW); 
}

#if ENABLE_ARTNET
void artnetCallback(const uint8_t* data, const uint16_t size) {
   digitalWrite(2, HIGH);
   led[0] = CRGB::Blue; FastLED.show();
   writeDMXData(data);
   digitalWrite(2, LOW);
   led[0] = CRGB::Black; FastLED.show();
}
#endif

void setupDMX() {
    const dmx_config_t config = DMX_DEFAULT_CONFIG;
    dmx_param_config(dmx_num, &config);

    const int tx_io_num = 17, rx_io_num = 16, rts_io_num = 21; //TODO define
    dmx_set_pin(dmx_num, tx_io_num, rx_io_num, rts_io_num);

    QueueHandle_t dmx_queue;
    dmx_driver_install(dmx_num, DMX_MAX_PACKET_SIZE, 10, &dmx_queue, ESP_INTR_FLAG_IRAM);
    dmx_set_mode(dmx_num, DMX_MODE_WRITE);
}

#if ARDUINO_OTA
void setupOTA() {
  ArduinoOTA.setHostname("esp-dmx-node");
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";
      Serial.println("[OTA] Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("[OTA] Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
}
#endif

void setupWifi() {
    WiFi.mode(WIFI_STA);

    if (passphrase != NULL)
        WiFi.begin(ssid, passphrase);
    else
        WiFi.begin(ssid);

    while (WiFi.status() != WL_CONNECTED) {
        led[0] = CRGB::Orange; FastLED.show();
        delay(500);
        led[0] = CRGB::Black; FastLED.show();
        Serial.print(".");
    }

    Serial.println("");
    Serial.print(F("connected to "));
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

#if ENABLE_MQTT
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  if (strcmp(topic, "esp-dmx/dmx/reset") == 0) {
    Serial.println("Received a MQTT dmx reset.");
    resetDMX();
  }

}

void mqttReport() {
    String address = WiFi.localIP().toString();
    String clockspeed = String(getCpuFrequencyMhz());
		mqttClient.publish("esp-dmx/ip", address.c_str());
    mqttClient.publish("esp-dmx/clockspeed", clockspeed.c_str());
}

void mqttReconnect() {
    String clientId = "esp-dmx-node-";
    clientId += String(random(0xffff), HEX);

    if (mqttClient.connect(clientId.c_str())) {
       Serial.println("Connected to MQTT.");
       mqttClient.subscribe("esp-dmx/#");
       mqttReport();
       led[0] = CRGB::Green; FastLED.show();
    } else {
        led[0] = CRGB::Red; FastLED.show();
        Serial.print("MQTT connection failed, rc=");
        Serial.println(mqttClient.state());
    }
}
#endif

void setup() {
    setCpuFrequencyMhz(240);
    Serial.begin(115200);
    pinMode(2,OUTPUT);

    #if ENABLE_LED
    FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(led, 1);
    led[0] = CRGB::Red;  FastLED.show();
    #endif

    // Setup wifi
    setupWifi();
    
    #if ARDUINO_OTA
    // Setup OTA
    setupOTA();
    #endif

    // Setup e131 and artnet
    #if ENABLE_E131
    e131.begin(E131_MULTICAST, UNIVERSE, UNIVERSE_COUNT);
    #endif
    
    #if ENABLE_ARTNET
    artnet.begin();
    artnet.subscribe(0, artnetCallback);
    #endif
    
    // Setup DMX output
    setupDMX();
    resetDMX();

    // Setup MQTT
    #if ENABLE_MQTT
    mqttClient.setServer(mqtt_server, 1883);
    mqttClient.setCallback(mqttCallback);
    mqttReconnect();
    #endif

    Serial.println("- Ready");
    delay(50);
    led[0] = CRGB::Black; FastLED.show();
}

void loop() {
    unsigned long currentMillis = millis();
    
    #if ENABLE_ARTNET
    artnet.parse();
    #endif

    #if ENABLE_MQTT
    mqttClient.loop();
    #endif

    #if ENABLE_E131
    if (!e131.isEmpty()) {
        led[0] = CRGB::Blue; FastLED.show();
        digitalWrite(2, HIGH);
        
        e131_packet_t packet;
        e131.pull(&packet);
        writeDMXData(packet.property_values);
        digitalWrite(2, LOW);
        led[0] = CRGB::Black; FastLED.show();
    }
    #endif

    // Do some checks every 5 seconds
    if (currentMillis - previousMillis >= 5000) {
        if (WiFi.status() != WL_CONNECTED) {
            led[0] = CRGB::Orange; FastLED.show();
            Serial.println("Reconnecting to wifi.");
            WiFi.disconnect();
            WiFi.reconnect();
        }
        
        #if ENABLE_MQTT
        if (!mqttClient.connected()) {
          led[0] = CRGB::Orange; FastLED.show();
            Serial.println("Reconnecting to MQTT");
            mqttReconnect();
            led[0] = CRGB::Black; FastLED.show();
        }
        #endif

        previousMillis = currentMillis;
    }
    

}