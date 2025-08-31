#include <Arduino.h>
#include "Globalku.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>  // For saving settings persistently
#include <PubSubClient.h> // MQTT Library
#include <ElegantOTA.h>
#include <ModbusMaster.h>
#define RXD2 33
#define TXD2 32


// Wi-Fi Access Point credentials
const char *ap_ssid = "ESP32_Configuration";
const char *ap_password = "12345678";
String ssid;
String password;
int ip_local1;   // Default IP
int ip_local2;   // Default IP
int ip_local3;   // Default IP
int ip_local4;   // Default IP
int ip_gateway1; // Default Gateway
int ip_gateway2; // Default Gateway
int ip_gateway3; // Default Gateway
int ip_gateway4; // Default Gateway
int netmask1;    // Default Netmask/Subnet
int netmask2;    // Default Netmask/Subnet
int netmask3;    // Default Netmask/Subnet
int netmask4;    // Default Netmask/Subnet
int primDNS1;    // Default PrimaryDNS
int primDNS2;    // Default PrimaryDNS
int primDNS3;    // Default PrimaryDNS
int primDNS4;    // Default PrimaryDNS
// slave 1
uint16_t modbus_address0;
uint16_t modbus_address1;
uint16_t modbus_address2;
uint16_t modbus_address3;
uint16_t modbus_address4;
uint16_t modbus_address5;
String type_data0 = "UINT16";
String type_data1 = "UINT16";
String type_data2 = "UINT16";
String type_data3 = "UINT16";
String type_data4 = "UINT16";
String type_data5 = "UINT16";
// slave 2
uint16_t modbus_address01;
uint16_t modbus_address11;
uint16_t modbus_address21;
uint16_t modbus_address31;
String type_data01 = "UINT16";
String type_data11 = "UINT16";
String type_data21 = "UINT16";
String type_data31 = "UINT16";
// komparasi suhu
String komSuhu = ">";
String komHum = ">";
String komLux = ">";
int Suhu;
int Hum;
int Lux;

IPAddress ip_localMikon(ip_local1, ip_local2, ip_local3, ip_local4);
IPAddress ip_gatwayMikon(ip_gateway1, ip_gateway2, ip_gateway3, ip_gateway4);
IPAddress netmask_mikon(netmask1, netmask2, netmask3, netmask4);
IPAddress primary_DNS(primDNS1, primDNS2, primDNS3, primDNS4);
unsigned long cyclesReconnect;
unsigned long ota_progress_millis = 0;

// MQTT Settings (placeholders, will be updated from web form)
String mqtt_server = "";
int mqtt_port = 1883;
String mqtt_username = "";
String mqtt_password = "";
String topic_publish = "";

// Modbus Settings
//slave id 1
String modbus_type = "RTU"; // RTU or TCP
int slave_id = 1;           // Default Slave ID
int baud_rate = 9600;
int data_bits = 8;
int stop_bits = 1;
String modbus_parity = "none"; // "none", "even", "odd"

//slave id 2
String modbus_type1 = "RTU"; // RTU or TCP
int slave_id_sensor = 2;           // Default Slave ID
int baud_rate1 = 9600;
int data_bits1 = 8;
int stop_bits1 = 1;
String modbus_parity1 = "none"; // "none", "even", "odd"

int host_tcp1 = 0;             // IPAddress ip_staticz(192, 168, 100, 23)
int host_tcp2 = 0;
int host_tcp3 = 0;
int host_tcp4 = 0;
int modbus_port = 502;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
Preferences preferences;

// Modbus Objects
ModbusMaster node;
ModbusMaster nodeSensor;
unsigned long old_time, time_input;

void webServer();
void readConfig();
void wifiAP();
void connectToMQTT();
uint32_t modbusParity(int dataBits, String parity, int stopsBits);
void onOTAStart();
void onOTAProgress(size_t current, size_t final);
void onOTAEnd(bool success);
void init_modbus_relay(int baudrate, uint8_t slaveID, uint32_t paritySerial);
void init_modbus_sensor(int baudrate, uint8_t slaveID, uint32_t paritySerial);
uint16_t read_CoilsRegister(uint16_t u16ReadAddress, uint16_t u16BitQty);
void write_coilRegister(bool state, uint16_t u16WriteAddress);
void read_DiscreteRegister(String nameVariable, uint16_t u16ReadAddress, uint16_t u16BitQty, const char *topicMqtt);
void read_HoldingRegister(uint16_t u16ReadAddress, uint16_t u16ReadQty, uint16_t , const char *topicMqtt);
void read_HoldingRegister(String nameVariable, uint16_t u16ReadAddress, uint16_t u16ReadQty, int16_t typeDataModbus, const char *topicMqtt);
void read_HoldingRegister(String nameVariable, uint16_t u16ReadAddress, uint16_t u16ReadQty, uint32_t typeDataModbus, const char *topicMqtt);
void read_HoldingRegister(String nameVariable, uint16_t u16ReadAddress, uint16_t u16ReadQty, int32_t typeDataModbus, const char *topicMqtt);
void read_HoldingRegister(String nameVariable, uint16_t u16ReadAddress, uint16_t u16ReadQty, float typeDataModbus, const char *topicMqtt);
void read_InputRegister(String nameVariable, uint16_t u16ReadAddress, uint16_t u16ReadQty, const char *topicMqtt);

uint16_t read_HoldingRegister(uint16_t u16ReadAddress, uint16_t u16ReadQty, const char *topicMqtt);
void komparasi(String kom_suhu, String kom_hum, uint16_t dataSuhu, uint16_t dataHum, float temp, float humidity);

void setup()
{
  // Initialize Serial Monitor
  Serial.begin(9600);

  readConfig();

  webServer();

  init_modbus_relay(baud_rate, slave_id, modbusParity(data_bits, modbus_parity, stop_bits));
  init_modbus_sensor(baud_rate1, slave_id_sensor, modbusParity(data_bits1, modbus_parity1, stop_bits1));
  mqttClient.setServer(mqtt_server.c_str(), mqtt_port);
}

void loop()
{
  uint16_t dataModus0 = 0;
  uint16_t dataModus1 = 0;
  uint16_t dataModus2 = 0;
  uint16_t dataModus3 = 0;
  uint32_t dataModus4;

  ElegantOTA.loop();
  if ((millis() - cyclesReconnect) > 5000)
  {
    if (!mqttClient.connected())
    {
      // Try to connect to MQTT broker
      connectToMQTT();
    }
    // // Keep MQTT connection alive
    mqttClient.loop();
    cyclesReconnect = millis();
  }

  unsigned long old_time1 = 0;
  unsigned long old_time2 = 0;
  unsigned long old_time3 = 0;
  unsigned long old_time4 = 0;

  if ((millis() - old_time1) >= 250) {
    dataModus0 = read_HoldingRegister(modbus_address01, 1, topic_publish.c_str());
    old_time1 = millis();
  }

  if ((millis() - old_time2) >= 250) {
    dataModus1 = read_HoldingRegister(modbus_address11, 1, topic_publish.c_str());
    old_time2 = millis();
  }

  if ((millis() - old_time3) >= 250) {
    dataModus2 = read_HoldingRegister(modbus_address21, 1, topic_publish.c_str());
    old_time3 = millis();
  }

  if ((millis() - old_time4) >= 250) {
    dataModus3 = read_HoldingRegister(modbus_address31, 1, topic_publish.c_str());
    old_time4 = millis();
  }

  komparasi(komSuhu, komHum, dataModus1, dataModus0, Suhu, Hum);
  Serial.println(topic_publish);
  delay(1000);
}

void wifiAP()
{
  // Assign the IPAddress values dynamically
  IPAddress ip_localMikon(ip_local1, ip_local2, ip_local3, ip_local4);
  IPAddress ip_gatwayMikon(ip_gateway1, ip_gateway2, ip_gateway3, ip_gateway4);
  IPAddress netmask_mikon(netmask1, netmask2, netmask3, netmask4);
  IPAddress primary_DNS(primDNS1, primDNS2, primDNS3, primDNS4);
  // Configure static IP
  if (!WiFi.config(ip_localMikon, ip_gatwayMikon, netmask_mikon, primary_DNS))
  {
    Serial.println("Failed to configure Static IP");
  }
  else
  {
    Serial.println("Static IP configured successfully.");
  }
  // // Debug output
  // Serial.println("Connect to WiFi " + String(ssid));
  // Serial.println("                " + String(password));
  // Serial.println("Configuring static IP...");
  // Serial.print("Local IP: ");
  // Serial.println(ip_localMikon);
  // Serial.print("Gateway: ");
  // Serial.println(ip_gatwayMikon);
  // Serial.print("Netmask: ");
  // Serial.println(netmask_mikon);
  // Serial.print("DNS: ");
  // Serial.println(primary_DNS);

  // Start in Station mode
  WiFi.mode(WIFI_AP_STA);
  // Configure static IP
  if (!WiFi.config(ip_localMikon, ip_gatwayMikon, netmask_mikon, primary_DNS))
  {
    Serial.println("Failed to configure Static IP");
  }
  else
  {
    Serial.println("Static IP configured successfully.");
  }

  // Connect to Wi-Fi as a client (STA)
  WiFi.begin(ssid.c_str(), password.c_str());
  Serial.println("Connecting to STA...");
  if (WiFi.status() != WL_CONNECTED)
  {
    for (size_t i = 0; i < 20; i++)
    {
      delay(500);
      Serial.print(".");
      if (WiFi.status() == WL_CONNECTED)
      {
        break;
      }
    }
  }
  Serial.println("\nSTA Connected!");
  Serial.print("STA IP Address: ");
  Serial.println(WiFi.localIP());

  // Start Wi-Fi as Access Point
  WiFi.softAP(ap_ssid, ap_password);
  Serial.println("Access Point started!");
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());
}

void webServer()
{
  wifiAP();

  // Serve configuration form
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/html", baseWebHtml); });

  // Handle form submission
  server.on("/saveConfig", HTTP_POST, [](AsyncWebServerRequest *request)
            {
    ssid = request->getParam("ssid", true)->value();
    password = request->getParam("password", true)->value();
    ip_local1 = request->getParam("ip_local1", true)->value().toInt();
    ip_local2 = request->getParam("ip_local2", true)->value().toInt();
    ip_local3 = request->getParam("ip_local3", true)->value().toInt();
    ip_local4 = request->getParam("ip_local4", true)->value().toInt();
    ip_gateway1 = request->getParam("ip_gateway1", true)->value().toInt();
    ip_gateway2 = request->getParam("ip_gateway2", true)->value().toInt();
    ip_gateway3 = request->getParam("ip_gateway3", true)->value().toInt();
    ip_gateway4 = request->getParam("ip_gateway4", true)->value().toInt();
    netmask1 = request->getParam("netmask1", true)->value().toInt();
    netmask2 = request->getParam("netmask2", true)->value().toInt();
    netmask3 = request->getParam("netmask3", true)->value().toInt();
    netmask4 = request->getParam("netmask4", true)->value().toInt();
    primDNS1 = request->getParam("primDNS1", true)->value().toInt();
    primDNS2 = request->getParam("primDNS2", true)->value().toInt();
    primDNS3 = request->getParam("primDNS3", true)->value().toInt();
    primDNS4 = request->getParam("primDNS4", true)->value().toInt();
    // String ip = request->hasParam("ip", true) ? request->getParam("ip", true)->value() : "";
    // String subnet = request->hasParam("subnet", true) ? request->getParam("subnet", true)->value() : "";
    // String gateway = request->hasParam("gateway", true) ? request->getParam("gateway", true)->value() : "";
    // mqtt_server = request->getParam("mqtt_server", true)->value();
    // mqtt_port = request->hasParam("mqtt_port", true) ? request->getParam("mqtt_port", true)->value().toInt() : 1883;
    // mqtt_username = request->hasParam("mqtt_username", true) ? request->getParam("mqtt_username", true)->value() : "";
    // mqtt_password = request->hasParam("mqtt_password", true) ? request->getParam("mqtt_password", true)->value() : "";
    mqtt_server = request->getParam("mqtt_server", true)->value();
    mqtt_port = request->getParam("mqtt_port", true)->value().toInt();
    mqtt_username = request->getParam("mqtt_username", true)->value();
    mqtt_password = request->getParam("mqtt_password", true)->value();
    topic_publish = request->getParam("topic_publish", true)->value();
    //slave id 1
    modbus_type = request->getParam("modbus_type", true)->value();
    slave_id = request->getParam("slave_id", true)->value().toInt();
    baud_rate = request->getParam("baud_rate", true)->value().toInt();
    data_bits = request->getParam("data_bits", true)->value().toInt();
    stop_bits = request->getParam("stop_bits", true)->value().toInt();
    modbus_parity = request->getParam("modbus_parity", true)->value();
    // slave 2
    modbus_type1 = request->getParam("modbus_type1", true)->value();
    slave_id_sensor = request->getParam("slave_id_sensor", true)->value().toInt();
    baud_rate1 = request->getParam("baud_rate1", true)->value().toInt();
    data_bits1 = request->getParam("data_bits1", true)->value().toInt();
    stop_bits1 = request->getParam("stop_bits1", true)->value().toInt();
    modbus_parity1 = request->getParam("modbus_parity1", true)->value();
    
    // host_tcp1 = request->getParam("host_tcp1", true)->value().toInt();
    // host_tcp2 = request->getParam("host_tcp2", true)->value().toInt();
    // host_tcp3 = request->getParam("host_tcp3", true)->value().toInt();
    // host_tcp4 = request->getParam("host_tcp4", true)->value().toInt();
    // modbus_port = request->getParam("modbus_port", true)->value().toInt();

    modbus_address0 = request->getParam("modbus_address0", true)->value().toInt();
    modbus_address1 = request->getParam("modbus_address1", true)->value().toInt();
    modbus_address2 = request->getParam("modbus_address2", true)->value().toInt();
    modbus_address3 = request->getParam("modbus_address3", true)->value().toInt();
    // modbus_address4 = request->getParam("modbus_address4", true)->value().toInt();
    // modbus_address5 = request->getParam("modbus_address5", true)->value().toInt();

    modbus_address01 = request->getParam("modbus_address01", true)->value().toInt();
    modbus_address11 = request->getParam("modbus_address11", true)->value().toInt();
    modbus_address21 = request->getParam("modbus_address21", true)->value().toInt();
    modbus_address31 = request->getParam("modbus_address31", true)->value().toInt();

    type_data0 = request->getParam("type_data0", true)->value();
    type_data1 = request->getParam("type_data1", true)->value();
    type_data2 = request->getParam("type_data2", true)->value();
    type_data3 = request->getParam("type_data3", true)->value();
    // type_data4 = request->getParam("type_data4", true)->value();
    // type_data5 = request->getParam("type_data5", true)->value();

    type_data01 = request->getParam("type_data01", true)->value();
    type_data11 = request->getParam("type_data11", true)->value();
    type_data21 = request->getParam("type_data21", true)->value();
    type_data31 = request->getParam("type_data31", true)->value();

    Suhu = request->getParam("Suhu", true)->value().toInt();
    Hum = request->getParam("Hum", true)->value().toInt();
    Lux = request->getParam("Lux", true)->value().toInt();
    komSuhu = request->getParam("komSuhu", true)->value();
    komHum = request->getParam("komHum", true)->value();
    komLux = request->getParam("komLux", true)->value();

    // Print received data to Serial Monitor
    Serial.println("Configuration Received:");
    Serial.println("SSID: " + ssid);
    Serial.println("Password: " + password);
    Serial.println("IPAddress: " + String(ip_local1) + "." + String(ip_local2) + "." + String(ip_local3) + "." + String(ip_local4));
    Serial.println("Gateway: " + String(ip_gateway1) + "." + String(ip_gateway2) + "." + String(ip_gateway3) + "." + String(ip_gateway4));
    Serial.println("Netmask: " + String(netmask1) + "." + String(netmask2) + "." + String(netmask3) + "." + String(netmask4));
    Serial.println("Primary DNS: " + String(primDNS1) + "." + String(primDNS2) + "." + String(primDNS3) + "." + String(primDNS4));
    Serial.println("MQTT Server: " +  mqtt_server);
    Serial.println("MQTT Port: " + String(mqtt_port));
    Serial.println("MQTT Username: " + mqtt_username);
    Serial.println("MQTT Password: " + mqtt_password);
    Serial.println("Modbus Type: " + modbus_type);
    Serial.println("Modbus SlaveID: " + String(slave_id));
    Serial.println("Modbus Baudrate: " + String(baud_rate));
    Serial.println("Modbus Databits: " + String(data_bits));
    Serial.println("Modbus Stopbits: " + String(stop_bits));
    Serial.println("Modbus Parity: " + modbus_parity);

    // Save configuration to Preferences
    preferences.begin("WiFiCreds", false);
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    preferences.putInt("ip_local1", ip_local1);
    preferences.putInt("ip_local2", ip_local2);
    preferences.putInt("ip_local3", ip_local3);
    preferences.putInt("ip_local4", ip_local4);
    preferences.putInt("ip_gateway1", ip_gateway1);
    preferences.putInt("ip_gateway2", ip_gateway2);
    preferences.putInt("ip_gateway3", ip_gateway3);
    preferences.putInt("ip_gateway4", ip_gateway4);
    preferences.putInt("netmask1", netmask1);
    preferences.putInt("netmask2", netmask2);
    preferences.putInt("netmask3", netmask3);
    preferences.putInt("netmask4", netmask4);
    preferences.putInt("primDNS1", primDNS1);
    preferences.putInt("primDNS2", primDNS2);
    preferences.putInt("primDNS3", primDNS3);
    preferences.putInt("primDNS4", primDNS4);
    preferences.putString("mqtt_server", mqtt_server);
    preferences.putInt("mqtt_port", mqtt_port);
    preferences.putString("mqtt_username", mqtt_username);
    preferences.putString("mqtt_password", mqtt_password);
    preferences.putString("topic_publish", topic_publish);
    // slave 1
    preferences.putString("modbus_type", modbus_type);
    preferences.putInt("slave_id", slave_id);
    preferences.putInt("baud_rate", baud_rate);
    preferences.putInt("data_bits", data_bits);
    preferences.putInt("stop_bits", stop_bits);
    preferences.putString("modbus_parity", modbus_parity);
    // slave 2
    preferences.putString("modbus_type1", modbus_type1);
    preferences.putInt("slave_id_sensor", slave_id_sensor);
    preferences.putInt("baud_rate1", baud_rate1);
    preferences.putInt("data_bits1", data_bits1);
    preferences.putInt("stop_bits1", stop_bits1);
    preferences.putString("modbus_parity1", modbus_parity1);

    // preferences.putInt("host_tcp1", host_tcp1);
    // preferences.putInt("host_tcp2", host_tcp2);
    // preferences.putInt("host_tcp3", host_tcp3);
    // preferences.putInt("host_tcp4", host_tcp4);
    // preferences.putInt("modbus_port", modbus_port);

    preferences.putInt("modbus_address0", modbus_address0);
    preferences.putInt("modbus_address1", modbus_address1);
    preferences.putInt("modbus_address2", modbus_address2);
    preferences.putInt("modbus_address3", modbus_address3);
    preferences.putInt("modbus_address4", modbus_address4);
    preferences.putInt("modbus_address5", modbus_address5);

    preferences.putInt("modbus_address01", modbus_address01);
    preferences.putInt("modbus_address11", modbus_address11);
    preferences.putInt("modbus_address21", modbus_address21);
    preferences.putInt("modbus_address31", modbus_address31);

    preferences.putString("type_data0", type_data0);
    preferences.putString("type_data1", type_data1);
    preferences.putString("type_data2", type_data2);
    preferences.putString("type_data3", type_data3);
    preferences.putString("type_data4", type_data4);
    preferences.putString("type_data5", type_data5);

    preferences.putString("type_data01", type_data01);
    preferences.putString("type_data11", type_data11);
    preferences.putString("type_data21", type_data21);
    preferences.putString("type_data31", type_data31);

    preferences.putInt("Suhu", Suhu);
    preferences.putInt("Hum", Hum);
    preferences.putInt("Lux", Lux);
    preferences.putString("komSuhu", komSuhu);
    preferences.putString("komHum", komHum);
    preferences.putString("komLux", komLux);
    preferences.end();

    // Respond to client
    request->send(200, "text/html", "Configuration Saved! ESP32 will restart to apply changes.");

    // Restart ESP32 to apply configuration (optional)
    delay(3000);
    ESP.restart(); });

  ElegantOTA.begin(&server, "admin", "Together1!"); // Start ElegantOTA

  // Start the server
  server.begin();
}

void readConfig()
{
  // Initialize MQTT client with saved configuration
  preferences.begin("WiFiCreds", true);
  ssid = preferences.getString("ssid", "");
  password = preferences.getString("password", "");
  ip_local1 = preferences.getInt("ip_local1", ip_local1);
  ip_local2 = preferences.getInt("ip_local2", ip_local2);
  ip_local3 = preferences.getInt("ip_local3", ip_local3);
  ip_local4 = preferences.getInt("ip_local4", ip_local4);
  ip_gateway1 = preferences.getInt("ip_gateway1", ip_gateway1);
  ip_gateway2 = preferences.getInt("ip_gateway2", ip_gateway2);
  ip_gateway3 = preferences.getInt("ip_gateway3", ip_gateway3);
  ip_gateway4 = preferences.getInt("ip_gateway4", ip_gateway4);
  netmask1 = preferences.getInt("netmask1", netmask1);
  netmask2 = preferences.getInt("netmask2", netmask2);
  netmask3 = preferences.getInt("netmask3", netmask3);
  netmask4 = preferences.getInt("netmask4", netmask4);
  primDNS1 = preferences.getInt("primDNS1", primDNS1);
  primDNS2 = preferences.getInt("primDNS2", primDNS2);
  primDNS3 = preferences.getInt("primDNS3", primDNS3);
  primDNS4 = preferences.getInt("primDNS4", primDNS4);
  mqtt_server = preferences.getString("mqtt_server", "");
  mqtt_port = preferences.getInt("mqtt_port", 1883);
  mqtt_username = preferences.getString("mqtt_username", "");
  mqtt_password = preferences.getString("mqtt_password", "");
  topic_publish = preferences.getString("topic_publish", "");
   // slave 1
   modbus_type = preferences.getString("modbus_type", modbus_type);
   slave_id = preferences.getInt("slave_id", slave_id);
   baud_rate = preferences.getInt("baud_rate", baud_rate);
   data_bits = preferences.getInt("data_bits", data_bits);
   stop_bits = preferences.getInt("stop_bits", stop_bits);
   modbus_parity = preferences.getString("modbus_parity", modbus_parity);
   // slave 2
   modbus_type1 = preferences.getString("modbus_type", modbus_type);
   slave_id_sensor = preferences.getInt("slave_id_sensor", slave_id_sensor);
   baud_rate1 = preferences.getInt("baud_rate1", baud_rate1);
   data_bits1 = preferences.getInt("data_bits1", data_bits1);
   stop_bits1 = preferences.getInt("stop_bits1", stop_bits1);
   modbus_parity1 = preferences.getString("modbus_parity1", modbus_parity1);

  // host_tcp1 = preferences.getInt("host_tcp1", host_tcp1);
  // host_tcp2 = preferences.getInt("host_tcp2", host_tcp2);
  // host_tcp3 = preferences.getInt("host_tcp3", host_tcp3);
  // host_tcp4 = preferences.getInt("host_tcp4", host_tcp4);
  // modbus_port = preferences.getInt("modbus_port", modbus_port);
  
  modbus_address0 = preferences.getInt("modbus_address0", modbus_address0);
  modbus_address1 = preferences.getInt("modbus_address1", modbus_address1);
  modbus_address2 = preferences.getInt("modbus_address2", modbus_address2);
  modbus_address3 = preferences.getInt("modbus_address3", modbus_address3);
  modbus_address4 = preferences.getInt("modbus_address4", modbus_address4);
  modbus_address5 = preferences.getInt("modbus_address5", modbus_address5);

  modbus_address01 = preferences.getInt("modbus_address01", modbus_address01);
  modbus_address11 = preferences.getInt("modbus_address11", modbus_address11);
  modbus_address21 = preferences.getInt("modbus_address21", modbus_address21);
  modbus_address31 = preferences.getInt("modbus_address31", modbus_address31);

  type_data0 = preferences.getString("type_data0", type_data0);
  type_data1 = preferences.getString("type_data1", type_data1);
  type_data2 = preferences.getString("type_data2", type_data2);
  type_data3 = preferences.getString("type_data3", type_data3);
  type_data4 = preferences.getString("type_data3", type_data4);
  type_data5 = preferences.getString("type_data3", type_data5);

  type_data01 = preferences.getString("type_data0", type_data01);
  type_data11 = preferences.getString("type_data1", type_data11);
  type_data21 = preferences.getString("type_data2", type_data21);
  type_data31 = preferences.getString("type_data3", type_data31);

  Suhu = preferences.getInt("Suhu", Suhu);
  Hum = preferences.getInt("Hum", Hum);
  Lux = preferences.getInt("Lux", Lux);
  komSuhu = preferences.getString("komSuhu", komSuhu);
  komHum = preferences.getString("komHum", komHum);
  komLux = preferences.getString("komLux", komLux);
  preferences.end();
}

// void readConfig(String variablez, const char *keyz, String defaultValuez)
// {
//   variablez = preferences.getString(keyz, defaultValuez);
//   preferences.end();
// }

// void readConfig(int variablez, const char *keyz, int32_t defaultValuez)
// {
//   variablez = preferences.getInt(keyz, defaultValuez);
//   preferences.end();
// }

void connectToMQTT()
{
  if (!mqttClient.connected())
  {
    Serial.print("Connecting to MQTT...");

    // Generate client ID
    String clientId = "ESP32-" + String(random(0xffff), HEX);

    // Try to connect
    if (mqttClient.connect(clientId.c_str(), mqtt_username.c_str(), mqtt_password.c_str()))
    {
      Serial.println("Connected to MQTT!");
      String payload;
      if (modbus_type == "RTU")
      {
        payload = "{\"SSID\":\"" + ssid + "\",\"Password\":\"" + password + "\"" +
                  ",\"IPAddress\":" + "\"" + String(ip_local1) + "." + String(ip_local2) + "." + String(ip_local3) + "." + String(ip_local4) + "\"" +
                  ",\"Gateway\":" + "\"" + String(ip_gateway1) + "." + String(ip_gateway2) + "." + String(ip_gateway3) + "." + String(ip_gateway4) + "\"" +
                  ",\"Netmask\":" + "\"" + String(netmask1) + "." + String(netmask2) + "." + String(netmask3) + "." + String(netmask4) + "\"" +
                  ",\"Primary DNS\":" + "\"" + String(primDNS1) + "." + String(primDNS2) + "." + String(primDNS3) + "." + String(primDNS4) + "\"" +
                  ",\"MQTT Server\":\"" + mqtt_server + "\",\"MQTT Port\":\"" + String(mqtt_port) + "\",\"MQTT Username\":\"" + mqtt_username + "\",\"MQTT Password\":\"" + mqtt_password + "\"" +
                  ",\"Modbus Type\":\"" + modbus_type + "\",\"Modbus SlaveID\":\"" + String(slave_id) + "\",\"Modbus Baudrate\":\"" + String(baud_rate) + "\",\"Modbus Databits\":\"" + String(data_bits) + "\"" +
                  ",\"Modbus Stopbits\":\"" + String(stop_bits) + "\",\"Modbus Parity\":\"" + modbus_parity + "\"}";
      }
      else if (modbus_type == "TCP")
      {
        payload = "{\"SSID\":\"" + ssid + "\",\"Password\":\"" + password + "\"" +
                  ",\"IPAddress\":" + "\"" + String(ip_local1) + "." + String(ip_local2) + "." + String(ip_local3) + "." + String(ip_local4) + "\"" +
                  ",\"Gateway\":" + "\"" + String(ip_gateway1) + "." + String(ip_gateway2) + "." + String(ip_gateway3) + "." + String(ip_gateway4) + "\"" +
                  ",\"Netmask\":" + "\"" + String(netmask1) + "." + String(netmask2) + "." + String(netmask3) + "." + String(netmask4) + "\"" +
                  ",\"Primary DNS\":" + "\"" + String(primDNS1) + "." + String(primDNS2) + "." + String(primDNS3) + "." + String(primDNS4) + "\"" +
                  ",\"MQTT Server\":\"" + mqtt_server + "\",\"MQTT Port\":\"" + String(mqtt_port) + "\",\"MQTT Username\":\"" + mqtt_username + "\",\"MQTT Password\":\"" + mqtt_password + "\"" +
                  ",\"Modbus Type\":\"" + modbus_type + "\",\"Modbus SlaveID\":\"" + String(slave_id) + "\"" +
                  ",\"Modbus TCP/IP host\":\"" + String(host_tcp1) + "." + String(host_tcp2) + "." + String(host_tcp3) + "." + String(host_tcp4) + "\",\"Modbus Port\":\"" + modbus_port + "\"}";
      }
      mqttClient.publish("debugger", payload.c_str());
    }
    else
    {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds");
    }
  }
}

uint32_t modbusParity(int dataBits, String parity, int stopsBits)
{
  uint32_t modbuzParity;
  if (dataBits == 8)
  {
    if (parity == "even")
    {
      if (stopsBits == 1) {
        modbuzParity = SERIAL_8E1;
      }
      else if (stopsBits == 2)
      {
        modbuzParity = SERIAL_8E2;
      }
      
      // return SERIAL_8E1;
    }
    else if (parity == "odd")
    {
      if (stopsBits == 1) {
        modbuzParity = SERIAL_8O1;
      }
      else if (stopsBits == 2)
      {
        modbuzParity = SERIAL_8O2;
      }
      // return SERIAL_8O1;
    }
    else if (parity == "none")
    {
      if (stopsBits == 1) {
        modbuzParity = SERIAL_8N1;
      }
      else if (stopsBits == 2)
      {
        modbuzParity = SERIAL_8N2;
      }
      // return SERIAL_8N1;
    }
  }
  else if (dataBits == 7)
  {
    if (parity == "even")
    {
      if (stopsBits == 1) {
        modbuzParity = SERIAL_7E1;
      }
      else if (stopsBits == 2)
      {
        modbuzParity = SERIAL_7E2;
      }
      // return SERIAL_7E1;
    }
    else if (parity == "odd")
    {
      if (stopsBits == 1) {
        modbuzParity = SERIAL_7O1;
      }
      else if (stopsBits == 2)
      {
        modbuzParity = SERIAL_7O2;
      }
      // return SERIAL_7O1;
    }
    else if (parity == "none")
    {
      if (stopsBits == 1) {
        modbuzParity = SERIAL_7N1;
      }
      else if (stopsBits == 2)
      {
        modbuzParity = SERIAL_7N2;
      }
      // return SERIAL_7N1;
    }
  }
  return modbuzParity;
}

void onOTAStart()
{
  // Log when OTA has started
  Serial.println("OTA update started!");
  // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final)
{
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000)
  {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success)
{
  // Log when OTA has finished
  if (success)
  {
    Serial.println("OTA update finished successfully!");
  }
  else
  {
    Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
}

void init_modbus_relay(int baudrate, uint8_t slaveID, uint32_t paritySerial)
{
  Serial2.begin(baudrate, paritySerial, RXD2, TXD2);
  // Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  node.begin(slaveID, Serial2);
  delay(100);
}

void init_modbus_sensor(int baudrate, uint8_t slaveID, uint32_t paritySerial)
{
  Serial2.begin(baudrate, paritySerial, RXD2, TXD2);
  // Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  nodeSensor.begin(slaveID, Serial2);
  delay(100);
}

uint16_t read_CoilsRegister(uint16_t u16ReadAddress, uint16_t u16BitQty)
{
  String returnVariable;
  uint16_t dataModbuz;
  if ((millis() - old_time) >= 250)
  {
    // ledState = !ledState;
    // digitalWrite(pinLed, ledState);

    uint8_t result;
    uint32_t tempdouble;
    uint16_t dataAddress;
    result = node.readCoils(u16ReadAddress, u16BitQty);
    if (result == node.ku8MBSuccess)
    {
      dataAddress = node.getResponseBuffer(0);
      returnVariable = String(dataAddress);
      dataModbuz = dataAddress;
    }
    else
    {
      Serial.println("failed read modbus4");
    }
    delay(100);
    old_time = millis();
  }
  return dataModbuz;
}

void write_coilRegister(bool state, uint16_t u16WriteAddress) {
  if ((millis() - old_time) >= 250) {
    uint8_t result = node.writeSingleCoil(u16WriteAddress, state);

    if (result == node.ku8MBSuccess) {
      Serial.println("relay state");
      Serial.println(state);
    } else {
      Serial.println("Write failed");
    }
    delay(100);
    old_time = millis();
  }
}

void read_DiscreteRegister(String nameVariable, uint16_t u16ReadAddress, uint16_t u16BitQty, const char *topicMqtt)
{
  String returnVariable;
  if ((millis() - old_time) >= 250)
  {
    // ledState = !ledState;
    // digitalWrite(pinLed, ledState);

    uint8_t result;
    uint32_t tempdouble;
    uint16_t dataAddress;
    result = node.readDiscreteInputs(u16ReadAddress, u16BitQty);
    if (result == node.ku8MBSuccess)
    {
      dataAddress = node.getResponseBuffer(0);
      Serial.print(nameVariable);
      Serial.println(dataAddress);
      returnVariable = String(dataAddress);
      mqttClient.publish(topicMqtt, returnVariable.c_str());
    }
    else
    {
      Serial.println("failed read modbus4");
      mqttClient.publish(topicMqtt, "failed read modbus");
    }
    delay(100);
    old_time = millis();
  }
}

void read_HoldingRegister(uint16_t u16ReadAddress, uint16_t u16ReadQty, uint16_t typeDataModbus, const char *topicMqtt)
{
  String returnVariable;
  if ((millis() - old_time) >= 250)
  {
    uint8_t result;
    uint32_t tempdouble;
    // uint16_t dataAddress;
    result = nodeSensor.readHoldingRegisters(u16ReadAddress, u16ReadQty);
    if (result == nodeSensor.ku8MBSuccess)
    {
      typeDataModbus = nodeSensor.getResponseBuffer(0);
      Serial.println(typeDataModbus);
      returnVariable = String(typeDataModbus);
      mqttClient.publish(topicMqtt, returnVariable.c_str());
    }
    else
    {
      Serial.println("failed read modbus4");
    }
    delay(100);
    old_time = millis();
  }
}

// void read_HoldingRegister(uint16_t u16ReadAddress, uint16_t u16ReadQty, int16_t typeDataModbus)
// {
//   String returnVariable;
//   if ((millis() - old_time) >= 250)
//   {
//     // ledState = !ledState;
//     // digitalWrite(pinLed, ledState);

//     uint8_t result;
//     uint32_t tempdouble;
//     // uint16_t dataAddress;
//     result = node.readHoldingRegisters(u16ReadAddress, u16ReadQty);
//     if (result == node.ku8MBSuccess)
//     {
//       typeDataModbus = node.getResponseBuffer(0);
//       returnVariable = String(typeDataModbus);
//     }
//     else
//     {
//       Serial.println("failed read modbus4");
//     }
//     delay(100);
//     old_time = millis();
//   }
// }

void read_HoldingRegister(String nameVariable, uint16_t u16ReadAddress, uint16_t u16ReadQty, uint32_t typeDataModbus, const char *topicMqtt)
{
  String returnVariable;
  if ((millis() - old_time) >= 250)
  {
    // ledState = !ledState;
    // digitalWrite(pinLed, ledState);

    uint8_t result;
    uint32_t tempdouble;
    // uint16_t dataAddress;
    result = node.readHoldingRegisters(u16ReadAddress, u16ReadQty); // note jika nilai tidak sesuai coba address -1
    if (result == node.ku8MBSuccess)
    {
      // Serial.println("success read modbus4");
      tempdouble = node.getResponseBuffer(1);
      tempdouble |= node.getResponseBuffer(0) << 16;
      typeDataModbus = *(uint32_t *)&tempdouble;
      Serial.print(nameVariable);
      Serial.println(typeDataModbus);
      returnVariable = String(typeDataModbus);
      mqttClient.publish(topicMqtt, returnVariable.c_str());
    }
    else
    {
      Serial.println("failed read modbus4");
      mqttClient.publish(topicMqtt, "failed read modbus");
    }
    delay(100);
    old_time = millis();
  }
}

void read_HoldingRegister(String nameVariable, uint16_t u16ReadAddress, uint16_t u16ReadQty, int32_t typeDataModbus, const char *topicMqtt)
{
  String returnVariable;
  if ((millis() - old_time) >= 250)
  {
    // ledState = !ledState;
    // digitalWrite(pinLed, ledState);

    uint8_t result;
    uint32_t tempdouble;
    // uint16_t dataAddress;
    result = node.readHoldingRegisters(u16ReadAddress, u16ReadQty); // note jika nilai tidak sesuai coba address -1
    if (result == node.ku8MBSuccess)
    {
      // Serial.println("success read modbus4");
      tempdouble = node.getResponseBuffer(1);
      tempdouble |= node.getResponseBuffer(0) << 16;
      typeDataModbus = *(int32_t *)&tempdouble;
      Serial.print(nameVariable);
      Serial.println(typeDataModbus);
      returnVariable = String(typeDataModbus);
      mqttClient.publish(topicMqtt, returnVariable.c_str());
    }
    else
    {
      Serial.println("failed read modbus4");
      mqttClient.publish(topicMqtt, "failed read modbus");
    }
    delay(100);
    old_time = millis();
  }
}

void read_HoldingRegister(String nameVariable, uint16_t u16ReadAddress, uint16_t u16ReadQty, float typeDataModbus, const char *topicMqtt)
{
  String returnVariable;
  if ((millis() - old_time) >= 250)
  {
    // ledState = !ledState;
    // digitalWrite(pinLed, ledState);

    uint8_t result;
    uint32_t tempdouble;
    // uint16_t dataAddress;
    result = node.readHoldingRegisters(u16ReadAddress, u16ReadQty); // note jika nilai tidak sesuai coba address -1
    if (result == node.ku8MBSuccess)
    {
      // Serial.println("success read modbus4");
      tempdouble = node.getResponseBuffer(1);
      tempdouble |= node.getResponseBuffer(0) << 16;
      typeDataModbus = *(float *)&tempdouble;
      Serial.print(nameVariable);
      Serial.println(typeDataModbus);
      returnVariable = String(typeDataModbus);
      mqttClient.publish(topicMqtt, returnVariable.c_str());
    }
    else
    {
      Serial.println("failed read modbus4");
      mqttClient.publish(topicMqtt, "failed read modbus");
    }
    delay(100);
    old_time = millis();
  }
}

void read_InputRegister(String nameVariable, uint16_t u16ReadAddress, uint16_t u16ReadQty, const char *topicMqtt)
{
  String returnVariable;
  float floatValuesAddress;
  if ((millis() - old_time) >= 250)
  {
    // ledState = !ledState;
    // digitalWrite(pinLed, ledState);

    uint8_t result;
    uint32_t tempdouble;
    uint16_t dataAddress;
    if (u16ReadQty < 2)
    {
      result = node.readInputRegisters(u16ReadAddress, u16ReadQty);
      if (result == node.ku8MBSuccess)
      {
        dataAddress = node.getResponseBuffer(0);
        Serial.print(nameVariable);
        Serial.println(dataAddress);
        returnVariable = String(dataAddress);
        mqttClient.publish(topicMqtt, returnVariable.c_str());
      }
      else
      {
        Serial.println("failed read modbus4");
        mqttClient.publish(topicMqtt, "failed read modbus");
      }
    }
    else if (u16ReadQty == 2)
    {
      result = node.readInputRegisters(u16ReadAddress, u16ReadQty); // note jika nilai tidak sesuai coba address -1
      if (result == node.ku8MBSuccess)
      {
        // Serial.println("success read modbus4");
        tempdouble = node.getResponseBuffer(1);
        tempdouble |= node.getResponseBuffer(0) << 16;
        floatValuesAddress = *(float *)&tempdouble;
        Serial.print(nameVariable);
        Serial.println(floatValuesAddress);
        returnVariable = String(floatValuesAddress);
        mqttClient.publish(topicMqtt, returnVariable.c_str());
      }
      else
      {
        Serial.println("failed read modbus4");
        mqttClient.publish(topicMqtt, "failed read modbus");
      }
    }
    delay(100);
    old_time = millis();
  }
}

void komparasi(String kom_suhu, String kom_hum, uint16_t dataSuhu, uint16_t dataHum, float temp, float humidity) {  
  // 0b0000000 = 0b6543210
  if (kom_hum == ">" ){
    if (dataHum > humidity ) {
      read_CoilsRegister(2, 4);
      node.setTransmitBuffer(0, 0b1010);
      node.writeMultipleCoils(2, 4);
      Serial.println("kipas nyala"); 
    }
   else if (dataHum < humidity ) {
      Serial.println("kipas mati");
      read_CoilsRegister(2, 4);
      node.setTransmitBuffer(0, 0b0101);
      node.writeMultipleCoils(3, 4);
    }
  }
  if (komSuhu == ">"){
    if (dataSuhu >temp) {
      read_CoilsRegister(0, 2);
      node.setTransmitBuffer(0, 0b00);
      node.writeMultipleCoils(0, 2);
    } else {
      read_CoilsRegister(0, 2);
      node.setTransmitBuffer(0, 0b11);
      node.writeMultipleCoils(0, 2);
    }
  } 
  // else if (kom_hum == "<") 
  // {
  //   if (dataHum < humidity) {
  //     read_CoilsRegister(modbus_address0, 6);
  //     node.setTransmitBuffer(0, 0b010100);
  //     node.writeMultipleCoils(0, 6);
  //   } else {
  //     read_CoilsRegister(modbus_address0, 6);
  //     node.setTransmitBuffer(0, 0b000011);
  //     node.writeMultipleCoils(0, 6);
  //   }
  // }
  // else if (kom_hum == ">=")
  // {
  //   if (dataHum >= humidity) {
  //     read_CoilsRegister(modbus_address0, 6);
  //     node.setTransmitBuffer(0, 0b000011);
  //     node.writeMultipleCoils(0, 6);
  //   } else
  //   {
  //     read_CoilsRegister(modbus_address0, 6);
  //     node.setTransmitBuffer(0, 0b010100);
  //     node.writeMultipleCoils(0, 6);
  //   }       
  // }
  // else if (kom_hum == "<=") {
  //   if (dataHum <= humidity) {
  //     read_CoilsRegister(modbus_address0, 6);
  //     node.setTransmitBuffer(0, 0b010100);
  //     node.writeMultipleCoils(0, 6);
  //   } else {
     
  //   }
  // } 

  // if (kom_suhu == ">"){
  //   if (dataSuhu > temp) {
  //     read_CoilsRegister(modbus_address0, 6);
  //     node.setTransmitBuffer(0, 0b010100);
  //     node.writeMultipleCoils(0, 6);
  //   } else {
  //     read_CoilsRegister(modbus_address0, 6);
  //     node.setTransmitBuffer(0, 0b000011);
  //     node.writeMultipleCoils(0, 6);
  //   }
  // }
  // else if (kom_suhu == "<") 
  // {
  //   if (dataSuhu < temp) {
  //     read_CoilsRegister(modbus_address0, 6);
  //     node.setTransmitBuffer(0, 0b000011);
  //     node.writeMultipleCoils(0, 6);
  //   } else {
  //     read_CoilsRegister(modbus_address0, 6);
  //     node.setTransmitBuffer(0, 0b010100);
  //     node.writeMultipleCoils(0, 6);
  //   }
  // }
  // else if (kom_suhu == ">=")
  // {
  //   if (dataSuhu >= temp) {
  //     read_CoilsRegister(modbus_address0, 6);
  //     node.setTransmitBuffer(0, 0b010100);
  //     node.writeMultipleCoils(0, 6);
  //   } else
  //   {
  //     read_CoilsRegister(modbus_address0, 6);
  //     node.setTransmitBuffer(0, 0b000011);
  //     node.writeMultipleCoils(0, 6);
  //   }       
  // }
  // else if (kom_suhu == "<=") {
  //   if (dataSuhu <= temp) {
  //     read_CoilsRegister(modbus_address0, 6);
  //     node.setTransmitBuffer(0, 0b000011);
  //     node.writeMultipleCoils(0, 6);
  //   } else {
  //     read_CoilsRegister(modbus_address0, 6);
  //     node.setTransmitBuffer(0, 0b010100);
  //     node.writeMultipleCoils(0, 6);
  //   }
  // } 
}

uint16_t read_HoldingRegister(uint16_t u16ReadAddress, uint16_t u16ReadQty, const char *topicMqtt) {
  String returnVariable;
  if ((millis() - old_time) >= 250) {
    uint8_t result;
    result = nodeSensor.readHoldingRegisters(u16ReadAddress, u16ReadQty);
    
    if (result == nodeSensor.ku8MBSuccess) {
      uint16_t value = nodeSensor.getResponseBuffer(0)/10; // Ambil data dari buffer
      returnVariable = String(value);
      mqttClient.publish(topicMqtt, returnVariable.c_str());
      
      return value; // Kembalikan nilai
    } else {
      Serial.println("Failed to read Modbus");
      return 0; // Return default value jika gagal
    }
  }
  return 0; // Return default jika belum waktunya membaca
}
