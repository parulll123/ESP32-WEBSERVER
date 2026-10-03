#pragma once
#include <Arduino.h>

// Rename fields to match the parser in web_configurator.cpp.
static const char kConfigPage[] PROGMEM = R"HTML(<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>ESP32 Configuration</title>
  <style>
    body{font-family:'Segoe UI',Tahoma,Geneva,Verdana,sans-serif;margin:0;padding:0;background:#f5f5f5;color:#333}
    h1,h2{text-align:center;color:#4CAF50}
    form{max-width:640px;margin:20px auto;padding:20px;background:#fff;box-shadow:0 4px 8px rgba(0,0,0,.1);border-radius:8px}
    label{display:block;margin-bottom:5px;font-weight:bold}
    .modbus_container{display:flex;gap:10px}
    input,select,button{width:100%;padding:10px;margin-bottom:15px;border:1px solid #ddd;border-radius:4px;box-sizing:border-box;font-size:16px}
    input:focus,select:focus{border-color:#4CAF50;outline:none;box-shadow:0 0 4px rgba(76,175,80,.5)}
    button{background:#4CAF50;color:#fff;font-size:18px;cursor:pointer;transition:background-color .3s ease}
    button:hover{background:#45a049}
    .ip-group,.rule-row{display:flex;gap:10px}
    .ip-group input{flex:1;text-align:center}
    .rule-row select{width:30%}
    .modbus-entry{display:flex;gap:10px;margin-bottom:10px}
    .modbus-entry input,.modbus-entry select{flex:1}
    @media(max-width:768px){form{padding:15px}.ip-group{flex-wrap:wrap}.ip-group input{width:calc(50% - 10px);margin-bottom:10px}}
    @media(max-width:480px){.ip-group input{width:100%}button{font-size:16px;padding:12px}.modbus-entry{flex-direction:column}}
  </style>
</head>
<body>
<h1>ESP32 Configuration</h1>
<form action="/saveConfig" method="POST">

  <h2>Wi-Fi Settings</h2>
  <label for="ssid">SSID (required):</label>
  <input type="text" id="ssid" name="ssid" placeholder="Wi-Fi SSID" required>
  <label for="password">Password (required):</label>
  <input type="password" id="password" name="password" placeholder="Wi-Fi password" required>

  <label>Static IP Address (required):</label>
  <div class="ip-group">
    <input type="number" name="ip0" min="0" max="255" placeholder="192" required>
    <input type="number" name="ip1" min="0" max="255" placeholder="168" required>
    <input type="number" name="ip2" min="0" max="255" placeholder="0"   required>
    <input type="number" name="ip3" min="0" max="255" placeholder="2"   required>
  </div>

  <label>Gateway Address (required):</label>
  <div class="ip-group">
    <input type="number" name="gw0" min="0" max="255" placeholder="192" required>
    <input type="number" name="gw1" min="0" max="255" placeholder="168" required>
    <input type="number" name="gw2" min="0" max="255" placeholder="0"   required>
    <input type="number" name="gw3" min="0" max="255" placeholder="1"   required>
  </div>

  <label>Subnet Mask (required):</label>
  <div class="ip-group">
    <input type="number" name="nm0" min="0" max="255" placeholder="255" required>
    <input type="number" name="nm1" min="0" max="255" placeholder="255" required>
    <input type="number" name="nm2" min="0" max="255" placeholder="255" required>
    <input type="number" name="nm3" min="0" max="255" placeholder="0"   required>
  </div>

  <label>Primary DNS (required):</label>
  <div class="ip-group">
    <input type="number" name="dns0" min="0" max="255" placeholder="8" required>
    <input type="number" name="dns1" min="0" max="255" placeholder="8" required>
    <input type="number" name="dns2" min="0" max="255" placeholder="8" required>
    <input type="number" name="dns3" min="0" max="255" placeholder="8" required>
  </div>

  <h2>MQTT Configuration</h2>
  <label for="mqttHost">Broker IP/URL (required):</label>
  <input type="text" id="mqttHost" name="mqttHost" placeholder="MQTT broker" required>
  <label for="mqttPort">Port (required):</label>
  <input type="number" id="mqttPort" name="mqttPort" placeholder="1883" required>
  <label for="mqttUser">Username (optional):</label>
  <input type="text" id="mqttUser" name="mqttUser" placeholder="MQTT username">
  <label for="mqttPass">Password (optional):</label>
  <input type="password" id="mqttPass" name="mqttPass" placeholder="MQTT password">
  <label for="mqttTopic">Publish Topic:</label>
  <input type="text" id="mqttTopic" name="mqttTopic" placeholder="Topic to publish">

  <h2>Modbus Configuration</h2>
  <div class="modbus_container">

    <!-- ==== Slave 1 ==== -->
    <div>
      <label for="s1Id">Slave ID 1:</label>
      <input type="number" id="s1Id" name="s1Id" placeholder="Slave ID">
      <label for="s1Proto">Protocol:</label>
      <select id="s1Proto" name="s1Proto">
        <option value="RTU">RTU</option>
      </select>
      <label for="s1Baud">Baud Rate:</label>
      <select id="s1Baud" name="s1Baud">
        <option value="9600">9600</option><option value="19200">19200</option>
        <option value="115200">115200</option>
      </select>
      <label for="s1Data">Data Bits:</label>
      <select id="s1Data" name="s1Data"><option value="8">8</option><option value="7">7</option></select>
      <label for="s1Stop">Stop Bits:</label>
      <select id="s1Stop" name="s1Stop"><option value="1">1</option><option value="2">2</option></select>
      <label for="s1Parity">Parity:</label>
      <select id="s1Parity" name="s1Parity">
        <option value="none">None</option><option value="even">Even</option><option value="odd">Odd</option>
      </select>

      <label>Registers</label>
      <div class="modbus-entry">
        <input type="number" name="s1Reg0" placeholder="Address">
        <select name="s1Type0">
          <option>UINT16</option><option>INT16</option><option>UINT32</option>
          <option>INT32</option><option>FLOAT32</option>
        </select>
      </div>
      <div class="modbus-entry">
        <input type="number" name="s1Reg1" placeholder="Address">
        <select name="s1Type1">
          <option>UINT16</option><option>INT16</option><option>UINT32</option>
          <option>INT32</option><option>FLOAT32</option>
        </select>
      </div>
      <div class="modbus-entry">
        <input type="number" name="s1Reg2" placeholder="Address">
        <select name="s1Type2">
          <option>UINT16</option><option>INT16</option><option>UINT32</option>
          <option>INT32</option><option>FLOAT32</option>
        </select>
      </div>
      <div class="modbus-entry">
        <input type="number" name="s1Reg3" placeholder="Address">
        <select name="s1Type3">
          <option>UINT16</option><option>INT16</option><option>UINT32</option>
          <option>INT32</option><option>FLOAT32</option>
        </select>
      </div>
    </div>

    <!-- ==== Slave 2 ==== -->
    <div>
      <label for="s2Id">Slave ID 2:</label>
      <input type="number" id="s2Id" name="s2Id" placeholder="Slave ID">
      <label for="s2Proto">Protocol:</label>
      <select id="s2Proto" name="s2Proto">
        <option value="RTU">RTU</option>
      </select>
      <label for="s2Baud">Baud Rate:</label>
      <select id="s2Baud" name="s2Baud">
        <option value="9600">9600</option><option value="19200">19200</option>
        <option value="115200">115200</option>
      </select>
      <label for="s2Data">Data Bits:</label>
      <select id="s2Data" name="s2Data"><option value="8">8</option><option value="7">7</option></select>
      <label for="s2Stop">Stop Bits:</label>
      <select id="s2Stop" name="s2Stop"><option value="1">1</option><option value="2">2</option></select>
      <label for="s2Parity">Parity:</label>
      <select id="s2Parity" name="s2Parity">
        <option value="none">None</option><option value="even">Even</option><option value="odd">Odd</option>
      </select>

      <label>Registers</label>
      <div class="modbus-entry">
        <input type="number" name="s2Reg0" placeholder="Address">
        <select name="s2Type0">
          <option>UINT16</option><option>INT16</option><option>UINT32</option>
          <option>INT32</option><option>FLOAT32</option>
        </select>
      </div>
      <div class="modbus-entry">
        <input type="number" name="s2Reg1" placeholder="Address">
        <select name="s2Type1">
          <option>UINT16</option><option>INT16</option><option>UINT32</option>
          <option>INT32</option><option>FLOAT32</option>
        </select>
      </div>
      <div class="modbus-entry">
        <input type="number" name="s2Reg2" placeholder="Address">
        <select name="s2Type2">
          <option>UINT16</option><option>INT16</option><option>UINT32</option>
          <option>INT32</option><option>FLOAT32</option>
        </select>
      </div>
      <div class="modbus-entry">
        <input type="number" name="s2Reg3" placeholder="Address">
        <select name="s2Type3">
          <option>UINT16</option><option>INT16</option><option>UINT32</option>
          <option>INT32</option><option>FLOAT32</option>
        </select>
      </div>
    </div>
  </div>

  <h2>Temperature Threshold</h2>
  <div class="rule-row">
    <select name="ctlTempCmp">
      <option value=">">&gt;</option><option value="<">&lt;</option>
      <option value=">=">&gt;=</option><option value="<=">&lt;=</option>
    </select>
    <input type="number" name="ctlTempThr" placeholder="Setpoint">
  </div>

  <h2>Humidity Threshold</h2>
  <div class="rule-row">
    <select name="ctlHumCmp">
      <option value=">">&gt;</option><option value="<">&lt;</option>
      <option value=">=">&gt;=</option><option value="<=">&lt;=</option>
    </select>
    <input type="number" name="ctlHumThr" placeholder="Setpoint">
  </div>

  <h2>Lux Threshold</h2>
  <div class="rule-row">
    <select name="ctlLuxCmp">
      <option value=">">&gt;</option><option value="<">&lt;</option>
      <option value=">=">&gt;=</option><option value="<=">&lt;=</option>
    </select>
    <input type="number" name="ctlLuxThr" placeholder="Setpoint">
  </div>

  <button type="submit">Save Configuration</button>
</form>
</body>
</html>)HTML";