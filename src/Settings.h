
#define version "1.0.7"
#define ap_ssid "AirbnkOpenGateway"

#define main_prefs "airbnk_prefs"

#define wifi_ssid_pref "wifi_ssid"
#define wifi_pwd_pref "wifi_pwd"
#define mqtt_ip_pref "mqtt_ip"
#define mqtt_port_pref "mqtt_port"
#define mqtt_user_pref "mqtt_user"
#define mqtt_pass_pref "mqtt_pass"
#define mqtt_topic_pref "mqtt_topic"
#define lock_mac_pref "lock_mac"

#define LED_BUILTIN 2
#define LED_ON 0

#define serviceUUID "0xFFF0"
#define characteristicUUID "0xFFF2"
#define statusCharacteristicUUID "0xFFF3"

#define scanInterval 0 // Define the interval in seconds between scans
#define singleScanTime 10 // Define the duration of a single scan in seconds
#define bleScanInterval 0x80 // Used to determine antenna sharing between Bluetooth and WiFi. Do not modify unless you are confident you know what you're doing
#define bleScanWindow 0x40 // Used to determine antenna sharing between Bluetooth and WiFi. Do not modify unless you are confident you know what you're doing

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
<title>Airbnk Gateway %VERSION%</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style> 
input[type=submit] {
  background-color: #5e9ca0;
  border: none;
  color: white;
  padding: 16px 32px;
  text-decoration: none;
  margin: 4px 2px;
  cursor: pointer;
}
</style>
</head><body>
<h1 style="color: #5e9ca0;">Airbnk Gateway %VERSION%</h1>
<h4>by @formatBCE</h4>
<form action="/update">
<p><input type="submit" value="Update firmware" /></p>
</form>
<h4>&nbsp;</h4>
<h4>CONFIGURATION</h4>
<p>Insert required data into fields below, and save configuration.</p>
<p>Gateway will reboot and connect to your WiFi and MQTT automatically.&nbsp;</p>
<p><span style="color: #ff0000;"><strong>PLEASE DOUBLE-CHECK ENTERED DATA BEFORE SAVING!</strong></span></p>
<p>After saving, there will be no way to change them.</p>
<p>&nbsp;</p>
<form action="/config">
<p><strong>WiFi (2.4 GHz)</strong></p>
<p>SSID: <input name="input1" type="text" /></p>
<p>Password: <input name="input2" type="text" /></p>
<p><strong>MQTT</strong></p>
<p>Broker IP: <input name="input3" type="text" value="192.168.0.1" /></p>
<p>Broker port: <input name="input4" type="number" value="1883" /></p>
<p>User: <input name="input5" type="text" /></p>
<p>Password: <input name="input6" type="text" /></p>
<p>Root topic: <input name="input7" type="text" value="airbnk_lock" /></p>
<p><strong>Target lock</strong></p>
<p>MAC address: <input name="input8" type="text" value="12:34:56:78:90:AB" /></p>
<p><input type="submit" value="Save and reboot" /></p>
</form>
</body></html>)rawliteral";
const char reset_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
<title>Airbnk Gateway %VERSION%</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style> 
input[type=submit] {
  background-color: #5e9ca0;
  border: none;
  color: white;
  padding: 16px 32px;
  text-decoration: none;
  margin: 4px 2px;
  cursor: pointer;
}
</style>
</head><body>
<h1 style="color: #5e9ca0;">Airbnk Gateway %VERSION%</h1>
<h4>by @formatBCE</h4>
<form action="/update">
<p><input type="submit" value="Update firmware" /></p>
</form>
<h4>&nbsp;</h4>
<h4>CURRENT CONFIGURATION</h4>
<p>WiFi SSID: %WIFI%</p>
<p>MQTT IP: %MQTT_IP%</p>
<p>MQTT port: %MQTT_PORT%</p>
<p>MQTT user: %MQTT_USER%</p>
<p>MQTT root topic: %MQTT_TOPIC%</p>
<p>Lock MAC address: %LOCK_MAC%</p>
<h4>&nbsp;</h4>
<h4>RESET CONFIGURATION</h4>
<p>You may reset gateway configuration on this page.</p>
<p>After resetting, connect to WiFi access point "AirbnkOpenGateway", and configure gateway from scratch.</p>
<form action="/reset">
<p><input type="submit" value="Reset configuration and restart" /></p>
</form>
</body></html>)rawliteral";
const char confirm_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
<title>Airbnk Gateway %VERSION%</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
</head><body>
<h1 style="color: #5e9ca0;">Airbnk Gateway %VERSION%</h1>
<h4>by @formatBCE</h4>
<p>Configuration saved, device restarted. You may close this page now.</p>
</body></html>)rawliteral";
const char* PARAM_INPUT_1 = "input1";
const char* PARAM_INPUT_2 = "input2";
const char* PARAM_INPUT_3 = "input3";
const char* PARAM_INPUT_4 = "input4";
const char* PARAM_INPUT_5 = "input5";
const char* PARAM_INPUT_6 = "input6";
const char* PARAM_INPUT_7 = "input7";
const char* PARAM_INPUT_8 = "input8";


// OTA
String style =
"<style>#file-input,input{width:100%;height:44px;border-radius:4px;margin:10px auto;font-size:15px}"
"input{background:#f1f1f1;border:0;padding:0 15px}body{background:#5e9ca0;font-size:14px;}"
"#file-input{padding:0;border:1px solid #ddd;line-height:44px;text-align:center;display:block;cursor:pointer}"
"#bar,#prgbar{background-color:#f1f1f1;border-radius:2px}#bar{background-color:#5e9ca0;width:0%;height:10px}"
"form{background:#fff;max-width:258px;margin:75px auto;padding:30px;border-radius:5px;text-align:center}"
".btn{background:#5e9ca0;color:#fff;cursor:pointer}</style>";
String update_html = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
"<input type='file' name='update' id='file' onchange='sub(this)' style=display:none>"
"<label id='file-input' for='file'>   Choose file...</label>"
"<input type='submit' class=btn value='Update'>"
"<br><br>"
"<div id='prg'></div>"
"<br><div id='prgbar'><div id='bar'></div></div><br></form>"
"<script>"
"function sub(obj){"
"var fileName = obj.value.split('\\\\');"
"document.getElementById('file-input').innerHTML = '   '+ fileName[fileName.length-1];"
"};"
"$('form').submit(function(e){"
"e.preventDefault();"
"var form = $('#upload_form')[0];"
"var data = new FormData(form);"
"$.ajax({"
"url: '/doUpdate',"
"type: 'POST',"
"data: data,"
"contentType: false,"
"processData:false,"
"xhr: function() {"
"var xhr = new window.XMLHttpRequest();"
"xhr.upload.addEventListener('progress', function(evt) {"
"if (evt.lengthComputable) {"
"var per = evt.loaded / evt.total;"
"if (per === 1) {"
"setTimeout(function(){"
"window.location.href = '/';"
"}, 5000);"
"}"
"$('#prg').html('Flashing progress: ' + Math.round(per*100) + '%');"
"$('#bar').css('width',Math.round(per*100) + '%');"
"}"
"}, false);"
"return xhr;"
"},"
"success:function(d, s) {"
"console.log('success!') "
"},"
"error: function (a, b, c) {"
"}"
"});"
"});"
"</script>" + style;
