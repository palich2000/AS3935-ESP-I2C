#include <AS3935.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <ESP8266httpUpdate.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266Ping.h>
#include <PubSubClient.h>                   // MQTT
#include "support.h"
#include "settings.h"

#define NO_STRIKE_DISTANCE 40
int distanceToLastStrike = NO_STRIKE_DISTANCE;
uint32_t utcTimeOfLastStrike = 0;

AS3935 as3935(0x03, D5);

const char* update_path = "/WebFirmwareUpgrade";
const char* update_username = "admin";
const char* update_password = "espP@ssw0rd";

const char* ssid = "indebuurt1";
const char* password = "VnsqrtnrsddbrN";
char my_hostname[33] = {};

WiFiManager wifiManager;

ESP8266WebServer web_server(80);
ESP8266HTTPUpdateServer httpUpdater;

WiFiClient EspClient;
PubSubClient MqttClient(EspClient);


void setup_web_server() {


    web_server.on("/emulate", HTTP_POST, [&]() {
	as3935.emulateInterrupt();
	String res("OK");
        web_server.send (200, "text/json", "{\"result\":\"" + res + "\"}");
    });

    web_server.on("/nf_up", HTTP_POST, [&]() {
        web_server.send (200, "text/json", "{\"NoiseFloor\":\"" + String(as3935.increaseNoiseFloor()) + "\"}");
    });

    web_server.on("/nf_down", HTTP_POST, [&]() {
	web_server.send (200, "text/json", "{\"NoiseFloor\":\"" + String(as3935.descreseNoiseFloor()) + "\"}");
    });

    web_server.on("/get", HTTP_POST, [&]() {
	auto nf = as3935.getNoiseFloor();
        auto wt = as3935.getWatchdogThreshold();
	auto sr = as3935.getSpikeRejection();
	auto ml = as3935.getMinimumLightning();
        web_server.send (200, "text/json", 
		"{\"NoiseFloor\":" + String(nf) + 
               ", \"WatchdogThreshold\":" + String(wt) + 
               ", \"SpikeRejection\":" + String(sr) + 
               ", \"MinimumLightning\":" + String(ml) + 
	"}");
    });

    web_server.on("/power", HTTP_POST, [&]() {
	String pwr = web_server.arg("pwr");
	if (pwr == "on") {
	    as3935.powerUp();
	}
	else if (pwr == "off") {
	    as3935.powerDown();
	}
	else {
	    web_server.send (200, "text/json","error");
	    return;
	}
	dumpRegs(0x00, 0x09);
	web_server.send (200, "text/json","OK");
    });

    web_server.on("/set", HTTP_POST, [&]() {
        int iNoiseFloor = 0;
        int iWatchdogThreshold = 0;
        int iSpikeRejection = 0;
	int iMinimumLightning =0;

	String strNoiseFloor = web_server.arg("NoiseFloor");
	String strWatchdogThreshold = web_server.arg("WatchdogThreshold");
	String strSpikeRejection = web_server.arg("SpikeRejection");
	String strMinimumLightning = web_server.arg("MinimumLightning");
	String result("{");

        if (strMinimumLightning.length() > 0) {
            result += "\"MinimumLightning\":";
	    iMinimumLightning = strMinimumLightning.toInt();
	    if (as3935.setMinimumLightning(iMinimumLightning)) {
		result += "true ,";
		RtcSettings.MinimumLightning = iMinimumLightning;
	    }
	    else {
		result += "false ,";
	    }
	}

        if (strWatchdogThreshold.length() > 0) {
            result += "\"WatchdogThreshold\":";
	    iWatchdogThreshold = strWatchdogThreshold.toInt();
	    if (as3935.setWatchdogThreshold(iWatchdogThreshold)) {
		result += "true ,";
		RtcSettings.WatchdogThreshold = iWatchdogThreshold;
	    }
	    else {
		result += "false ,";
	    }
	}

        if (strSpikeRejection.length() > 0) {
	    result += "\"SpikeRejection\":";
	    iSpikeRejection = strSpikeRejection.toInt();
	    if (as3935.setSpikeRejection(iSpikeRejection)) {
                result += "true ,";
		RtcSettings.SpikeRejection = iSpikeRejection;
            }
            else {
                result += "false ,";
            }
	}
        if (strNoiseFloor.length() > 0) {
	    result += "\"NoiseFloor\":";
	    iNoiseFloor = strNoiseFloor.toInt();
	    if (as3935.setNoiseFloor(iNoiseFloor)) {
                result += "true ,";
		RtcSettings.NoiseFloor = iNoiseFloor;
            }
            else {
                result += "false ,";
            }
	}
	result += "}";
	RtcSettingsSave();
	outputCalibrationValues();
	recalibrate();
	dumpRegs(0x00, 0x09);
	web_server.send (200, "text/json", result);
    });
    
    web_server.on("/regs", HTTP_POST, [&]() {
	String tmp_s("OK");
	/*
        for (uint8_t i = 0; i <= 0x08; i++) {
	    uint8_t reg = as3935.readRegister(i);
	    tmp_s += "reg[" + String(i) + "]:0x" + String(reg, HEX) + " 0b" + String(reg, BIN) + " ";
        }
	*/
	dumpRegs(0x00, 0x09);
	web_server.send (200, "text/json", "{\"regs\": \"" + tmp_s  + "\"}");
    });

    web_server.on("/reset", HTTP_POST, [&]() {
	as3935.powerDown();
	SYSLOG(LOG_INFO, "reset");
	web_server.send (200, "text/json", "OK");
	web_server.close();
	web_server.stop();
        RtcSettingsSave();
        ESP.restart();
    });

    web_server.on("/set_default", HTTP_POST, [&]() {
	SYSLOG(LOG_INFO, "setDefault");
	as3935.setDefault();
	web_server.send (200, "text/json", "OK");
    });

    web_server.on("/clear_stats", HTTP_POST, [&]() {
	SYSLOG(LOG_INFO, "clearStats");
	as3935.clearStats();
	distanceToLastStrike = NO_STRIKE_DISTANCE;
	web_server.send (200, "text/json", "OK");
    });

    web_server.on("/esp", HTTP_POST, [&]() {
        String firmware = web_server.arg("firmware");
        web_server.close();
        SYSLOG(LOG_INFO, "Begin update firmwate: %s", firmware.c_str());
        HTTPUpdateResult ret = ESPhttpUpdate.update(firmware, "1.0.0");
        web_server.begin();
        switch(ret) {
        case HTTP_UPDATE_FAILED:
            web_server.send (500, "text/json", "{\"result\":false,\"msg\":\"" + ESPhttpUpdate.getLastErrorString() + "\"}");
            Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
            break;
        case HTTP_UPDATE_NO_UPDATES:
            web_server.send (304, "text/json", "{\"result\":true,\"msg\":\"Update not necessary.\"}");
            Serial.println("Update not necessary.");
            break;
        case HTTP_UPDATE_OK:
            web_server.send (200, "text/json", "{\"result\":true,\"msg\":\"Update OK.\"}");
            Serial.println("Update OK. Rebooting....");
            RtcSettingsSave();
            ESP.restart();
            break;
        }
    });

    httpUpdater.setup(&web_server, update_path, update_username, update_password);
    web_server.begin();
    MDNS.addService("http", "tcp", 80);
}

void outputCalibrationValues() {
  as3935.interruptEnable(false);
  delay(50);
  for (byte i = 0; i <= 0x0F; i++) {
    int frequency = as3935.tuneAntenna(i);
    long fullFreq = (long) frequency*160;  // multiply with clock-divider, and 10 (because measurement is for 100ms)
    SYSLOG(LOG_INFO,"tune antenna to capacitor %d gives frequency: %d = %ld Hz", i, frequency, fullFreq);
    delay(10);
  }
  as3935.interruptEnable(true);
}

void recalibrate() {
  delay(50);
  long int freq;
  uint8_t cap = as3935.getBestTune(freq); 
  SYSLOG(LOG_INFO, "antenna calibration picks value: %d freq: %ld Hz", cap, freq);
  delay(50);
}

void as3935_init() {

    WiFi.forceSleepBegin();

    as3935.begin(D2, D1);
    as3935.setDefault();
    as3935.setOutdoor();
    as3935.setMaskDisturber(1);

    outputCalibrationValues();
    recalibrate();

    SYSLOG(LOG_INFO,"AS3935 NoiseFloor: %d set: %s", RtcSettings.NoiseFloor, as3935.setNoiseFloor(RtcSettings.NoiseFloor)==true?"OK":"FAIL");
    SYSLOG(LOG_INFO,"AS3935 SpikeRejection: %d set: %s", RtcSettings.SpikeRejection, as3935.setSpikeRejection(RtcSettings.SpikeRejection)==true?"OK":"FAIL");
    SYSLOG(LOG_INFO,"AS3935 WatchdogThreshold: %d set: %s", RtcSettings.WatchdogThreshold, as3935.setWatchdogThreshold(RtcSettings.WatchdogThreshold)==true?"OK":"FAIL");
    SYSLOG(LOG_INFO,"AS3935 MinimumLightning: %d set: %s", RtcSettings.MinimumLightning, as3935.setMinimumLightning(RtcSettings.MinimumLightning)==true?"OK":"FAIL");
  
    outputCalibrationValues();
    recalibrate();

    as3935.clearStats();

    dumpRegs(0x00, 0x09);

    distanceToLastStrike = NO_STRIKE_DISTANCE;

    as3935.interruptEnable(true);
    //WiFi.forceSleepWake();
}

void
setup()
{
	pinMode(BUILTIN_LED, OUTPUT);
	digitalWrite(BUILTIN_LED, LOW);
	Serial.begin(115200);
	Serial.println();
	Serial.println(__FILE__);
	Serial.println("+");

	SettingsLoad();
	SyslogInit();
	
	wifiManager.setConfigPortalTimeout(180);

	WiFi.hostname(my_hostname);
	WiFi.begin(ssid, password);
	if (!wifiManager.autoConnect(ssid, password)) {
	    Serial.println("failed to connect, we should reset as see if it connects");
	    delay(3000);
	    ESP.reset();
	    delay(5000);	
	}
	MDNS.begin(my_hostname);
	ping_sever(WiFi.gatewayIP().toString().c_str());
	OsWatchInit();

	setup_web_server();

	SYSLOG(LOG_INFO, "=======Begin setup======");
	SYSLOG(LOG_INFO, "Restart reason: %s", ESP.getResetReason().c_str());
	SYSLOG(LOG_INFO, "IP address: %s", WiFi.localIP().toString().c_str());
	SYSLOG(LOG_INFO, "Chip ID = %08X", ESP.getChipId());
	SYSLOG(LOG_INFO, "Buil timestamp: %s", __TIMESTAMP__);
	SYSLOG(LOG_INFO, "Sketch size: %d", ESP.getSketchSize());
	SYSLOG(LOG_INFO, "Free space: %d", ESP.getFreeSketchSpace());
	SYSLOG(LOG_INFO, "Flash Chip ID: %d", ESP.getFlashChipId());
	SYSLOG(LOG_INFO, "Flash Chip Real Size: %d", ESP.getFlashChipRealSize());
	SYSLOG(LOG_INFO, "Flash Chip Size: %d", ESP.getFlashChipSize());
	SYSLOG(LOG_INFO, "Flash Chip Speed: %d", ESP.getFlashChipSpeed());
	SYSLOG(LOG_INFO, "Flash Chip Mode: %d", ESP.getFlashChipMode());


	RtcInit();

	MQTT_Reconnect();
	MQTT_send_state();

	digitalWrite(BUILTIN_LED, HIGH);
	SYSLOG(LOG_INFO, "=======End setup======");
}

void I2cScan(char *devs, unsigned int devs_len)
{
  byte error;
  byte address;
  byte any = 0;
  char tstr[10];

  for (address = 1; address <= 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (0 == error) {
      snprintf_P(tstr, sizeof(tstr), PSTR(" 0x%02x"), address);
      strncat(devs, tstr, devs_len);
      any = 1;
    }
    else if (4 == error) {
      snprintf_P(devs, devs_len, PSTR("I2CScan: Unknown error at 0x%2x"), address);
    }
  }
  if (!any) {
    snprintf_P(devs, devs_len, PSTR("I2CScan: No devices found"));
  }
}

void 
ping_sever(const char * server) {
  Serial.print("Pinging ");
  Serial.print(server);
  Serial.print(": ");

  auto pingResult = Ping.ping(server, 3);

  if (pingResult) {
    Serial.print("SUCCESS! RTT = ");
    Serial.print(Ping.averageTime());
    Serial.println(" ms");
  } else {
    Serial.println("FAILED!");
  }
}

const char * mqtt_host = "192.168.0.106";
int mqtt_port = 8883;
const char * mqtt_client = my_hostname;
const char * mqtt_user = "owntracks";
const char * mqtt_pwd = "zhopa";

bool
MQTT_Reconnect(void)
{
    if (WL_CONNECTED != WiFi.status()) return false;

    static uint32_t next_try_connect_time = 0;

    if (MqttClient.connected()) return true;

    if (GetUTCTime() < next_try_connect_time) return false;

    next_try_connect_time = GetUTCTime() + 10;

    MqttClient.setServer(mqtt_host, mqtt_port);

    if (MqttClient.connect(mqtt_client, mqtt_user, mqtt_pwd)) {
	SYSLOG(LOG_INFO, "mqtt server %s:%d connected",mqtt_host, mqtt_port)
	return true;
    } else {
	SYSLOG(LOG_ERR, "Unable connect to mqtt server %s:%d rc %d",mqtt_host, mqtt_port, MqttClient.state());
    }
    return false;
}


typedef struct {
    int l_count;
    int d_count;
    long energy;
} LightningHistory_t;

#define LightningHistoryLen 60
LightningHistory_t LightningHistory[LightningHistoryLen] = {};


void
wifiOn() {
    as3935.powerDown();
    WiFi.forceSleepWake();

    uint32_t startTime = GetUTCTime();;

    while ((WL_CONNECTED != WiFi.status()) && (GetUTCTime() - startTime < 10)) {
	delay(100);
    }

    if (GetUTCTime() - startTime >= 10) {
	SYSLOG(LOG_ERR, "WiFi not connected");
    }
}

void
wifiOff() {
    SysLogFlush();
    EspClient.flush();
    MqttClient.disconnect();
    WiFi.forceSleepBegin();
    delay(100);
    as3935.powerUp();
}


void
MQTT_send_sensor(void)
{
    int strikes = 0;
    int disturbs = 0;
    long energy = 0;
    for (int i = 0; i < LightningHistoryLen; i++) {
	strikes += LightningHistory[i].l_count;
	energy  += LightningHistory[i].energy;
	disturbs += LightningHistory[i].d_count;
    }
    if (strikes) {
	energy = energy / strikes;
    }
    char buffer[255] = {};
    char voltage[16];
    dtostrfd((double)ESP.getVcc() / 1000, 3, voltage);
    snprintf_P(buffer, sizeof(buffer) - 1, PSTR("{\"Time\":\"%s\",\"AS3935\":{\"Strikes\": %d,\"Energy\": %ld,\"Disturbes\": %d,\"Distance\": %d}}"),
		GetDateAndTime().c_str(), strikes, energy, disturbs, distanceToLastStrike);
    String topic = String("tele/") + my_hostname + String("/SENSOR");

    if (MQTT_Reconnect()) {
	MQTT_publish(topic.c_str(),buffer);
    } else {
	SYSLOG(LOG_ERR, "MQTT not connected");
    }
}

void 
every_hour(void)
{
    ntpSyncTime();
    MQTT_send_state();
    dumpRegs(0x00, 0x09);
}


void 
every_minute(void)
{
    auto now = GetUTCTime();

    if (GetUTCTime() - utcTimeOfLastStrike >= 15 * 60) {
	distanceToLastStrike =  NO_STRIKE_DISTANCE;
    }

    wifiOn();
    MQTT_send_sensor();
    if (now % 3600 == 0){
        every_hour();
    }
    wifiOff();
}

void
every_second(void)
{
    MQTT_Reconnect();
    static bool not_cleared_stat = true;
    if ((not_cleared_stat) && (utc_time > 0)) {
	not_cleared_stat = false;
	as3935_init();
    }
}

void 
main_loop(void) 
{
    static uint32_t last_utc_time = 0;
    if (last_utc_time != utc_time) {
	auto now = last_utc_time = GetUTCTime();
	LightningHistory[now % LightningHistoryLen] = {};
	every_second();
	if (now % 60 == 0){
	    every_minute();
	}
    }
}

uint8_t dumpRegs(uint8_t s, uint8_t n) {
  uint8_t  i;
  uint8_t  reg;
  uint8_t  err = 0;
  uint8_t  r;

  for (i=0; i<n; i++) {
    r = i + s;
    reg = as3935.readRegister(r);
    if ((err = as3935.transmitError()) != 0) {
      SYSLOG(LOG_ERR, "readRegister 0x%02x read error %d", r, err);
      break;
    }
    REG_u rr;
    rr.data = reg;

    switch(r) {
	case 0:
	    SYSLOG(LOG_INFO, "Reg[0x%02x] PWD: 0x%02x AFE_GB:0x%02x", r, rr.R0.PWD,  rr.R0.AFE_GB);
	    break;
	case 1:
	    SYSLOG(LOG_INFO, "Reg[0x%02x] WDTH:0x%02x NF_LEV:0x%02x", r, rr.R1.WDTH,  rr.R1.NF_LEV);
	    break;
	case 2:
	    SYSLOG(LOG_INFO, "Reg[0x%02x] SREJ:0x%02x MIN_NUM_LIGHT:0x%02x CL_STAT:0x%02x", r, rr.R2.SREJ,  rr.R2.MIN_NUM_LIGHT, rr.R2.CL_STAT);
	    break;
	case 3:
	    SYSLOG(LOG_INFO, "Reg[0x%02x] INT: 0x%02x MASK_DIST:0x%02x LCO_FDIV:0x%02x", r, rr.R3.INT,  rr.R3.MASK_DIST, rr.R3.LCO_FDIV);
	    break;
	case 7:
	    SYSLOG(LOG_INFO, "Reg[0x%02x] DISTANCE:0x%02x", r, rr.R7.DISTANCE);
	    break;
	case 8:
	    SYSLOG(LOG_INFO, "Reg[0x%02x] TUN_CAP:0x%02x DISP_TRCO:0x%02x DISP_SRCO:0x%02x DISP_LCO:0x%02x", r, rr.R8.TUN_CAP,  rr.R8.DISP_TRCO, rr.R8.DISP_SRCO, rr.R8.DISP_LCO);
	    break;
	default:
	    SYSLOG(LOG_INFO, "Reg[0x%02x] 0x%02x", r, reg);
    }
    delay(5);
  }
  return(err);
}


void
loop()
{
	main_loop();
	if (as3935.waitingInterrupt()) {
            delay(3);
            unsigned long time = as3935.timeFromLastInterrupt();
	    uint8_t int_src = as3935.getInterrupt();
	    if (as3935.transmitError() == 0) {
		int dist = as3935.getDistance();
        	uint32_t energy = as3935.getStrikeEnergyRaw();
		if (0 == int_src) {
    		    SYSLOG(LOG_INFO, "interrupt source result not expected");
		} else {
		    if(AS3935_INT_STRIKE & int_src) {
			int i = utc_time % LightningHistoryLen;
			LightningHistory[i].l_count++;
			LightningHistory[i].energy += energy;
			distanceToLastStrike = dist;
			utcTimeOfLastStrike = GetUTCTime();
			SYSLOG(LOG_INFO, "Lightning detected! Distance to strike: %d kilometers energy: %u time: %ld", dist, energy, time);
		    }
		    if(AS3935_INT_DISTURBER & int_src) {
			int i = utc_time % LightningHistoryLen;
			LightningHistory[i].d_count++;
			SYSLOG(LOG_INFO, "Disturber detected: %d", LightningHistory[i].d_count);
		    }
		    if(AS3935_INT_NOISE & int_src)  {
			SYSLOG(LOG_INFO, "Noise level too high");
		    }
		}
	    }
	}
	OsWatchLoop();
        web_server.handleClient();
        MqttClient.loop();
}
