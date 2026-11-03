#include <DHT.h> 
#include <SoftwareSerial.h> 
//Pin definitions 
#define DHT_PIN        2 
#define TANK_PIN       3 
#defineVALVE_B_PIN    5 
#define PUMP_PIN       6 
#defineVALVE_A_PIN    7 
#define GSM_RX_PIN    10 
#define GSM_TX_PIN    11 
#define SOIL_A_PIN    A0 
#define SOIL_B_PIN    A1 
#define RAIN_PIN      A2 
// Calibration constants 
const int DRY_A    = 1022. 
const int WET_A    = 382. 
const int THRESH_A = 766. 
const int DRY_B    = 1023. 
const int WET_B    = 395. 
const int THRESH_B = 772. 
68 
 

const int SOIL_DISCONNECTED_HIGH = 1020. 
const int SOIL_DISCONNECTED_LOW = 10; const int RAIN_DISCONNECTED_HIGH = 1015; const int RAIN_DISCONNECTED_LOW = 5; const int RAIN_THRESHOLD = 800; 
const int HUMIDITY_SPIKE = 80; 
//Timing constants 
#if DEMO_MODE 
 const unsigned long SENSOR_INTERVAL = 30000UL; 
 const unsigned long COOLDOWN_PERIOD = 120000UL; #else 
 const unsigned long SENSOR_INTERVAL = 900000UL;  const unsigned long COOLDOWN_PERIOD = 3600000UL; #endif 
const unsigned long SMS_CHECK_INTERVAL = 8000UL; const unsigned long WEATHER_INTERVAL   = 1800000UL; const unsigned long BALANCE_INTERVAL   = 604800000UL; const unsigned longVALVE_PRE_DELAY    = 500UL; 
const unsigned long PUMP_RUN_TIME      = 4000UL; 
const unsigned longVALVE_POST_DELAY   = 300UL; 
const int MAX_SENSOR_FAILURES = 3; 
// ── Farmer phone and API config 
69 
 

const char FARMER_NUMBER[] = "+2348127773481"; const char OWM_CITY_ID[]   = "2332459"; 
const char APN[]            = "internet"; 
// Objects 
DHT           dht(DHT_PIN, DHT22); 
SoftwareSerial gsm(GSM_RX_PIN, GSM_TX_PIN); 
// Timer variables 
unsigned long lastSensorCheck  = 0; 
unsigned long lastSMSCheck     = 0; 
unsigned long lastWeatherCheck = 0; 
unsigned long lastBalanceCheck = 0; 
// Irrigation state 
unsigned long cooldownA      = 0; 
unsigned long cooldownB      = 0; 
unsigned long lastIrrigationA = 0; 
unsigned long lastIrrigationB = 0; 
// Cached sensor values (used in STATUS reply) int   lastMoistA    = 0; 
int   lastMoistB    = 0; 
bool  lastTankOK    = true; 
70 
 

float lastTemp      = 0.0; 
float lastHumidity  = 0.0; 
int   lastRainADC   = 1023; 
// Error counters per sensor 
// Counts consecutive invalid readings before sending an alert int dhtFailCount   = 0; 
int soilAFailCount = 0; 
int soilBFailCount = 0; 
int rainFailCount  = 0; 
int tankFailCount  = 0; 
// Alert sent flags — prevents the same alert being sent on every cycle bool dhtAlertSent   = false; 
bool soilAAlertSent = false; 
bool soilBAlertSent = false; 
bool rainAlertSent  = false; 
bool tankAlertSent  = false; 
// Weather forecast cached result 
bool rainForecast = false; 
// Emergency stop flag 
bool stopFlag = false; 
71 
 

// SETUP 
void setup() { 
 Serial.begin(9600); 
 gsm.begin(9600); 
 // All actuator pins go LOW immediately 
 pinMode(VALVE_A_PIN, OUTPUT); digitalWrite(VALVE_A_PIN, LOW);  pinMode(VALVE_B_PIN, OUTPUT); digitalWrite(VALVE_B_PIN, LOW);  pinMode(PUMP_PIN,    OUTPUT); digitalWrite(PUMP_PIN,    LOW); 
 // Input pins 
 pinMode(TANK_PIN,   INPUT);   // external 10kΩ pull-up, plain INPUT  pinMode(SOIL_A_PIN, INPUT); 
 pinMode(SOIL_B_PIN, INPUT); 
 pinMode(RAIN_PIN,   INPUT); 
 dht.begin(); 
 Serial.println(F("=== Solar Irrigation System v2 — Boot ===")); 
 #if DEMO_MODE 
	 Serial.println(F("Mode: DEMO (30-second cycles)")); 
 #else 
	 Serial.println(F("Mode: FIELD (15-minute cycles)")); 
 #endif 
72 
 

 delay(3000);   // GSM settling time 
 // GSM initialisation with response check 
 if (!gsmInitOK()) { 
		 Serial.println(F("WARNING: GSM module did not respond on boot.")); 		 Serial.println(F("Check wiring on D10/D11 and SIM900A power supply."));  } else { 
		 sendSMS(FARMER_NUMBER, "Irrigation system online and ready."); 		 Serial.println(F("Boot SMS sent.")); 
 } 
 lastSensorCheck  = millis() - SENSOR_INTERVAL; 
 lastSMSCheck     = millis() - SMS_CHECK_INTERVAL; 
 lastWeatherCheck = millis() - WEATHER_INTERVAL; 
 lastBalanceCheck = millis() - BALANCE_INTERVAL; 
} 
bool gsmInitOK() { 
 for (int attempt = 0; attempt < 3; attempt++) { 
		 gsm.println(F("AT")); 
		 delay(500); 
		 String r = ""; 
		 while (gsm.available()) r += (char)gsm.read(); 
		 if (r.indexOf("OK") >= 0) { 
			 gsmSend("AT+CMGF=1");          delay(500); 
			 gsmSend("AT+CNMI=1,2,0,0,0"); delay(500); 
73 
 

			 return true; 
		 } 
		 delay(1000); 
 } 
 return false; 
} 
// MAIN LOOP 
void loop() { 
 unsigned long now = millis(); 
 if (now - lastSensorCheck  >= SENSOR_INTERVAL)  { lastSensorCheck  = now; runSensorAndIrrigation(); } 
 if (now - lastSMSCheck     >= SMS_CHECK_INTERVAL){ lastSMSCheck     = now; checkSMSInbox();          } 
 if (now - lastWeatherCheck >= WEATHER_INTERVAL)  { lastWeatherCheck = now; fetchWeatherForecast();   } 
 if (now - lastBalanceCheck >= BALANCE_INTERVAL)  { lastBalanceCheck = now; checkSimBalance();        } 
} 
// SUBSYSTEM 1 — SENSOR ACQUISITION & IRRIGATION DECISION void runSensorAndIrrigation() { 
 Serial.println(F("\n--- Sensor Cycle ---")); 
 //1. DHT22 (temperature & humidity)  
 float temp     = dht.readTemperature(); 
 float humidity = dht.readHumidity(); 
74 
 

 if (isnan(temp) || isnan(humidity) || temp < -40.0 || temp > 80.0 
		 || humidity < 0.0 || humidity > 100.0) { 
	 dhtFailCount++; 
	 Serial.print(F("DHT22 invalid reading. Fail count: ")); 
	 Serial.println(dhtFailCount); 
	 if (dhtFailCount >= MAX_SENSOR_FAILURES && !dhtAlertSent) { 
		 sendSMS(FARMER_NUMBER, 
			 "SENSOR ALERT: DHT22 temperature/humidity sensor is not responding. " 			 "Check wiring on pin D2. System continues using last known values."); 		 dhtAlertSent = true; 
	 } 
 } else { 
	 lastTemp      = temp; 
	 lastHumidity  = humidity; 
	 dhtFailCount  = 0; 
	 dhtAlertSent  = false; 
 } 
 // 2. Tank level sensor (XKC-Y25)  
 bool reading1 = (digitalRead(TANK_PIN) == LOW);  delay(100); 
 bool reading2 = (digitalRead(TANK_PIN) == LOW);  delay(100); 
 bool reading3 = (digitalRead(TANK_PIN) == LOW); 
75 
 

 bool tankReadingsConsistent = (reading1 == reading2 && reading2 == reading3);  if (!tankReadingsConsistent) { 
	 tankFailCount++; 
	 Serial.print(F("Tank sensor unstable. Fail count: ")); 
	 Serial.println(tankFailCount); 
	 if (tankFailCount >= MAX_SENSOR_FAILURES && !tankAlertSent) { 		 sendSMS(FARMER_NUMBER, 
			 "SENSOR ALERT: Tank level sensor giving unstable readings. " 
			 "Check wiring on pin D3 and 10k pull-up resistor. " 
			 "Irrigation paused until sensor is stable."); 
		 tankAlertSent = true; 
	 } 
	 lastTankOK = false; 
 } else { 
	 bool tankOK = reading1; 
	 lastTankOK   = tankOK; 
	 tankFailCount = 0; 
	 tankAlertSent = false; 
 } 
 //3. Rain gauge potentiometer 
 int rainADC = readAvg(RAIN_PIN); 
 lastRainADC = rainADC; 
 bool rainGaugeValid  = (rainADC > RAIN_DISCONNECTED_LOW && 
76 
 

				 rainADC < RAIN_DISCONNECTED_HIGH); 
 bool rainGaugeActive = false; 
 if (!rainGaugeValid) { 
	 rainFailCount++; 
	 Serial.print(F("Rain gauge out of range (ADC=")); 
	 Serial.print(rainADC); Serial.print(F("). Fail count: ")); 
	 Serial.println(rainFailCount); 
	 if (rainFailCount >= MAX_SENSOR_FAILURES && !rainAlertSent) { 		 sendSMS(FARMER_NUMBER, 
			 "SENSOR ALERT: Rain gauge potentiometer reading is out of range. " 			 "Check wiring on pin A2. Rain gauge will be ignored until resolved."); 		 rainAlertSent = true; 
	 } 
 } else { 
	 rainGaugeActive = (rainADC < RAIN_THRESHOLD); 
	 rainFailCount   = 0; 
	 rainAlertSent   = false; 
 } 
 // 4. Humidity spike detection 
 bool humiditySpike = (dhtFailCount < MAX_SENSOR_FAILURES && 	 lastHumidity >= HUMIDITY_SPIKE); 
 //5. Composite rain signal (OR of valid sources) 
77 
 

 bool rainSignal = rainGaugeActive || humiditySpike || rainForecast; 
 // 6. Soil moisture sensors 
 int rawA = readAvg(SOIL_A_PIN); 
 int rawB = readAvg(SOIL_B_PIN); 
 bool soilAValid = (rawA > SOIL_DISCONNECTED_LOW && 
				 rawA < SOIL_DISCONNECTED_HIGH); 
 bool soilBValid = (rawB > SOIL_DISCONNECTED_LOW && 
				 rawB < SOIL_DISCONNECTED_HIGH); 
 if (!soilAValid) { 
	 soilAFailCount++; 
	 Serial.print(F("Soil sensor A out of range (ADC=")); 
	 Serial.print(rawA); Serial.print(F("). Fail count: ")); 
	 Serial.println(soilAFailCount); 
	 if (soilAFailCount >= MAX_SENSOR_FAILURES && !soilAAlertSent) { 		 sendSMS(FARMER_NUMBER, 
			 "SENSOR ALERT: Soil moisture sensor A (dry bed, pin A0) " 
			 "is reading out of range. Check probe insertion and wiring. " 
			 "Bed A irrigation is paused until sensor recovers."); 
		 soilAAlertSent = true; 
	 } 
 } else { 
	 soilAFailCount = 0; 
78 
 

	 soilAAlertSent = false; 
	 lastMoistA = toPercent(rawA, DRY_A, WET_A); 
 } 
 // Handle Sensor B validity 
 if (!soilBValid) { 
	 soilBFailCount++; 
	 Serial.print(F("Soil sensor B out of range (ADC=")); 
	 Serial.print(rawB); Serial.print(F("). Fail count: ")); 
	 Serial.println(soilBFailCount); 
	 if (soilBFailCount >= MAX_SENSOR_FAILURES && !soilBAlertSent) { 		 sendSMS(FARMER_NUMBER, 
			 "SENSOR ALERT: Soil moisture sensor B (wet bed, pin A1) " 
			 "is reading out of range. Check probe insertion and wiring. " 
			 "Bed B irrigation is paused until sensor recovers."); 
		 soilBAlertSent = true; 
	 } 
 } else { 
	 soilBFailCount = 0; 
	 soilBAlertSent = false; 
	 lastMoistB = toPercent(rawB, DRY_B, WET_B); 
 } 
 // 7. Serial data log  
 Serial.print(F("ms="));       Serial.print(millis()); 
 Serial.print(F(",mA="));      Serial.print(soilAValid ? lastMoistA : -1); 
79 
 

 Serial.print(F("%,mB="));     Serial.print(soilBValid ? lastMoistB : -1);  Serial.print(F("%,tank="));   Serial.print(lastTankOK ? "OK" : "LOW");  Serial.print(F(",rain="));    Serial.print(rainSignal ? "YES" : "NO"); 
 Serial.print(F(",gauge="));   Serial.print(rainADC); 
 Serial.print(F(",hum="));     Serial.print(lastHumidity, 0); 
 Serial.print(F("%,temp="));   Serial.print(lastTemp, 1); 
 Serial.print(F("C,fcst="));   Serial.println(rainForecast ? "RAIN" : "CLEAR"); 
 //8. Tank check  
 if (!lastTankOK) { 
	 Serial.println(F("TANK LOW or UNSTABLE — skipping both beds")); 	 if (!tankAlertSent) { 
		 sendSMS(FARMER_NUMBER, 
			 "WARNING: Tank is low or sensor is unstable. " 
			 "Irrigation paused. Please check water tank and sensor."); 
		 tankAlertSent = true; 
	 } 
	 return; 
 } 
 //9. Per-bed evaluation 
 if (soilAValid) { 
	 evaluateBed("A", rawA, lastMoistA, THRESH_A,VALVE_A_PIN, 		 cooldownA, lastIrrigationA, rainSignal); 
80 
 

 } else { 
	 Serial.println(F("Bed A skipped — sensor invalid"));  } 
 if (soilBValid) { 
		 evaluateBed("B", rawB, lastMoistB, THRESH_B,VALVE_B_PIN, 			 cooldownB, lastIrrigationB, rainSignal); 
 } else { 
		 Serial.println(F("Bed B skipped — sensor invalid")); 
 } 
} 
// Per-bed irrigation decision 
void evaluateBed(const char* name, int rawADC, int pct, int threshold, 	 int valvePin, unsigned long &cooldown, 
	 unsigned long &lastIrrigation, bool rainSignal) { 
 Serial.print(F("Bed ")); Serial.print(name); 
 Serial.print(F(" ADC:")); Serial.print(rawADC); 
 Serial.print(F(" (")); Serial.print(pct); Serial.print(F("%)")); 
 // 1. Moisture check 
 if (rawADC <= threshold) { 
	 Serial.println(F(" — moist, no action")); 
81 
 

	 return; 
 } 
 // 2. Cooldown check 
 if (cooldown != 0 && millis() - cooldown < COOLDOWN_PERIOD) { 	 unsigned long rem = (COOLDOWN_PERIOD - (millis() - cooldown)) / 1000; 	 Serial.print(F(" — cooldown, ")); 
	 Serial.print(rem); Serial.println(F("s left")); 
	 return; 
 } 
 // 3. Rain signal check 
 if (rainSignal) { 
	 Serial.println(F(" — DRY but rain signal active, withheld")); 	 String msg = "Bed "; msg += name; 
	 msg += " is dry but rain detected. Irrigation withheld."; 	 sendSMS(FARMER_NUMBER, msg.c_str()); 
	 return; 
 } 
 // 4. All clear — irrigate 
 Serial.print(F(" — DRY, irrigating")); 
 irrigateBed(name, valvePin, cooldown, lastIrrigation); } 
82 
 

//Safe irrigation sequence 
void irrigateBed(const char* name, int valvePin, 
		 unsigned long &cooldown, unsigned long &lastIrrigation) {  stopFlag = false; 
 // Open valve 
 digitalWrite(valvePin, HIGH); 
 Serial.print(F(" → valve OPEN")); 
 if (waitWithStop(VALVE_PRE_DELAY)) { 	 digitalWrite(valvePin, LOW); 
	 Serial.println(F(" → STOPPED before pump")); 	 return; 
 } 
 // Start pump 
 digitalWrite(PUMP_PIN, HIGH); 
 Serial.print(F(" → pump ON")); 
 if (waitWithStop(PUMP_RUN_TIME)) { 
 digitalWrite(PUMP_PIN, LOW); 
 delay(VALVE_POST_DELAY); 
 digitalWrite(valvePin, LOW); 
83 
 

		 Serial.println(F(" → STOPPED mid-cycle")); 
		 return; 
 } 
 // Stop pump 
 digitalWrite(PUMP_PIN, LOW); 
 Serial.print(F(" → pump OFF")); 
 delay(VALVE_POST_DELAY); 
 digitalWrite(valvePin, LOW); 
 Serial.println(F(" → valve CLOSED")); 
 cooldown       = millis(); 
 lastIrrigation = millis(); 
 String msg = "Bed "; msg += name; 
 msg += " irrigated ("; 
 msg += (valvePin ==VALVE_A_PIN) ? lastMoistA : lastMoistB;  msg += "% moisture). Cooldown started."; 
 sendSMS(FARMER_NUMBER, msg.c_str()); 
} 
bool waitWithStop(unsigned long duration) { 
 unsigned long start = millis(); 
 while (millis() - start < duration) { 
		 if (stopFlag) return true; 
		 if (gsm.available()) { 
			 String line = gsm.readStringUntil('\n'); 
			 if (line.indexOf("STOP") >= 0) stopFlag = true; 
84 
 

		 } 
		 delay(200); 
 } 
 return false; 
} 
// SUBSYSTEM 2 — SMS COMMUNICATION void checkSMSInbox() { 
 gsm.println(F("AT+CMGL=\"REC UNREAD\""));  delay(800); 
 String response = ""; 
 unsigned long start = millis(); 
 while (millis() - start < 1500) { 
	 while (gsm.available()) response += (char)gsm.read();  } 
 if (response.indexOf("+CMGL:") < 0) return; 
 int cmglIdx = response.indexOf("+CMGL:"); 
 while (cmglIdx >= 0) { 
	 int commaPos  = response.indexOf(",", cmglIdx); 
	 String msgIdx = response.substring(cmglIdx + 7, commaPos); 	 msgIdx.trim(); 
85 
 

 int q1     = response.indexOf("\"", commaPos);  int q2     = response.indexOf("\"", q1 + 1);  int q3     = response.indexOf("\"", q2 + 1);  int q4     = response.indexOf("\"", q3 + 1);  String sender = response.substring(q3 + 1, q4);  sender.trim(); 
 int bodyStart = response.indexOf("\n", q4); 
 int bodyEnd   = response.indexOf("\n", bodyStart + 1);  if (bodyEnd < 0) bodyEnd = response.length(); 
 String body = response.substring(bodyStart + 1, bodyEnd);  body.trim(); 
 body.toUpperCase(); 
 Serial.print(F("SMS from: ")); Serial.print(sender);  Serial.print(F(" | Body: ")); Serial.println(body); 
 if (isRegisteredFarmer(sender)) { 
	 handleFarmerCommand(body); 
 } else if (isNetworkAlert(sender, body)) { 
	 String fwd = "NETWORK ALERT: "; 
	 fwd += response.substring(bodyStart + 1, bodyEnd); 	 sendSMS(FARMER_NUMBER, fwd.c_str()); 
86 
 

	 Serial.println(F("Network alert forwarded.")); 
 } else { 
	 Serial.println(F("Unknown sender — discarded."));  } 
 // Delete processed message 
 gsm.print(F("AT+CMGD=")); gsm.println(msgIdx);  delay(400); 
		 cmglIdx = response.indexOf("+CMGL:", cmglIdx + 1);  } 
} 
bool isRegisteredFarmer(const String &sender) { 
 String farmerLast10 = String(FARMER_NUMBER).substring( 			 String(FARMER_NUMBER).length() - 10);  String senderLast10 = sender.length() >= 10 
		 ? sender.substring(sender.length() - 10) 
		 : sender; 
 return (senderLast10 == farmerLast10); 
} 
bool isNetworkAlert(const String &sender, const String &body) {  bool shortCode = (sender.length() <= 6 && isAllDigits(sender)); 
87 
 

 bool hasKeyword = (body.indexOf("BALANCE") >= 0 || 
			 body.indexOf("DATA")    >= 0 || 
			 body.indexOf("EXPIR")   >= 0 || 
			 body.indexOf("RECHARGE")>= 0 || 
			 body.indexOf("CREDIT")  >= 0 || 
			 body.indexOf("LOW")     >= 0); 
 return shortCode && hasKeyword; 
} 
bool isAllDigits(const String &s) { 
 for (unsigned int i = 0; i < s.length(); i++) { 
		 if (!isdigit(s[i])) return false; 
 } 
 return true; 
} 
void handleFarmerCommand(const String &cmd) { 
 if (cmd == "STATUS") { 
		 handleStatus(); 
 } else if (cmd == "IRRIGATE A") { 
		 handleManualIrrigate("A",VALVE_A_PIN, cooldownA, lastIrrigationA);  } else if (cmd == "IRRIGATE B") { 
		 handleManualIrrigate("B",VALVE_B_PIN, cooldownB, lastIrrigationB);  } else if (cmd == "STOP") { 
		 stopFlag = true; 
		 digitalWrite(PUMP_PIN,    LOW); 
88 
 

		 digitalWrite(VALVE_A_PIN, LOW); 
		 digitalWrite(VALVE_B_PIN, LOW); 
		 sendSMS(FARMER_NUMBER, "STOP received. Pump halted, valves closed."); 		 Serial.println(F("STOP executed.")); 
 } else { 
		 // Unknown command — inform farmer 
		 String msg = "Unknown command received: "; 
		 msg += cmd; 
		 msg += ".Valid commands: STATUS, IRRIGATE A, IRRIGATE B, STOP"; 		 sendSMS(FARMER_NUMBER, msg.c_str()); 
		 Serial.print(F("Unknown command: ")); Serial.println(cmd); 
 } 
} 
void handleStatus() { 
 unsigned long now = millis(); 
 String msg = "STATUS:\n"; 
 // Soil moisture — flag if sensor was invalid 
 msg += "Bed A: "; 
 if (soilAFailCount >= MAX_SENSOR_FAILURES) { 
		 msg += "SENSOR ERROR"; 
 } else { 
		 msg += lastMoistA; msg += "%"; 
 } 
 msg += "\nBed B: "; 
89 
 

 if (soilBFailCount >= MAX_SENSOR_FAILURES) { 
		 msg += "SENSOR ERROR"; 
 } else { 
		 msg += lastMoistB; msg += "%"; 
 } 
 msg += "\nTank: "; 
 msg += (lastTankOK ? "OK" : "LOW"); 
 msg += "\nTemp: "; 
 if (dhtFailCount >= MAX_SENSOR_FAILURES) { 
		 msg += "SENSOR ERROR"; 
 } else { 
		 msg += (int)lastTemp; msg += "C  Hum:"; msg += (int)lastHumidity; msg += "%";  } 
 msg += "\nRain: "; 
 msg += (lastRainADC < RAIN_THRESHOLD ? "Active" : "None"); 
 msg += "\nLast irrig A: "; 
 msg += (lastIrrigationA > 0) ? String((now - lastIrrigationA)/60000) + "min ago" : "never"; 
 msg += "\nLast irrig B: "; 
 msg += (lastIrrigationB > 0) ? String((now - lastIrrigationB)/60000) + "min ago" : "never"; 
 sendSMS(FARMER_NUMBER, msg.c_str()); 
 Serial.println(F("STATUS sent.")); 
} 
void handleManualIrrigate(const char* name, int valvePin, 
90 
 

			 unsigned long &cooldown, 
			 unsigned long &lastIrrigation) { 
 if (!lastTankOK) { 
		 sendSMS(FARMER_NUMBER, "Cannot irrigate — tank is LOW or sensor unstable."); 
		 return; 
 } 
 // Check if the relevant sensor has failed 
 if 	((valvePin 	==VALVE_A_PIN 	&& 	soilAFailCount 	>= 
MAX_SENSOR_FAILURES) || 
 (valvePin 	==VALVE_B_PIN 	&& 	soilBFailCount 	>= 
MAX_SENSOR_FAILURES)) { 
		 String msg = "WARNING: Soil sensor for Bed "; 
		 msg += name; 
		 msg += " is faulty. Proceeding with manual irrigation anyway."; 
		 sendSMS(FARMER_NUMBER, msg.c_str());} 
 String confirm = "Manual irrigation started for Bed "; confirm += name; 
 sendSMS(FARMER_NUMBER, confirm.c_str()); 
 irrigateBed(name, valvePin, cooldown, lastIrrigation); 
} 
// SUBSYSTEM 3 — WEATHER FORECAST (Open-Meteo, no API key needed) void fetchWeatherForecast() { 
 Serial.println(F("Weather: opening GPRS bearer...")); 
 gsmSend("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""); delay(300); 
91 
 

 String apnCmd = "AT+SAPBR=3,1,\"APN\",\"";  apnCmd += APN; apnCmd += "\""; 
 gsmSend(apnCmd.c_str()); delay(300); 
 gsmSend("AT+SAPBR=1,1"); delay(5000); 
 String sapbr = gsmQuery("AT+SAPBR=2,1", 2000); 
 if (sapbr.indexOf(",1,") < 0) { 
	 Serial.println(F("Weather: GPRS did not open. Retaining last forecast.")); 	 gsmSend("AT+SAPBR=0,1"); 
	 return; 
 } 
 gsmSend("AT+HTTPINIT");            delay(300); 
 gsmSend("AT+HTTPPARA=\"CID\",1"); delay(300); 
 // Open-Meteo — no API key needed, free, no registration 
 gsmSend("AT+HTTPPARA=\"URL\",\"https://api.open-
meteo.com/v1/forecast?latitude=9.07&longitude=7.4&current=rain,showers&timezo ne=GMT\""); 
 delay(300); 
 gsmSend("AT+HTTPACTION=0"); delay(10000); 
 String resp = gsmQuery("AT+HTTPREAD", 5000); 
 gsmSend("AT+HTTPTERM"); delay(300); 
 gsmSend("AT+SAPBR=0,1"); delay(1000); 
 // Check response is not empty 
92 
 

 if (resp.length() < 20) { 
	 Serial.println(F("Weather: empty response. Retaining last forecast.")); 	 return; 
 } 
 // Parse "rain": value 
 float rainVal    = 0.0; 
 float showersVal = 0.0; 
 int rainPos = resp.indexOf("\"rain\":"); 
 if (rainPos >= 0) { 
	 int valStart = rainPos + 7; 
	 int valEnd   = resp.indexOf(",", valStart); 
	 if (valEnd < 0) valEnd = resp.indexOf("}", valStart); 
	 rainVal = resp.substring(valStart, valEnd).toFloat(); 
 } 
 int showersPos = resp.indexOf("\"showers\":"); 
 if (showersPos >= 0) { 
	 int valStart = showersPos + 10; 
	 int valEnd   = resp.indexOf(",", valStart); 
	 if (valEnd < 0) valEnd = resp.indexOf("}", valStart); 
	 showersVal = resp.substring(valStart, valEnd).toFloat(); 
 } 
 // If either value is above 0.0 mm, rain is active 
 rainForecast = (rainVal > 0.0 || showersVal > 0.0); 
 Serial.print(F("Weather: rain="));   Serial.print(rainVal); 
93 
 

 Serial.print(F("mm showers="));      Serial.print(showersVal); 
 Serial.print(F("mm → forecast: ")); Serial.println(rainForecast ? "RAIN" : "CLEAR"); 
 if (rainForecast) { 
		 sendSMS(FARMER_NUMBER, "Weather: rain detected in your area. Irrigation will be withheld."); 
 } 
} 
// SUBSYSTEM 4 — BALANCE MONITORING 
void checkSimBalance() { 
 Serial.println(F("Balance: sending USSD...")); 
 // MTN: *556#  |  Airtel: *123#  |  Glo: *124*1#  |  9mobile: *228# 
 gsmSend("AT+CUSD=1,\"*556#\",15"); 
 delay(8000); 
 String ussd = ""; 
 unsigned long start = millis(); 
 while (millis() - start < 3000) { 
		 while (gsm.available()) ussd += (char)gsm.read(); 
 } 
 if (ussd.indexOf("+CUSD:") < 0) { 
		 Serial.println(F("Balance: no USSD response received.")); 
		 return; 
 } 
 int q1 = ussd.indexOf("\""); 
94 
 

 int q2 = ussd.indexOf("\"", q1 + 1); 
 String balInfo = ussd.substring(q1 + 1, q2); 
 balInfo.trim(); 
 if (balInfo.length() == 0) { 
	 Serial.println(F("Balance: USSD response was empty.")); 	 return; 
 } 
 String msg = "WEEKLY BALANCE:\n"; msg += balInfo;  sendSMS(FARMER_NUMBER, msg.c_str()); 
 Serial.print(F("Balance forwarded: ")); Serial.println(balInfo); } 
// UTILITY FUNCTIONS 
int readAvg(int pin) { 
 long sum = 0; 
 for (int i = 0; i < 20; i++) { 
		 sum += analogRead(pin); 
		 delay(25); 
 } 
 return (int)(sum / 20); 
} 
int toPercent(int raw, int dryVal, int wetVal) { 
95 
 

 return constrain(map(raw, dryVal, wetVal, 0, 100), 0, 100); 
} 
void gsmSend(const char* cmd) { 
 gsm.println(cmd); 
 delay(150); 
 while (gsm.available()) Serial.write(gsm.read()); 
} 
String gsmQuery(const char* cmd, unsigned long timeout) { 
 gsm.println(cmd); 
 String resp = ""; 
 unsigned long start = millis(); 
 while (millis() - start < timeout) { 
		 while (gsm.available()) resp += (char)gsm.read(); 
 } 
 return resp; 
}void sendSMS(const char* number, const char* message) { 
 gsm.print(F("AT+CMGS=\"")); gsm.print(number); gsm.println(F("\""));  delay(500); 
 gsm.print(message); 
 delay(100); 
 gsm.write(26);   // Ctrl+Z 
 delay(3000); 
 Serial.print(F("SMS → ")); Serial.println(number);}
