#include <esp_task_wdt.h>
#include <esp32-hal-log.h>

#if DEBUG
#define DUMP_AT_COMMANDS
#endif

// Cellular Modem
#define TINY_GSM_DEBUG Serial
#define TINY_GSM_MODEM_BG96
#define BG95_UART_TX 4
#define BG95_UART_RX 15
#define BG95_PWR_KEY 2
#define BG95_RESET 12
#define BG95_WAKEUP_LTE 25
#define SerialAT Serial1
#define SerialPMS Serial2
#ifndef TINY_GSM_YIELD
#define TINY_GSM_YIELD() \
  { delay(TINY_GSM_YIELD_MS); }
#endif
// LED
#define NUM_LEDS 4
#define LED_PIN 32
#if DEBUG
#define BRIGHTNESS  25
#else
#define BRIGHTNESS  255
#endif

#define TINY_GSM_YIELD_MS 50
#define TINY_GSM_RX_BUFFER  1024U

#include <Arduino.h>
#include <TinyGsmClient.h>
#include <FastLED.h>

// GSM
bool disable_cell = false;
constexpr char apn[] = "hologram";
constexpr char gprsUser[] = "";
constexpr char gprsPass[] = "";
int cell_connect_count = 0;

// Setup checks
bool check_cell = false;

CRGB leds[NUM_LEDS];

// Initialize GSM modem
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

// Initialize GSM client
TinyGsmClient client(modem);

/**
 * UTILITY FUNCTIONS
*/

bool set_led_color(String value)
{
    if (value == "red")
    {
        log_d("Setting LED to %s", value.c_str());
        leds[0] = CRGB::Red;
        leds[1] = CRGB::Red;
        leds[2] = CRGB::Red;
        leds[3] = CRGB::Red;
        FastLED.show();
        return true;
    } else if (value == "green"){
        log_d("Setting LED to %s", value.c_str());
        leds[0] = CRGB::Green;
        leds[1] = CRGB::Green;
        leds[2] = CRGB::Green;
        leds[3] = CRGB::Green;
        FastLED.show();
        return true;
    } else if (value == "blue"){
        log_d("Setting LED to %s", value.c_str());
        leds[0] = CRGB::Blue;
        leds[1] = CRGB::Blue;
        leds[2] = CRGB::Blue;
        leds[3] = CRGB::Blue;
        FastLED.show();
        return true;
    } else if (value == "white"){
        log_d("Setting LED to %s", value.c_str());
        leds[0] = CRGB::White;
        leds[1] = CRGB::White;
        leds[2] = CRGB::White;
        leds[3] = CRGB::White;
        FastLED.show();
        return true;
    } else if (value == "black"){
        log_d("Setting LED to %s", value.c_str());
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Black;
        leds[2] = CRGB::Black;
        leds[3] = CRGB::Black;
        FastLED.show();
        return true;
    } else if (value == "purple"){
        log_d("Setting LED to %s", value.c_str());
        leds[0] = CRGB::Purple;
        leds[1] = CRGB::Purple;
        leds[2] = CRGB::Purple;
        leds[3] = CRGB::Purple;
        FastLED.show();
        return true;
    } else if (value == "darkOrange"){
        log_d("Setting LED to %s", value.c_str());
        leds[0] = CRGB::DarkOrange;
        leds[1] = CRGB::DarkOrange;
        leds[2] = CRGB::DarkOrange;
        leds[3] = CRGB::DarkOrange;
        FastLED.show();
        return true;
    } else if (value == "pink"){
        log_d("Setting LED to %s", value.c_str());
        leds[0] = CRGB::Pink;
        leds[1] = CRGB::Pink;
        leds[2] = CRGB::Pink;
        leds[3] = CRGB::Pink;
        FastLED.show();
        return true;
    } else if (value == "yellow"){
        log_d("Setting LED to %s", value.c_str());
        leds[0] = CRGB::Yellow;
        leds[1] = CRGB::Yellow;
        leds[2] = CRGB::Yellow;
        leds[3] = CRGB::Yellow;
        FastLED.show();
        return true;
    }

    return false;
}

bool removeRPLMN()
{
    modem.sendAT(GF("+crsm=176,28542,0,0,11"));
    if (modem.waitResponse(5000, GF("FFFFFFFFFFFFFFFFFFFFFF")) != 1){
        log_d("Stored network found, clearing it.");
        modem.sendAT(GF("+crsm=214,28542,0,0,11,\"FFFFFFFFFFFFFFFFFFFFFF\""));
        if (modem.waitResponse(10000, GF("144,0,\"\"")) == 1){
            log_d("Network cleared.");
            modem.restart();
            return true;
        } else {
            log_d("Could not clear the network.");
        }
    } else {
        return true;
    }

    return false;
}

bool c_connected()
{
    return modem.isGprsConnected();
}

bool c_connect()
{
    log_d("In cell connect");

    cell_connect_count += 1;

    //if (cell_connect_count == 1){
    //    bool _remove_rplmn = removeRPLMN();
    //    if (!_remove_rplmn){
    //        log_d("Could not remove stored network.");
    //        return false;
    //    }else{
    //        set_led_color("yellow");
    //        delay(1000);
    //    }
    //}

    set_led_color("white");

    log_d("Waiting for network...");
    if (!modem.waitForNetwork(1200000L, true)) {
        log_d(" fail");
        set_led_color("red");
        return false;
    }
    log_d(" success");

    set_led_color("blue");

    delay(1000);

    if (modem.isNetworkConnected()) { log_d("Network connected"); }

    // GPRS connection parameters are usually set after network registration
    log_d("Connecting to GPRS");

    int _before_gprs = millis();

    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        log_d(" fail");
        set_led_color("red");

        return false;
    }
    log_d(" success");

    if (modem.isGprsConnected()) { 
        log_d("GPRS connected");

        int csq = modem.getSignalQuality();
        log_d("Signal stength: %s", String(csq));

        set_led_color("purple");

        return true; 
    } else {
        set_led_color("red");
    }

    log_d("GPRS NOT connected");

    return false;
}

/**
 * CORE FUNCTIONS
*/

void loop()
{
    delay(50);
}

void setup()
{
    Serial.begin(9600);
    Serial.setDebugOutput(true);

    delay(6000);

    log_d("Resetting, and powering on modem");

    pinMode(BG95_PWR_KEY, OUTPUT);
    pinMode(BG95_RESET, OUTPUT);
    pinMode(BG95_WAKEUP_LTE, OUTPUT);
    digitalWrite(BG95_RESET, LOW);
    digitalWrite(BG95_WAKEUP_LTE, LOW);
    // This code was found on a BG95 Arduino library, but doesn't seem to be necessary
    // Found it works better without it.
    //digitalWrite(BG95_PWR_KEY, HIGH);
    //delay(2000);
    digitalWrite(BG95_PWR_KEY, LOW);

    FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);
    set_led_color("black");

    /*
    The main UART interface supports 9600 bps, 19200 bps, 38400 bps, 57600 bps, 115200 bps,
230400 bps, 460800 bps and 921600 bps baud rates, and the default is 115200 bps. 
    */
    SerialAT.begin(115200, SERIAL_8N1, BG95_UART_TX, BG95_UART_RX, false);

    delay(6000);

    // Factory default
    bool ret = modem.factoryDefault();
    log_d("Modem factory default: %s", ret ? "OK" : "fail");

    modem.sendAT(GF("+QPRTPARA=1"));
    modem.waitResponse(30000L);
    
    modem.sendAT(GF("+QPRTPARA=3"));
    modem.waitResponse(30000L);

    //log_d("Initializing modem...");
    //modem.init();
    log_d("Restarting modem");
    modem.restart();

    delay(6000);

    String modemInfo = modem.getModemInfo();
    log_d("Modem Info: %s", modemInfo.c_str());
    int status = modem.getSimStatus();
    log_d("SIM Status: %s", String(status));

    //bool _set_apn = modem.gprsConnect(apn);
    //log_d("Setting APN");

    bool _remove_rplmn = removeRPLMN();
    log_d("Removing any stored entries for networks.");

    // Verbose error messages
    //modem.sendAT(GF("+CMEE=2"));
    //modem.waitResponse();

    //modem.sendAT(GF("+QCFG=\"BAND\""));
    //modem.waitResponse();
    
    //modem.sendAT(GF("+QCFG=\"nwscanmode\""));
    //modem.waitResponse();

    //modem.sendAT(GF("+QCFG=\"nwscanseq\""));
    //modem.waitResponse(10000L);

    // This doesn't take affect until a restart
    // Configure the RAT search sequence in the following order: LTE Cat M1 -> GSM -> LTE Cat NB1.
    //modem.sendAT(GF("+QCFG=\"nwscanseq\",020103,1"));
    //modem.waitResponse(10000L);

    //modem.sendAT(GF("+QCFG=\"nwscanseq\""));
    //modem.waitResponse(10000L);

    //modem.sendAT(GF("+COPS=0"));
    //modem.waitResponse();

    //modem.sendAT(GF("+CMNB=?"));
    //modem.waitResponse();

    //modem.sendAT(GF("+CMNB?"));
    //modem.waitResponse();

    //modem.sendAT(GF("+CDGCONT?"));
    //modem.waitResponse();

    //Set preferred mode to CAT-M
    //AT+CMNB=1

    //modem.sendAT(GF("+QPINC?"));
    //modem.waitResponse();

    // Unlock a SIM PUK request
    //modem.sendAT(GF("+CPIN=23513244,1234"));
    //modem.waitResponse();
    // Status of SIM LOCK
    //modem.sendAT(GF("+CLCK=\"SC\",2"));
    //modem.waitResponse();
    // Turn off SIM PIN
    //modem.sendAT(GF("+CLCK=\"SC\",0,\"1234\""));
    //modem.waitResponse();

    String _iccid = modem.getSimCCID();
    log_i("SIMICCID|||%s|||", _iccid.c_str());
    
    // https://www.dragino.com/downloads/downloads/datasheet/other_vendors/BG95/Software/Quectel_BG95&BG77&BG600L_Series_QuecCell_Application_Note_V1.0.pdf
    //modem.sendAT(GF("+QENG=?"));
    //modem.waitResponse();

    //modem.sendAT(GF("+QENG=\"servingcell\""));
    //modem.waitResponse();

    //modem.sendAT(GF("+QCFG=\"iotopmode\""));
    //modem.waitResponse();

    //modem.sendAT(GF("+QCFG=\"servicedomain\""));
    //modem.waitResponse();

    //modem.sendAT(GF("+CREG=1"));
    //modem.waitResponse();

    //modem.sendAT(GF("+CREG?"));
    //modem.waitResponse();

    //modem.sendAT(GF("+CGREG=1"));
    //modem.waitResponse();

    //modem.sendAT(GF("+CGREG?"));
    //modem.waitResponse();

    // Returning "no network service"
    //modem.sendAT(GF("+CGATT=1"));
    //modem.waitResponse(180000L);
    
    //modem.sendAT(GF("+CGATT?"));
    //modem.waitResponse();
    
    //modem.sendAT(GF("+COPS?"));
    //modem.waitResponse();

    //modem.sendAT(GF("+CFUN?"));
    //modem.waitResponse();

    //modem.sendAT(GF("+CGREG?"));
    //modem.waitResponse();

    //modem.sendAT(GF("+CREG?"));
    //modem.waitResponse();
    
    //modem.sendAT(GF("+COPS=?"));
    //modem.waitResponse(360000L);

    modem.sendAT(GF("+ATI"));
    modem.waitResponse();

    modem.sendAT(GF("+CGMI"));
    modem.waitResponse();

    modem.sendAT(GF("+CGMM"));
    modem.waitResponse();

    modem.sendAT(GF("+GMM"));
    modem.waitResponse();

    modem.sendAT(GF("+CPIN?"));
    modem.waitResponse();

    modem.sendAT(GF("+CCID"));
    modem.waitResponse();

    modem.sendAT(GF("+CRSM=176,28539,0,0,12"));
    modem.waitResponse();

    modem.sendAT(GF("+CFUN?"));
    modem.waitResponse();

    modem.sendAT(GF("+CSQ"));
    modem.waitResponse();

    modem.sendAT(GF("+CREG?"));
    modem.waitResponse();

    modem.sendAT(GF("+CGREG?"));
    modem.waitResponse();

    modem.sendAT(GF("+CEREG?"));
    modem.waitResponse();

    modem.sendAT(GF("+CGDCONT?"));
    modem.waitResponse();

    modem.sendAT(GF("+CGACT?"));
    modem.waitResponse();

    // Force TMOBILE, to avoid Verizon, which is not allowed currently
    //modem.sendAT(GF("+COPS=0,0,\"TMO\""));
    //modem.waitResponse();

    // Force AT&T
    //modem.sendAT(GF("+COPS=2,0,\"310410\""));
    //modem.waitResponse();

    //at+cops=1,2,"310260",8 # T-Mobile
    //at+cops=1,2,"310410",8 # AT&T
    //at+cops=1,2,"311480",8 # Verizon
    //at+cops=0,0 # To set it back to automatic selection

    modem.sendAT(GF("+COPS?"));
    modem.waitResponse(10000L);
    
    modem.sendAT(GF("+COPS=0,0"));
    modem.waitResponse(30000L);

    modem.sendAT(GF("+COPS?"));
    modem.waitResponse(10000L);

    modem.sendAT(GF("+COPS=?"));
    modem.waitResponse(300000L);

    bool _cell = c_connect();
    log_d("After cell");

    if (c_connected()){
        log_d("Connected");
        log_i("ALL CHECKS PASSED");
    } else {
        log_d("CONNECTION FAILED");
    }

    modem.sendAT(GF("+COPS?"));
    modem.waitResponse();
}