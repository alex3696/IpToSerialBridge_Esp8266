#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
// #include <WiFiServer.h>
/*
   Bridge UDP/TCP <-> serial.
*/
//----------------------------------------------------------------------
// output debugging information to UART1 TX
//#define DEBUG_PRINT
#ifdef DEBUG_PRINT
#define DPPRINTLN(...) Serial1.println(__VA_ARGS__)
#define DPPRINT(...) Serial1.print(__VA_ARGS__)
#else
#define DPPRINTLN(...)
#define DPPRINT(...)
#endif
//----------------------------------------------------------------------
// Serial config
const uint32_t RS_Speed = 2000000;	      // (921600) скорость соединения по Serial
const uint32_t RS_pin = 12;			          // выход для переключения направления передачи
const uint32_t Server_Timeout = 2000;     // таймаут сервера
#define BUF_MAX 256 // позволяет полностью загрузить весь пакет modbus

const size_t charTimeoutMicros = 1000000 / (RS_Speed /10) * 4; //20; // pause indicating the end of a continuous packet, таймаут Serial
uint8_t ipBuf[BUF_MAX];
uint8_t serBuf[BUF_MAX];
size_t serCount = 0;
size_t serLastByteMicros = 0;

const size_t channelTimeoutMillis = 2000; // Сброс источника через 2 сек тишины
size_t lastActivityMillis = 0;     // Для сброса lastSource

// откуда пришёл последний пакет
enum Source
{
  NONE,
  SOURCE_TCP,
  SOURCE_UDP
};
Source lastSource = NONE;
IPAddress udpRemoteIp;
uint16_t udpRemotePort;

//----------------------------------------------------------------------
// default WiFi client config
// String Statuses[] =  { "WL_IDLE_STATUS=0", "WL_NO_SSID_AVAIL=1", "WL_SCAN_COMPLETED=2", "WL_CONNECTED=3", "WL_CONNECT_FAILED=4", "WL_CONNECTION_LOST=5", "WL_DISCONNECTED=6"};
//  defaul WiFi AP config
WiFiMode wifi_mode = WiFiMode::WIFI_STA;
String ssidAPbase = "bs3";
String ssidAP = "";
String passwordAP = "spsksiam";
IPAddress ip_static(10, 10, 10, 10);
IPAddress ip_gateway(10, 10, 10, 1);
IPAddress ip_subnet(255, 255, 255, 0);
IPAddress ip_dns(10, 10, 10, 1);
//----------------------------------------------------------------------
const int Port = 502;
WiFiUDP udp;
WiFiServer server(Port);
WiFiClient tcpClient;
int wifiCondition = 0;
const size_t connectCheckPeriod = 300;
size_t connectCheckLast = 0;
//-----------------------------------------------------------------------------
// Quick pin output
void IRAM_ATTR WritePin(uint8_t pin, uint8_t val)
{
  if (pin < 16)
  {
    if (val)
      GPOS = (1 << pin);
    else
      GPOC = (1 << pin);
  }
  else if (pin == 16)
  {
    if (val)
      GP16O |= 1;
    else
      GP16O &= ~1;
  }
}
//----------------------------------------------------------------------
// ESP-12E module's onboard LED, used as status indicator
#define BUILTIN_LED 2
int LedState = 0;

void BUILTIN_LED_OFF()
{
  if (0 == LedState)
    return;
  WritePin(BUILTIN_LED, HIGH);
  LedState = 0;
}
void BUILTIN_LED_ON()
{
  if (LedState)
    return;
  WritePin(BUILTIN_LED, LOW);
  LedState = 1;
}
void BUILTIN_LED_TOOGLE()
{
  if (LedState)
    BUILTIN_LED_OFF();
  else
    BUILTIN_LED_ON();
}

//-----------------------------------------------------------------------------
unsigned long blinkPrevios = 0;
const unsigned long blinkPeriod = 1500;
void StatusBlink()
{
  switch (lastSource)
  {
    default: break;
    case SOURCE_TCP:
    case SOURCE_UDP:
      BUILTIN_LED_ON(); return;
      //blinkPeriod = 200; break;
  }
  if (millis() - blinkPrevios > blinkPeriod)
  {
    BUILTIN_LED_TOOGLE();
    blinkPrevios = millis();
  }
}
//-----------------------------------------------------------------------------
void ClearSerial()
{
  int clrRetry = BUF_MAX;
  size_t avail = 0;
  while (clrRetry-- && (avail = Serial.available()))
  {
    DPPRINT(F("serial reset the incoming byte = ")); DPPRINTLN(avail, DEC);
    size_t toRead = (avail > sizeof(ipBuf)) ? sizeof(ipBuf) : avail;
    if (toRead)
    {
      Serial.read(serBuf, toRead);
      serBuf[toRead - 1] = 0;
      DPPRINTLN((char*)buf);
    }
  }
}
//----------------------------------------------------------------------
void setup()
{
  ESP.wdtEnable(3000);
#ifdef DEBUG_PRINT
  // boot log when reset 74880
  // DEBUG SERIAL SETUP
  Serial1.begin(115200, SERIAL_8N1);
  Serial1.setTimeout(0);
  DPPRINTLN("---");
  DPPRINTLN("DEBUG SERIAL - open OK");
#else
  pinMode(BUILTIN_LED, OUTPUT);
  BUILTIN_LED_OFF();
#endif

  // SERIAL SETUP
  Serial.setRxBufferSize(2048);
  Serial.setTimeout(0);// не будем ожидать байты
  Serial.begin(RS_Speed, SERIAL_8N1);
  DPPRINTLN("setup serial port speed - OK");

  // SETUP WIFI MODE, VIA PIN_14
  // 1=WIFI_STA; 0=WIFI_AP
  pinMode(14, INPUT_PULLUP); //
  // digitalWrite(14, HIGH);

  ssidAP = ssidAPbase + "_" + WiFi.macAddress();

  int colon_index = ssidAP.indexOf(':');
  while (-1 != colon_index)
  {
    ssidAP.remove(colon_index, 1);
    colon_index = ssidAP.indexOf(':');
  }
  DPPRINTLN(ssidAP.c_str());
  WiFi.hostname(ssidAPbase); // common wifi config

  wifi_mode = digitalRead(14) ? WIFI_STA : WIFI_AP;

  if (WiFiMode::WIFI_STA == wifi_mode)
  {
    DPPRINT("WiFiMode::WIFI_STA ");
    if (!WiFi.getAutoConnect())
      WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
    int reconect_attempt = 10;
    while (--reconect_attempt && WiFi.status() != WL_CONNECTED)
    {
      ESP.wdtFeed();
      DPPRINT("try WiFi reconect attempt ");
      DPPRINTLN(reconect_attempt, DEC);
      BUILTIN_LED_TOOGLE();
      delay(150);
    } // while(--reconect_attempt)

    if (WiFi.status() != WL_CONNECTED) // autoconnect failed
    {
      DPPRINTLN("WiFi NOT reconected");
      WifiConnectReset();
    }
    else
    {
      DPPRINTLN("WiFi Reconnected");
      DPPRINT(F("IP address: "));
      DPPRINTLN(WiFi.localIP());
      WiFi.printDiag(Serial1);
      DPPRINT(F("RSSI: "));
      DPPRINTLN(WiFi.RSSI());
      DPPRINT(F("BSSID: "));
      DPPRINTLN(WiFi.BSSIDstr());
      StartIpServer();
    }
  }
  else
  {
    DPPRINT("WiFiMode::WIFI_AP ");
    WifiConnectReset();
  }
  DPPRINTLN("setup wifi common config - OK");

  DPPRINTLN("setup serial port alt output");
  Serial.flush();
  Serial.pins(15, 13);
  pinMode(RS_pin, OUTPUT);
  digitalWrite(RS_pin, 0);
  ClearSerial();

  wifi_set_sleep_type(NONE_SLEEP_T);
  // wifi_set_sleep_type(MODEM_SLEEP_T);
  // wifi_set_sleep_type(LIGHT_SLEEP_T);
}
//----------------------------------------------------------------------
void StartIpServer()
{
  udp.begin(Port);
  DPPRINTLN("UDP started");
  server.begin();
  server.setNoDelay(true);

  // https://ru.wikipedia.org/wiki/Алгоритм_Нейгла
  // https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/client-class.html
  wifiCondition = 10;
  DPPRINT("TCP server started: ");
  DPPRINTLN(WiFi.localIP());
}
//----------------------------------------------------------------------
void WifiConnectReset()
{
  DPPRINTLN("Wifi connect reset");
  ESP.wdtFeed();
  udp.stop();
  DPPRINTLN("UDP stopped");
  tcpClient.stop();
  server.stop();
  DPPRINTLN("TCP stopped");
  WiFi.disconnect(true);
  WiFi.softAPdisconnect(true);
  switch (wifi_mode)
  {
    default:
    case WIFI_STA:
      wifiCondition = 0;
      break;
    case WIFI_AP:
      wifiCondition = 2;
      break;
      // case WIFI_AP_STA:break;
      // case WIFI_OFF: break;
  }
}
//----------------------------------------------------------------------
void WifiConnectAP()
{
  DPPRINTLN("Start WIFI_AP");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssidAP.c_str(), passwordAP.c_str());
  WiFi.softAPConfig(ip_static, ip_gateway, ip_subnet);
  StartIpServer();
}
//----------------------------------------------------------------------
// Подключение к WiFi точке доступа и старт сервера
void WifiConnectSTA()
{
  DPPRINTLN("Try connect WIFI_STA");
  WiFi.mode(WIFI_STA);
  // WiFi.config(ip_static, ip_gateway, ip_subnet);
  // for(uint32_t i=0; i< sizeof(ssid)/sizeof(char); ++i)
  {
    WiFi.begin(ssidAP.c_str(), passwordAP.c_str());
    BUILTIN_LED_ON();
    if (WL_CONNECTED == WiFi.waitForConnectResult())
    {
      StartIpServer();
      return;
    }
  }
  WifiConnectReset();
}
//-----------------------------------------------------------------------------
void TransparentIo()
{
  size_t avail = 0;
  size_t toRead = 0;

  // ip to serial
  if (tcpClient && tcpClient.connected())
  {
    avail = tcpClient.available();
    toRead = (avail > sizeof(ipBuf)) ? sizeof(ipBuf) : avail;
    if (toRead)
    {
      toRead = tcpClient.read(ipBuf, toRead);
      DPPRINT(F("ip Read = ")); DPPRINTLN(toRead, DEC);
      //digitalWrite(RS_pin, 1); // Передача
      Serial.write(ipBuf, toRead);
      //Serial.flush();
      //digitalWrite(RS_pin, 0); // Прием
      DPPRINT(F("serial Write = ")); DPPRINTLN(toRead, DEC);
      lastSource = SOURCE_TCP;
      lastActivityMillis = millis();
      //serLastByteMicros = micros();
    }
  }
  else // UDP
  {
    avail = udp.parsePacket();
    toRead = (avail > sizeof(ipBuf)) ? sizeof(ipBuf) : avail;
    if (toRead > 0)
    {
      DPPRINT(F("UDP Read = ")); DPPRINTLN(toRead, DEC);
      udpRemoteIp = udp.remoteIP();
      udpRemotePort = udp.remotePort();
      udp.read(ipBuf, toRead);
      //digitalWrite(RS_pin, 1); // Передача
      Serial.write(ipBuf, toRead);
      //Serial.flush();
      //digitalWrite(RS_pin, 0); // Прием
      lastSource = SOURCE_UDP;
      lastActivityMillis = millis();
      //lastByteMicros = micros();
    }
  }
  // вычитываем всё, что есть в serial в буфер
  while (0 < (avail = Serial.available()) && sizeof(serBuf) > serCount )
  {
    size_t rest = sizeof(serBuf) - serCount;
    toRead = (avail > rest) ? rest : avail;
    if (toRead)
    {
      toRead = Serial.read(serBuf + serCount, toRead);
      DPPRINT(F("serial Readed = ")); DPPRINTLN(toRead, DEC);
      serCount += toRead;
      serLastByteMicros = micros();
    }
  }
  // Сборка и отправка по таймауту "тишины" (микросекунды)
  if ( sizeof(serBuf) <= serCount
       || (0 < serCount && (micros() - serLastByteMicros > charTimeoutMicros)))
  {
    switch (lastSource)
    {
      default: break;
      case SOURCE_TCP:
        tcpClient.write(serBuf, serCount);
        //tcpClient.flush();
        DPPRINT(F("serial to TcpIp = ")); DPPRINTLN(serCount, DEC);
        break;
      case SOURCE_UDP:
        udp.beginPacket(udpRemoteIp, udpRemotePort);
        udp.write(serBuf, serCount);
        udp.endPacket();
        //udp.flush();
        break;
    }
    serCount = 0;
  }
  // сбрасываем зависший источник
  if (lastSource != NONE && (millis() - lastActivityMillis > channelTimeoutMillis))
    lastSource = NONE;
}
//-----------------------------------------------------------------------------
void CheckTcpConnect()
{
  if ((millis() - connectCheckLast < connectCheckPeriod))
    return;
  connectCheckLast = millis();
  if (WIFI_STA == WiFi.getMode() && WL_CONNECTED != WiFi.status())
  {
    DPPRINTLN("WIFI_STA disconected - do RESET");
    WifiConnectReset();
    return;
  }
  WiFiClient newClient = server.accept();
  if (newClient)
  {
    if (tcpClient)
      tcpClient.stop();
    tcpClient = newClient;
    // Configure keep-alive with custom parameters
    // idle_sec: time before sending first keep-alive (default 7200 seconds)
    // intv_sec: interval between keep-alive probes (default 75 seconds)
    // count: number of probes before connection is considered broken (default 9)
    // tcpClient.disableKeepAlive();
    tcpClient.keepAlive(5, 3, 3);
    tcpClient.setNoDelay(true);
    //tcpClient.setTimeout(Server_Timeout);// default 1000
    DPPRINT(F("Connected socket client: "));
    DPPRINTLN(tcpClient.remoteIP());
    DPPRINT(F("condition = ")); DPPRINTLN(wifiCondition, DEC);
    ClearSerial();
  }
}
//-----------------------------------------------------------------------------
void WifiConnect()
{
  switch (wifiCondition)
  {
    default: // WiFi отключен
      DPPRINTLN("unknown condition RESET");
      WifiConnectReset();
      break;
    case 0: // пробуем подключиться к точке доступа
      WifiConnectSTA();
      break;
    case 2: // стартуем точку доступа
      WifiConnectAP();
      break;
    case 10: // Ожидание подключения и обработка
      CheckTcpConnect();
      TransparentIo();
      break; // case 10
  } // switch (wifiCondition)
} // void WifiConnect()
//----------------------------------------------------------------------
void loop()
{
  ESP.wdtFeed();
  WifiConnect();
  StatusBlink();
  //yield();
}
