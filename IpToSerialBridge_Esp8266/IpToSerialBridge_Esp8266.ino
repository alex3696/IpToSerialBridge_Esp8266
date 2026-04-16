#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
// #include <WiFiServer.h>
/*
 * Bridge UDP/TCP <-> serial.
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
#define RS_Speed 2000000	// (921600) скорость соединения по Serial
#define RS_pin 12			// выход для переключения направления передачи
#define RS_Timeout 10		// таймаут Serial
#define RS_StTimeout 500	// таймаут приема Serial
#define RS_StTimeout23 2500 // таймаут приема Serial для команды 23
#define Server_Timeout 2000 // таймаут сервера
#define BUF_MAX 256
//----------------------------------------------------------------------
#define bswap16(x) (((unsigned short)(x)) << 8 | ((unsigned short)(x)) >> 8)
//----------------------------------------------------------------------
// default WiFi client config
// String Statuses[] =  { "WL_IDLE_STATUS=0", "WL_NO_SSID_AVAIL=1", "WL_SCAN_COMPLETED=2", "WL_CONNECTED=3", "WL_CONNECT_FAILED=4", "WL_CONNECTION_LOST=5", "WL_DISCONNECTED=6"};
//----------------------------------------------------------------------
//  defaul WiFi AP config
WiFiMode wifi_mode = WiFiMode::WIFI_STA;
String ssidAPbase = "bs3";
String passwordAP = "spsksiam";
IPAddress ip_static(10, 10, 10, 10);
IPAddress ip_gateway(10, 10, 10, 1);
IPAddress ip_subnet(255, 255, 255, 0);
IPAddress ip_dns(10, 10, 10, 1);
//----------------------------------------------------------------------
WiFiUDP udp;
WiFiServer server(502);
WiFiClient tcpClient;
uint8_t wifiCondition = 0;
uint8_t buf[BUF_MAX];
String ssidAP = "";

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
uint8_t LedState = 0;

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
unsigned long gPreviosCycleQty = 0;
void idle_blink();

//----------------------------------------------------------------------
//----- Таблица для вычисления CRC: -----
const uint16_t Crc16Table[] PROGMEM = {
  0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
  0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
  0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
  0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
  0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
  0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
  0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
  0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
  0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
  0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
  0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
  0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
  0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
  0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
  0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
  0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
  0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
  0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
  0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
  0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
  0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
  0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
  0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
  0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
  0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
  0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
  0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
  0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
  0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
  0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
  0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
  0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};
//----------------------------------------------------------------------
uint16_t CRC_modbus;
void CRC_16(uint8_t *b, uint16_t count)
{
  CRC_modbus = 0xffff;
  for (uint16_t i = 0; i < count; i++)
  {
    uint8_t ptr = (CRC_modbus & 0x00ff) ^ ((uint8_t)b[i] & 0x00ff);
    CRC_modbus = pgm_read_word_near(Crc16Table + ptr) ^ (CRC_modbus >> 8);
  }
}
//-----------------------------------------------------------------------------
void ClearSerial()
{
  int clrRetry = BUF_MAX;
  int avail = 0;
  while (clrRetry-- && (avail = Serial.available()))
  {
    DPPRINT(F("serial reset the incoming byte = ")); DPPRINTLN(avail, DEC);
    int toRead = (avail >= BUF_MAX) ? BUF_MAX : avail;
    Serial.readBytes(buf, toRead);
    buf[toRead - 1] = 0;
    DPPRINTLN((char*)buf);
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
  Serial.setTimeout(0);//RS_Timeout
  Serial.begin(RS_Speed, SERIAL_8N1);
  DPPRINTLN("setup serial port speed - OK");

  // WIFI SETUP
  // PIN_14
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
  udp.begin(502);
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

//----------------------------------------------------------------------
int SerialBufCheck(int c)
{
  int packet_ok = 0, crc_ok = 0;

  if (c >= 5) // Минимальная длина ответа 5 байт (exception)
  {
    if ((buf[1] & 0x80) == 0) // Если нормальный ответ (нет exception)
    {
      switch (buf[1])
      {
        case 3:
        case 23:
          if (c == buf[2] + 5)
            packet_ok = 1;
          break;

        case 6:
        case 16:
          if (c == 8)
            packet_ok = 1;
          break;

        default:
          break;
      }
    }
    else
      packet_ok = 1;

    if (packet_ok)
    {
      CRC_16(buf, c); // проверка контрольной суммы
      if (CRC_modbus == 0)
        crc_ok = 1;
    }
  }

  return crc_ok;
}
//-----------------------------------------------------------------------------
int ServerReceive(int avail)
{
  int packet_ok = 0, crc_ok = 0, c = 0;

  if (avail >= 8)
  {
    c = tcpClient.readBytes(buf, avail); // минимальный пакет Modbus-RTU
    if (c == avail)
    {
      switch (buf[1])
      {
        case 3:
        case 6:
          if (c == 8)
            packet_ok = 1;
          break;

        case 16:
          if (c == buf[6] + 7 + 2)
            packet_ok = 1;
          break;

        case 23:
          if (c >= 13 && c == buf[10] + 11 + 2)
            packet_ok = 1;
          break;

        default:
          break;
      }

      if (packet_ok)
      {
        CRC_16(buf, c); // проверка контрольной суммы
        if (CRC_modbus == 0)
          crc_ok = 1;
      }
    }
  }
  return crc_ok;
}
//-----------------------------------------------------------------------------
void ModbusIo()
{
  int avail = tcpClient.available();
  if (avail > BUF_MAX)
    avail = BUF_MAX;
  if (avail)
  {
    if (ServerReceive(avail))
    {
      int rxCnt = 0, bufCnt = 0, rxCorrect = 0;
      char* bufPtr = (char*)buf;
      unsigned long lastTick = millis();
      unsigned long rsStTimeout = buf[1] == 23 ? RS_StTimeout23 : RS_StTimeout;
      BUILTIN_LED_ON();
      Serial.flush();
      Serial.write(buf, avail);
      DPPRINTLN("Begin data IO");
      DPPRINT(F("20: TCP data avail = ")); DPPRINTLN(avail, DEC);
      while (rxCorrect == 0 && ((millis() - lastTick) < rsStTimeout))
      {
        int serAvail = Serial.available();
        if (serAvail > 0)
        {
          // 1. Рассчитываем, сколько байт реально можем принять
          int toRead = (bufCnt + serAvail > BUF_MAX) ? BUF_MAX - bufCnt : serAvail;
          if (0 < toRead)
          {
            DPPRINT(F("24: Serial data avail = ")); DPPRINTLN(toRead, DEC);
            rxCnt = Serial.readBytes(bufPtr, toRead);
            DPPRINT(F("25: condition = ")); DPPRINTLN(wifiCondition, DEC);
            bufPtr += rxCnt;
            bufCnt += rxCnt;
            rxCorrect = SerialBufCheck(bufCnt);
          }
          // 4. Если буфер заполнился, а пакет так и не распознан — выходим
          if (bufCnt >= BUF_MAX/* && !rxCorrect*/)
          {
            DPPRINTLN(F("Buffer overflow, no valid packet, flush serial"));
            DPPRINT(F(" condition = ")); DPPRINTLN(wifiCondition, DEC);
            Serial.flush();
            break;
          }
        }//if (serAvail > 0)
      }//while

      BUILTIN_LED_OFF();
      if (rxCorrect)
      {
        DPPRINTLN(F("valid packet, Send to TCP"));
        tcpClient.write(buf, bufCnt); // отсылаем ответ ModbusTCP-клиенту
      }
      else
      {
        DPPRINT(F("no valid packet, bufCnt = ")); DPPRINTLN(bufCnt, DEC);
        DPPRINTLN("flush serial");
        Serial.flush();
      }
      gPreviosCycleQty = millis();
      LedState = 0;
      DPPRINT("End data IO");
      DPPRINT(F(" condition = ")); DPPRINTLN(wifiCondition, DEC);
    }
    else
      tcpClient.flush();
  }//if (avail)
}
//-----------------------------------------------------------------------------
const uint32_t charTimeoutMicros = 20;    // Пауза внутри пакета
uint8_t ipBuf[BUF_MAX];
uint8_t serBuf[BUF_MAX];
uint8_t serPos = 0;
unsigned long serLastByteMicros = 0;

const uint32_t channelTimeoutMillis = 2000; // Сброс источника через 2 сек тишины
unsigned long lastActivityMillis = 0; // Для сброса lastSource
enum Source { NONE, SOURCE_TCP, SOURCE_UDP };
Source lastSource = NONE;
IPAddress udpRemoteIp;
uint16_t udpRemotePort;



void TransparentIo()
{
  //BUILTIN_LED_ON();
  uint32_t avail = 0;
  uint32_t toRead = 0;

  // ip to serial
  if (tcpClient && tcpClient.connected())
  {
    avail = tcpClient.available();
    toRead = (avail > sizeof(ipBuf)) ? sizeof(ipBuf) : avail;
    if (toRead)
    {
      toRead = tcpClient.readBytes(ipBuf, toRead);
      DPPRINT(F("ip Read = ")); DPPRINTLN(toRead, DEC);
      Serial.write(ipBuf, toRead);
      Serial.flush();
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
      Serial.write(ipBuf, toRead);
      Serial.flush();
      lastSource = SOURCE_UDP;
      lastActivityMillis = millis();
      //lastByteMicros = micros();
    }
  }
  // вычитываем всё, что есть в serial в буфер
  while (0 < (avail = Serial.available()) && serPos < sizeof(serBuf))
  {
    size_t rest = sizeof(serBuf) - serPos;
    toRead = (avail > rest) ? rest : avail;
    if (toRead)
    {
      toRead = Serial.readBytes(serBuf + serPos, toRead);
      DPPRINT(F("serial Readed = ")); DPPRINTLN(toRead, DEC);
      serPos += toRead;
      serLastByteMicros = micros();
    }
  }
  // Сборка и отправка по таймауту "тишины" (микросекунды)
  if (0 < serPos && (micros() - serLastByteMicros > charTimeoutMicros))
  {
    switch (lastSource)
    {
      default: break;
      case SOURCE_TCP:
        tcpClient.write(serBuf, serPos);
        //tcpClient.flush();
        DPPRINT(F("serial to TcpIp = ")); DPPRINTLN(serPos, DEC);
        break;
      case SOURCE_UDP:
        udp.beginPacket(udpRemoteIp, udpRemotePort);
        udp.write(serBuf, serPos);
        udp.endPacket();
        //udp.flush();
        break;
    }
    serPos = 0;
  }
  // сбрасываем зависший источник
  if (lastSource != NONE && (millis() - lastActivityMillis > channelTimeoutMillis))
    lastSource = NONE;

  //BUILTIN_LED_OFF();
}
//-----------------------------------------------------------------------------
static uint32_t connectCheckPeriod = 300;
static uint32_t connectCheckLast = 0;

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
      if ((millis() - connectCheckLast > connectCheckPeriod))
      {
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
          //wifi_set_sleep_type(NONE_SLEEP_T);
          // wifi_set_sleep_type(MODEM_SLEEP_T);
          // wifi_set_sleep_type(LIGHT_SLEEP_T);
          DPPRINT(F("Connected socket client: "));
          DPPRINTLN(tcpClient.localIP());
          DPPRINT(F("condition = ")); DPPRINTLN(wifiCondition, DEC);
          ClearSerial();
        }
      }
      //ModbusIo();
      TransparentIo();
      break; // case 10
  } // switch (wifiCondition)
} // void WifiConnect()
//----------------------------------------------------------------------
void idle_blink()
{
  long unsigned int blinkPeriod = 1500;
  switch (lastSource)
  {
    default: break;
    case SOURCE_TCP:
    case SOURCE_UDP:
      BUILTIN_LED_ON(); return;
      //blinkPeriod = 200; break;
  }
  if (millis() - gPreviosCycleQty > blinkPeriod)
  {
    BUILTIN_LED_TOOGLE();
    gPreviosCycleQty = millis();
  }
}
//----------------------------------------------------------------------
void loop()
{
  ESP.wdtFeed();
  WifiConnect();
  idle_blink();
  //yield();
}
