//debug options
bool debug = false;
bool gpsDebug = false;
bool writeDebug = false;
bool gammaDebug = false;
bool timeDebug = false;

//LED pin left to right (/sd/gps/power/lidar)
int powerLedRed = 4;
int powerLedBlue = 6;
int powerLedGreen = 5;
int gpsLedRed = 24;
int gpsLedBlue = 26;
int gpsLedGreen = 25;
int sdLedRed = 27;
int sdLedBlue = 29;
int sdLedGreen = 28;
int lidarLedRed = 30;
int lidarLedBlue = 32;
int lidarLedGreen = 31;

//gps init
#include "NMEAGPS.h"
#define gpsSerial Serial1
unsigned int gpsBitRate = 115200;
const byte ppsPin = 2;
double gpsLat, gpsLon, gpsAlt = 0;
byte gpsYear, gpsMonth, gpsDay = 0;
byte gpsHour, gpsMinute, gpsSecond, gpsCentisecond = 0;
byte gpsSat = 0;
static NMEAGPS  NMgps;
static gps_fix  fix;
char gpsFreq[] = "$PMTK220,300*2D"; //Данные GPS каждые 300мс
//char gpsFreq[] = "$PMTK220,100*2F"; //Данные GPS каждые 100мс
//char gpsFreq[] = "$PMTK220,500*2B" //Данные GPS каждые 500мс
uint64_t const gpsTimeLimit = 1000; //как часто записывать координаты в файл, сколько прерываний будет пропущено.
uint64_t gpsTimeCount = 0; //Служебная переменная
unsigned long gpsReadTrashhold = 1500;
unsigned long gpsCount; //обработка работы светодиода - индикатора gps
bool gpsTimeOwn;

//Timers
IntervalTimer gpsTimer;
#include <Time.h>
#include <TimeLib.h>
unsigned int gpsTrackTime = 1; //как часто пишется трек, сек
bool needTimeSyncFlag = true;
uint8_t timeSyncTimerLimit = 30;
uint32_t timeSyncTimer = 0;


//sdCard
#include <SD.h>
#include <SPI.h>
const byte SD_PIN_CHIP_SELECT = BUILTIN_SDCARD;
String dataFileName = "DATA.txt";
String dataFileHeader = "RTC_DT\tDATETIME\tLAT\tLON\tALT\tSAT\tSPECTRE\n";
String gpsFileName = "gpsDATA.txt";
String gpsFileHeader = "DATETIME\tLAT\tLON\tALT\tSAT\n";
String ldrFileName = "ldrDATA.txt";
String ldrFileHeader = "RTC_DT\tDATETIME\tLAT\tLON\tALT\tSAT\tDIST\n";
String logFileName = "LOG.txt";
String logFileHeader = "RTC_DT\tMASSAGE\n";
bool sdCardExist;
int writeSize, fileSize1, fileSize2;

//Блок переменных для обработки получения данных со считывателя гамма данных
#define gammaSerial Serial3

//блок обработки данных лидара
#include <Wire.h>
#include <LIDARLite.h>
byte adress = 0x62; //Lidar adress
LIDARLite myLidarLite;
bool lidarFoundFlag = false;
uint32_t lidarTimerTrashhold = 1;
IntervalTimer lidarTimer;

void setup() {
  Serial1.begin(gpsBitRate);
  initLED();
  delay(200);
  digitalWrite(powerLedGreen, 1);
  initGps();
  // задаем время библиотеки Time при помощи RTC на Teensy 3.0:
  setSyncProvider(getTeensy3Time);
  setSyncInterval(5);
  delay(100);
  Serial3.begin(115200);
  delay(100);
  digitalWrite(gpsLedRed, 1);
  initSdCard(SD_PIN_CHIP_SELECT);
  delay(200);
  Serial.begin(115200);
  initLogFile();
  initDataFile();
  initGpsFile();

  pinMode(ppsPin, INPUT);
  attachInterrupt(ppsPin, ppsInterrupt, RISING);
  initLidarFile();
  initLidar();
  logStr("Device ON");
}

void loop() {
  gpsPortRead();
  readGammaSerial();
}

void foo(){
  noInterrupts();
  logStr("log");
  interrupts();
}


//проверка синхронизации часов
void checkTimeSync(){
  if((millis() - timeSyncTimer) > (timeSyncTimerLimit*1000)){
    if (timeDebug){
      String loggingStr = "Now: ";
      loggingStr += (now());
      loggingStr += "; gps time is: ";
      loggingStr += (fix.dateTime_cs/100);
      logStr(loggingStr);
      Serial.println(loggingStr);
    }
    if ((fix.dateTime.year != year()) || (fix.dateTime.month != month()) || (fix.dateTime.date != day()) || (fix.dateTime.hours != hour()) || (fix.dateTime.minutes != minute()) || (round(fix.dateTime.seconds + fix.dateTime_cs/100) != second())){
      needTimeSyncFlag = true;
    }
    timeSyncTimer = millis();
  }
}

//синхронизировать часы с gps если gps дает время и установлен флаг needTimSyncFlag
void syncRTC_GPS(){
  if (needTimeSyncFlag){
    if (fix.status >= 2){
      gpsPortRead();
      uint8_t hour = fix.dateTime.hours;
      uint8_t minute = fix.dateTime.minutes;
      uint8_t second = round(fix.dateTime.seconds + fix.dateTime_cs/100);
      uint8_t day = fix.dateTime.date;
      uint8_t month = fix.dateTime.month;
      uint8_t year = fix.dateTime.year;
      setTime(hour, minute, second, day, month, year);
      Teensy3Clock.set(now()); // задаем RTC
      // if (timeDebug){
      //   String loggingStr = "Now gps time: ";
      //   loggingStr += "; rtc time is: ";
      //   loggingStr += (now());
      //   logStr(loggingStr);
      //   Serial.println(loggingStr);
      // }
      needTimeSyncFlag = false;
    }
  }
}

//служебная функция возвращает время rtc модуля teensy
time_t getTeensy3Time(){
  return Teensy3Clock.get();
}

//Выставление частоты gpsSat
void initGps() {
  delay(200);
  Serial1.write("$PMTK220,300*2D");
  delay(100);
  Serial1.write(0x0D);
  delay(100);
  Serial1.write(0x0A);
  delay(1000);
  Serial1.write("$PMTK220,300*2D");
  delay(100);
  Serial1.write(0x0D);
  delay(100);
  Serial1.write(0x0A);
  delay(1000);
  Serial1.write("$PMTK220,300*2D");
  delay(100);
  Serial1.write(0x0D);
  delay(100);
  Serial1.write(0x0A);
}
//Первичная инициализация и тест светодиодов.
void initLED(){
  //indication init
  pinMode(powerLedRed, OUTPUT);
  pinMode(powerLedBlue, OUTPUT);
  pinMode(powerLedGreen, OUTPUT);
  pinMode(gpsLedRed, OUTPUT);
  pinMode(gpsLedBlue, OUTPUT);
  pinMode(gpsLedGreen, OUTPUT);
  pinMode(sdLedRed, OUTPUT);
  pinMode(sdLedBlue, OUTPUT);
  pinMode(sdLedGreen, OUTPUT);
  pinMode(lidarLedRed, OUTPUT);
  pinMode(lidarLedBlue, OUTPUT);
  pinMode(lidarLedGreen, OUTPUT);

  digitalWrite(powerLedRed, 1);
  digitalWrite(gpsLedRed, 1);
  digitalWrite(sdLedRed, 1);
  digitalWrite(lidarLedRed, 1);
  delay(1000);
  digitalWrite(powerLedRed, 0);
  digitalWrite(gpsLedRed, 0);
  digitalWrite(sdLedRed, 0);
  digitalWrite(lidarLedRed, 0);
  digitalWrite(powerLedBlue, 1);
  digitalWrite(gpsLedBlue, 1);
  digitalWrite(sdLedBlue, 1);
  digitalWrite(lidarLedBlue, 1);
  delay(1000);
  digitalWrite(powerLedBlue, 0);
  digitalWrite(gpsLedBlue, 0);
  digitalWrite(sdLedBlue, 0);
  digitalWrite(lidarLedBlue, 0);
  digitalWrite(powerLedGreen, 1);
  digitalWrite(gpsLedGreen, 1);
  digitalWrite(sdLedGreen, 1);
  digitalWrite(lidarLedGreen, 1);
  delay(1000);
  digitalWrite(powerLedGreen, 0);
  digitalWrite(gpsLedGreen, 0);
  digitalWrite(sdLedGreen, 0);
  digitalWrite(lidarLedGreen, 0);
}

//инициализация лидара.
void initLidar(){
  Wire.begin();
  byte error;
  Wire.beginTransmission(adress);
  error = Wire.endTransmission();
  if (error == 0){
    logStr("Lidar found");
    digitalWrite(lidarLedRed, 0);
    digitalWrite(lidarLedGreen, 1);
    digitalWrite(lidarLedBlue, 0);
    lidarFoundFlag = true;
  } else if (error == 4){
    logStr("Unknown error at lidar port");
    digitalWrite(lidarLedRed, 1);
    digitalWrite(lidarLedGreen, 0);
    digitalWrite(lidarLedBlue, 0);
  } else {
    logStr("No lidar found");
    digitalWrite(lidarLedRed, 0);
    digitalWrite(lidarLedGreen, 0);
    digitalWrite(lidarLedBlue, 0);
  }

  if (lidarFoundFlag){
    myLidarLite.begin(0, true);
    myLidarLite.configure(3);
    lidarTimer.begin(getLidarDistance, lidarTimerTrashhold*1000000);
  }
}

//Блок инициализации файлов
void initGpsFile(){
  logStr("Gps File init.");
  if (debug){
    Serial.println("");
  }
  File gpsFile = SD.open(gpsFileName.c_str(), FILE_READ);
  if (!((SD.exists(gpsFileName.c_str()))&&(gpsFile.size()>0))){
    gpsFile.close();
    writeToFile(gpsFileName, gpsFileHeader);
  }
}

void initDataFile(){
  File dataFile = SD.open(dataFileName.c_str(), FILE_READ);
  if (!((SD.exists(dataFileName.c_str()))&&(dataFile.size()>0))){
    dataFile.close();
    writeToFile(dataFileName, dataFileHeader);
  }
}

void initLidarFile(){
  logStr("Lidar File init.");
  bool writeStatus = false;
  if (debug){
    Serial.println("lidar file init");
    Serial.print("Lidar fileName: ");
    Serial.print(ldrFileName);
    Serial.print("; Lidar ldrFileHeader: ");
    Serial.print(ldrFileHeader);
  }
  File ldrFile = SD.open(ldrFileName.c_str(), FILE_READ);
  if (!((SD.exists(ldrFileName.c_str()))&&(ldrFile.size()>0))){
    ldrFile.close();
    writeStatus = writeToFile(ldrFileName, ldrFileHeader);
  }
  if (debug){
    Serial.print("write lidar file init: ");
    Serial.println(writeStatus);
  }
}

void initLogFile(){
  noInterrupts();
  logStr("Log File init.");
  File logFile = SD.open(logFileName.c_str(), FILE_READ);
  if (!((SD.exists(logFileName.c_str()))&&(logFile.size()>0))){
    logFile.close();
    writeToFile(logFileName, logFileHeader);
  }
  interrupts();
}

//записать строку в Лог файл.
void logStr(String loggingString){
  String logStr = "";
  logStr += getRtcTime();
  logStr += "\t";
  logStr += loggingString;
  logStr += "\n";
  writeToFile(logFileName, logStr);
}

//если есть данные в порту gps - читает их. После чего меняет цвет индикатора.
//Зеленый - нет координат, но есть время, с gps
//синий - более 5 спутников, есть координаты
void gpsPortRead(){
  while (NMgps.available(gpsSerial)) {
    fix = NMgps.read();
    gpsCount = millis();
  }
  if ((fix.status < 2) || (millis() - gpsCount > gpsReadTrashhold)){
    digitalWrite(gpsLedRed, 1);
    digitalWrite(gpsLedBlue, 0);
    digitalWrite(gpsLedGreen, 0);
    gpsTimeOwn = false;
  } else{
    gpsTimeOwn = true;
    if (fix.satellites <= 5){
      digitalWrite(gpsLedRed, 0);
      digitalWrite(gpsLedBlue, 0);
      digitalWrite(gpsLedGreen, 1);
    } else {
      digitalWrite(gpsLedRed, 0);
      digitalWrite(gpsLedBlue, 1);
      digitalWrite(gpsLedGreen, 0);
    }
  }
}

// Write string to file with size control
bool writeToFile(String fileName, String writeString){
  noInterrupts();
  bool status = false;
  File file = SD.open(fileName.c_str(), FILE_WRITE);  // как СД передать в функцию? c_str - новая функция
  digitalWrite(sdLedBlue, 0);
  if (writeDebug) {
    Serial.print(writeString);
  }
  fileSize1 = file.size();
  writeSize = file.print(writeString);

  file.flush();
  fileSize2 = file.size();
  if ((fileSize2 >= fileSize1+writeSize)&&(writeSize > 0)){
      digitalWrite(sdLedBlue, 1);
      status = true;
    } else {
      digitalWrite(sdLedRed, 1);
      String errorString = "";
      errorString += "File write error. Write Size: ";
      errorString += writeSize;
      errorString += "; start file size: ";
      errorString += fileSize1;
      errorString += "; end file size: ";
      errorString += fileSize2;
      errorString += "; File name: ";
      errorString += fileName;
      errorString += ";\n";
      status = false;
    }
  if (writeDebug) {
    Serial.print("Write to File Func : filesize1 ");
    Serial.print(fileSize1);
    Serial.print(", writed:");
    Serial.print(writeSize);
    Serial.print(", Filesize2: ");
    Serial.println(fileSize2);
  }
  file.close();
  interrupts();
  return status;
}

// проверка наличия sd карты. Если нет - зажеч красный огонек.
void initSdCard(byte SD_PIN_CS) {
  if (!SD.begin(SD_PIN_CS)) {
    if (writeDebug){
      Serial.println("initialization failed!");
    }
    digitalWrite(sdLedRed, 1);
    sdCardExist = false;
    return;
  } else{
    sdCardExist = true;
  }
}

//Возвращает из последнего сообщения gps строку в виде Время/координаты/число спутников
String formFullGPSstring(gps_fix gpsFix){
  noInterrupts();
  String gpsString = "";
  gpsString += getGpsTime(gpsFix);
  gpsString += "\t";
  gpsString += String(gpsFix.latitude(), 7);
  gpsString += "\t";
  gpsString += String(gpsFix.longitude(), 7);
  gpsString += "\t";
  gpsString += gpsFix.altitude();
  gpsString += "\t";
  gpsString += gpsFix.satellites;
  interrupts();
  return gpsString;
}

//Возвращает из последнего сообщения gps строку в виде Время (dd.mm.yyyyThh:mm:ss.ms)
String getGpsTime(gps_fix gpsFix){
  noInterrupts();
  String gpsTimeString = "";
  int tempData;
  tempData = gpsFix.dateTime.date;
  if (tempData < 10){
    gpsTimeString +="0";
  }
  gpsTimeString += tempData;
  gpsTimeString += ".";
  tempData = gpsFix.dateTime.month;
  if (tempData < 10){
    gpsTimeString +="0";
  }
  gpsTimeString += gpsFix.dateTime.month;
  gpsTimeString += ".";
  gpsTimeString += gpsFix.dateTime.full_year();
  gpsTimeString += "T";
  tempData = gpsFix.dateTime.hours;
  if (tempData < 10){
    gpsTimeString +="0";
  }
  gpsTimeString += gpsFix.dateTime.hours;
  gpsTimeString += ":";
  tempData = gpsFix.dateTime.minutes;
  if (tempData < 10){
    gpsTimeString +="0";
  }
  gpsTimeString += gpsFix.dateTime.minutes;
  gpsTimeString += ":";
  tempData = gpsFix.dateTime.seconds;
  if (tempData < 10){
    gpsTimeString +="0";
  }
  gpsTimeString += gpsFix.dateTime.seconds;
  gpsTimeString += ".";
  gpsTimeString += gpsFix.dateTime_ms();
  interrupts();
  return gpsTimeString;
}

//Возвращает время rtc
String getRtcTime(){
  noInterrupts();
  String rtcTimeString = "";
  int tempData;
  tempData = day();
  if (tempData < 10){
    rtcTimeString +="0";
  }
  rtcTimeString += tempData;
  rtcTimeString += ".";
  tempData = month();
  if (tempData < 10){
    rtcTimeString +="0";
  }
  rtcTimeString += tempData;
  rtcTimeString += ".";
  rtcTimeString +=  year();
  rtcTimeString += "T";
  tempData = hour();
  if (tempData < 10){
    rtcTimeString +="0";
  }
  rtcTimeString += tempData;
  rtcTimeString += ":";
  tempData = minute();
  if (tempData < 10){
    rtcTimeString +="0";
  }
  rtcTimeString += tempData;
  rtcTimeString += ":";
  tempData = second();
  if (tempData < 10){
    rtcTimeString +="0";
  }
  rtcTimeString += tempData;
  interrupts();
  return rtcTimeString;
}

// обработка события прихода ссообщения gps
void ppsInterrupt(){
  noInterrupts();
  gpsPortRead();
  checkTimeSync();
  syncRTC_GPS();
  writeGpsWP();
  interrupts();
}

//записывает gps точку в файл gps трека
void writeGpsWP(){
  if ((millis() - gpsTimeCount)  > gpsTimeLimit){
    String gpsString = "";
    gpsString += formFullGPSstring(fix);
    gpsString += "\n";
    writeToFile(gpsFileName, gpsString);
    gpsTimeCount = millis();
  }
}

//Принятие данных из UART порта и запись в файл.
void readGammaSerial(){
  while (Serial3.available()){
    String readBuf = "";
    String dataString = "";
    while (!readBuf.endsWith("\n")){
      readBuf += Serial3.readString();
    }
    dataString += getRtcTime();
    dataString += "\t";
    dataString += formFullGPSstring(fix);
    dataString += "\t";
    dataString += readBuf;
    writeToFile(dataFileName, dataString);
    if (gammaDebug){
      Serial.print(dataString);
    }
  }
}

void getLidarDistance(){
  digitalWrite(lidarLedRed, 0);
  digitalWrite(lidarLedGreen, 0);
  digitalWrite(lidarLedBlue, 0);
  String ldrString = "";
  ldrString += getRtcTime();
  ldrString += "\t";
  ldrString += formFullGPSstring(fix);
  ldrString += "\t";
  ldrString += myLidarLite.distance();
  ldrString += "\n";
  bool writeStatus = writeToFile(ldrFileName, ldrString);
  if (writeStatus){
    digitalWrite(lidarLedRed, 0);
    digitalWrite(lidarLedGreen, 0);
    digitalWrite(lidarLedBlue, 1);
  } else {
    digitalWrite(lidarLedRed, 1);
    digitalWrite(lidarLedGreen, 0);
    digitalWrite(lidarLedBlue, 0);
  }
}
