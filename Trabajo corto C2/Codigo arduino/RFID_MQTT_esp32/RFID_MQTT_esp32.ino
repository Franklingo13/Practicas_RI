
#include <DHT.h>
#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

//Librerías para lectura RFID
#include <Wire.h>
#include <SPI.h> //Librería para comunicación SPI
#include <UNIT_PN532.h> //Librería Modificada

#include <Servo.h>

//Conexiones SPI del ESP32
#define PN532_SCK  (18)
#define PN532_MOSI (23)
#define PN532_SS   (5)
#define PN532_MISO (19)
uint8_t DatoRecibido[4]; //Para almacenar los datos

UNIT_PN532 nfc(PN532_SS);// Línea enfocada para la comunicación por SPI
// Funcion auxiliar para mostrar el buffer
void printArray(byte *buffer, byte bufferSize) {
   for (byte i = 0; i < bufferSize; i++) {
      Serial.print(buffer[i] < 0x10 ? " 0" : " ");
      Serial.print(buffer[i], HEX);
   }
}




// Conexión WiFi
/*const char* SSID = "POCOX3";
const char* SSID_PASSWORD = "pocox5597";*/

//const char* SSID = "Alejate_de_mi_WiFi";
//const char* SSID_PASSWORD = "0107100596nand74ls00";

const char* SSID = "Franklingo13";
const char* SSID_PASSWORD = "gatoconbotas";

//const char* SSID = "PCJD";
//const char* SSID_PASSWORD = "12345678";

// Conexión con mosquitto Broker
//#define MQTT_HOST IPAddress(192, 168, 0, 53) 
// host movil
#define MQTT_HOST "192.168.43.233"
//#define MQTT_HOST "192.168.137.198"
//#define MQTT_HOST "192.168.0.53" 
#define MQTT_PORT 1883

//__________________MQTT Topics____________________________
#define MQTT_PUB_UID  "esp32/PN532"
#define MQTT_SUB_OUT_TEMP "esp32/OutputControl"
#define MQTT_SUB_DOOR "esp32/DoorControl"


const int OutPin = 22;
const int DoorPin = 21;




//Creación de un objeto AsyncMqttClient
AsyncMqttClient mqttClient;
//temporizadores para reconexión 
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

// variables para monitorear el tiempo entre lecturas
unsigned long previousMillis = 0;   
const long interval = 10000;  

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(SSID, SSID_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %dn", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); 
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);

  uint16_t packetIdSub = mqttClient.subscribe(MQTT_SUB_OUT_TEMP, 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);

  //suscripcion para control de la puerta
  uint16_t packetIdSub2 = mqttClient.subscribe(MQTT_SUB_DOOR, 2);
  Serial.print("Subscribing Puerta at QoS 2, packetId: ");
  Serial.println(packetIdSub2);

  
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

// subscriptor MQTT
void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}
void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

//mensaje para OutPin
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("\n Publish received.");
  Serial.print("topic: ");
  Serial.println(topic);
  int TopLen = 17;
  String TopicName;
  for (int i = 0; i < TopLen; i++) {
    TopicName += (char)topic[i];
  }
   Serial.print("este es el nombre del topic: ");
   Serial.println(TopicName);

  
  String messageTemp;
  for (int i = 0; i < len; i++) {
    messageTemp += (char)payload[i];
  }
    Serial.print("Message: ");
    Serial.println(messageTemp);
  
  if (messageTemp == "ON"){
  digitalWrite(OutPin, HIGH); 
  Serial.println("Refrigeración encendida");
  }
  if (messageTemp == "OFF"){
  digitalWrite(OutPin, LOW); 
  Serial.println("Refrigeración apagada");
  }
  if (messageTemp == "OPEN"){
  digitalWrite(DoorPin, HIGH); 
  Serial.println("Bodega abierta");
  }
  if (messageTemp == "CLOSE"){
  digitalWrite(DoorPin, LOW); 
  Serial.println("Bodega cerrada");
  }


  
}

//_____________funcion para validación de UID____________________
  uint8_t validUID[4] = { 0x39, 0xB3, 0x43, 0xE8 };  // UID:FRANK
  uint8_t jdUID[4] = { 0x32, 0x1F, 0xFA, 0x21 };  // UID:JUAN DIEGO
  uint8_t llavero[4] = {0x11, 0xA5, 0x5D, 0x26}; // UID: LLAVERO
//Función para comparar dos vectores
bool isEqualArray(uint8_t* arrayA, uint8_t* arrayB, uint8_t length)
{
  for (uint8_t index = 0; index < length; index++)
  {
    if (arrayA[index] != arrayB[index]) return false;
  }
  return true;
}
//___________________________________________________



void rfid(){
  //_____ lectura de RFID_________________________________
  nfc.begin(); //Comienza la comunicación del PN532
  delay(1000);
  uint32_t versiondata = nfc.getFirmwareVersion();//Obtiene información de la placa
  if (! versiondata) { //Si no se encuentra comunicación con la placa --->
    Serial.print("No se encontró la placa PN53x");
    while (1); // Detener
  }
  Serial.print("Chip encontrado PN5");
  Serial.println((versiondata >> 24) & 0xFF, HEX); //Imprime en el serial que version de Chip es el lector
  Serial.print("Firmware ver. ");
  Serial.print((versiondata >> 16) & 0xFF, DEC); // Imprime que versión de firmware tiene la placa
  Serial.print('.'); Serial.println((versiondata >> 8) & 0xFF, DEC);
  //Establezca el número máximo de reintentos para leer de una tarjeta.
  nfc.setPassiveActivationRetries(0xFF);
  nfc.SAMConfig(); //Configura la placa para realizar la lectura
  Serial.println("Esperando una tarjeta ISO14443A ...");
  return;
}
//___________________________________________________________________________________

void setup() {
  Serial.begin(115200);
  Serial.println();
   pinMode(OutPin, OUTPUT);
   pinMode(DoorPin, OUTPUT);
   delay(1000);
  rfid();

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  // puerta mqttClient.onMessage(onMqttMessage2);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  connectToWifi();
  delay(5000);
  
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;


    unsigned long currentMillis = millis();
    /*
  
    // Publica un mensaje MQTT en el topic esp32/dht/temperature
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP_DHT, 0, true, String(temperature_DHT).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_TEMP_DHT, packetIdPub1);
    Serial.printf("Message: %.2f \n", temperature_DHT);

    // Publica un mensaje MQTT en el topic esp32/dht/humidity
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM_DHT, 1, true, String(humidity_DHT).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM_DHT, packetIdPub2);
    Serial.printf("Message: %.2f \n", humidity_DHT);*/
  }
  //======================RFID=========================================
  boolean LeeTarjeta; //Variable para almacenar la detección de una tarjeta
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Búfer para almacenar el UID devuelto
  uint8_t LongitudUID; //Variable para almacenar la longitud del UID de la tarjeta
  //Recepción y detección de los datos de la tarjeta y almacenamiento en la variable "LeeTarjeta"
  LeeTarjeta = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &LongitudUID, 5000);
  //Se detecto un tarjeta RFID
  if (LeeTarjeta) {
    Serial.println("Tarjeta encontrada!");
    Serial.print("Longitud del UID: ");
    Serial.print(LongitudUID, DEC); //Imprime la longitud de los datos de la tarjeta en decimal
    Serial.println(" bytes");
    Serial.print("Valor del UID: ");
    // Imprime los datos almacenados en la tarjeta en Hexadecimal
    Serial.print("UID: "); printArray(uid, LongitudUID);
    Serial.println("");
    if (isEqualArray(uid, validUID, LongitudUID))
    {
      Serial.println("Franklin Conectado");
      // Publica un mensaje MQTT en el topic  "esp32/DoorControl"
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_SUB_DOOR, 0, true, String("OPEN").c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_SUB_DOOR, packetIdPub3);
    Serial.println("");
    //Serial.printf("Message: %.2f \n", temperature_DHT);
      //delay(5000);
      // Publica un mensaje MQTT en el topic  "esp32/EPN532"
    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_UID, 0, true, String("Franklin").c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_UID, packetIdPub4);
    Serial.println("");
    }
    else if (isEqualArray(uid, jdUID, LongitudUID)){
      // Publica un mensaje MQTT en el topic  "esp32/DoorControl"
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_SUB_DOOR, 0, true, String("OPEN").c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_SUB_DOOR, packetIdPub3);
    Serial.println("");
    //Serial.printf("Message: %.2f \n", temperature_DHT);
      // Publica un mensaje MQTT en el topic  "esp32/EPN532"
    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_UID, 0, true, String("Juan D").c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_UID, packetIdPub4);
    Serial.println("");
    }
    else if(isEqualArray(uid, llavero, LongitudUID)){
      // Publica un mensaje MQTT en el topic  "esp32/OutControl"
    uint16_t packetIdPub6 = mqttClient.publish(MQTT_SUB_OUT_TEMP, 0, true, String("ON").c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_SUB_OUT_TEMP, packetIdPub6);
    Serial.println("");
    //Serial.printf("Message: %.2f \n", temperature_DHT);
      // Publica un mensaje MQTT en el topic  "esp32/EPN532"
      }
    else
    {
      Serial.println("Tarjeta invalida");
      // Publica un mensaje MQTT en el topic  "esp32/DoorControl"
    uint16_t packetIdPub5 = mqttClient.publish(MQTT_SUB_DOOR, 0, true, String("Acceso denegado").c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_SUB_DOOR, packetIdPub5);
    Serial.println("");
  }
  }
  //Si no se detecta tarjeta
  else
  {
    Serial.println("Se agotó el tiempo de espera de una tarjeta");
    
  }
  //======================RFID=========================================


 

  
}
