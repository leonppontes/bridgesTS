#define TINY_GSM_MODEM_SIM800 
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include "local_config.h"
#include <DHT.h>
#include <Arduino_SNMP_Manager.h>

#define TOKEN "BBFF-tkFmFn3Ie1OdwGbL0FznbTw5tT5Lx7"
#define TOPIC "/v1.6/devices/bridgets"
#define DEVICE_ID "606df16f1d84727f0d3a660a"
#define MQTT_SERVER "things.ubidots.com"
#define MQTT_PORT 1883 
#define INTERVAL 10000


//Modem serial comunication
HardwareSerial SerialGSM(1);
TinyGsm modemGSM(SerialGSM);
TinyGsmClient gsmClient(modemGSM);
//MQTT Client
PubSubClient client(MQTT_SERVER, MQTT_PORT, gsmClient);
//Last send/refresh time
uint32_t lastTime = 0;
//Temperature and door variables
DHT dht(15, DHT11);
float temperature; 
const int doorPin = 21; 
int doorStatus;

IPAddress target(192, 168, 12, 24);
const char *community = "public";
const int snmpVersion = 0; // SNMP Version 1 = 0, SNMP Version 2 = 1
const char *oidServiceCountInt = ".1.3.6.1.4.1.1773.1.3.208.2.2.3.0";      // Integer sysServices
int servicesResponse = 0;   //talvez precise mudar para long
// A UDP instance to let us send and receive packets over UDP.
EthernetUDP Udp;
SNMPManager snmp = SNMPManager(community);             // Starts an SMMPManager to listen to replies to get-requests
SNMPGet snmpRequest = SNMPGet(community, snmpVersion); // Starts an SMMPGet instance to send requests
ValueCallback *callbackServices;   // Blank callback pointer for each OID

void getSNMP()
{
  // Build a SNMP get-request add each OID to the request
  
  snmpRequest.addOIDPointer(callbackServices);
  snmpRequest.setIP(Ethernet.localIP()); // IP of the listening MCU
  snmpRequest.setUDP(&Udp);
  snmpRequest.setRequestID(rand() % 5555);
  snmpRequest.sendTo(target);
  snmpRequest.clearOIDList();
}

void setupGSM()
{
  Serial.println("GSM configuration started");
  SerialGSM.begin(9600, SERIAL_8N1, 4, 2, false);
  delay(4000);
  Serial.println(modemGSM.getModemInfo());

  //Initiate modem
  if (!modemGSM.restart())
  {
    Serial.println("Restarting GSM Modem failure");
    delay(2000);
    //ESP.restart();
    return;
  } else {Serial.println("Modem restarted");}

  //Waits for network
  Serial.println("Tryng to find network...");
  if (!modemGSM.waitForNetwork()) 
  {
    Serial.println("Can't locate network");
    delay(3000);
    return;
  } else {Serial.println("Found network");}

  //Conecta à rede gprs (APN, usuário, senha)
  if (!modemGSM.gprsConnect("claro.com.br", "claro", "claro")) {
    Serial.println("GPRS connection failure");
    delay(10000);
    return;
  } {Serial.println("GPRS connection success");}

  Serial.println("SETUP GSM SUCCESS");
}

void connectMQTTServer() {
  Serial.println("Connecting to the MQTT server...");
  //Se conecta ao device que definimos
  if (client.connect(DEVICE_ID, TOKEN, "")) {
    //Se a conexão foi bem sucedida
    Serial.println("MQTT Connected");
  } else {
    //Se ocorreu algum erro
    Serial.print("error = ");
    Serial.println(client.state());
    delay(10000);
    //ESP.restart();
  }
}

String createJsonString() 
{ 
  float nivel =  (servicesResponse/1.00) - 256;
  String data = "{";
  data+="\"door\":";
  data+=String(doorStatus, 2);
  data+=",";
  data+="\"temperature\":";
  data+=String(temperature, 2);
  data+=",";
  data+="\"level\":";
  data+=String(nivel, 2); 
  data+="}";
  return data;
}

void publishMQTT()
{
  //Cria o json que iremos enviar para o server MQTT
  String msg = createJsonString();
  Serial.print("Publish message: ");
  Serial.println(msg);
  //Publicamos no tópico
  int status = client.publish(TOPIC, msg.c_str());
  Serial.println("Status: " + String(status));//Status 1 se sucesso ou 0 se deu erro
}

void WizReset() {
    Serial.print("Resetting Wiz W5500 Ethernet Board...  ");
    pinMode(RESET_P, OUTPUT);
    digitalWrite(RESET_P, HIGH);
    delay(250);
    digitalWrite(RESET_P, LOW);
    delay(50);
    digitalWrite(RESET_P, HIGH);
    delay(350);
    Serial.println("Done.");
}

void prt_hwval(uint8_t refval) {
    switch (refval) {
    case 0:
        Serial.println("No hardware detected.");
        break;
    case 1:
        Serial.println("WizNet W5100 detected.");
        break;
    case 2:
        Serial.println("WizNet W5200 detected.");
        break;
    case 3:
        Serial.println("WizNet W5500 detected.");
        break;
    default:
        Serial.println
            ("UNKNOWN - Update espnow_gw.ino to match Ethernet.h");
    }
}

void prt_ethval(uint8_t refval) {
    switch (refval) {
    case 0:
        Serial.println("Uknown status.");
        break;
    case 1:
        Serial.println("Link flagged as UP.");
        break;
    case 2:
        Serial.println("Link flagged as DOWN. Check cable connection.");
        break;
    default:
        Serial.println
            ("UNKNOWN - Update espnow_gw.ino to match Ethernet.h");
    }
}

void setup() 
{
  Serial.begin(115200);
  dht.begin();
  setupGSM(); 
  connectMQTTServer();
  delay(2000);
  pinMode(doorPin, INPUT_PULLUP);
  Serial.println("\n\tStarting W5500 configuration\r\n");
  Ethernet.init(5);           // GPIO5 on the ESP32.
  WizReset();
  Serial.println("Starting ETHERNET connection...");
  Ethernet.begin(eth_MAC, eth_IP, eth_DNS, eth_GW, eth_MASK);
  delay(200);
  Serial.print("Ethernet IP is: ");
  Serial.println(Ethernet.localIP());
  Serial.print("Checking connection.");
  bool rdy_flag = false;
  for (uint8_t i = 0; i <= 20; i++) {
      if ((Ethernet.hardwareStatus() == EthernetNoHardware) || (Ethernet.linkStatus() == LinkOFF)) {
            Serial.print(".");
            rdy_flag = false;
            delay(80);
        } else {
            rdy_flag = true;
            break;
        }
    }
    if (rdy_flag == false) {
        Serial.println
            ("\n\r\tHardware fault, or cable problem... cannot continue.");
        Serial.print("Hardware Status: ");
        prt_hwval(Ethernet.hardwareStatus());
        Serial.print("   Cable Status: ");
        prt_ethval(Ethernet.linkStatus());
        while (true) {
            delay(10);          // Halt.
        }
    } else {
        Serial.println(" OK");
    }
  snmp.setUDP(&Udp); // give snmp a pointer to the UDP object
  snmp.begin();      // start the SNMP Manager
  // Get callbacks from creating a handler for each of the OID
  callbackServices = snmp.addIntegerHandler(target, oidServiceCountInt, &servicesResponse);
}

void loop() 
{
  delay(1000);
  doorStatus = digitalRead(doorPin);
  temperature = dht.readTemperature();
  
  if(!client.connected())
  {
    connectMQTTServer();
  }

  snmp.loop(); 

  unsigned long now = millis();
  if(now - lastTime > INTERVAL)
  {
    getSNMP();
    publishMQTT();
    lastTime = now;
  }
    // Wait to see if a reply is available.
    delay(1000);
    
    // You only need to call maintain if you're using DHCP.
    // Ethernet.maintain();
}