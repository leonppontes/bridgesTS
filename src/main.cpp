#define TINY_GSM_MODEM_SIM800 
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <DHT.h>

#define TOPIC "bridge/0001/test"
#define MQTT_SERVER "broker.hivemq.com"
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
const int doorPin = 18; 
int doorStatus;


void setupGSM()
{
  Serial.println("Configuracao do GSM");
  SerialGSM.begin(9600, SERIAL_8N1, 4, 2, false);
  delay(6000);
  Serial.println(modemGSM.getModemInfo());

  //Initiate modem
  if (!modemGSM.restart())
  {
    Serial.println("Restarting GSM Modem falhou");
    delay(3000);
    return;
  } else {Serial.println("Modem restartou");}

  //Waits for network
  Serial.println("Tentando achar rede...");
  if (!modemGSM.waitForNetwork()) 
  {
    Serial.println("Falha em achar redes");
    delay(3000);
    return;
  } else {Serial.println("Achou rede");}

  //Conecta à rede gprs (APN, usuário, senha)
  if (!modemGSM.gprsConnect("claro.com.br", "claro", "claro")) {
    Serial.println("GPRS Conexao falhou");
    delay(10000);
    return;
  } {Serial.println("Conectou na GPRS");}

  Serial.println("SETUP GSM SUCCESS");
}

void connectMQTTServer() {
  Serial.println("Conectando ao MQTT Server...");
  //Se conecta ao device que definimos
  if (client.connect("Bridge 0001")) {
    //Se a conexão foi bem sucedida
    Serial.println("MQTT Connectado");
  } else {
    //Se ocorreu algum erro
    Serial.print("error = ");
    Serial.println(client.state());
    delay(10000);
    ESP.restart();
  }
}

String createJsonString() 
{  
  String data = "{";
  data+="\"door\":";
  data+=String(doorStatus, 2);
  data+=",";
  data+="\"temperature\":";
  data+=String(temperature, 2); 
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

void setup() 
{
  Serial.begin(115200);
  dht.begin();
  //setupGSM(); 
  //connectMQTTServer();
  delay(2000);
  pinMode(doorPin, INPUT_PULLUP);
}

void loop() 
{
  delay(1000);
  doorStatus = digitalRead(doorPin);
  temperature = dht.readTemperature();
  Serial.println("Status: " + String(doorStatus, 2));
  Serial.println("Status: " + String(temperature, 2));
  /*if(!client.connected())
  {
    connectMQTTServer();
  }
   //Tempo decorrido desde o boot em milissegundos
  unsigned long now = millis();

  //Se passou o intervalo de envio
  if(now - lastTime > INTERVAL)
  {
    //Publicamos para o server mqtt
    publishMQTT();
    //Atualizamos o tempo em que foi feito o último envio
    lastTime = now;
  }*/
}

