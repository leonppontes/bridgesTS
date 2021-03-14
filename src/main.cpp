#define TINY_GSM_MODEM_SIM800 //Tipo de modem que estamos usando
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <SPI.h>

//Tópico onde vamos postar os dados de temperatura e umidade 
#define TOPIC "bridge/0001/test" 

//URL do MQTT Server
#define MQTT_SERVER "broker.hivemq.com"

//Porta padrão do MQTT
#define MQTT_PORT 1883 

//Intervalo entre os envios
#define INTERVAL 10000

gpio_num_t  doorSensorPIN = GPIO_NUM_12;  // Reed GPIO PIN:
gpio_num_t GPIO_INPUT_IO_TRIGGER = doorSensorPIN;

//Canal serial que vamos usar para comunicarmos com o modem.
HardwareSerial SerialGSM(1);
TinyGsm modemGSM(SerialGSM);
TinyGsmClient gsmClient(modemGSM);

//Cliente MQTT, passamos a url do server, a porta
//e o cliente GSM
PubSubClient client(MQTT_SERVER, MQTT_PORT, gsmClient);

//Tempo em que o último envio/refresh foi feito
uint32_t lastTime = 0;

float temperature = 25.0; //Variável onde iremos armazenar o valor da temperatura

int doorState = digitalRead(doorSensorPIN);

void setupGSM()
{
  Serial.println("Configuracao do GSM");
  //Inicializamos a serial onde está o modem
  SerialGSM.begin(9600, SERIAL_8N1, 4, 2, false);
  delay(6000);

  //Mostra informação sobre o modem
  Serial.println(modemGSM.getModemInfo());

  //Inicializa o modem
  if (!modemGSM.restart())
  {
    Serial.println("Restarting GSM Modem falhou");
    delay(3000);
//    ESP.restart();
    return;
  } else {Serial.println("Modem restartou");}

  //Espera pela rede
  Serial.println("Tentando achar rede...");
  if (!modemGSM.waitForNetwork()) 
  {
    Serial.println("Falha");
    delay(3000);
    //ESP.restart();
    return;
  } else {Serial.println("Achou rede");}

  //Conecta à rede gprs (APN, usuário, senha)
  if (!modemGSM.gprsConnect("claro.com.br", "claro", "claro")) {
    Serial.println("GPRS Conexao falhou");
    delay(10000);
    //ESP.restart();
    return;
  } {Serial.println("CONECTOU NA GPRS");}

  Serial.println("Setup GSM Success");
}

void connectMQTTServer() {
  Serial.println("Conectando ao MQTT Server...");
  //Se conecta ao device que definimos
  if (client.connect("Bridge 0001")) {
    //Se a conexão foi bem sucedida
    Serial.println("MQTT Connecteado");
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
  data+=String(doorState, 2);
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
  setupGSM(); //Inicializa e configura o modem GSM
  connectMQTTServer(); //Conectamos ao mqtt server
  //Espera 2 segundos
  delay(2000);
  pinMode(GPIO_INPUT_IO_TRIGGER,INPUT_PULLUP);
}

void loop() 
{
  //Se desconectou do server MQTT
  if(!client.connected())
  {
    //Mandamos conectar
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
  }
}

