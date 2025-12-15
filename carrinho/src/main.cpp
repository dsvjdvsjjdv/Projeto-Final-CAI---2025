//---------Includes---------//
#include <Arduino.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_ADS1X15.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include "INTERNET.h"
#include "Certificados.h"
#include "Funcoes.h"
#include <VL53L0X.h>
#include <Preferences.h>
#include "carrinho.h"
#include <Wire.h>
#include <Adafruit_PN532.h>
#include <Adafruit_NeoPixel.h>

//---------Instancias---------//
Preferences Prefs;
Adafruit_ADS1115 ads;
WiFiClientSecure espClient;
PubSubClient mqtt(espClient);
VL53L0X SensorProximidade;
Carrinho carrinho(mcp);
Adafruit_NeoPixel ledRGB(1, 48, NEO_GRB);

//---------MQTT---------//
const int mqtt_port = 8883;
const char *mqtt_id = "brigwrluihvtrsdrtgyugou";
const char *mqtt_PUB = "NovaFuture/esp32/Enviar";
const char *mqtt_SUB = "NovaFuture/esp32/Receber";

//---------Variaveis---------//
int leituraA;
float temperaturaDirFrente;

int leituraB;
float temperaturaEsqFrente;

int leituraC;
float temperaturaEsqTras;

int leituraD;
float temperaturaDirTras;

bool SetaDirAtivada = 0;
bool SetaEsqAtivada = 0;
static unsigned long AtualizaPiscaSetaDir = 0;
static unsigned long AtualizaPiscaSetaEsq = 0;
static bool EstadoSetaDir = 0;
static bool EstadoSetaEsq = 0;
int ModoDeFuncionamento = 1;
bool FarolEstado = 0;

// Flags de estado (sem enum)
static bool emCorrida = false;       // está seguindo a linha?
static bool emAtrasoPartida = false; // aguardando atraso pós-largada?

//---------Instanciar Funções---------//
void Callback(char *topic, byte *payload, unsigned int lenght);
void conectaMQTT();
void DesligarFarois();
void LigarFarois();
void DesligarAlerta();
void LigarAlerta();
void ModoDeMovimentacao(int ModoUsado);

void setup()
{
  conectaWiFi();
  Serial.begin(115200);
  espClient.setCACert(AWS_ROOT_CA);
  espClient.setCertificate(AWS_CERT);
  espClient.setPrivateKey(AWS_KEY);
  mqtt.setServer(AWS_BROKER, mqtt_port);
  mqtt.setBufferSize(2048);
  mqtt.setCallback(Callback);
  inicializarMCP();
  inicializarPWM();
  Wire.begin(8, 9);
  carrinho.begin();
  ads.begin();
  ledRGB.setPixelColor(0, ledRGB.Color(0, 0, 0));
  ledRGB.begin();
  ledRGB.setBrightness(200);
  carrinho.setPID(6.0f, 0.5f, 0.5f);
  carrinho.setVelocidades(25.0f, 0.0f);
  Prefs.begin("WorkSpace", false);
  ModoDeFuncionamento = Prefs.getInt("Modo", 1);
  Prefs.end();
  if (!SensorProximidade.init())
  {
    Serial.print("falha no sensor!!");
    while (1)
    {
    }
  }
  SensorProximidade.setMeasurementTimingBudget(20000); //* Tempo maximo em leitura do sensor em micro _segundo
  SensorProximidade.startContinuous(25);               //* Ativa a Leitura automatica do sensor a cada 25 ms.
  SensorProximidade.setTimeout(100);                   //* Limita o tempo de Leitura com erro para nao travar.
}

#define CALIBRACAO false

void loop()
{
  ReconectarWiFi();

  if (!mqtt.connected())
  {
    conectaMQTT();
  }

  mqtt.loop();

  static unsigned long AtualizaTemperatura = 0;
  unsigned long agoraTemperatura = millis();
  if (agoraTemperatura - AtualizaTemperatura > 1000)
  {
    leituraA = ads.readADC_SingleEnded(0);     // 0 ate 3
    temperaturaDirFrente = leituraA * 0.01875; // converter a temperatura do LM35

    leituraB = ads.readADC_SingleEnded(2);     // 0 ate 3
    temperaturaEsqFrente = leituraB * 0.01875; // converter a temperatura do LM35

    leituraC = ads.readADC_SingleEnded(3);   // 0 ate 3
    temperaturaEsqTras = leituraC * 0.01875; // converter a temperatura do LM35

    leituraD = ads.readADC_SingleEnded(1);   // 0 ate 3
    temperaturaDirTras = leituraD * 0.01875; // converter a temperatura do LM35
    AtualizaTemperatura = agoraTemperatura;
  }
  if (SetaDirAtivada == 1)
  {
    unsigned long agoraSetaDir = millis();
    if (agoraSetaDir - AtualizaPiscaSetaDir > 1000)
    {
      EstadoSetaDir = !EstadoSetaDir;
      AtualizaPiscaSetaDir = agoraSetaDir;
    }
    if (EstadoSetaDir == 0)
    {
      SetaDireitaDes();
    }
    else
    {
      SetaDireita();
    }
  }
  else
  {
    EstadoSetaDir = 0;
    SetaDireitaDes();
  }

  if (SetaEsqAtivada == 1)
  {
    unsigned long agoraSetaEsq = millis();
    if (agoraSetaEsq - AtualizaPiscaSetaEsq > 1000)
    {
      EstadoSetaEsq = !EstadoSetaEsq;
      AtualizaPiscaSetaEsq = agoraSetaEsq;
    }
    if (EstadoSetaEsq == 0)
    {
      SetaEsquerdaDes();
    }
    else
    {
      SetaEsquerda();
    }
  }
  else
  {
    EstadoSetaEsq = 0;
    SetaEsquerdaDes();
  }

  int distanciaCm = SensorProximidade.readRangeContinuousMillimeters() / 10;

  static unsigned long tempo_pub = 0;
  unsigned long agora = millis();
  if (agora - tempo_pub > 1000)
  {
    JsonDocument doc;

    doc["TemperaturaDirFrente"] = temperaturaDirFrente;
    doc["TemperaturaEsqFrente"] = temperaturaEsqFrente;
    doc["TemperaturaDirTras"] = temperaturaDirTras;
    doc["TemperaturaEsqTras"] = temperaturaEsqTras;
    doc["SensorProximidadeFrente"] = distanciaCm;
    doc["EstadoSetaDir"] = EstadoSetaDir;
    doc["EstadoSetaEsq"] = EstadoSetaEsq;
    doc["FarolEstado"] = FarolEstado;
    doc["Timestamp"] = agora;
    doc["ModoDeFuncionamento"] = ModoDeFuncionamento;

    String mensagem;
    serializeJsonPretty(doc, mensagem);
    bool status = mqtt.publish(mqtt_PUB, mensagem.c_str());
    tempo_pub = agora;
  }
}
void conectaMQTT()
{
  while (!mqtt.connected())
  {
    Serial.println("CONECTANDO AO AWS IoT Core...");

    if (mqtt.connect(mqtt_id))
    {
      Serial.println("CONECTADO!");
      mqtt.subscribe(mqtt_SUB);
    }
    else
    {
      Serial.print("falha, rc=");
      Serial.print(mqtt.state());
      Serial.println("TENTANDO DENOVO");
      delay(5000);
    }
  }
}
void Callback(char *topic, byte *payload, unsigned int lenght)
{
  String msg((char *)payload, lenght);

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, msg);

  if (error)
  {
    Serial.print("Erro no JSON: ");
    Serial.println(error.c_str());
    return;
  }

  bool ImpactoFrente = doc["SensorProximidadeFrenteAlerta"].as<bool>();

  int DirecaoJoy = doc["Direcao"].as<int>();
  int DirecaoMit = doc["Controle_Carrinho"].as<int>();
  bool EsterEgg = doc["Estereeg_Carrinho"].as<bool>();

  if (doc["ModoDeFuncionamento"].is<int>())
  {
    ModoDeFuncionamento = doc["ModoDeFuncionamento"].as<int>();
    Prefs.begin("WorkSpace", false);
    Prefs.putInt("Modo", ModoDeFuncionamento);
    Prefs.end();
  }

  if (ImpactoFrente == 1)
  {
    LigarAlerta();
    paratodosMotores();
    ledRGB.setPixelColor(0, ledRGB.Color(255, 0, 0));
    ledRGB.show();
  }
  else
  {
    DesligarAlerta();
    ledRGB.setPixelColor(0, ledRGB.Color(0, 0, 0));
    ledRGB.show();
  }

  if (ModoDeFuncionamento == 1)
  {
    ModoDeMovimentacao(DirecaoJoy);
  }
  else if (ModoDeFuncionamento == 2)
  {
    ModoDeMovimentacao(DirecaoMit);
  }
  else
  {
    paratodosMotores();
    SetaDirAtivada = 0;
    SetaEsqAtivada = 0;
    DesligarAlerta();
    DesligarFarois();
  }
}

void LigarAlerta()
{
  FarolDirTras();
  FarolEsqTras();
}

void DesligarAlerta()
{
  FarolDirTrasDes();
  FarolEsqTrasDes();
}

void LigarFarois()
{
  FarolDirFrente();
  FarolEsqFrente();
}

void DesligarFarois()
{
  FarolDirFrenteDes();
  FarolEsqFrenteDes();
}

void ModoDeMovimentacao(int ModoUsado)
{
  switch (ModoUsado)
  {
  case 1:
    frente();
    SetaDirAtivada = 0;
    SetaEsqAtivada = 0;
    FarolEstado = 0;
    DesligarAlerta();
    LigarFarois();
    break;

  case 2:
    DiagonalDirc();
    SetaDirAtivada = 0;
    SetaEsqAtivada = 1;
    FarolEstado = 0;
    DesligarAlerta();
    DesligarFarois();
    break;

  case 3:
    direita();
    SetaDirAtivada = 0;
    SetaEsqAtivada = 1;
    FarolEstado = 0;
    DesligarAlerta();
    DesligarFarois();
    break;

  case 4:
    DiagonalDirb();
    SetaDirAtivada = 0;
    SetaEsqAtivada = 1;
    FarolEstado = 0;
    LigarAlerta();
    DesligarFarois();
    break;

  case 5:
    tras();
    SetaDirAtivada = 0;
    SetaEsqAtivada = 0;
    FarolEstado = 1;
    LigarAlerta();
    DesligarFarois();
    break;

  case 6:
    DiagonalEsqb();

    SetaDirAtivada = 1;
    SetaEsqAtivada = 0;
    FarolEstado = 0;
    LigarAlerta();
    DesligarFarois();
    break;

  case 7:
    esquerda();
    SetaEsqAtivada = 0;
    SetaDirAtivada = 1;
    FarolEstado = 0;
    DesligarAlerta();
    DesligarFarois();
    break;

  case 8:

    DiagonalEsqc();
    SetaEsqAtivada = 0;
    SetaDirAtivada = 1;
    FarolEstado = 0;
    DesligarAlerta();
    DesligarFarois();
    break;

  default:
    paratodosMotores();
    SetaDirAtivada = 0;
    SetaEsqAtivada = 0;
    FarolEstado = 0;
    DesligarAlerta();
    DesligarFarois();
    break;
  }
}
