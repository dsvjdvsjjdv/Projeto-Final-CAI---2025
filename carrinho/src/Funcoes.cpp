#include <Arduino.h>
#include <Adafruit_MCP23X17.h>
#include <Funcoes.h>

// instancias do objeto
Adafruit_MCP23X17 mcp;

// definicoes
#define PinM0Dir 18
#define PinM0Esq 3
#define PinM1Dir 10
#define PinM1Esq 46
#define PinM2Dir 13
#define PinM2Esq 14
#define PinM3Dir 12
#define PinM3Esq 11
#define I2C_SDA 8
#define I2C_SCL 9
#define Freq_PWM 20000 // frequencia do PWM 20kHz
#define resol_PWM 8    // resolucao do PWM em 8 bits 0 ~ 255
#define erroLinha INT16_MAX

// pinmotor[motor][direcao]
const uint8_t PinMotor[4][2] = {
    {PinM0Esq, PinM0Dir},
    {PinM1Esq, PinM1Dir},
    {PinM2Esq, PinM2Dir},
    {PinM3Esq, PinM3Esq}};

const uint8_t PinSeguidor[8] = {0, 1, 2, 3, 4, 5, 6, 7};
#define pinSeguidorHabilitado 8 // habilita os led infra vermelho dos sensores

const uint8_t chMotor[4][2]{
    {0, 1},
    {2, 3},
    {4, 5},
    {6, 7}};

// Variaveis PID
float kp = 4.0f;
float ki = 0.5f;
float kd = 1.2f;

float VelocidadeFrente = 20.0f; // nao ultrapassar de 20
float omegaMax = 40.0f;
float ErroAnterior = 0.0f;
float integralAcumulada = 0.0f;
uint32_t instanteAnteriorMs = 0;
// uint32_t -> unsigned long

// prototipos

void paratodosMotores();
void SetaDireita();
void SetaDireitaDes();
void SetaEsquerda();
void SetaEsquerdaDes();
void FarolDirFrente();
void FarolDirFrenteDes();
void FarolEsqFrenteDes();
void FarolEsqFrente();
void FarolDirTras();
void FarolDirTrasDes();
void FarolEsqTras();
void FarolEsqTrasDes();
void DiagonalEsqb();
void frente();
void tras();
void direita();
void esquerda();
void esquerdaVolta();
void direitaVolta();
void driftCima();
void driftBaixo();
void driftCimaEsq();
void driftBaixoEsq();
void DiagonalDirc();
void DiagonalEsqc();
void DiagonalDirb();
void DiagonalEsqb();
void CurvaSuaveParaDireita();
void CurvaSuaveParaEsquerda();
void ReSuaveParaDireita();
void ReSuaveParaEsquerda();
uint8_t Velocidade(uint8_t);

void inicializarMCP()
{
  if (!mcp.begin_I2C())
  {
    Serial.println("Erro ao iniciar MCP23017!");
    while (1)
      ;
  }
  mcp.pinMode(10, OUTPUT);
  mcp.pinMode(11, OUTPUT);
  mcp.pinMode(12, OUTPUT);
  mcp.pinMode(13, OUTPUT);
  mcp.pinMode(14, OUTPUT);
  mcp.pinMode(15, OUTPUT);
}

void inicializarPWM()
{
  // Inicializar os 8 canais (4 motores × 2 direções)
  for (int m = 0; m < 4; m++)
  {
    for (int d = 0; d < 2; d++)
    {
      uint8_t canal = chMotor[m][d]; // canal LEDC definido na matriz
      uint8_t pino = PinMotor[m][d]; // pino correspondente ao motor

      ledcSetup(canal, Freq_PWM, resol_PWM); // configura frequência e resolução
      ledcAttachPin(pino, canal);            // liga o pino ao canal
      ledcWrite(canal, 0);                   // inicia com duty 0
    }
  }
}

void paratodosMotores()
{
  for (char i = 0; i < 4; i++)
  {
    ledcWrite(chMotor[i][0], 0);
    ledcWrite(chMotor[i][1], 0);
  }
}

void SetaDireita()
{
  mcp.digitalWrite(13, HIGH);
}

void SetaDireitaDes()
{
  mcp.digitalWrite(13, LOW);
}

void SetaEsquerda()
{
  mcp.digitalWrite(10, HIGH);
}
void SetaEsquerdaDes()
{
  mcp.digitalWrite(10, LOW);
}

void FarolDirFrente()
{
  mcp.digitalWrite(12, HIGH);
}
void FarolDirFrenteDes()
{
  mcp.digitalWrite(12, LOW);
}

void FarolEsqFrente()
{
  mcp.digitalWrite(11, HIGH);
}

void FarolEsqFrenteDes()
{
  mcp.digitalWrite(11, LOW);
}

void FarolDirTras()
{
  mcp.digitalWrite(14, HIGH);
}

void FarolDirTrasDes()
{
  mcp.digitalWrite(14, LOW);
}

void FarolEsqTras()
{
  mcp.digitalWrite(15, HIGH);
}

void FarolEsqTrasDes()
{
  mcp.digitalWrite(15, LOW);
}

uint8_t Velocidade(uint8_t valor)
{
  return valor != 0
             ? map(valor, 0, 100, 150, 255)
             : 0;
}

void frente()
{
  ledcWrite(chMotor[0][0], Velocidade(10));
  ledcWrite(chMotor[1][0], Velocidade(10));
  ledcWrite(chMotor[2][0], Velocidade(10));
  ledcWrite(chMotor[3][0], Velocidade(10));
}
void tras()
{
  ledcWrite(chMotor[0][1], Velocidade(10));
  ledcWrite(chMotor[1][1], Velocidade(10));
  ledcWrite(chMotor[2][1], Velocidade(10));
  ledcWrite(chMotor[3][1], Velocidade(10));
}

void direita()
{
  ledcWrite(chMotor[0][0], Velocidade(10));
  ledcWrite(chMotor[1][1], Velocidade(10));
  ledcWrite(chMotor[2][1], Velocidade(10));
  ledcWrite(chMotor[3][0], Velocidade(10));
}

void esquerda()
{
  ledcWrite(chMotor[0][1], Velocidade(10));
  ledcWrite(chMotor[1][0], Velocidade(10));
  ledcWrite(chMotor[2][0], Velocidade(10));
  ledcWrite(chMotor[3][1], Velocidade(10));
}

void esquerdaVolta()
{
  ledcWrite(chMotor[0][1], Velocidade(10));
  ledcWrite(chMotor[1][0], Velocidade(10));
  ledcWrite(chMotor[2][1], Velocidade(10));
  ledcWrite(chMotor[3][0], Velocidade(10));
}

void direitaVolta()
{
  ledcWrite(chMotor[0][0], Velocidade(10));
  ledcWrite(chMotor[1][1], Velocidade(10));
  ledcWrite(chMotor[2][0], Velocidade(10));
  ledcWrite(chMotor[3][1], Velocidade(10));
}

void driftCima()
{
  ledcWrite(chMotor[0][0], Velocidade(10));
  ledcWrite(chMotor[1][1], Velocidade(10));
}

void driftBaixo()
{
  ledcWrite(chMotor[2][0], Velocidade(10));
  ledcWrite(chMotor[3][1], Velocidade(10));
}

void driftCimaEsq()
{
  ledcWrite(chMotor[0][1], Velocidade(10));
  ledcWrite(chMotor[1][0], Velocidade(10));
}

void driftBaixoEsq()
{
  ledcWrite(chMotor[2][1], Velocidade(10));
  ledcWrite(chMotor[3][0], Velocidade(10));
}

void DiagonalEsqc()
{
  ledcWrite(chMotor[1][0], Velocidade(10));
  ledcWrite(chMotor[2][0], Velocidade(10));
}

void DiagonalDirc()
{
  ledcWrite(chMotor[0][0], Velocidade(10));
  ledcWrite(chMotor[3][0], Velocidade(10));
}

void DiagonalEsqb()
{
  ledcWrite(chMotor[0][1], Velocidade(10));
  ledcWrite(chMotor[3][1], Velocidade(10));
}

void DiagonalDirb()
{
  ledcWrite(chMotor[1][1], Velocidade(10));
  ledcWrite(chMotor[2][1], Velocidade(10));
}

void CurvaSuaveParaDireita()
{
  ledcWrite(chMotor[0][0], Velocidade(30));
  ledcWrite(chMotor[1][0], Velocidade(10));
  ledcWrite(chMotor[2][0], Velocidade(30));
  ledcWrite(chMotor[3][0], Velocidade(10));
}

void CurvaSuaveParaEsquerda()
{
  ledcWrite(chMotor[0][0], Velocidade(10));
  ledcWrite(chMotor[1][0], Velocidade(30));
  ledcWrite(chMotor[2][0], Velocidade(10));
  ledcWrite(chMotor[3][0], Velocidade(30));
}

void ReSuaveParaDireita()
{
  ledcWrite(chMotor[0][1], Velocidade(30));
  ledcWrite(chMotor[1][1], Velocidade(10));
  ledcWrite(chMotor[2][1], Velocidade(30));
  ledcWrite(chMotor[3][1], Velocidade(10));
}

void ReSuaveParaEsquerda()
{
  ledcWrite(chMotor[0][1], Velocidade(10));
  ledcWrite(chMotor[1][1], Velocidade(30));
  ledcWrite(chMotor[2][1], Velocidade(10));
  ledcWrite(chMotor[3][1], Velocidade(30));
}