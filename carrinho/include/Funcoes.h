#ifndef Funcoes_H
#define Funcoes_H

extern Adafruit_MCP23X17 mcp;


void paratodosMotores(void);
void SetaDireita(void);
void SetaDireitaDes (void);
void SetaEsquerda (void);
void SetaEsquerdaDes(void);
void FarolDirFrente (void);
void FarolDirFrenteDes (void);
void FarolEsqFrenteDes (void);
void FarolEsqFrente (void);
void FarolDirTras (void);
void FarolDirTrasDes (void);
void FarolEsqTras (void);
void FarolEsqTrasDes (void);
void DiagonalEsqb(void);
void frente(void);
void tras (void);
void direita(void);
void esquerda(void);
void esquerdaVolta(void);
void direitaVolta(void);
void driftCima(void);
void driftBaixo(void);
void driftCimaEsq(void);
void driftBaixoEsq(void);
void DiagonalDirc(void);
void DiagonalEsqc(void);
void DiagonalDirb(void);
void DiagonalEsqb(void);
void CurvaSuaveParaDireita(void);
void CurvaSuaveParaEsquerda(void);
void ReSuaveParaDireita(void);
void ReSuaveParaEsquerda(void);
uint8_t Velocidade(uint8_t );
void inicializarMCP();
void inicializarPWM(void);


#endif