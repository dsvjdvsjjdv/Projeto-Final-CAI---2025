#include <Arduino.h>
#include <WiFi.h>
#include "INTERNET.h"
#include "senhas.h"

//? REDE PARA SE CONECTAR

//? CONSTANTE DE TEMPO
const unsigned long tempodeconexao = 20000;
const unsigned long tempodereconexao = 1000;

void conectaWiFi()
{

    Serial.printf("CONECTANDO AO WIFI: %s", SSID);

    WiFi.begin(SSID, SENHA); //* INICIA O WIFI E SE CONECTA A REDE

    unsigned long tempoInicialWiFi = millis();

    while (WiFi.status() != WL_CONNECTED && millis() - tempoInicialWiFi < tempodeconexao) //* Verifica se o estado do wifi é diferente de conectado
    {
        Serial.print(".");
        delay(500);
    }

    if (WiFi.status() == WL_CONNECTED) //* verifica se o estado do wifi é igual conectado
    {
        Serial.println("\n WiFi conectado com sucesso!");
        Serial.print("ENDEREÇO IP:");
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.println("\nERROA AO CONECTAR NO WIFI verifique a senha e a rede");
    }
}
void ReconectarWiFi()
{
    unsigned long tempoAtual = millis();
    static unsigned long tempoDaUltimaConexao = 0;

    if (tempoAtual - tempoDaUltimaConexao > tempodereconexao)
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println("\n Conexao Perdida! Tentando reconectar...");
            conectaWiFi();
        }
        tempoDaUltimaConexao = tempoAtual;
    }
}