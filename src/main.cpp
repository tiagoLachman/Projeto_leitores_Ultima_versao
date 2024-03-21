/* Programa para monitoramento de Extrusoras CONDICIONADO PARA TEAR COM PULSOS E INTERRUPÇÃOES

    Criação data: 04-09-2020

    Autor:
     -Pericles Silva
    Procopio Ind. e Comercio LTDA.

    Versão Arduino Beta 2.9.0
*/

#include <SPI.h>
#include <Ethernet.h>
//#include <SD.h>
#include <SdFat.h>
#include <DS3232RTC.h> // https://github.com/JChristensen/DS3232RTC
#include "RTCDS1307.h"
#include <ArduinoJson.h>
#include <ArduinoHttpClient.h>
#include <avr/wdt.h> // for self reset
//#include <Watchdog.h>
#include "EEPROM.h"

#define pinBotao 18 // pino da interrupção
#define para 30

#define pino32 32
#define pino36 36

/*****/
#ifdef ARDUINO_ARCH_SAMD
#include <SDU.h>                         // prepends to this sketch a 'second stage SD bootloader'
const char *BIN_FILENAME = "UPDATE.BIN"; // expected by the SDU library
#endif
#ifdef ARDUINO_ARCH_AVR
const char *BIN_FILENAME = "firmware.bin"; // expected by zevero avr_boot
#endif

const short VERSION = 19;

#ifndef SDCARD_SS_PIN
#define SDCARD_SS_PIN 4
#endif

#ifdef ARDUINO_SAM_ZERO // M0
#define Serial SerialUSB
#endif
/******/

#define REQ_BUF_SZ 20
char HTTP_req[REQ_BUF_SZ] = { 0 }; // buffered HTTP request stored as null terminated string

// Watchdog watchdog;
RTCDS1307 rtc(0x68); // Declaração do rtc

DS3232RTC RTC(0x68);

//byte aMAC[] = { 0x70, 0xB3, 0xD5, 0x0A, 0xC7, 0xC8 }; // Vetor para armazenar endereço aMac
byte aMAC[6];     // = {0x70, 0xB3, 0xD5, 0x0A, 0xC7, 0x43}; //Vetor para armazenar endereço aMac
byte aIP[4];      // = { 192, 168,  23,  11 };      //Vetor para armazenar o endereço de IP
byte aSubnet[4];  // = { 255, 255, 252, 0 }; //Vetor para armazenar o endereço de SubRede
byte aGateway[4]; // = { 192, 168,  20,  11 }; //Vetor para armazenar o endereço de GATEWAY
byte aDNS[4];     // = {   8,   8,   8,   8 }; //Vetor para armazenar o endereço de DNS

// byte ip[4] = {192, 168, 20, 232};//Endereço de IP do Mega

EthernetServer server(80); // Cria um server na porta 80
EthernetClient client;     // Objeto para lidar com clientes conectado ao server

unsigned long int nIdFalha    = 0;       // Id Falhas
unsigned long int nIdFalhaAnt = 0;       // Id Falhas

unsigned long int nBatidas;       // Contador do total de batidas
unsigned long int nBatidasMes;    // Contador do total de batidas
unsigned long int nBatidasOP;     // Contador de batidas desde que foi reiniciado o limite de batidas
unsigned long int nBatidasBobina; // Contador de batidas desde que foi reiniciado o limite de batidas
unsigned long int nFimProducao;   // Limite de batidas
unsigned long int nFimBobina;     // Limite de batidas
int nFalhas = 0;                  // Variavel que armazena as falhas
int nRPM = 0;                     // Variavel que contém o rpm da maquina
int nInicio = 0;                  // Variavel para indicar que hardware foi iniciado

int nTam1 = 0;
int nTam2 = 0;

//#####################################################
// Variavel para ativar batidas de teste              #
//                                                    #
int nTeste    = 0;   // 0 = Normal -  1 = Teste       #
int nVelTeste = 3.6; // 3.8 * 60 = 228 RPM            #
//                                                    #
//                                                    #
//#####################################################

String cMinN  = "";
String cSecN  = "";
String cMinN2 = "";
String cSecN2 = "";

// Variaveis para controle de mudanças
String cOPAnt;


int nFalhasAnt = 0; // Variavel que armazena as falhas
int nRPMAnt    = 0;

unsigned long int nBatidasAnt       = 0;
unsigned long int nBatidasMesAnt    = 0;
unsigned long int nBatidasOPAnt     = 0;
unsigned long int nFimProducaoAnt   = 0;
unsigned long int nBatidasBobinaAnt = 0;
unsigned long int nFimBobinaAnt     = 0;

// uint8_t nSec, nMin, nHora, nDia, nMes, nAno; //Variaveis que armazenam o tempo do RTC
// uint8_t W;                                   //Variável descartável
byte nSec, nMin, nHora, nDia, nMes, nAno;
byte W;

String cOP;                // Codigo OP
String cMAQUINA;           // Nome da maquina
String stringnFimProducao; // String do limite de batidas

String cMAC;
String cIP; // String que armazena o endereço de IP
String cSubnet;
String cGateway;
String cDNS;
String readString = ""; // Variável que será utilizada para armazenar dados enviados pelo usuário WEB
String NovaData, NovaHora;
String cData, cDataOld, cHorario, cDataOldAnt, cHorarioAnt;

String cTurnoAInicio = "05:50:01";
String cTurnoBInicio = "14:10:01";
String cTurnoCInicio = "22:30:01";

String cTurnoAFim = "14:10:00";
String cTurnoBFim = "22:30:30";
String cTurnoCFim = "05:50:00";

int nPino1 = 0, nPino2 = 0, nPino3 = 0, nPino4 = 0, nPino5 = 0, nPino6 = 0, nPino7 = 0;

int volatile pino = 19;

const int pino_sensor1 = 23; // Pino dos sensores de falha
const int pino_sensor2 = 24;
const int pino_sensor3 = 25;
const int pino_sensor4 = 26;
const int pino_sensor5 = 27;
const int pino_sensor6 = 28;
const int pino_sensor7 = 29;

unsigned long ultimo_valor_salvo = 0;

volatile byte pulsos;

int variacao = 3; // Variável para detectar a !(variação) do rpm

unsigned long timeold; // Armazena tempo
unsigned long millis_old = 0;
static unsigned long millis_anterior;

unsigned int pulsos_por_volta = 1;

void (*funcReset)(void) = 0; // Função que reseta o atmega2560

boolean _lOK = false;
int _nCodError = 0;


File myFile; // Objetos que são usados para abrir arquivos do cartão SD
File myFile2;
File last_Data;

File MACTXT;
File ipTXT;
File SubnetTXT;
File GatewayTXT;
File DNSTXT;
File fArquivo;
File fMaquina;

// file system
SdFat SD;
File fRename;

const int chipSelect = 4;
// store error strings in flash to save RAM
#define error(s) SD.errorHalt(F(s))

void Procura_Dados();
void Guarda_Dados();

void contador()
{
  // Incrementa contador
  nFalhas = 0;    // Reseta as falhas
  variacao = 800; // Fala ao programa que há pulsos
  pulsos++;
}

void CarregaDataHora()
{

  String cDia = "";
  String cMes = "";
  String cHora = "";
  String cMin = "";
  String cSec = "";

  if (day() < 10)
  {
    cDia = "0" + String(day());
  }
  else
  {
    cDia = String(day());
  }

  if (month() < 10)
  {
    cMes = "0" + String(month());
  }
  else
  {
    cMes = String(month());
  }

  if (hour() < 10)
  {
    cHora = "0" + String(hour());
  }
  else
  {
    cHora = String(hour());
  }

  if (minute() < 10)
  {
    cMin = "0" + String(minute());
    cMinN = "0" + String(minute());
  }
  else
  {
    cMin = String(minute());
    cMinN = String(minute());
  }

  if (second() < 10)
  {
    cSec = "0" + String(second());
    cSecN = "0" + String(second());
  }
  else
  {
    cSec = String(second());
    cSecN = String(second());
  }

  cData = cDia + "/" + cMes + "/" + String(year());
  cDataOld = String(year()) + cMes + cDia;
  cHorario = cHora + ":" + cMin + ":" + cSec;

  // Verifica se é inicio de Mês e zera batidas Mes.
  if(cDia == "01"){
    if(cHorario == "00:00:01"){
      nBatidasMes = 0;
    }
  }
}

void botaoAcionado()
{
  nBatidas++;
  nBatidasMes++;
  nBatidasOP++;
  nBatidasBobina++;

  if (millis() - timeold >= 1000)
  {
    // Desabilita interrupcoes durante o calculo
    noInterrupts();
    nRPM = (60 * 1000 / pulsos_por_volta) / (millis() - timeold) * pulsos;
    pulsos = 0;
//    Serial.print("RPM = ");
//    Serial.println(nRPM);

    // Habilita interrupcoes
    interrupts();
    timeold = millis();
  }
}

// Busca dados maquinas
void LeituraMaquina()
{
  //--------------------------Digital Input---------------------------------------
  if (nFalhas == 0)
  {
    if (pino_sensor1)
    { // Entrada 23  URDUME =1
      if (digitalRead(pino_sensor1) != LOW && nFalhas != 1)
      {
        nFalhas = 1;
        Serial.println("1");
      }
    }

    if (pino_sensor2)
    {
      // Entrada 24 TRAMA = 2
      if (digitalRead(pino_sensor2) != LOW && nFalhas != 2)
      {
        nFalhas = 2;
        Serial.println("2");
      }
    }

    if (pino_sensor3)
    {
      // Entrada 25  FALHA GERAL = 4
      if (digitalRead(pino_sensor3) != LOW && nFalhas != 3)
      {
        nFalhas = 3;
        Serial.println("3");
      }
    }

    if (pino_sensor4)
    {
      // Entrada 26
      if (digitalRead(pino_sensor4) != LOW && nFalhas != 4)
      {
        nFalhas = 4;
        Serial.println("4");
      }
    }

    if (pino_sensor5)
    {
      if (digitalRead(pino_sensor5) != LOW && nFalhas != 5)
      {
        nFalhas = 5;
        Serial.println("5");
      }
    }

    if (pino_sensor6)
    {
      // Entrada 28
      if (digitalRead(pino_sensor6) != LOW && nFalhas != 6)
      {
        nFalhas = 6;
        Serial.println("6");
      }
    }

  if (nTeste != 1)
  {
    if (nRPM == 0 && nFalhas == 0)
    {
      nFalhas = 7;
      Serial.println("7");
    }
  }

    /*    if (pino_sensor7){            SERÁ AJUSTADO POSTERIORMENTE
          //Entrada 29
          if (digitalRead(pino_sensor7) == LOW && nFalhas != 8){
            nFalhas = 8;
            Serial.println("8");
          }
        }
    */
  }

  if (variacao < 0)
  {
    nRPM = 0; // Se não tiver pulsos,
  }
  else
  {
    variacao--;
  }

  CarregaDataHora();
  delay(1);

/*
  if (nBatidasOP == nFimProducao)
  {
    digitalWrite(48, HIGH);
    delay(100000);
    digitalWrite(48, LOW);
    Serial.println("OP concluiu a metragem!");
  }

  if (nBatidasOP >= nFimProducao)
  {
    digitalWrite(49, HIGH);
  }

  if (nBatidasOP <= nFimProducao)
  {
    digitalWrite(49, LOW);
  }

  // Verifica Tamanho Bobina
  if (nBatidasBobina == nFimBobina)
  {
    digitalWrite(48, HIGH);
    delay(100000);
    digitalWrite(48, LOW);
    Serial.println("Bobina concluiu a metragem!");
  }

  if (nBatidasBobina >= nFimBobina)
  {
    digitalWrite(49, HIGH);
  }

  if (nBatidasBobina <= nFimBobina)
  {
    digitalWrite(49, LOW);
  }
*/

  if (cMinN != cMinN2)
  {
    cMinN2 = cMinN;
    Guarda_Dados();
  }

  // Gera Batidas manual para teste
  if (nTeste == 1)
  {
    if(nFalhas == 7 || nFalhas == 0){
      if (cSecN != cSecN2)
      {
        nBatidas       += nVelTeste;
        nBatidasOP     += nVelTeste;
        nBatidasBobina += nVelTeste;
        nBatidasMes    += nVelTeste;

        cSecN2 = cSecN;
      }
    }
  }

  if (nFalhasAnt != nFalhas || nInicio == 0 || cOP != cOPAnt || nFimProducao != nFimProducaoAnt || nFimBobina != nFimBobinaAnt)
  {
    fArquivo = SD.open("Data.txt", FILE_WRITE);

    if (fArquivo)
    {
      fArquivo.flush();

      StaticJsonDocument<200> Dados;

      if (nInicio == 0)
      {
        nInicio++;
      }
      else
      {
        Dados["DATA"]           = cDataOld;
        Dados["HORA"]           = cHorario;
        Dados["FALHA"]          = nFalhasAnt;
        Dados["RPM"]            = nRPM;
        Dados["BATIDAS"]        = nBatidasAnt;
        Dados["BATIDAS_MES"]    = nBatidasMesAnt;
        Dados["BATIDAS_OP"]     = nBatidasOPAnt;
        Dados["BATIDAS_BOBINA"] = nBatidasBobinaAnt;
        Dados["OP"]             = cOP;
        Dados["FimProducao"]    = nFimProducao;
        Dados["FimBobina"]      = nFimBobina;
        Dados["Leitura"]        = "C";
        Dados["ID_FALHA"]       = nIdFalhaAnt;

        // Verifica status da Producao
        if (nBatidasOP >= nFimProducao)
        {
          Dados["Status"] = "FimProducao";
        }
        else if (nBatidasBobina >= nFimBobina)
        {
          Dados["Status"] = "FimBobina";
        }
        else
        {
          Dados["Status"] = "Produzindo";
        }

        serializeJson(Dados, fArquivo);
        fArquivo.print("\n");

        // serializeJson(Dados, Serial);
        // Serial.println();
      }

      // Grava Dados Atuais
      Dados["DATA"]           = cDataOld;
      Dados["HORA"]           = cHorario;
      Dados["FALHA"]          = nFalhas;
      Dados["RPM"]            = nRPM;
      Dados["BATIDAS"]        = nBatidas;
      Dados["BATIDAS_MES"]    = nBatidasMes;
      Dados["BATIDAS_OP"]     = nBatidasOP;
      Dados["BATIDAS_BOBINA"] = nBatidasBobina;
      Dados["OP"]             = cOP;
      Dados["FimProducao"]    = nFimProducao;
      Dados["FimBobina"]      = nFimBobina;
      Dados["Leitura"]        = "C";
      Dados["ID_FALHA"]       = nIdFalha;

      // Verifica status da Producao
      if (nBatidasOP >= nFimProducao)
      {
        Dados["Status"] = "FimProducao";
      }
      else if (nBatidasBobina >= nFimBobina)
      {
        Dados["Status"] = "FimBobina";
      }
      else
      {
        Dados["Status"] = "Produzindo";
      }

      serializeJson(Dados, fArquivo);
      fArquivo.print("\n");

      // serializeJson(Dados, Serial);
      // Serial.println();

      fArquivo.close(); // fecha o arquivo

      // Ajuda dados com novos valores
      cOPAnt            = cOP;
      nFalhasAnt        = nFalhas;
      nBatidasAnt       = nBatidas;
      nBatidasMesAnt    = nBatidasMes;
      nBatidasOPAnt     = nBatidasOP;
      nBatidasBobinaAnt = nBatidasBobina;
      nFimProducaoAnt   = nFimProducao;
      nFimBobinaAnt     = nFimBobina;
      nIdFalhaAnt       = nIdFalha;
      
      if(nFalhas != 0){
        nIdFalha++;
      }

      Guarda_Dados();
    } else {
      Serial.println();
      Serial.println("Não encontrou Data.txt 1");
      Serial.println();
      //break;
    }
  } else if(nFalhas == 0){
    nFalhasAnt        = nFalhas;
    nBatidasAnt       = nBatidas;
    nBatidasMesAnt    = nBatidasMes;
    nBatidasOPAnt     = nBatidasOP;
    nIdFalhaAnt       = nIdFalha;
    
    //Serial.println(nFalhas);
  }

  // Força gravação nas trocar de turno
  CarregaDataHora();
  delay(1);

  if (cTurnoAInicio == cHorario ||
      cTurnoBInicio == cHorario ||
      cTurnoCInicio == cHorario ||
      cTurnoAFim == cHorario ||
      cTurnoBFim == cHorario ||
      cTurnoCFim == cHorario)
  {

    fArquivo = SD.open("Data.txt", FILE_WRITE);

    if (fArquivo)
    {
      fArquivo.flush();

      StaticJsonDocument<200> Dados;

      // Grava Dados Atuais
      Dados["DATA"]           = cDataOld;
      Dados["HORA"]           = cHorario;
      Dados["FALHA"]          = nFalhas;
      Dados["RPM"]            = nRPM;
      Dados["BATIDAS"]        = nBatidas;
      Dados["BATIDAS_MES"]    = nBatidasMes;
      Dados["BATIDAS_OP"]     = nBatidasOP;
      Dados["BATIDAS_BOBINA"] = nBatidasBobina;
      Dados["OP"]             = cOP;
      Dados["FimProducao"]    = nFimProducao;
      Dados["FimBobina"]      = nFimBobina;
      Dados["Leitura"]        = "C";
      Dados["ID_FALHA"]       = nIdFalha;

      // Verifica status da Producao
      if (nBatidasOP >= nFimProducao)
      {
        Dados["Status"] = "FimProducao";
      }
      else if (nBatidasBobina >= nFimBobina)
      {
        Dados["Status"] = "FimBobina";
      }
      else
      {
        Dados["Status"] = "Produzindo";
      }

      serializeJson(Dados, fArquivo);
      fArquivo.print("\n");

      fArquivo.close(); // fecha o arquivo

      Guarda_Dados();

      delay(900);
    } else {
      Serial.println();
      Serial.println("Não encontrou Data.txt 2");
      Serial.println();
      //break;
    }
  }

  // Força gravação nas troca de horarios
  CarregaDataHora();
  delay(1);

  if (cHorario.substring(3, 8) == "59:59" || cHorario.substring(3, 8) == "00:00")
  {
    fArquivo = SD.open("Data.txt", FILE_WRITE);

    if (fArquivo)
    {
      fArquivo.flush();

      StaticJsonDocument<200> Dados;

      // Grava Dados Atuais
      //Dados["MAQUINA"]        = cMAQUINA;
      Dados["DATA"]           = cDataOld;
      Dados["HORA"]           = cHorario;
      Dados["FALHA"]          = nFalhas;
      Dados["RPM"]            = nRPM;
      Dados["BATIDAS"]        = nBatidas;
      Dados["BATIDAS_MES"]    = nBatidasMes;
      Dados["BATIDAS_OP"]     = nBatidasOP;
      Dados["BATIDAS_BOBINA"] = nBatidasBobina;
      Dados["OP"]             = cOP;
      Dados["FimProducao"]    = nFimProducao;
      Dados["FimBobina"]      = nFimBobina;
      Dados["Leitura"]        = "C";
      Dados["ID_FALHA"]       = nIdFalha;

      // Verifica status da Producao
      if (nBatidasOP >= nFimProducao)
      {
        Dados["Status"] = "FimProducao";
      }
      else if (nBatidasBobina >= nFimBobina)
      {
        Dados["Status"] = "FimBobina";
      }
      else
      {
        Dados["Status"] = "Produzindo";
      }

      serializeJson(Dados, fArquivo);
      fArquivo.print("\n");

      fArquivo.close(); // fecha o arquivo

      Guarda_Dados();

      delay(900);
    } else {
      Serial.println();
      Serial.println("Não encontrou Data.txt 3");
      Serial.println();
      //break;
    }
  }
}


void setup()
{

  Serial.begin(115200); // Inicia a comunicação Serial

  pinMode(SDCARD_SS_PIN, OUTPUT);
  digitalWrite(SDCARD_SS_PIN, HIGH); // to disable SD card while Ethernet initializes

  Serial.println();
  Serial.println();
  Serial.println("############################");
  Serial.println("|                          |");
  Serial.println("|  TEAR_VERSAO_FULL_2.9." + String(VERSION) + "  |");
  Serial.println("|                          |");
  Serial.println("############################");

  rtc.begin();              // Inicia o RTC
  setSyncProvider((getExternalTime)RTC.get()); // the function to get the time from the RTC

  Serial.println();

  if (timeStatus() != timeSet)
  {
    Serial.println("Unable to sync with the RTC");
  }
  else
  {
    Serial.println("RTC has set the system time");
  }

  Serial.println();

  pinMode(pino, INPUT);

  ///// digitalRead(pino32) se 1
  pinMode(pino32, INPUT); // Pino Preto
  pinMode(pino36, INPUT); // Pino 1 Vermelho

  pulsos = 0; // Zera variáveis importantes
  nRPM = 0;
  timeold = 0;

  //  setSyncProvider(RTC.get);   // the function to get the time from the RTC

  pinMode(para, OUTPUT);
  pinMode(pinBotao, INPUT_PULLUP);
  pinMode(pino_sensor1, INPUT);
  pinMode(pino_sensor2, INPUT);
  pinMode(pino_sensor3, INPUT);
  pinMode(pino_sensor4, INPUT);
  pinMode(pino_sensor5, INPUT);
  pinMode(pino_sensor6, INPUT);
  pinMode(pino_sensor7, INPUT);

  Serial.println("Iniciando o cartao SD...");
  if(!SD.begin(chipSelect, SPI_HALF_SPEED))
    //SD.initErrorHalt();  
  //if (!SD.begin(SDCARD_SS_PIN))
  {
    Serial.println("Erro ao inicializar o cartão!");
    Serial.println();
    Serial.println("Reiniciando sistema em 5 segundos....");
    delay(5000);
                
    #ifdef __AVR__      
      wdt_enable(WDTO_15MS);
      while (true);
    #else
      NVIC_SystemReset();
    #endif
    
    // don't continue:
    while (true);
  }
  Serial.println("Cartao iniciado com SUCESSO!!");

  if(SD.remove(BIN_FILENAME)){
    Serial.println();
    Serial.println("Firmware.bin removido com sucesso...");
    Serial.println();
  }

  Procura_Dados(); // Procura os dados salvos no cartão SD

  Serial.println("Conectando...");
  delay(1);
  // Ethernet.begin(mac, ip, dns, gateway, subnet);
  Ethernet.begin(aMAC, aIP, aGateway, aSubnet, aDNS); // Inicializa a Ethernet Shield

  delay(1);
  int status_rede = Ethernet.linkStatus();

  Serial.println(status_rede);

  if (status_rede == LinkON)
  {
    Serial.println("Conectado!");
  }
  else
  {
    Serial.println("Nao conectado!");
    Serial.print("IP:");

    for (int i = 0; i < 4; i++)
    {
      Serial.print(aIP[i]);
      Serial.print(".");
    }

    Serial.println();
    Serial.print("Mac:");

    for (int i = 0; i < 6; i++)
    {
      Serial.print(aMAC[i], HEX);
      Serial.print(".");
    }

    Serial.println();
    Serial.print("Erro:");

    switch (status_rede)
    {
    case Unknown:
      Serial.println("Desconhecido.");
      break;
    case LinkOFF:
      Serial.println("Desconectado.");
      break;
    case LinkON:
      Serial.println("Sem erros");
      break;
    default:
      Serial.println("Nao listado");
      break;
    }

    Serial.println();
    Serial.println("Reiniciando sistema em 5 segundos....");
    delay(5000);
                
    #ifdef __AVR__      
      wdt_enable(WDTO_15MS);
      while (true);
    #else
      NVIC_SystemReset();
    #endif
      
    // don't continue:
    while (true);
  }

  server.begin(); // Inicia esperando por requisições dos clientes (Browsers)
  attachInterrupt(4, contador, RISING);
  attachInterrupt(digitalPinToInterrupt(pinBotao), botaoAcionado, FALLING);

  // watchdog.enable(Watchdog::TIMEOUT_8S);
}

void reiniciarRobocore()
{
  // Função que reinicia o Robocore
  noInterrupts();

  Serial.println("REINICIANDO ROBOCORE AGUARDE...");
  delay(5000);

  funcReset(); // Essa função tem um bug que faz com que, quando executada, retorne à instrução 0000

  // don't continue:
  while (true);
}

// Função para ver se há quaisquer caracteres não numéricos em uma String
bool verificaChar(String *lol)
{
  char m[lol->length() + 1]; // O mais 1 é para a String lol conseguir passar todos os caracteres para o vetor char
  lol->toCharArray(m, lol->length() + 1, 0);

  for (int x = 0; x < lol->length(); x++)
  {
    if (!isDigit(m[x]))
    {
      // isDigit retorna true se for um numero
      return false;
    }
  }

  return true;
}

void pegar_tempo()
{
  rtc.getDate(nAno, nMes, nDia, W); // Pega Ano,Mes,Dia e Semana(W)
  rtc.getTime(nHora, nMin, nSec);   // Pega hora, minuto e segundo
}

void Procura_Dados()
{

  String valor = "";

  Serial.println("");
  Serial.println("############################################");
  Serial.println("#                                          #");
  Serial.println("#        CARREGANDO DADOS DE REDE          #");
  Serial.println("#                                          #");
  Serial.println("############################################");
  Serial.println("#                                          #");

  // Busca IP
  if (SD.exists("MAC.txt"))
  {
    // Verifica se ip.txt existe no cartão CASO CONTRARIO O OPERADOR DEVE NECESSARIAMENTE CRIAR UM IP.txt NO CARTÃO MANUALMENTE

    MACTXT = SD.open("MAC.txt", FILE_READ); // Abre o arquivo ip.txt para ler
    MACTXT.flush();                         // limpa o buffer do objeto

    if (MACTXT)
    { // verifica se foi realizado a abertura com sucesso    
      char* pBuffer;                              // Declare a pointer to your buffer.
      {
          unsigned int fileSize = MACTXT.size();  // Get the file size.
          pBuffer = (char*)malloc(fileSize + 1);  // Allocate memory for the file and a terminating null char.
          MACTXT.read(pBuffer, fileSize);         // Read the file into the buffer.
          pBuffer[fileSize] = '\0';               // Add the terminating null char.
//          Serial.println(pBuffer);                // Print the file to the serial monitor.
//          MACTXT.close();                         // Close the file.
      }

      sscanf(pBuffer, "%2x%2x%2x%2x%2x%2x", &aMAC[0], &aMAC[1], &aMAC[2], &aMAC[3], &aMAC[4], &aMAC[5] );
      // *** Use the buffer as needed here. ***
      free(pBuffer); 

      Serial.print("#  MAC    : ");
      for (int i = 0; i < 6; i++)
      {
        Serial.print(aMAC[i], HEX);
        if(i < 5)
        Serial.print(":");
      }
      Serial.print("               #");
      Serial.println("");
      
      //Serial.println("#  MAC    : " + valor + "              #");
    }
  }
  else
  {
    Serial.println("#  MAC: Arquivo não encontrado                   #");
  }

  MACTXT.close(); // fecha o txt
  valor = "";     // zera a String para ser usada abaixo

  // Busca IP
  if (SD.exists("ip.txt"))
  {
    // Verifica se ip.txt existe no cartão CASO CONTRARIO O OPERADOR DEVE NECESSARIAMENTE CRIAR UM IP.txt NO CARTÃO MANUALMENTE

    ipTXT = SD.open("ip.txt", FILE_READ); // Abre o arquivo ip.txt para ler
    ipTXT.flush();                        // limpa o buffer do objeto

    if (ipTXT)
    { // verifica se foi realizado a abertura com sucesso

      while (ipTXT.available())
      { // enquanto tiver informações no txt
        valor += (char)ipTXT.read();
      }
      Serial.println("#  IP     : " + valor + "                  #");
    }

    int a = -1; // variável com valor -1 por motivos de programação

    for (int x = 0; x < 4; x++)
    {                                            // for com o tamanho do aIP
      aIP[x] = (valor.substring(a + 1).toInt()); // pega o primeiro endereço do ip antes do ponto
      if (valor.indexOf(".") != -1)
      {
        a = valor.indexOf("."); // Verifica se há algum ponto ainda
      }

      valor.setCharAt(a, ' '); // remove o ponto pra não bugar o if
    }
  }
  else
  {
    Serial.println("#  IP: Arquivo não encontrado                    #");
  }

  ipTXT.close(); // fecha o txt
  valor = "";    // zera a String para ser usada abaixo

  // Busca MASCARA
  if (SD.exists("SUBNET.txt"))
  {
    // Verifica se ip.txt existe no cartão CASO CONTRARIO O OPERADOR DEVE NECESSARIAMENTE CRIAR UM IP.txt NO CARTÃO MANUALMENTE

    SubnetTXT = SD.open("SUBNET.txt", FILE_READ); // Abre o arquivo ip.txt para ler
    SubnetTXT.flush();                            // limpa o buffer do objeto

    if (SubnetTXT)
    { // verifica se foi realizado a abertura com sucesso

      while (SubnetTXT.available())
      { // enquanto tiver informações no txt
        valor += (char)SubnetTXT.read();
      }

      Serial.println("#  SUBNET : " + valor + "                 #");
    }

    int a = -1; // variável com valor -1 por motivos de programação

    for (int x = 0; x < 4; x++)
    {                                                // for com o tamanho do aIP
      aSubnet[x] = (valor.substring(a + 1).toInt()); // pega o primeiro endereço do ip antes do ponto
      if (valor.indexOf(".") != -1)
      {
        a = valor.indexOf("."); // Verifica se há algum ponto ainda
      }

      valor.setCharAt(a, ' '); // remove o ponto pra não bugar o if
    }
  }
  else
  {
    Serial.println("#  SUBNET : Arquivo não encontrado         #");
  }

  SubnetTXT.close(); // fecha o txt
  valor = "";        // zera a String para ser usada abaixo

  // Busca GATEWAY
  if (SD.exists("GATEWAY.txt"))
  {
    // Verifica se ip.txt existe no cartão CASO CONTRARIO O OPERADOR DEVE NECESSARIAMENTE CRIAR UM IP.txt NO CARTÃO MANUALMENTE

    GatewayTXT = SD.open("GATEWAY.txt", FILE_READ); // Abre o arquivo ip.txt para ler
    GatewayTXT.flush();                             // limpa o buffer do objeto

    if (GatewayTXT)
    { // verifica se foi realizado a abertura com sucesso

      while (GatewayTXT.available())
      { // enquanto tiver informações no txt
        valor += (char)GatewayTXT.read();
      }
      Serial.println("#  GATEWAY: " + valor + "            #");
    }

    int a = -1; // variável com valor -1 por motivos de programação

    for (int x = 0; x < 4; x++)
    {                                                 // for com o tamanho do aIP
      aGateway[x] = (valor.substring(a + 1).toInt()); // pega o primeiro endereço do ip antes do ponto
      if (valor.indexOf(".") != -1)
      {
        a = valor.indexOf("."); // Verifica se há algum ponto ainda
      }

      valor.setCharAt(a, ' '); // remove o ponto pra não bugar o if
    }
  }
  else
  {
    Serial.println("#  GATEWAY: Arquivo não encontrado         #");
  }

  GatewayTXT.close(); // fecha o txt
  valor = "";         // zera a String para ser usada abaixo

  // Busca DNS
  if (SD.exists("DNS.txt"))
  {
    // Verifica se ip.txt existe no cartão CASO CONTRARIO O OPERADOR DEVE NECESSARIAMENTE CRIAR UM IP.txt NO CARTÃO MANUALMENTE

    DNSTXT = SD.open("DNS.txt", FILE_READ); // Abre o arquivo ip.txt para ler
    DNSTXT.flush();                         // limpa o buffer do objeto

    if (DNSTXT)
    { // verifica se foi realizado a abertura com sucesso

      while (DNSTXT.available())
      { // enquanto tiver informações no txt
        valor += (char)DNSTXT.read();
      }
      Serial.println("#  DNS    : " + valor + "                        #");
    }

    int a = -1; // variável com valor -1 por motivos de programação

    for (int x = 0; x < 4; x++)
    {                                             // for com o tamanho do aIP
      aDNS[x] = (valor.substring(a + 1).toInt()); // pega o primeiro endereço do ip antes do ponto
      if (valor.indexOf(".") != -1)
      {
        a = valor.indexOf("."); // Verifica se há algum ponto ainda
      }

      valor.setCharAt(a, ' '); // remove o ponto pra não bugar o if
    }
  }
  else
  {
    Serial.println("#  DNS    : Arquivo não encontrado         #");
  }

  Serial.println("#                                          #");
  Serial.println("############################################");
  Serial.println();

  DNSTXT.close(); // fecha o txt
  valor = "";     // zera a String para ser usada abaixo

  // Carrega dados anteriores
  if (SD.exists("lastval.txt"))
  {
    last_Data = SD.open("lastval.txt", FILE_READ);
    last_Data.flush();

    if (last_Data)
    {
      while (last_Data.available())
      {
        // Exibe o conteúdo do Arquivo
        valor += (char)last_Data.read();
      }

      last_Data.close();

      if (SD.exists("lastval.txt"))
      {
    
        Serial.println();

        Serial.println("####################################");
        Serial.println("#                                  #");
        Serial.println("#         DADOS CARREGADOS         #");
        Serial.println("#                                  #");
        Serial.println("####################################");
        Serial.println("#                                  #");

        
        StaticJsonDocument<384> Dados;

        // Deserialize the JSON document
        DeserializationError error = deserializeJson(Dados, valor);
      
        // Test if parsing succeeds.
        if (error) {
          Serial.print(F("deserializeJson() erro: "));
          Serial.println(error.f_str());
                  
          Serial.println("ERRO deserializeJson....");
          Serial.println();
          Serial.println("Reiniciando sistema em 5 segundos....");
          
          delay(5000);
                      
          #ifdef __AVR__      
            wdt_enable(WDTO_15MS);
            while (true);
          #else
            NVIC_SystemReset();
          #endif
          
          // don't continue:
          while (true);
        }

        String cMAQUINA2       = Dados["MAQUINA"];
        String cOP2            = Dados["OP"];
        nBatidasMes            = Dados["BatidasMes"];
        nBatidas               = Dados["Batidas"];
        nBatidasOP             = Dados["BatidasOP"];
        nBatidasBobina         = Dados["BatidasBobina"];
        nFimBobina             = Dados["FimBobina"];
        nFimProducao           = Dados["FimProducao"];
        String cTurnoAInicio2  = Dados["TurnoAInicio"];
        String cTurnoBInicio2  = Dados["TurnoBInicio"];
        String cTurnoCInicio2  = Dados["TurnoCInicio"];
        String cTurnoAFim2     = Dados["TurnoAFim"];
        String cTurnoBFim2     = Dados["TurnoBFim"];
        String cTurnoCFim2     = Dados["TurnoCFim"];
        nIdFalha               = Dados["ID_FALHA"];

        if(cOP2 != "null"){
          cOP = cOP2;
        }
        if(cMAQUINA2 != "null")
          cMAQUINA = cMAQUINA2;

        Serial.print("#   cMaquina       :  ");
        Serial.println(cMAQUINA);

        Serial.print("#   cOP            :  ");
        Serial.println(cOP);

        Serial.print("#   nBatidas       :  ");
        Serial.println(nBatidas);

        //nBatidasMes = valor.substring(valor.indexOf("BatidasMes=") + 11, valor.indexOf("BatidasOP=") - 1).toInt();
        Serial.print("#   nBatidasMes    :  ");
        Serial.println(nBatidasMes);

        //nBatidasOP = valor.substring(valor.indexOf("BatidasOP=") + 10, valor.indexOf("BatidasBobina=") - 1).toInt();
        Serial.print("#   nBatidasOP     :  ");
        Serial.println(nBatidasOP);

        //nBatidasBobina = valor.substring(valor.indexOf("BatidasBobina=") + 14, valor.indexOf("FimBobina=") - 1).toInt();
        Serial.print("#   nBatidasBobina :  ");
        Serial.println(nBatidasBobina);

        //nFimBobina = valor.substring(valor.indexOf("FimBobina=") + 10, valor.indexOf("FimProducao=") - 1).toInt();
        Serial.print("#   nFimBobina     :  ");
        Serial.println(nFimBobina);

        //nFimProducao = valor.substring(valor.indexOf("FimProducao=") + 12, valor.indexOf("TurnoAInicio=") - 1).toInt();
        Serial.print("#   nFimProducao   :  ");
        Serial.println(nFimProducao);

        Serial.println("#                                  #");
        Serial.println("####################################");
        Serial.println("#                                  #");
        Serial.println("#         LISTA DE TURNOS          #");
        Serial.println("#                                  #");
        Serial.println("####################################");
        Serial.println("#                                  #");

        // Inicio
        if (cTurnoAInicio2 != "" && cTurnoAInicio2 != "null")
        {
          cTurnoAInicio = cTurnoAInicio2;
        }
        Serial.print("#   cTurnoAInicio   :  ");
        Serial.println(cTurnoAInicio + "    #");

        if (cTurnoBInicio2 != "" && cTurnoBInicio2 != "null")
        {
          cTurnoBInicio = cTurnoBInicio2;
        }
        Serial.print("#   cTurnoBInicio   :  ");
        Serial.println(cTurnoBInicio + "    #");

        if (cTurnoCInicio2 != "" && cTurnoCInicio2 != "null")
        {
          cTurnoCInicio = cTurnoCInicio2;
        }
        Serial.print("#   cTurnoCInicio   :  ");
        Serial.println(cTurnoCInicio + "    #");

        // Fim
        if (cTurnoAFim2 != "" && cTurnoAFim2 != "null")
        {
          cTurnoAFim = cTurnoAFim2;
        }
        Serial.print("#   cTurnoAFim      :  ");
        Serial.println(cTurnoAFim + "    #");

        if (cTurnoBFim2 != "" && cTurnoBFim2 != "null")
        {
          cTurnoBFim = cTurnoBFim2;
        }
        Serial.print("#   cTurnoBFim      :  ");
        Serial.println(cTurnoBFim + "    #");

        if (cTurnoCFim2 != "" && cTurnoCFim2 != "null")
        {
          cTurnoCFim = cTurnoCFim2;
        }
        Serial.print("#   cTurnoCFim      :  ");
        Serial.println(cTurnoCFim + "    #");

        Serial.println("#                                  #");
        Serial.println("####################################");
        Serial.println();
      }
      else
      {
        last_Data.close();

        Serial.println("ERRO lastval não encontrado....");
        Serial.println();
        Serial.println("Reiniciando sistema em 5 segundos....");
        
        delay(5000);
                    
        #ifdef __AVR__      
          wdt_enable(WDTO_15MS);
          while (true);
        #else
          NVIC_SystemReset();
        #endif
        
        // don't continue:
        while (true);
      }
    }
    else
    {
      last_Data.close();

      Serial.println("ERRO, não foi possível abrir arquivo lastval...");
      Serial.println();
      Serial.println("Reiniciando sistema em 5 segundos....");

      delay(5000);
            
      #ifdef __AVR__      
        wdt_enable(WDTO_15MS);
        while (true);
      #else
        NVIC_SystemReset();
      #endif

      // don't continue:
      while (true);
    }
  }

  pegar_tempo();
}

void Guarda_Dados()
{
  // guarda dados no lastval.txt
  if(SD.exists("lastval.txt")){
    SD.remove("lastval.txt");                       // remove para guardar novas informações

    last_Data = SD.open("lastval.txt", FILE_WRITE); // Abre o lastval para escrever
    last_Data.flush();                              // Limpa o buffer

    if (last_Data) // verifica se existe o arquivo lastval.txt
    {    
      StaticJsonDocument<200> Dados;

      // Grava Dados Atuais
      Dados["MAQUINA"]       = cMAQUINA;
      Dados["OP"]            = cOP;
      Dados["Batidas"]       = nBatidas;
      Dados["BatidasMes"]    = nBatidasMes;
      Dados["BatidasOP"]     = nBatidasOP;
      Dados["BatidasBobina"] = nBatidasBobina;
      Dados["FimBobina"]     = nFimBobina;
      Dados["FimProducao"]   = nFimProducao;
      Dados["TurnoAInicio"]  = cTurnoAInicio;
      Dados["TurnoBInicio"]  = cTurnoBInicio;
      Dados["TurnoCInicio"]  = cTurnoCInicio;
      Dados["TurnoAFim"]     = cTurnoAFim;
      Dados["TurnoBFim"]     = cTurnoBFim;
      Dados["TurnoCFim"]     = cTurnoCFim;
      Dados["ID_FALHA"]      = nIdFalha;
/*
      Serial.println();
      serializeJson(Dados, Serial);
      Serial.println();
*/
      serializeJson(Dados, last_Data);
      last_Data.close();
      // Serial.println("Dados Gravados com sucesso!");
    } else {
      
      Serial.println("ERRO, não foi possível abrir arquivo lastval...");
      Serial.println();
      Serial.println("Reiniciando sistema em 5 segundos....");

      delay(5000);
            
      #ifdef __AVR__      
        wdt_enable(WDTO_15MS);
        while (true);
      #else
        NVIC_SystemReset();
      #endif

      // don't continue:
      while (true);
    }
  }
}

// DOWNLOAD FIRMEWARE
void handleSketchDownload()
{

  const char *SERVER = "192.168.20.7"; // must be string for HttpClient
  const unsigned short SERVER_PORT = 3000;
  const char *PATH = "/firmwares/update-v%d.bin";
  const unsigned long CHECK_INTERVAL = 5000;

  static unsigned long previousMillis;

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis < CHECK_INTERVAL)
    return;
  previousMillis = currentMillis;

  EthernetClient transport;
  HttpClient http(transport, SERVER, SERVER_PORT);
  http.setTimeout(5000);

  char buff[32];
  snprintf(buff, sizeof(buff), PATH, VERSION + 1);

  Serial.println();
  Serial.print("Procurando arquivo de atualização: ");
  Serial.println(buff);

  http.get(buff);  

  int statusCode = http.responseStatusCode();
  Serial.print("Conexão status: ");
  Serial.println(statusCode);
  if (statusCode != 200)
  {
    _lOK = false;
    _nCodError = statusCode;

    http.stop();
    return;
  }

  File file = SD.open(BIN_FILENAME, O_CREAT | O_WRITE);
  if (!file)
  {
    _lOK = false;
    _nCodError = 98;

    http.stop();
    Serial.println("Não foi possível criar o arquivo bin. Atualização não ira continuar.");
    return;
  }

  long length = http.contentLength();
  if (length == HttpClient::kNoContentLengthHeader)
  {
    _lOK = false;
    _nCodError = 99;

    http.stop();
    Serial.println("Server didn't provide Content-length header. Can't continue with update.");
    return;
  }

  //Serial.print("Download do firmware finalizado com o tamanho: ");
  //Serial.print(length);
  //Serial.println(" bytes");
  
  byte clientBuf[128];
  int clientCount = 0;
  while (http.available())
  {
    clientBuf[clientCount] = http.read();
    clientCount++;
    if (clientCount > 127)
    {
      file.write(clientBuf, 128); 
      clientCount = 0;
    }
  }

  // final <64 byte cleanup packet
  if (clientCount > 0){
    file.write(clientBuf, clientCount);
    clientCount = 0;
  }

  Serial.println();

  Serial.print("Download do firmware finalizado com o tamanho: ");
  Serial.print(file.size());
  Serial.print(" bytes");

  nTam1 = file.size();
  Serial.println();
  
  if(nTam1 != nTam2){
    Serial.println("Erro ao realizar download!");
    Serial.print("Tamanho: ");
    Serial.print(String(nTam2));    
    Serial.print(" bytes");

    SD.remove(BIN_FILENAME);
    file.close();
    http.stop();

    return; 
  }

  file.close();
  http.stop();

  if (clientCount > 0)
  {
    _lOK = false;
    _nCodError = 100;
    
    SD.remove(BIN_FILENAME);
    Serial.print("Timeout downloading update file at ");
    Serial.print(clientCount);
    Serial.println(" bytes. Can't continue with update.");
    return;
  }

  Serial.println("Arquivos de atualização salvo. Reset em progresso....");
  Serial.flush();

  _lOK = true;

}


void loop()
{
  // watchdog.reset();
  char cLinha = (char)"";

  client = server.available(); // Tenta pegar uma conexão com o cliente (Browser)
  //Serial.println(client);

  LeituraMaquina();

  /*                                          ATIVAR SOMENTE NO FUTURO. BOTAO DE RESET
    //Botao Preto Direita
    if(digitalRead(pino32) == 1){
      Serial.println("");
      Serial.println("Zerando Batidas OP....");

      nBatidasOP = 0;

      delay(500);
    }

    //Botao Vermelho Direita
    if(digitalRead(pino36) == 1){
      Serial.println("");
      Serial.println("Zerando Batidas Bobina....");

      nBatidasBobina = 0;

      delay(500);
    }
  */

  if (client)
  { // Existe um cliente em conexão ?
    boolean currentLineIsBlank = true;
    String currentLine = "";

    while (client.connected())
    {
      if (client.available())
      {                         // os dados do cliente estão disponiveis para serem lidos
        cLinha = client.read(); // lê 1 byte (character) do cliente

        if (readString.length() < 100)
        {
          readString += cLinha;
        }

        // a ultima linha da requisição do cliente é branca e termina com o caractere \n
        // responde para o cliente apenas após a última linha recebida
        if (cLinha == '\n' && currentLineIsBlank)
        {

          //-------------------------------------------- AJUSTA DATA --------------------------------------------
          if (readString.indexOf("/Data=") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("Data", 3);  // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("Data") + 4; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "Data")
            {
              NovaData = readString.substring(readString.indexOf("Data=") + 5, readString.indexOf("      ")).toInt();

              nDia = NovaData.substring(6, 8).toInt();
              nMes = NovaData.substring(4, 6).toInt();
              nAno = NovaData.substring(2, 4).toInt();

              // rtc.getDate(nAno, nMes, nDia, W);
              rtc.setDate(nAno, nMes, nDia);
              setSyncProvider((getExternalTime)RTC.get());
              RTC.set(now());

              Serial.println("");
              Serial.println("Data Ajustada");
              Serial.println(String(nDia) + "/" + String(nMes) + "/" + String(nAno));
              Serial.println("");

              StaticJsonDocument<200> Dados;

              String cDia = "";
              String cMes = "";

              if (day() < 10)
              {
                cDia = "0" + String(day());
              }
              else
              {
                cDia = String(day());
              }

              if (month() < 10)
              {
                cMes = "0" + String(month());
              }
              else
              {
                cMes = String(month());
              }

              Dados["DATA"] = String(year()) + cMes + cDia;
              Dados["Status"] = "OK";

              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: application/json;charset=utf-8");
              client.println("Access-Control-Allow-Origin: *");
              client.println("Connection: close");
              client.println();

              serializeJson(Dados, client);

              readString = "";
              break;
            }
          }

          //-------------------------------------------- AJUSTA HORA --------------------------------------------
          if (readString.indexOf("/Hora=") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("Hora", 2);  // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("Hora") + 4; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "Hora")
            {
              NovaHora = readString.substring(readString.indexOf("Hora=") + 5, readString.indexOf("      ")).toInt();

              if (NovaHora.length() == 5)
              {
                nHora = NovaHora.substring(0, 1).toInt();
                nMin = NovaHora.substring(1, 3).toInt();
                nSec = NovaHora.substring(3, 5).toInt();
              }
              else
              {
                nHora = NovaHora.substring(0, 2).toInt();
                nMin = NovaHora.substring(2, 4).toInt();
                nSec = NovaHora.substring(4, 6).toInt();
              }

              // rtc.getTime(nHora, nMin, nSec);
              rtc.setTime(nHora, nMin, nSec);
              setSyncProvider((getExternalTime)RTC.get());
              RTC.set(now());

              Serial.println("");
              Serial.println("Hora Ajustada");
              Serial.println(String(nHora) + ":" + String(nMin) + ":" + String(nSec));

              Serial.println("");

              StaticJsonDocument<200> Dados;

              String cHora = "";
              String cMin = "";
              String cSec = "";

              if (hour() < 10)
              {
                cHora = "0" + String(hour());
              }
              else
              {
                cHora = String(hour());
              }

              if (minute() < 10)
              {
                cMin = "0" + String(minute());
              }
              else
              {
                cMin = String(minute());
              }

              if (second() < 10)
              {
                cSec = "0" + String(second());
              }
              else
              {
                cSec = String(second());
              }

              Dados["HORA"] = cHora + ":" + cMin + ":" + cSec;
              Dados["Status"] = "OK";

              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: application/json;charset=utf-8");
              client.println("Access-Control-Allow-Origin: *");
              client.println("Connection: close");
              client.println();

              serializeJson(Dados, client);

              readString = "";
              break;
            }
          }

          //-------------------------------------------- REALIZA MUDANÇA DE ARQUIVO NO SD PARA LISTAGEM NOVA ---------------------------F-----------------
          if (readString.indexOf("/DEL") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("DEL", 3);  // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("DEL") + 3; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "DEL")
            {
              StaticJsonDocument<200> Dados;

              if (SD.exists("DataRead.txt"))
              {
                SD.remove("DataRead.txt");
                Serial.println("Removendo DataRead.txt...");
                Dados["REMOVIDO"] = "SIM";
              }
              else
              {
                Dados["REMOVIDO"] = "NAO";
              }

              digitalWrite(48, LOW);
              digitalWrite(49, LOW);

              Dados["Status"] = "OK";

              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: application/json;charset=utf-8");
              client.println("Access-Control-Allow-Origin: *");
              client.println("Connection: close");
              client.println();

              serializeJson(Dados, client);

              readString = "";
              break;
            }
          }

          // Lista dados gravados no Cartao
          if (readString.indexOf("/Listar") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("Listar", 3);  // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("Listar") + 6; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "Listar")
            {

              Serial.println("Lendo SD...");

              boolean lOK = false;

              myFile = SD.open("Data.txt"); // abre o arquivo da pagina WEB
              if (!SD.exists("DataRead.txt"))
              {
                if (myFile)
                {

                  delay(1);
                  
                  if(myFile.rename("DataRead.txt")) {
                    Serial.println("Arquivo renomeado para DataRead.txt");
                  } else {      
                    //error("DataRead.txt");

                    Serial.println("Arquivo não encontrado DataRead.txt");

                    myFile.close(); // fecha o arquivo
                    myFile2.close();

                    readString = "";
                    break;
                  }
                }
                
                SD.remove("Data.txt");
                myFile2.close();
                myFile2 = SD.open("DataRead.txt"); // abre o arquivo da pagina WEB
              }
              else
              {
                myFile2 = SD.open("DataRead.txt"); // abre o arquivo da pagina WEB
              }

              if (myFile2)
              {
                Serial.println("Download dados do SD...");
                
                delay(1);

                if (HTTP_req, "GET /DataRead.txt")
                {
                  client.println("HTTP/1.1 200 OK");
                  client.println("Content-Disposition: attachment; filename=DataRead.txt");
                  client.println("Connection: close");
                  client.println();
                 
                  CarregaDataHora();
                  Serial.println("Inicio: " + cHorario);

                  byte clientBuf[128];
                  int clientCount = 0;
                  while (myFile2.available())
                  {
                    lOK = true;
                    clientBuf[clientCount] = myFile2.read();
                    clientCount++;

                    if (clientCount > 127)
                    {
//                      LeituraMaquina();
                      client.write(clientBuf, 128);
                      clientCount = 0;

                      // Perde 2 Kb no download, estudar furamente se realmente a necessidade dessas funções abaixo                                           
                      if(client.connected() == 0){
                        Serial.println("Cliente desconectado, download interrompido!");

                        lOK = false;
                        break;
                      }
                    }
                  }

                  CarregaDataHora();
                  Serial.println("Fim: " + cHorario);

                  // final <64 byte cleanup packet
                  if (clientCount > 0)
                    client.write(clientBuf, clientCount);
                }
              }

              if (!lOK)
              {
                CarregaDataHora();
                delay(1);
                
                if (HTTP_req, "GET /DataRead.txt")
                {
                  client.println("HTTP/1.1 200 OK");
                  client.println("Content-Disposition: attachment; filename=DataRead.txt");
                  client.println("Connection: close");
                  client.println();

                  // Caso nao entre em nehuma anterior mostra dados
                  StaticJsonDocument<200> Dados;

                  String cDia = "";
                  String cMes = "";
                  String cHora = "";
                  String cMin = "";
                  String cSec = "";

                  //Dados["MAQUINA"]        = cMAQUINA;
                  Dados["DATA"]           = cDataOld;
                  Dados["HORA"]           = cHorario;
                  Dados["FALHA"]          = nFalhas;
                  Dados["RPM"]            = nRPM;
                  Dados["BATIDAS"]        = nBatidas;
                  Dados["BATIDAS_MES"]    = nBatidasMes;
                  Dados["BATIDAS_OP"]     = nBatidasOP;
                  Dados["BATIDAS_BOBINA"] = nBatidasBobina;
                  Dados["OP"]             = cOP;
                  Dados["FimProducao"]    = nFimProducao;
                  Dados["FimBobina"]      = nFimBobina;
                  Dados["Leitura"]        = "A";
                  Dados["ID_FALHA"]       = nIdFalha;

                  // Verifica status da Producao
                  if (nBatidasOP >= nFimProducao)
                  {
                    Dados["Status"] = "FimProducao";
                  }
                  else
                  {
                    Dados["Status"] = "Produzindo";
                  }
                 
                  serializeJson(Dados, client);
                }
                
              }

              myFile.close(); // fecha o arquivo
              myFile2.close();

              readString = "";
              break;
            }
          }

          //-------------------------------------------- REALIZA MUDANÇA FIM DE BOBINA --------------------------------------------
          if (readString.indexOf("/FimBobina=") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("FimBobina", 3);  // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("FimBobina") + 9; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "FimBobina")
            {
              unsigned long Posicao1 = readString.indexOf("FimBobina=") + 10; // pega a posição do = antes do valor
              unsigned long Posicao2 = readString.indexOf("HTTP/") - 1;       // pega a posição do fim depois do valor

              String nTotal;

              nTotal = readString.substring(Posicao1, Posicao2);
              nTotal.trim();

              if (verificaChar(&nTotal))
              {
                // Verifica se tem algum caractere não numérico inserido pelo usuário
                nFimBobina = nTotal.toInt();

                Serial.println("Fim de Bobina Ajustado");
                Serial.println(nFimBobina);
                // unsigned long Ajuste = nFimProducao;
                // stringnFimProducao = nFimProducao;
                readString = "";
                // nBatidasOP = 0;

                LeituraMaquina();
              }

              StaticJsonDocument<200> Dados;
              Dados["Fim_Bobina"] = String(nTotal);
              Dados["Status"] = "OK";

              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: application/json;charset=utf-8");
              client.println("Access-Control-Allow-Origin: *");
              client.println("Connection: close");
              client.println();

              serializeJson(Dados, client);

              readString = "";
              break;
            }
          }

          //-------------------------------------------- REALIZA MUDANÇA FIM DE PRODUCAO --------------------------------------------
          if (readString.indexOf("/FimProducao=") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("FimProducao", 3);   // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("FimProducao") + 11; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "FimProducao")
            {
              unsigned long Posicao1 = readString.indexOf("FimProducao=") + 12; // pega a posição do = antes do valor
              unsigned long Posicao2 = readString.indexOf("HTTP/") - 1;         // pega a posição do fim depois do valor

              String nTotal;

              nTotal = readString.substring(Posicao1, Posicao2);
              nTotal.trim();

              if (verificaChar(&nTotal))
              {
                // Verifica se tem algum caractere não numérico inserido pelo usuário
                nFimProducao = nTotal.toInt();

                Serial.println("Fim de Producao Ajustado");
                Serial.println(nFimProducao);
                // unsigned long Ajuste = nFimProducao;
                // stringnFimProducao = nFimProducao;
                readString = "";
                // nBatidasOP = 0;

                LeituraMaquina();
              }

              StaticJsonDocument<200> Dados;
              Dados["Fim_Producao"] = String(nTotal);
              Dados["Status"] = "OK";

              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: application/json;charset=utf-8");
              client.println("Access-Control-Allow-Origin: *");
              client.println("Connection: close");
              client.println();

              serializeJson(Dados, client);

              readString = "";
              break;
            }
          }

          //-------------------------------------------- REALIZA MUDANÇA NUMERO OP --------------------------------------------
          if (readString.indexOf("/OP=") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("OP", 3); // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("OP") + 2;

            if (readString.substring(Posicao1, Posicao2) == "OP")
            {
              int long Posicao3 = readString.indexOf("OP=") + 3;   // pega a posição do = antes do valor
              int long Posicao4 = readString.indexOf("HTTP/") - 1; // pega a posição do f depois do valor
              cOP = readString.substring(Posicao3, Posicao4);
              Serial.println("Numero da OP:");
              Serial.println(cOP);

              LeituraMaquina();

              StaticJsonDocument<200> Dados;
              Dados["OP"] = cOP;
              Dados["Status"] = "OK";

              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: application/json;charset=utf-8");
              client.println("Access-Control-Allow-Origin: *");
              client.println("Connection: close");
              client.println();

              serializeJson(Dados, client);

              Guarda_Dados();
              readString = "";
              break;
            }
          }

          //-------------------------------------------- PARA TEAR --------------------------------------------
          if (readString.indexOf("/PARA") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("PARA", 3);  // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("PARA") + 4; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "PARA")
            {
              digitalWrite(48, HIGH);
              Serial.println("TEAR PARA");

              readString = "";
              break;
            }
          }

          //-------------------------------------------- DELETA ARQUIVO DE LISTAGEM PARA TRAZER LISTAGEM ATUAL --------------------------------------------
          if (readString.indexOf("/DEL") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("DEL", 3);  // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("DEL") + 3; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "DEL")
            {
              StaticJsonDocument<200> Dados;

              if (SD.exists("DataRead.txt"))
              {
                SD.remove("DataRead.txt");
                Serial.println("Removendo DataRead.txt...");
                Dados["REMOVIDO"] = "SIM";
              }
              else
              {
                Dados["REMOVIDO"] = "NAO";
              }

              Dados["Status"] = "OK";

              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: application/json;charset=utf-8");
              client.println("Access-Control-Allow-Origin: *");
              client.println("Connection: close");
              client.println();

              serializeJson(Dados, client);

              readString = "";
              break;
            }
          }

          //-------------------------------------------- REALIZA MUDANÇA MAC --------------------------------------------
          if (readString.indexOf("/MAC=") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("MAC", 4);  // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("MAC") + 3; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "MAC")
            {
              // Comando que muda o IP da placa, ao usar ele corretamente, a placa reinicia com o novo IP
              noInterrupts();
              cMAC = readString.substring(readString.indexOf("MAC=") + 4, readString.indexOf("HTTP/1.1") - 1);
              cMAC.trim(); // remove espaços
              Serial.println("Novo MAC:");
              Serial.println(cMAC);

              // Começa a gravar o novo IP pra quando for reiniciado poder ter a informação dele
              SD.remove("MAC.txt");
              delay(1000);

              ipTXT = SD.open("MAC.txt", FILE_WRITE);
              if (ipTXT)
              {
                int a = -1;
                ipTXT.print(cMAC.substring(a + 1));
                cMAC.setCharAt(a, ' ');

                ipTXT.close();

                StaticJsonDocument<200> Dados;
                Dados["MAC"] = cMAC;
                Dados["Status"] = "OK";

                client.println("HTTP/1.1 200 OK");
                client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
                client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
                client.println("Content-Type: application/json;charset=utf-8");
                client.println("Access-Control-Allow-Origin: *");
                client.println("Connection: close");
                client.println();

                serializeJson(Dados, client);
                readString = "";

                delay(3000);         // Espera 3 segundos para que a placa tenha tempo de lidar com buffer
                reiniciarRobocore(); // Reinicia a placa

                break;
              }
              else
              {
                Serial.print("Não foi possível abrir MACTXT");
              }
              break;
            }
          }

          //-------------------------------------------- REALIZA MUDANÇA IP --------------------------------------------
          if (readString.indexOf("/IP=") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("IP", 3);  // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("IP") + 2; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "IP")
            {
              // Comando que muda o IP da placa, ao usar ele corretamente, a placa reinicia com o novo IP
              noInterrupts();
              cIP = readString.substring(readString.indexOf("IP=") + 3, readString.indexOf("HTTP/1.1") - 1);
              cIP.trim(); // remove espaços
              Serial.println("Novo IP:");
              Serial.println(cIP);

              // Começa a gravar o novo IP pra quando for reiniciado poder ter a informação dele
              SD.remove("ip.txt");
              delay(1000);

              ipTXT = SD.open("ip.txt", FILE_WRITE);
              if (ipTXT)
              {
                int a = -1;
                ipTXT.print(cIP.substring(a + 1));
                cIP.setCharAt(a, ' ');

                ipTXT.close();

                StaticJsonDocument<200> Dados;
                Dados["IP"] = cIP;
                Dados["Status"] = "OK";

                client.println("HTTP/1.1 200 OK");
                client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
                client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
                client.println("Content-Type: application/json;charset=utf-8");
                client.println("Access-Control-Allow-Origin: *");
                client.println("Connection: close");
                client.println();

                serializeJson(Dados, client);
                readString = "";

                delay(3000);         // Espera 3 segundos para que a placa tenha tempo de lidar com buffer
                reiniciarRobocore(); // Reinicia a placa

                break;
              }
              else
              {
                Serial.print("Não foi possível abrir ipTXT");
              }
              break;
            }
          }

          //-------------------------------------------- REALIZA MUDANÇA SUBNET --------------------------------------------
          if (readString.indexOf("  ") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("SUBNET", 3);  // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("SUBNET") + 6; // pega a posição do fim depois do valor

            if (readString.substring(Posicao1, Posicao2) == "SUBNET")
            {
              // Comando que muda o IP da placa, ao usar ele corretamente, a placa reinicia com o novo IP
              noInterrupts();
              cSubnet = readString.substring(readString.indexOf("SUBNET=") + 7, readString.indexOf("HTTP/1.1") - 1);
              cSubnet.trim(); // remove espaços
              Serial.println("Novo SUBNET:");
              Serial.println(cSubnet);

              // Começa a gravar o novo IP pra quando for reiniciado poder ter a informação dele
              SD.remove("SUBNET.txt");
              delay(1000);

              SubnetTXT = SD.open("SUBNET.txt", FILE_WRITE);
              if (SubnetTXT)
              {
                int a = -1;

                SubnetTXT.print(cSubnet.substring(a + 1));
                cSubnet.setCharAt(a, ' ');

                SubnetTXT.close();

                StaticJsonDocument<200> Dados;
                Dados["SUBNET"] = cSubnet;
                Dados["Status"] = "OK";

                client.println("HTTP/1.1 200 OK");
                client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
                client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
                client.println("Content-Type: application/json;charset=utf-8");
                client.println("Access-Control-Allow-Origin: *");
                client.println("Connection: close");
                client.println();

                serializeJson(Dados, client);
                readString = "";

                delay(3000);         // Espera 3 segundos para que a placa tenha tempo de lidar com buffer
                reiniciarRobocore(); // Reinicia a placa
                break;
              }
              else
              {
                Serial.print("Não foi possível abrir SubnetTXT");
              }
              break;
            }
          }

          //-------------------------------------------- REALIZA MUDANÇA GATEWAY --------------------------------------------
          if (readString.indexOf("/GATEWAY=") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("GATEWAY", 8);  // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("GATEWAY") + 7; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "GATEWAY")
            {
              // Comando que muda o IP da placa, ao usar ele corretamente, a placa reinicia com o novo IP
              noInterrupts();
              cGateway = readString.substring(readString.indexOf("GATEWAY=") + 8, readString.indexOf("HTTP/1.1") - 1);
              cGateway.trim(); // remove espaços
              Serial.println("Novo GATEWAY:");
              Serial.println(cGateway);

              // Começa a gravar o novo IP pra quando for reiniciado poder ter a informação dele
              SD.remove("GATEWAY.txt");
              delay(1000);

              GatewayTXT = SD.open("GATEWAY.txt", FILE_WRITE);
              if (GatewayTXT)
              {
                int a = -1;
                GatewayTXT.print(cGateway.substring(a + 1));
                cGateway.setCharAt(a, ' ');

                GatewayTXT.close();

                StaticJsonDocument<200> Dados;

                Dados["SUBNET"] = cGateway;
                Dados["Status"] = "OK";

                client.println("HTTP/1.1 200 OK");
                client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
                client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
                client.println("Content-Type: application/json;charset=utf-8");
                client.println("Access-Control-Allow-Origin: *");
                client.println("Connection: close");
                client.println();

                serializeJson(Dados, client);

                readString = "";
                delay(3000);         // Espera 3 segundos para que a placa tenha tempo de lidar com buffer
                reiniciarRobocore(); // Reinicia a placa

                break;
              }
              else
              {
                Serial.print("Não foi possível abrir GatewayTXT");
              }
              break;
            }
          }

          //-------------------------------------------- REALIZA MUDANÇA DNS --------------------------------------------
          if (readString.indexOf("/DNS=") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("DNS", 4);  // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("DNS") + 3; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "DNS")
            {
              // Comando que muda o IP da placa, ao usar ele corretamente, a placa reinicia com o novo IP
              noInterrupts();
              cDNS = readString.substring(readString.indexOf("DNS=") + 4, readString.indexOf("HTTP/1.1") - 1);
              cDNS.trim(); // remove espaços
              Serial.println("Novo DNS:");
              Serial.println(cDNS);

              // Começa a gravar o novo IP pra quando for reiniciado poder ter a informação dele
              SD.remove("DNS.txt");
              delay(1000);

              DNSTXT = SD.open("DNS.txt", FILE_WRITE);
              if (DNSTXT)
              {
                int a = -1;
                DNSTXT.print(cDNS.substring(a + 1));
                cDNS.setCharAt(a, ' ');

                DNSTXT.close();

                StaticJsonDocument<200> Dados;

                Dados["GATEWAY"] = cDNS;
                Dados["Status"] = "OK";

                client.println("HTTP/1.1 200 OK");
                client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
                client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
                client.println("Content-Type: application/json;charset=utf-8");
                client.println("Access-Control-Allow-Origin: *");
                client.println("Connection: close");
                client.println();

                serializeJson(Dados, client);

                readString = "";
                delay(3000);         // Espera 3 segundos para que a placa tenha tempo de lidar com buffer
                reiniciarRobocore(); // Reinicia a placa

                break;
              }
              else
              {
                Serial.print("Não foi possível abrir DNSTXT");
              }
              break;
            }
          }

          //-------------------------------------------- AJUSTA TURNO A --------------------------------------------
          if (readString.indexOf("/TurnoAInicio") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("TurnoAInicio", 3);   // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("TurnoAInicio") + 12; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "TurnoAInicio")
            {
              String cAjustaTurno = readString.substring(readString.indexOf("TurnoAInicio=") + 13, readString.indexOf("HTTP/1.1") - 1);

              String cHora = cAjustaTurno.substring(0, 2);
              String cMin = cAjustaTurno.substring(2, 4);
              String cSec = cAjustaTurno.substring(4, 6);

              Serial.println("");
              Serial.println("Turno A Inicio Ajustado");
              Serial.println(cHora + ":" + cMin + ":" + cSec);

              Serial.println("");

              cTurnoAInicio = cHora + ":" + cMin + ":" + cSec;

              StaticJsonDocument<200> Dados;

              Dados["Turno"] = "A Inicio";
              Dados["HORA"] = cHora + ":" + cMin + ":" + cSec;
              Dados["Status"] = "OK";

              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: application/json;charset=utf-8");
              client.println("Access-Control-Allow-Origin: *");
              client.println("Connection: close");
              client.println();

              serializeJson(Dados, client);

              Guarda_Dados();
              readString = "";
              break;
            }
          }

          //-------------------------------------------- AJUSTA TURNO B --------------------------------------------
          if (readString.indexOf("/TurnoBInicio") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("TurnoBInicio", 3);   // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("TurnoBInicio") + 12; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "TurnoBInicio")
            {
              String cAjustaTurno = readString.substring(readString.indexOf("TurnoB Inicio=") + 13, readString.indexOf("HTTP/1.1") - 1);

              String cHora = cAjustaTurno.substring(0, 2);
              String cMin = cAjustaTurno.substring(2, 4);
              String cSec = cAjustaTurno.substring(4, 6);

              Serial.println("");
              Serial.println("Turno B Inicio Ajustado");
              Serial.println(cHora + ":" + cMin + ":" + cSec);

              Serial.println("");

              cTurnoBInicio = cHora + ":" + cMin + ":" + cSec;

              StaticJsonDocument<200> Dados;

              Dados["Turno"] = "B Inicio";
              Dados["HORA"] = cHora + ":" + cMin + ":" + cSec;
              Dados["Status"] = "OK";

              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: application/json;charset=utf-8");
              client.println("Access-Control-Allow-Origin: *");
              client.println("Connection: close");
              client.println();

              serializeJson(Dados, client);

              Guarda_Dados();
              readString = "";
              break;
            }
          }

          //-------------------------------------------- AJUSTA TURNO C --------------------------------------------
          if (readString.indexOf("/TurnoCInicio") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("TurnoCInicio", 3);   // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("TurnoCInicio") + 12; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "TurnoCInicio")
            {
              String cAjustaTurno = readString.substring(readString.indexOf("TurnoCInicio=") + 13, readString.indexOf("HTTP/1.1") - 1);

              String cHora = cAjustaTurno.substring(0, 2);
              String cMin = cAjustaTurno.substring(2, 4);
              String cSec = cAjustaTurno.substring(4, 6);

              Serial.println("");
              Serial.println("Turno C Inicio Ajustado");
              Serial.println(cHora + ":" + cMin + ":" + cSec);

              Serial.println("");

              cTurnoCInicio = cHora + ":" + cMin + ":" + cSec;

              StaticJsonDocument<200> Dados;

              Dados["Turno"] = "C Inicio";
              Dados["HORA"] = cHora + ":" + cMin + ":" + cSec;
              Dados["Status"] = "OK";

              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: application/json;charset=utf-8");
              client.println("Access-Control-Allow-Origin: *");
              client.println("Connection: close");
              client.println();

              serializeJson(Dados, client);

              Guarda_Dados();
              readString = "";
              break;
            }
          }

          if (readString.indexOf("/TurnoAFim") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("TurnoAFim", 3);  // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("TurnoAFim") + 9; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "TurnoAFim")
            {
              String cAjustaTurno = readString.substring(readString.indexOf("TurnoAFim=") + 10, readString.indexOf("HTTP/1.1") - 1);

              String cHora = cAjustaTurno.substring(0, 2);
              String cMin = cAjustaTurno.substring(2, 4);
              String cSec = cAjustaTurno.substring(4, 6);

              Serial.println("");
              Serial.println("Turno A Fim Ajustado");
              Serial.println(cHora + ":" + cMin + ":" + cSec);

              Serial.println("");

              cTurnoAFim = cHora + ":" + cMin + ":" + cSec;

              StaticJsonDocument<200> Dados;

              Dados["Turno"] = "A Fim";
              Dados["HORA"] = cHora + ":" + cMin + ":" + cSec;
              Dados["Status"] = "OK";

              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: application/json;charset=utf-8");
              client.println("Access-Control-Allow-Origin: *");
              client.println("Connection: close");
              client.println();

              serializeJson(Dados, client);

              Guarda_Dados();
              readString = "";
              break;
            }
          }

          if (readString.indexOf("/TurnoBFim") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("TurnoBFim", 3);  // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("TurnoBFim") + 9; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "TurnoBFim")
            {
              String cAjustaTurno = readString.substring(readString.indexOf("TurnoBFim=") + 10, readString.indexOf("HTTP/1.1") - 1);

              String cHora = cAjustaTurno.substring(0, 2);
              String cMin = cAjustaTurno.substring(2, 4);
              String cSec = cAjustaTurno.substring(4, 6);

              Serial.println("");
              Serial.println("Turno B Fim Ajustado");
              Serial.println(cHora + ":" + cMin + ":" + cSec);

              Serial.println("");

              cTurnoBFim = cHora + ":" + cMin + ":" + cSec;

              StaticJsonDocument<200> Dados;

              Dados["Turno"] = "B Fim";
              Dados["HORA"] = cHora + ":" + cMin + ":" + cSec;
              Dados["Status"] = "OK";

              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: application/json;charset=utf-8");
              client.println("Access-Control-Allow-Origin: *");
              client.println("Connection: close");
              client.println();

              serializeJson(Dados, client);

              Guarda_Dados();
              readString = "";
              break;
            }
          }

          if (readString.indexOf("/TurnoCFim") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("TurnoCFim", 3);  // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("TurnoCFim") + 9; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "TurnoCFim")
            {
              String cAjustaTurno = readString.substring(readString.indexOf("TurnoCFim=") + 10, readString.indexOf("HTTP/1.1") - 1);

              String cHora = cAjustaTurno.substring(0, 2);
              String cMin = cAjustaTurno.substring(2, 4);
              String cSec = cAjustaTurno.substring(4, 6);

              Serial.println("");
              Serial.println("Turno C Fim Ajustado");
              Serial.println(cHora + ":" + cMin + ":" + cSec);

              Serial.println("");

              cTurnoCFim = cHora + ":" + cMin + ":" + cSec;

              StaticJsonDocument<200> Dados;

              Dados["Turno"] = "C Fim";
              Dados["HORA"] = cHora + ":" + cMin + ":" + cSec;
              Dados["Status"] = "OK";

              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: application/json;charset=utf-8");
              client.println("Access-Control-Allow-Origin: *");
              client.println("Connection: close");
              client.println();

              serializeJson(Dados, client);

              Guarda_Dados();
              readString = "";
              break;
            }
          }

          if (readString.indexOf("/ListaTurno") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("ListaTurno", 3);   // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("ListaTurno") + 10; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "ListaTurno")
            {

              StaticJsonDocument<200> Dados;

              Dados["Turno A"] = cTurnoAInicio + " - " + cTurnoAFim;
              Dados["Turno B"] = cTurnoBInicio + " - " + cTurnoBFim;
              Dados["Turno C"] = cTurnoCInicio + " - " + cTurnoCFim;

              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: application/json;charset=utf-8");
              client.println("Access-Control-Allow-Origin: *");
              client.println("Connection: close");
              client.println();

              serializeJson(Dados, client);

              readString = "";
              break;
            }
          }

          if (readString.indexOf("/UPDATE") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("UPDATE", 3);  // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("UPDATE") + 6; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "UPDATE")
            {
              Posicao1 = readString.indexOf("UPDATE/", 4) + 7;
              Posicao2 = readString.indexOf("UPDATE/")    + 13;

              nTam1 = 0;
              nTam2 = readString.substring(Posicao1, Posicao2).toInt();
        
              _lOK = false;
              _nCodError = 0;
              
              Guarda_Dados();

              handleSketchDownload();

              StaticJsonDocument<200> Dados;

              const char *PATH = "update-v%d.bin";  
              char buff[32];
              snprintf(buff, sizeof(buff), PATH, VERSION + 1);

              if(_lOK){
                Dados["UPDATE"]  = "OK";
                Dados["ARQUIVO"] = buff;
                Dados["VERSAO"]  = "2.9." + String(VERSION+1);
              } else {
                // _nCodError = -2  - Servidor não encontrado
                // _nCodError = 98  - Erro ao Abrir arquivo
                // _nCodError = 99  - Erro ao Carregar arquivo
                // _nCodError = 100 - Erro no Download

                Dados["UPDATE"]   = _nCodError; 
                Dados["ARQUIVO"]  = buff;
                Dados["VERSAO"]   = "2.9." + String(VERSION);        
                Dados["DOWNLOAD"] = String(nTam1);
                Dados["TAMANHO"]  = String(nTam2);
              }

              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: application/json;charset=utf-8");
              client.println("Access-Control-Allow-Origin: *");
              client.println("Connection: close");
              client.println();

              serializeJson(Dados, client);

              if(_lOK){
                EEPROM.write(0x1FF, 0xF0);
                delay(1000);
                
                client.stop(); // termina a conexão
                
                #ifdef __AVR__
                  delay(1000);
                  
                  wdt_enable(WDTO_15MS);
                  while (true);
                #else
                  NVIC_SystemReset();
                #endif
              }

              readString = "";
              break;
            }
          }

          //-------------------------------------------- PROGRAMAÇÂO DE TEARES --------------------------------------------
          if (readString.indexOf("/CARREGAOP1") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("CARREGAOP1", 3);   // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("CARREGAOP1") + 10; // pega a posição do fim depois do valor

            unsigned long Posicao3;
            String nTotal;
            String cLinha;

            if (readString.substring(Posicao1, Posicao2) == "CARREGAOP1")
            {
              // Busca OP
              cLinha = readString.substring(Posicao2 + 1, 255);

              unsigned long Posicao3 = cLinha.indexOf("/");
              cOP = cLinha.substring(0, Posicao3);

              // Busca Batida OP
              cLinha = cLinha.substring(Posicao3 + 1, 255);
              Posicao3 = cLinha.indexOf("/");

              nTotal = cLinha.substring(0, Posicao3);
              nTotal.trim();

              if (verificaChar(&nTotal))
              {
                // Verifica se tem algum caractere não numérico inserido pelo usuário
                nFimProducao = nTotal.toInt();
              }

              // Busca Batida Bobinas
              cLinha = cLinha.substring(Posicao3 + 1, 255);
              Posicao3 = cLinha.indexOf(" ");

              nTotal = cLinha.substring(0, Posicao3);
              nTotal.trim();

              if (verificaChar(&nTotal))
              {
                // Verifica se tem algum caractere não numérico inserido pelo usuário
                nFimBobina = nTotal.toInt();
              }

              nBatidasBobina = 0;
              nBatidasOP = 0;

              delay(100);

              StaticJsonDocument<200> Dados;

              Dados["OP"]             = cOP;
              Dados["nFimProducao"]   = nFimProducao;
              Dados["nFimBobina"]     = nFimBobina;
              Dados["nBatidasBobina"] = 0;
              Dados["nBatidasOP"]     = 0;
              Dados["Status"]         = "OK";

              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: application/json;charset=utf-8");
              client.println("Access-Control-Allow-Origin: *");
              client.println("Connection: close");
              client.println();

              serializeJson(Dados, client);

              Guarda_Dados();
              readString = "";
              break;
            }
          }

          if (readString.indexOf("/CARREGAOP2") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("CARREGAOP2", 3);   // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("CARREGAOP2") + 10; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "CARREGAOP2")
            {
              String cPath = "/CARREGAOP2/";
              int statusCode;
              String response;
              String nTotal;
              String cLinha;

              EthernetClient c;
              HttpClient http(c, "192.168.20.7", 3000);
              http.setTimeout(5000);

              for (int i = 0; i < 4; i++)
              {
                cPath += aIP[i];
                if (i != 3)
                {
                  cPath += ".";
                }
              }

              // Serial.println(cPath);
              http.get(cPath);

              delay(500);

              // read the status code and body of the response
              statusCode = http.responseStatusCode();
              // Serial.println(statusCode);
              response = http.responseBody();
              // Serial.println(response);

              Serial.println();
              http.stop();

              StaticJsonDocument<200> Dados;

              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: application/json;charset=utf-8");
              client.println("Access-Control-Allow-Origin: *");
              client.println("Connection: close");
              client.println();

              if (statusCode == 200)
              {
                if (response == "{ Status: PN }")
                {
                  Dados["Status"] = "NP";
                }
                else
                {
                  // Busca OP
                  cLinha = response.substring(0, 255);

                  unsigned long Posicao3 = cLinha.indexOf("/");
                  cOP = cLinha.substring(0, Posicao3);

                  // Busca Batida OP
                  cLinha = cLinha.substring(Posicao3 + 1, 255);
                  Posicao3 = cLinha.indexOf("/");

                  nTotal = cLinha.substring(0, Posicao3);
                  nTotal.trim();

                  if (verificaChar(&nTotal))
                  {
                    // Verifica se tem algum caractere não numérico inserido pelo usuário
                    nFimProducao = nTotal.toInt();
                  }

                  // Busca Batida Bobinas
                  cLinha = cLinha.substring(Posicao3 + 1, 255);
                  Posicao3 = cLinha.indexOf(" ");

                  nTotal = cLinha.substring(0, Posicao3);
                  nTotal.trim();

                  if (verificaChar(&nTotal))
                  {
                    // Verifica se tem algum caractere não numérico inserido pelo usuário
                    nFimBobina = nTotal.toInt();
                  }

                  nBatidasBobina = 0;
                  nBatidasOP = 0;

                  delay(100);

                  Dados["OP"]             = cOP;
                  Dados["nFimProducao"]   = nFimProducao;
                  Dados["nFimBobina"]     = nFimBobina;
                  Dados["nBatidasBobina"] = 0;
                  Dados["nBatidasOP"]     = 0;
                  Dados["Status"]         = "OK";
                }
              }
              else
              {
                Dados["Status"] = "ERRO";
              }

              serializeJson(Dados, client);

              Guarda_Dados();
              readString = "";
              break;
            }
          }
          //-------------------------------------------------------------------------------------------------------------

          //-------------------------------------------- ZERA VALOS TEAR --------------------------------------------
          if (readString.indexOf("/ZERAR") > 0)
          {                                                           // Zera os valores abaixo
            unsigned long Posicao1 = readString.indexOf("ZERAR", 3);  // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("ZERAR") + 5; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "ZERAR")
            {

              stringnFimProducao = "";
              nFimProducao = 0;
              nFimBobina = 0;
              cOP = "00000000000";
              nBatidasOP = 0;
              nBatidasBobina = 0;
              nBatidasMes = 0;

              LeituraMaquina();

              StaticJsonDocument<200> Dados;

              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: application/json;charset=utf-8");
              client.println("Access-Control-Allow-Origin: *");
              client.println("Connection: close");
              client.println();

              Dados["Status"] = "OK";

              serializeJson(Dados, client);

              Guarda_Dados();
              readString = "";
              break;
            }
          }

          // VERSÂO
          if (readString.indexOf("/VERSAO") > 0)
          {                                                            // Zera os valores abaixo
            unsigned long Posicao1 = readString.indexOf("VERSAO", 3);  // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("VERSAO") + 6; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "VERSAO")
            {
              client.println();
              client.println("    #######################################");
              client.println("    |                                     |");
              client.println("    |       -----------------------       |");
              client.println("    |       TEAR_VERSAO_FULL_2.9." + String(VERSION) + "       |");
              client.println("    |       -----------------------       |");
              client.println("    |                                     |");
              client.println("    |               Autor:                |");
              client.println("    |                                     |");
              client.println("    |         - Pericles Silva            |");
              client.println("    |                                     |");
              client.println("    |                                     |");
              client.println("    |                                     |");
              client.println("    |    Procopio Ind. e Comercio LTDA.   |");
              client.println("    |                                     |");
              client.println("    |                                     |");
              client.println("    #######################################");

              readString = "";
              break;
            }
          }

          if (readString.indexOf("/MAQUINA") > 0)
          {
            unsigned long Posicao1 = readString.indexOf("MAQUINA", 3);   // pega a posição do = antes do valor
            unsigned long Posicao2 = readString.indexOf("MAQUINA") + 7; // pega a posição do fim depois do valor
            if (readString.substring(Posicao1, Posicao2) == "MAQUINA")
            {

              cMAQUINA = readString.substring(readString.indexOf("MAQUINA=") + 8, readString.indexOf("HTTP/1.1") - 1);

              Serial.println("Nome Maquina:");
              Serial.println(cMAQUINA);
              Serial.println();

              StaticJsonDocument<200> Dados;
              
              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: application/json;charset=utf-8");
              client.println("Access-Control-Allow-Origin: *");
              client.println("Connection: close");
              client.println();

              Dados["MAQUINA"] = cMAQUINA;
              Dados["Status"]  = "OK";

              serializeJson(Dados, client);

              Guarda_Dados();
              readString = "";
              break;
            }
          }


          // Caso nao entre em nehuma anterior mostra dados
          CarregaDataHora();
          delay(1);

          StaticJsonDocument<200> Dados;

          String cDia = "";
          String cMes = "";
          String cHora = "";
          String cMin = "";
          String cSec = "";

          Dados["MAQUINA"]        = cMAQUINA;
          Dados["DATA"]           = cDataOld;
          Dados["HORA"]           = cHorario;
          Dados["FALHA"]          = nFalhas;
          Dados["RPM"]            = nRPM;
          Dados["BATIDAS"]        = nBatidas;
          Dados["BATIDAS_MES"]    = nBatidasMes;
          Dados["BATIDAS_OP"]     = nBatidasOP;
          Dados["BATIDAS_BOBINA"] = nBatidasBobina;
          Dados["OP"]             = cOP;
          Dados["FimProducao"]    = nFimProducao;
          Dados["FimBobina"]      = nFimBobina;
          Dados["Leitura"]        = "A";
          Dados["ID_FALHA"]       = nIdFalha;          

          // Verifica status da Producao
          if (nBatidasOP >= nFimProducao)
          {
            Dados["Status"] = "FimProducao";
          }
          else
          {
            Dados["Status"] = "Produzindo";
          }

          client.println("HTTP/1.1 200 OK");
          client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
          client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
          client.println("Content-Type: application/json;charset=utf-8");
          client.println("Access-Control-Allow-Origin: *");
          client.println("Connection: close");
          client.println();

          serializeJson(Dados, client);

          if (readString.indexOf("/RESET") > 0)
          {
            nBatidasOP = 0;
            reiniciarRobocore();
          }

          if (readString.indexOf("/apaga_mes") > 0)
          {
            nBatidasMes = 0;
            readString = "";
          }

          readString = "";
          break;
        }

        // toda linha de texto recebida do cliente termina com os caracteres \r\n
        if (cLinha == '\n')
        {
          // ultimo caractere da linha do texto recebido
          // iniciando nova linha com o novo caractere lido
          currentLineIsBlank = true;
        }
        else if (cLinha != '\r')
        {
          // um caractere de texto foi recebido do cliente
          currentLineIsBlank = false;
        }
      } // fim do if (client.available())
    }   // fim do while (client.connected())

    delay(1);      // da um tempo para o WEB Browser receber o texto
    client.stop(); // termina a conexão
  } else {
    //Serial.println("teste");
  } // fim do if (client)
} // fim do loop
