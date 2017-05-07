/**
   Bibliotecas utilizadas
   https://github.com/Seeed-Studio/CAN_BUS_Shield
   https://github.com/tiagopossato/ctmNectar
*/

#include <avr/wdt.h>
#include <EEPROM.h>
#include "SPI.h"
#include "mcp_can.h"
#include "ctmNectar.h"
#include <Wire.h>
#include "ClosedCube_HDC1080.h"
#include "definicoes.h"

#define DEBUG
#define CENTRAL_ID 0x00

const int SPI_CS_PIN = 10;
//Cria objeto para manipular o controlador CAN
MCP_CAN CAN(SPI_CS_PIN);// Set CS pin

//Cria objeto para manipular o sensor i2c
ClosedCube_HDC1080 hdc1080;


//Estrutura com as sensorConfigurações do módulo
struct {
  uint8_t endereco;
  uint16_t intervaloEnvio; //Intervalo entre o envio das leituras, em segundos
  uint16_t intervaloLeitura; //Intervalo entre as leituras, em segundos
} sensorConfig;

//estrutura com os dados analogicos do sensor
struct {
  float temperaturaAr;
  float umidadeAr;
  float umidadeSolo;
} dados;

int8_t entradasDigitais[8];

// Contadores
uint32_t msUltimoEnvio = 0;
uint32_t msUltimaLeitura = 0;

/**
   Função para resetar o programa
   Utiliza o WatchDog Timer
*/
void reiniciar() {
#if defined(DEBUG)
  Serial.println("Reiniciando...");
#endif
  wdt_enable(WDTO_15MS);
  while (1);
}

// ----------------------
// Setup() & Loop()
// ----------------------

void setup()
{
#if defined(DEBUG)
  Serial.begin(115200);
  Serial.println(F(""));
  Serial.println(F("----------------------------------------"));
#endif
  leSensorConfig();

  if (sensorConfig.intervaloLeitura < 2 || sensorConfig.intervaloLeitura > 3600 ) {
    sensorConfig.intervaloLeitura = 2;
    salvarSensorConfig();
  }
  if (sensorConfig.intervaloEnvio <= 0 || sensorConfig.intervaloEnvio > 3600 ) {
    sensorConfig.intervaloEnvio = 1;
    salvarSensorConfig();
  }
  if (sensorConfig.endereco == 0) {
    sensorConfig.endereco = 1;
    salvarSensorConfig();
  }

  /*INTERVALO FIXO PARA FINS DE SIMULAÇÃO*/
  sensorConfig.intervaloLeitura = 2;
  sensorConfig.intervaloEnvio = 1000;

  if (CAN_OK != CAN.begin(CAN_100KBPS))              // init can bus : baudrate = 100k
  {
#if defined(DEBUG)
    Serial.println(F("CAN BUS Shield init fail"));
    Serial.println(F(" Init CAN BUS Shield again"));
#endif
    delay(250);
    reiniciar();
  }

  hdc1080.begin(0x40);

#if defined(DEBUG)
  Serial.println(F("Remota: CAN BUS init ok!"));
#endif

  unsigned char msgCfg[3] = {especial, 1, 1};
  CAN.sendMsgBuf(sensorConfig.endereco, 0, sizeof(msgCfg), msgCfg);

  //Inicializa o Watchdog
  wdt_enable(WDTO_250MS);
}

void loop() {
  //-------INICIO DA COMUNICAÇÃO------------
  // Verifica se recebeu uma mensagem
  if (!pacoteRecebido()) {
    // Se não recebeu, verifica se está na hora de enviar os dados
    if (millis() > msUltimoEnvio + (sensorConfig.intervaloEnvio * 1000)) {
      /*SIMULAÇÃO*/
      randomSeed(analogRead(A3));
      sensorConfig.endereco = (uint8_t) random(1, 6);
       msUltimoEnvio = millis();
      /*FIM DA SIMULAÇÃO*/
      lerDados();
      //enviarDados();
      //msUltimoEnvio atualizado na funcao enviarDados, para incluir nas contagens quando os dados são enviados por solicitação do mestre e não por tempo
    }
  }
  //-------FIM DA COMUNICAÇÃO------------
  if (millis() > msUltimaLeitura + (sensorConfig.intervaloLeitura * 1000)) {
    lerDados();
    msUltimaLeitura = millis();
    if (CAN.checkError() != 0) {
      Serial.print("Erro na CAN: ");
      Serial.println(CAN.checkError());
      reiniciar();
    }
  }
  wdt_reset();  //  reseta o watchdog
}

/**
   Le dados dos sensores
*/
void lerDados() {
  boolean flagEnviar = false;
  float tmp = 0;

  /*LE TEMPERATURA */
  tmp = hdc1080.readTemperature();
  if (abs(tmp - dados.temperaturaAr) > 0.5) flagEnviar = true;
  dados.temperaturaAr = tmp;

  /*LE UMIDADE DO AR*/
  tmp = hdc1080.readHumidity();
  if (abs(tmp - dados.umidadeAr) > 0.5) flagEnviar = true;
  dados.umidadeAr = tmp;

  /*SIMULA UMIDADE DO SOLO*/
  dados.umidadeSolo = (float) (random(-600, 300)/10.0);

  /*SIMULA ENTRADAS DIGITAIS*/
//  for (char i = 0; i < 8; i++) {
//    entradasDigitais[i] = (uint8_t)random(0, 2);
//  }
  /*FIM DA SIMULAÇÃO*/

#if defined(DEBUG)
  Serial.print("T=");
  Serial.print(dados.temperaturaAr);
  Serial.print("C, RH=");
  Serial.print(dados.umidadeAr);
  Serial.print("%uR, Solo=");
  Serial.print(dados.umidadeSolo);
  Serial.println("%");
#endif
  //if (flagEnviar) enviarDados();

}


/**
   Le os sensorConfiguracoes salvas na meméria EEPROM
*/
void leSensorConfig() {
  EEPROM.get(0, sensorConfig);
#if defined(DEBUG)
  Serial.print(F("endereco="));
  Serial.println(sensorConfig.endereco);
  Serial.print(F("intervaloLeitura="));
  Serial.println(sensorConfig.intervaloLeitura);
  Serial.print(F("intervaloEnvio="));
  Serial.println(sensorConfig.intervaloEnvio);
#endif
}

/**
   Escreve na memória EEPROM a configuração do sensor
*/
void salvarSensorConfig() {
  EEPROM.put(0, sensorConfig);
  leSensorConfig();
}

/**
   Envia configurações para a central
*/
void enviarConfig() {
  unsigned char msgCfg[2] = {0};
  //Envia intervalo entre as leituras
  msgCfg[0] = SEND_READ_TIME;
  msgCfg[1] = sensorConfig.intervaloLeitura;
  CAN.sendMsgBuf(sensorConfig.endereco, 0, sizeof(msgCfg), msgCfg);
  delay(10);

  //Envia intervalo de envio de dados para a central
  msgCfg[0] = SEND_TIME;
  msgCfg[1] = sensorConfig.intervaloEnvio;
  CAN.sendMsgBuf(sensorConfig.endereco, 0, sizeof(msgCfg), msgCfg);
}

/**
   Envia dados para a central
*/
void enviarDados() {
  enviarTemperatura();
  delay(10);
  enviarUmidadeAr();
  delay(10);
  enviarUmidadeSolo();

  //envia entradas digitais
  for (char i = 0; i < 8; i++) {
    delay(10);
    lerEntradaDigital(i);
  }
  msUltimoEnvio = millis();
}

void enviarTemperatura() {
  int16_t tmp = 0;
  unsigned char msg[4] = {0};

  msg[0] = (uint8_t)entradaAnalogica;
  msg[1] = (uint8_t)temperatura;
  tmp = (int16_t)(dados.temperaturaAr * 100.0);
  msg[2] = highByte(tmp);
  msg[3] = lowByte(tmp);

  CAN.sendMsgBuf(sensorConfig.endereco, 0, sizeof(msg), msg);

}

void enviarUmidadeAr() {
  int16_t tmp = 0;
  unsigned char msg[4] = {0};

  msg[0] = (uint8_t)entradaAnalogica;
  msg[1] = (uint8_t)umidadeAr;
  tmp = (int16_t)(dados.umidadeAr * 100.0);
  msg[2] = highByte(tmp);
  msg[3] = lowByte(tmp);

  CAN.sendMsgBuf(sensorConfig.endereco, 0, sizeof(msg), msg);
}

void enviarUmidadeSolo() {
  int16_t tmp = 0;
  unsigned char msg[4] = {0};

  msg[0] = (uint8_t)entradaAnalogica;
  msg[1] = (uint8_t)umidadeSolo;
  tmp = (int16_t)(dados.umidadeSolo * 100.0);
  msg[2] = highByte(tmp);
  msg[3] = lowByte(tmp);

  CAN.sendMsgBuf(sensorConfig.endereco, 0, sizeof(msg), msg);
}


void lerEntradaDigital(int8_t entrada) {
  Serial.println(entrada);
  if (entrada >= 0 && entrada < 8) {
    unsigned char msg[4] = {0};

    msg[0] = (uint8_t)entradaDigital;
    msg[1] = (uint8_t)entrada;
    /*SIMULAÇÃO*/
    msg[2] = entradasDigitais[entrada];
    CAN.sendMsgBuf(sensorConfig.endereco, 0, sizeof(msg), msg);
  }
}

void escreveSaidaDigital(uint8_t saida, int16_t valor) {
#if defined(DEBUG)
  if (valor == 1){
    Serial.print(F("Ligando pino: "));
    entradasDigitais[saida] = true;
  }
  else{
    entradasDigitais[saida] = false;
    Serial.print(F("Desligando pino: "));
  }
  Serial.println(saida);
#endif
}

bool pacoteRecebido() {
  //verifica se foi recebido pacote
  if (CAN.checkReceive() != CAN_MSGAVAIL) {
    return false;
  }

  unsigned char buf[8];
  unsigned char len = 0;

  //Estrutura que vai receber a mensagem
  struct {
    uint8_t idRede;
    uint8_t tipoGrandeza;
    uint8_t grandeza;
    int16_t valor;
  } canPkt;

  //Le a mensagem da placa
  CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

  //Verifica o ID do transmissor
  if (CAN.getCanId() != CENTRAL_ID) {
#if defined(DEBUG)
    Serial.println(F("Mensagem nao enviada pela central"));
#endif
    return false;
  }

  //Pega o ID do destino
  canPkt.idRede = buf[0];

  /*SIMULAÇÃO*/
  if (canPkt.idRede != 6) {
    sensorConfig.endereco = canPkt.idRede;
  }
  /*FIM SIMULAÇÃO*/


  //confere se é para este dispositivo
  if (canPkt.idRede != sensorConfig.endereco) {
    //    leSensorConfig();
    //#if defined(DEBUG)
    //    Serial.print(F("Esta mensagem nao e para este dispositivo: "));
    //    Serial.print(canPkt.canId);
    //    Serial.print(F(" != "));
    //    Serial.println(sensorConfig.endereco);
    //#endif
    return false;
  }

  //Pega o comando enviado
  canPkt.tipoGrandeza = buf[1];
  canPkt.grandeza = buf[2];
  canPkt.valor = word(buf[3], buf[4]);

  //verifica o tipo de grandeza
  switch (canPkt.tipoGrandeza) {
    /*--------entradas analogicas-----------*/
    case entradaAnalogica: {
#if defined(DEBUG)
        Serial.println(F("entradaAnalogica"));
#endif
        switch (canPkt.grandeza) {
          case temperatura:
            {
#if defined(DEBUG)
              Serial.println(F("temperatura"));
#endif
              enviarTemperatura();
              break;
            };
          case umidadeAr:
            {
#if defined(DEBUG)
              Serial.println(F("umidadeAr"));
#endif
              enviarUmidadeAr();
              break;
            };
          case umidadeSolo:
            {
#if defined(DEBUG)
              Serial.println(F("umidadeSolo"));
#endif
              enviarUmidadeSolo();
              break;
            };
        };
        break;
      };
    /*--------fim das entradas analogicas-----------*/

    /*--------entradas digitais-----------*/
    case entradaDigital: {
#if defined(DEBUG)
        Serial.println(F("entradaDigital"));
#endif
        lerEntradaDigital(canPkt.grandeza);
        break;
      };
    /*--------fim das entradas digitais-----------*/

    /*--------saidas digitais-----------*/
    case saidaDigital: {
      canPkt.valor = canPkt.valor/100;
#if defined(DEBUG)
        Serial.println(F("saidaDigital"));
        Serial.println(canPkt.valor);
#endif
        escreveSaidaDigital(canPkt.grandeza, canPkt.valor);
        break;
      };
      /*--------fim das saidas digitais-----------*/
  };
  return true;
}
