/**
   Bibliotecas utilizadas
   https://github.com/Seeed-Studio/CAN_BUS_Shield
   https://github.com/tiagopossato/overCAN
*/

#include <avr/wdt.h>
#include <EEPROM.h>
#include "SPI.h"
#include "mcp_can.h"
#include "ctmNectar.h"
#include <Wire.h>
#include "ClosedCube_HDC1080.h"

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

//estrutura com os dados a serem enviados
struct {
  float temperaturaAr;
  float umidadeAr;
} dados;

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

  unsigned char msgCfg[1] = {ONLINE};
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
      lerDados();
      enviarDados();
      //msUltimoEnvio atualizado na funcao enviarDados, para incluir nas contagens quando os dados são enviados por solicitação do mestre e não por tempo
    }
  }
  //-------FIM DA COMUNICAÇÃO------------
  if (millis() > msUltimaLeitura + (sensorConfig.intervaloLeitura * 1000)) {
    lerDados();
    msUltimaLeitura = millis();
    if (CAN.checkError() != 0) {
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

  tmp = hdc1080.readTemperature();
  if (abs(tmp - dados.temperaturaAr) > 0.5) flagEnviar = true;
  dados.temperaturaAr = tmp;

  tmp = hdc1080.readHumidity();
  if (abs(tmp - dados.umidadeAr) > 0.5) flagEnviar = true;
  dados.umidadeAr = tmp;

#if defined(DEBUG)
  Serial.print("T=");
  Serial.print(dados.temperaturaAr);
  Serial.print("C, RH=");
  Serial.print(dados.umidadeAr);
  Serial.println("%");
#endif
  if (flagEnviar) enviarDados();

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
  msUltimoEnvio = millis();
}

void enviarTemperatura() {
  unsigned char msg[4] = {0};

  msg[0] = ANALOG_VALUE;
  msg[1] = TEMPERATURA;
  msg[2] = (int)dados.temperaturaAr;
  msg[3] = (int)((dados.temperaturaAr - (int)dados.temperaturaAr) * 100);

  CAN.sendMsgBuf(sensorConfig.endereco, 0, sizeof(msg), msg);

}

void enviarUmidadeAr() {
  unsigned char msg[4] = {0};

  msg[0] = ANALOG_VALUE;
  msg[1] = UMIDADE_AR;
  msg[2] = (int)dados.umidadeAr;
  msg[3] = (int)((dados.umidadeAr - (int)dados.umidadeAr) * 100);

  CAN.sendMsgBuf(sensorConfig.endereco, 0, sizeof(msg), msg);
}

void enviarUmidadeSolo() {
  return;
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
    unsigned int canId;
    unsigned char comando;
    unsigned char msg[6];
  } canPkt;

  //Le a mensagem da placa
  CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

  //Pega o ID do transmissor
  if (CAN.getCanId() != CENTRAL_ID) {
#if defined(DEBUG)
    Serial.println(F("Mensagem nao enviada pela central"));
#endif
    return false;
  }

  //Pega o ID do destino
  canPkt.canId = buf[0];
  //confere se é para este dispositivo
  if (canPkt.canId != sensorConfig.endereco) {
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
  canPkt.comando = buf[1];

  //verifica o comando
  switch (canPkt.comando) {
    case SEND_CONFIG: {
#if defined(DEBUG)
        Serial.println(F("SEND_CONFIG"));
#endif
        enviarConfig();
      };
      break;

    case SEND_DATA: {
#if defined(DEBUG)
        Serial.println(F("SEND_DATA"));
#endif
        enviarDados();
      };
      break;

    case CHANGE_ID: {
#if defined(DEBUG)
        Serial.println(F("CHANGE_ID"));
#endif
        if (buf[2] > 0) sensorConfig.endereco = buf[2];
        //salva novas configuracoes
        salvarSensorConfig();
        reiniciar();
      };
      break;

    case CHANGE_SEND_TIME: {
#if defined(DEBUG)
        Serial.println(F("CHANGE_SEND_TIME"));
#endif
        if (buf[2] >= 0) sensorConfig.intervaloEnvio = buf[2];
        //salva novas configuracoes
        salvarSensorConfig();
        enviarDados();
      };
      break;

    case CHANGE_READ_TIME: {
#if defined(DEBUG)
        Serial.println(F("CHANGE_READ_TIME"));
#endif
        if (buf[2] > 2) {
          sensorConfig.intervaloLeitura = buf[2];
        } else {
          sensorConfig.intervaloLeitura = 2;
        }
        //salva novas configuracoes
        salvarSensorConfig();
        enviarDados();
      };
      break;

#if defined(DEBUG)
    default:
      Serial.println(F("Comando nao reconhecido"));
#endif
  }
  return true;
}
