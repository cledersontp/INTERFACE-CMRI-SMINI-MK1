// 
// INTERFACE 48 CONTROLES CMRI SMINI - Versao 1.7
// By Clederson T. Przybysz - clederson_p@yahoo.com.br
// expressoarduino.blogspot.com
// Criação: Outubro/2017 - Ultima Revisão: Dez/2018
//
// Release Note: 
// 1.3: Correção/Melhoria das mensagens de configuração;
//      Correção Bug Valor dos Blocos de Saida em Modo Standalone sem Modulos PWM configurados;
// 1.4: Melhoria mensagens de configuração com acesso direto a configuracao de angulo dos servos;
// 1.5: Correção da transmissão de valores das entradas;
// 1.6: Correção do modo Configuração que só passa a ser exibido se pino A3 estiver em nivel Alto;
// 1.7: Correção do modo Configuração habilita com nivel Alto. Alterado o pino A3 para PullUP eliminando a necessidade de Jumper;

//Copyright Notes Interface 48 Controles:
// O SOFTWARE É FORNECIDO "NO ESTADO EM QUE SE ENCONTRAM", SEM GARANTIA DE QUALQUER TIPO, EXPRESSA OU IMPLÍCITA, MAS NÃO SE LIMITANDO ÀS GARANTIAS DE COMERCIALIZAÇÃO.  
// EM NENHUMA CIRCUNSTÂNCIA, O AUTOR/TITULAR DE DIREITOS AUTORAIS SE RESPONSABILIZA POR QUALQUER RECLAMAÇÃO, DANOS OU OUTRA RESPONSABILIDADE, 
// SEJA EM AÇÃO DE CONTRATO, DELITO OU DE OUTRA FORMA, DECORRENDO DESTE SOFTWARE OU RELACIONADO AO SEU  USO.
//
//Copyright Notes CRMI Library AND AutoRS485 Library: 
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
//OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// Copyright Notes Adafruit PWMServoDriver Library:
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.


#include <Auto485.h>
#include <CMRI.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <EEPROM.h>

Auto485 SerialRs485(2);
CMRI cmrinode(0,24, 48, SerialRs485); //0 - Endereco Base, 24 Entradas(Sensores), 48 Saidas(Controles), ControleSerial

Adafruit_PWMServoDriver modServo0 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver modServo1 = Adafruit_PWMServoDriver(0x41);

#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define DEFAULT_PULSE_WIDTH   1500
#define FREQUENCY             50

unsigned long TimerAtualizaServos;

#define TempoPulso 100
#define TempoAcionamento 25

// Pino Modo Config - Pino configurado como PullUP e deve ser ligado em nivel baixo para exibir o menu configuração
#define configPin A3 


//Pinos ShiftOut 74HC595
#define clockShiftOutPin 3 //SH_CP 74HC595 Pin 11 - Laranja
#define latchShiftOutPin 4 //ST_CP 74HC595 Pin 12- Roxo
#define dataShiftOutPin 5  //DS 74HC595 Pin 14 - Branco

//Pinos ShiftIn CD4021
#define clockShiftInPin A0 //Clock CD4021 Pin 10 - Amarelo
#define latchShiftInPin A1 //Latch CD4021 Pin 9 - Verde
#define dataShiftInPin A2  //DS CD4021 Pin 3 - Azul



unsigned long TimerDesligaAMV;
byte AtualizaValorSaida=1;
byte EtapaConfig;
byte LoopCarregaEntradas=0;

byte EnderecoNode=0;
byte NumModulosPWM;
byte ModoOperacao=0;   //0-Standalone / 1-Node CMRI
byte ValorEntradas[6]; //Cada byte tem 8 bit - 8x6 48 bits (48 Entradas Standalone / 24 Entradas NodeCMRI)
byte ValorSaidas[6];   //Cada byte tem 8 bit - 8x6 48 bits
byte TipoSaidas[6];    //0- Normal, 1- PWN 16 Canais, 2- AMV Bobina
byte PortaIniBlocoSaida[6];  //Primeira Porta do Bloco de Saida 
byte ValorPortasSaidas[12]; //AMVs de Bobinas Precisam de Duas Portas para Controle (NSaidas x 2)
byte LoopResetBobina[48]; //Loop para Reset do Valor de Bobina 
byte TempoResetBobina[6]; //Numero de Loops para Reset da Bobina

byte PosServo[32];     //Posicao Atual do Servo
byte NovaPosServo[32]; //Nova Posicao do Servo

byte ModoConfig=0; //0. Desabilita / 1. Habilita configurações ao iniciar o Arduino

void setup() 
{
  SerialRs485.begin(19200);
  
  //Verifica pino A3 ligado em nivel Alto ou Baixo
  pinMode(configPin, INPUT_PULLUP);
  ModoConfig = digitalRead(configPin);
  
  if (ModoConfig==0) SerialRs485.println("Pressine 'C' para Configuracao...");
  if (ModoConfig==0) SerialRs485.flush();
  if (ModoConfig==0) AguardaModoConfig();  
  

  //Carrega Configuraçoes 
  CarregaConfiguracoes();

  //Pinos Shift Out
  pinMode(latchShiftOutPin, OUTPUT);
  pinMode(clockShiftOutPin, OUTPUT);
  pinMode(dataShiftOutPin, OUTPUT);

  //define pin modes
  pinMode(latchShiftInPin, OUTPUT);
  pinMode(clockShiftInPin, OUTPUT); 
  pinMode(dataShiftInPin, INPUT);

  //Endereco Node
  cmrinode.set_address(EnderecoNode);
  
  if (ModoConfig==0)  {
    SerialRs485.print("Modo Operacao: ");
    if  (ModoOperacao == 0) {
      SerialRs485.println("Stand Alone");
    }
    else
    {
      SerialRs485.println("Node CMRI");
      SerialRs485.print("Endereco Node: ");
      SerialRs485.println(EnderecoNode);
    }
    //Modulo PWM
    SerialRs485.print("Modulos PWM: ");
    SerialRs485.println(NumModulosPWM);
  
    for(int i = 0; i < 6; i++) {
      if (TipoSaidas[i] != 1) {
        SerialRs485.print("Tipo Saida Bloco ");
        SerialRs485.print(i+1);
        SerialRs485.print(": ");
        SerialRs485.print(TipoSaidas[i]);
        if (TipoSaidas[i] == 2) {
          SerialRs485.print(" Tempo Acionamento:");
          SerialRs485.print(TempoResetBobina[i]);
        }
        SerialRs485.println("");
      }
    }
    SerialRs485.println("Interface Ativa");
    SerialRs485.flush();
  }
}



void loop() {
  if (ModoOperacao == 1) cmrinode.process();
    

  //Carrega Valores de Entrada
  if (LoopCarregaEntradas==0) {
    CarregaValoresEntradas();
    LoopCarregaEntradas++; 
  }

  //Verifica Mudança Nos Valores das Saidas
  for(int i = 0; i < 6; i++) {
    byte NovoValor;
    if  (ModoOperacao == 0) {
      NovoValor = ValorEntradas[i];
    }
    else
    {
      NovoValor = cmrinode.get_byte(i);
    }
    if (NovoValor != ValorSaidas[i]) {
      ComparaBitsSaidas(NovoValor, i);
    }
  }

  
  //Verifica Posicao dos Servos - Intervalo 0,1 segundos
  if (millis()-TimerAtualizaServos>TempoPulso && NumModulosPWM>0) {
    for(int i = 0; i < 32; i++) {
      //Incrementa Posicao do Servo
      if (NovaPosServo[i] > PosServo[i]) {
        PosServo[i]++;
        AtualizaPosicaoServo(i, PosServo[i]);
      }
      if (NovaPosServo[i] < PosServo[i]) {
        PosServo[i]--;
        AtualizaPosicaoServo(i, PosServo[i]);
      }
    }
    TimerAtualizaServos = millis();
  }

   
  //Verifica Reset AMV Bobina
  if (millis()-TimerDesligaAMV>TempoAcionamento) {
    for (byte i = 0; i < 48; i++) {
      if (LoopResetBobina[i] > 0) {
        LoopResetBobina[i]++;
        byte bloco = i/8;
        if (LoopResetBobina[i]>TempoResetBobina[bloco]) ResetAMVBobina(i);
      }
    }
    TimerDesligaAMV = millis();
    LoopCarregaEntradas++;
    if (LoopCarregaEntradas>20) LoopCarregaEntradas=0;
  }
   
 
  //Atualizacao das Portas de Saidas
  if (AtualizaValorSaida>0) AtualizaSaida();
}

void ComparaBitsSaidas(byte tmpValor, byte bloco) {
  //Compara os bit de Novo Valor
  for(byte i = 0; i < 8; i++) {
    if (bitRead(tmpValor, i) != bitRead(ValorSaidas[bloco],i)) {
      switch (TipoSaidas[bloco]) {
        
        case 0:
          NovoValorSaida(bloco, i, bitRead(tmpValor, i));
          break;
        case 1:
          NovoValorServo(bloco * 8 + i, bitRead(tmpValor, i));
          break;
        case 2:
          NovoValorAMVBobina(bloco, i, bitRead(tmpValor, i));
          break;
      }
    }
  }
  ValorSaidas[bloco] = tmpValor;
}

void NovoValorServo(byte nServo, byte NovoValor) {
  if (NovoValor==1) {
    NovaPosServo[nServo] = PosicaoUmServo(nServo); 
  }
  else
  {
    NovaPosServo[nServo] = PosicaoZeroServo(nServo); 
  }
}

byte PosicaoZeroServo(byte nServo) {
  byte Angulo;
  Angulo = EEPROM.read(20 + nServo);
  if (Angulo==0||Angulo>180) Angulo=5;
  return Angulo;
}

byte PosicaoUmServo(byte nServo) {
  byte Angulo;
  Angulo = EEPROM.read(52 + nServo);
  if (Angulo==0||Angulo>180) Angulo=35;
  return Angulo;
}


void NovoValorSaida(byte bloco, byte nSaida, byte NovoValor){
  byte BlocoSaida = PortaIniBlocoSaida[bloco]/8;
  bitWrite(ValorPortasSaidas[BlocoSaida], nSaida, NovoValor);
  AtualizaValorSaida=1; 
}

void NovoValorAMVBobina(byte bloco, byte nSaida, byte NovoValor){
  byte BlocoSaida = PortaIniBlocoSaida[bloco]/8;
  byte PortaSaida; 
  LoopResetBobina[bloco*8 + nSaida] = 1;
  if (nSaida>3) {
    BlocoSaida = BlocoSaida+1;
    nSaida = nSaida - 4;
  }
  PortaSaida = nSaida * 2;
  if (NovoValor==0) {
    bitWrite(ValorPortasSaidas[BlocoSaida], PortaSaida, 0);
    bitWrite(ValorPortasSaidas[BlocoSaida], PortaSaida + 1, 1);
  }
  else
  {
    bitWrite(ValorPortasSaidas[BlocoSaida], PortaSaida, 1);
    bitWrite(ValorPortasSaidas[BlocoSaida], PortaSaida + 1, 0);
  }
  AtualizaValorSaida=1;
}

void ResetAMVBobina(byte nSaida){
  byte bloco = nSaida/8;
  byte BlocoSaida = PortaIniBlocoSaida[bloco]/8;
  byte PortaSaida;
  LoopResetBobina[nSaida] = 0;
  nSaida = nSaida - BlocoSaida * 8;
 
  if (nSaida>3) {
    BlocoSaida = BlocoSaida+1;
    nSaida = nSaida - 4;
  }
  PortaSaida = nSaida * 2;
  bitWrite(ValorPortasSaidas[BlocoSaida], PortaSaida, 0);
  bitWrite(ValorPortasSaidas[BlocoSaida], PortaSaida + 1, 0);
  AtualizaValorSaida=1;
}

void AtualizaPosicaoServo(byte nServo, int Angulo) {
  if (nServo <16) {
    modServo0.setPWM(nServo, 0, ConverteAnguloPulso(Angulo));
  }
  else if (nServo <32) {
    modServo1.setPWM(nServo-16, 0, ConverteAnguloPulso(Angulo));
  } 
}

int ConverteAnguloPulso(int angle){
  int pulse_wide, analog_value;
  pulse_wide   = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  //Serial.println(analog_value);
  return analog_value;
}

void AtualizaSaida() {
  // Habilita Envio Dados
  digitalWrite(latchShiftOutPin, LOW);
  for (int i = 11; i >= 0; i--) {
    //Envia Bits
    shiftOut(dataShiftOutPin, clockShiftOutPin, MSBFIRST, ValorPortasSaidas[i]); 
  }
  //Finaliza Envio Dados
  digitalWrite(latchShiftOutPin, HIGH);
  AtualizaValorSaida=0;
}

void CarregaValoresEntradas() {
  byte nEntradas=3;
  if (ModoOperacao==0) nEntradas=6;

  digitalWrite(latchShiftInPin,1);
  //set it to 1 to collect parallel data, wait
  delayMicroseconds(20);
  //set it to 0 to transmit data serially  
  digitalWrite(latchShiftInPin,0);
  for (byte i = 0; i < nEntradas; i++) {
    byte NovoValor = shiftIn(dataShiftInPin, clockShiftInPin);
    if (NovoValor != ValorEntradas[i]) {
      ValorEntradas[i] = NovoValor;
      //Se modo Node Atualiza Valor Saida
      if (ModoOperacao == 1) cmrinode.set_byte(i, NovoValor);
    }
  }
}

byte shiftIn(int myDataPin, int myClockPin) { 
  int i;
  int temp = 0;
  int pinState;
  byte myDataIn = 0;

  for (i=7; i>=0; i--)
  {
    digitalWrite(myClockPin, 0);
    delayMicroseconds(0.2);
    temp = digitalRead(myDataPin);
    if (temp) {
      pinState = 1;
      myDataIn = myDataIn | (1 << i);
    }
    else {
      pinState = 0;
    }
    digitalWrite(myClockPin, 1);
  }
  return myDataIn;
}

void AguardaModoConfig() {
  TimerDesligaAMV = millis();

  while (millis()-TimerDesligaAMV<5000) {
    //Comunicacao Serial Recebida
    if (SerialRs485.available()) {
       byte valS = SerialRs485.read(); 
       if (valS == 67) {
          SerialRs485.println(" ");
          SerialRs485.println("Modo Configuracao");
          SerialRs485.println(" ");
          delay(1000);
          while (SerialRs485.available() > 0) {
            valS = SerialRs485.read();
            
          }
          while (EtapaConfig<99) {
            EntraConfiguracao();
          }
          TimerDesligaAMV = TimerDesligaAMV+5000;
       }
    }
  }
}


void EntraConfiguracao() {
  byte vRec=0;
  byte vRecebido=0;

  if (EtapaConfig==0) SerialRs485.print("Modo [0]Standalone [1]CMRI [9]CfgServo:");
  if (EtapaConfig==1) SerialRs485.print("End.Node [0-125]:");  
  if (EtapaConfig==2) SerialRs485.print("Modulos PWM [0-2]:");
  if (EtapaConfig>=3&&EtapaConfig<=8) {
    SerialRs485.print("Tipo Saida Bloco ");
    SerialRs485.print(EtapaConfig-2);
    SerialRs485.print(" [0]OnOff [2]Pulso");
    SerialRs485.print(":");
  }
  if (EtapaConfig>=9&&EtapaConfig<=14) {
    SerialRs485.print("Tempo Acionamento Bloco ");
    SerialRs485.print(EtapaConfig-7);
    SerialRs485.print(":");
  }
  if (EtapaConfig==15) {
    SerialRs485.println("");
    SerialRs485.print("Configurar Servo:");
  }
  if (EtapaConfig>=20&&EtapaConfig<=51) {
    SerialRs485.print("Angulo Min Servo ");
    SerialRs485.print(EtapaConfig-19);
    SerialRs485.print(":");
  }
  if (EtapaConfig>=52&&EtapaConfig<=83) {
    SerialRs485.print("Angulo Max Servo ");
    SerialRs485.print(EtapaConfig-51);
    SerialRs485.print(":");
  }
  SerialRs485.flush();
  
  while (vRec == 0) {
    if (SerialRs485.available() > 0) {
      byte inChar = SerialRs485.read();
      if (inChar==10||inChar==13) {
        vRec=1;
      }
      else
      {
        vRecebido = vRecebido * 10 + (inChar-48);
      }
    }
  }
  
  
  //Valida Configuracao Recebida
  SerialRs485.println(vRecebido);
  
  if (EtapaConfig==0&&(vRecebido==0||vRecebido==1||vRecebido==9)) {
    if (vRecebido<9) {
      EEPROM.write(0, vRecebido);  
      ModoOperacao=vRecebido;
      if (vRecebido==0) {
        EtapaConfig=2;
      }
      else
      {
        EtapaConfig=1;
      }
    }
    else
    {
      CarregaConfiguracoes();
      EtapaConfig=15;
    }
  }
  else if (EtapaConfig==1&&vRecebido>=0&&vRecebido<127) {
    EEPROM.write(1, vRecebido);  
    EnderecoNode=vRecebido;
    EtapaConfig++;
  }
  else if (EtapaConfig==2&&vRecebido>=0&&vRecebido<3) {
    EEPROM.write(2, vRecebido); 
    NumModulosPWM=vRecebido;
    if (vRecebido==0) EtapaConfig++;
    if (vRecebido>0) {
      TipoSaidas[0]=1;
      TipoSaidas[1]=1;
      EEPROM.write(3, 1);
      EEPROM.write(4, 1);  
      EtapaConfig=5;
    }
    if (vRecebido>1) {
      TipoSaidas[2]=1;
      TipoSaidas[3]=1;
      EEPROM.write(5, 1);
      EEPROM.write(6, 1);  
      EtapaConfig=7;
    }
  }
  else if (EtapaConfig>=3&&EtapaConfig<=8&&vRecebido>=0&&vRecebido<3) {
    EEPROM.write(EtapaConfig, vRecebido); 
    TipoSaidas[EtapaConfig-3]=vRecebido;
    EtapaConfig++;
    
    //Verifica Proximo Etapa
    if (EtapaConfig==9) {
      byte FimBusca=0;
      while (FimBusca==0) {
        if (TipoSaidas[EtapaConfig-9] != 2) {
          EEPROM.write(EtapaConfig, 2);
          TempoResetBobina[EtapaConfig-9] = 2;
          EtapaConfig++;
        }
        else
        {
          FimBusca=1;
        }
        if (EtapaConfig>14) FimBusca=1;
        if (EtapaConfig==15) {
          if (NumModulosPWM==0) EtapaConfig=99;
          SerialRs485.println("Fim Configuracao");
        }
      }
    }
  }
  else if (EtapaConfig>=9&&EtapaConfig<=14&&vRecebido>1&&vRecebido<255) {
    EEPROM.write(EtapaConfig, vRecebido);
    TempoResetBobina[EtapaConfig-9] = vRecebido;
    EtapaConfig++;
    //Verifica Proximo Etapa  
    byte FimBusca=0;
    while (FimBusca==0) {
      if (TipoSaidas[EtapaConfig-9] != 2) {
        EEPROM.write(EtapaConfig, 2);
        TempoResetBobina[EtapaConfig-9] = 2;
        EtapaConfig++;
      }
      else
      {
        FimBusca=1;
      }
      if (EtapaConfig>14) FimBusca=1;
    }
    if (EtapaConfig==15) {
      if (NumModulosPWM==0) EtapaConfig=99;
      SerialRs485.println("Fim Configuracao");
    }
  }
  else if (EtapaConfig==15&&vRecebido>=0&&vRecebido<= 16*NumModulosPWM) {
    EtapaConfig= 20 + vRecebido-1;
    if (vRecebido==0) {
      EtapaConfig=99;
      SerialRs485.println("Fim Configuracao");
    }
  }
  else if (EtapaConfig>=20&&EtapaConfig<=51&&vRecebido>0&&vRecebido<180) {
    EEPROM.write(EtapaConfig, vRecebido);
    AtualizaPosicaoServo(EtapaConfig-20, vRecebido);
    EtapaConfig = EtapaConfig + 32;  
  }
  else if (EtapaConfig>=52&&EtapaConfig<=83&&vRecebido>0&&vRecebido<180) {
    EEPROM.write(EtapaConfig, vRecebido);
    AtualizaPosicaoServo(EtapaConfig-52, vRecebido);
    EtapaConfig = 15;  
  }
}


void CarregaConfiguracoes() {
  
  //Carrega Modo Operacao
  ModoOperacao = EEPROM.read(0);
  
  //Carrega Endereco Node 
  EnderecoNode = EEPROM.read(1);
  
  //Carrega Numero de Modulos PWW
  NumModulosPWM = EEPROM.read(2);
  //Inicia I2C
   if (NumModulosPWM>0) {
    modServo0.begin();
    modServo0.setPWMFreq(FREQUENCY);
  }
  if (NumModulosPWM>1) {
    modServo1.begin();
    modServo1.setPWMFreq(FREQUENCY);
  }

  //Configura Posicao Inicial dos Servos
  if (NumModulosPWM>0) {
    for(int i = 0; i < 32; i++) {
      NovaPosServo[i] = PosicaoZeroServo(i); 
    }
  }
  
  //Carrega Tipo das Portas de Saida - 0- Normal, 1- PWN 16 Canais, 2- AMV Bobina
  TipoSaidas[0] = EEPROM.read(3);
  TipoSaidas[1] = EEPROM.read(4);
  TipoSaidas[2] = EEPROM.read(5);
  TipoSaidas[3] = EEPROM.read(6);
  TipoSaidas[4] = EEPROM.read(7);
  TipoSaidas[5] = EEPROM.read(8);
  
  //Tempo Reset Bobina
  TempoResetBobina[0] = EEPROM.read(9);
  TempoResetBobina[1] = EEPROM.read(10);
  TempoResetBobina[2] = EEPROM.read(11);
  TempoResetBobina[3] = EEPROM.read(12);
  TempoResetBobina[4] = EEPROM.read(13);
  TempoResetBobina[5] = EEPROM.read(14);
  
  //Calcula Primeira Porta do Endereco de Saida
  byte IniBloco=0;
  byte FimBloco=0;
  for (byte i = 0; i < 6; i++) {
    if (TipoSaidas[i]==1) {
      FimBloco=0;
    }
    else
    {
      //Inicia Na Ultima Porta Configurada
      IniBloco=FimBloco;
      //Incrementa 1 ao Endereco da Porta Incial
      if (FimBloco>0) IniBloco++;  
      //Seta Valor Porta
      PortaIniBlocoSaida[i]=IniBloco;
      //Calcula Posicao Final da Porta
      if (TipoSaidas[i]==0) {
        FimBloco = FimBloco + 7;
      }
      else
      {
        FimBloco = FimBloco + 15;
      }
    }
  }
}

