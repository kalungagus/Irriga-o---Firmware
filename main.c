//**************************************************************************************************
//                                     CONTROLE DE IRRIGA��O
//
// Data:         26/05/2019
// Autor:        Gustavo Adolpho Souteras Barbosa
// Descri��o:    O presente programa faz o controle b�sico de bombas, baseando-se em
//               leituras dos sensores de umidade de solo instalados nela. O controle
//               � feito atrav�s da USB, mas tamb�m pode ser feito automaticamente atrav�s
//               do firmware
//
//**************************************************************************************************
#include "HardwareProfile.h"

//**************************************************************************************************
// FUS�VEIS
//**************************************************************************************************
#pragma config FOSC=HSPLL_HS
#pragma config CPUDIV=OSC1_PLL2
#pragma config PLLDIV=5
#pragma config USBDIV=2
#pragma config CCP2MX=ON
#pragma config WDT=OFF
#pragma config WDTPS=32768
#pragma config MCLRE=ON
#pragma config LVP=OFF
#pragma config VREGEN=ON
#pragma config IESO=OFF
#pragma config PWRT=OFF
#pragma config BOR=ON
#pragma config CP0=OFF
#pragma config CP1=OFF
#pragma config CP2=OFF
#pragma config CP3=OFF
#pragma config CPB=OFF
#pragma config CPD=OFF
#pragma config WRT0=OFF
#pragma config WRT1=OFF
#pragma config WRT2=OFF
#pragma config WRT3=OFF
#pragma config WRTB=OFF
#pragma config WRTC=OFF
#pragma config WRTD=OFF

//**************************************************************************************************
// DEFINES
//**************************************************************************************************
#define ACIONA_BOMBA1           PORTDbits.RD0
#define ACIONA_BOMBA2           PORTDbits.RD1
#define ACIONA_BOMBA3           PORTDbits.RD2
#define ACIONA_BOMBA4           PORTDbits.RD3
#define LED1                    PORTBbits.RB5

//==================================================================================================
// Comandos da USB
//==================================================================================================
#define CMD_ACIONAR_BOMBA       0x80
#define CMD_READ_SENSOR         0x81

//**************************************************************************************************
// EEPROM
//**************************************************************************************************
#pragma romdata eedata_scn=0xF00000
#pragma romdata

//**************************************************************************************************
// PROT�TIPOS DE FUN��ES
//**************************************************************************************************
void execEvent(void);
void LowISR(void);
void HighISR(void);

//**************************************************************************************************
// ESTRUTURAS DE DADOS
//**************************************************************************************************

//**************************************************************************************************
// VARI�VEIS GLOBAIS
//**************************************************************************************************
unsigned char gucUSBReceiveBuffer[7];
unsigned char ucTxBuffer[7];
unsigned int guiSensorVal[4];

//**************************************************************************************************
// INTERRUP��ES
//**************************************************************************************************
//==================================================================================================
// Tratamento dos vetores de interrup��o do PIC18
//==================================================================================================
#pragma interruptlow LowISR nosave=section(".tmpdata")
void LowISR(void)
{
    PIR1bits.TMR2IF=0;
}
	
#pragma interrupt HighISR
void HighISR(void)
{
    if(INTCONbits.INT0IF)
    {
        INTCONbits.INT0IF = 0;
    }
}
	
#pragma code lowVector=0x18
void LowVector(void){_asm goto LowISR _endasm}
#pragma code highVector=0x8
void HighVector(void){_asm goto HighISR _endasm}
#pragma code // Volta para a se��o padr�o de c�digo

//**************************************************************************************************
// FUN��ES
//**************************************************************************************************
// =================================================================================================
// Fun��o:     _USBCBInitEP
// Descri��o:  Esta fun��o � chamada quando o dispositivo se torna inicializado,
//             que ocorre depois do host enviar uma requisi��o SET_CONFIGURATION
// 					       (com wValue diferente de 0). Esta fun��o de callback deve inicializar 
//					        os endpoints para o uso do dispositivo, de acordo com a configura��o
//					        atual.
// Par�metros: nenhum
// Retorno:    nenhum
// =================================================================================================
void USBCBInitEP(void)
{
    CDCInitEP();
}

// =================================================================================================
// Fun��o:     _USBCBStdSetDscHandler
// Descri��o:  A fun��o de callback USBCBStdSetDscHandler() � chamada
//					        quando uma requisi��o SETUP, bRequest: SET_DESCRIPTOR
//					        � recebida. Tipicamente requisi��es SET_DESCRIPTOR n�o
//					        s�o usadas na maioria das aplica��es, e � opcional dar
//					        suporte a este tipo de requisi��o.
// Par�metros: nenhum
// Retorno:    nenhum
// =================================================================================================
void USBCBStdSetDscHandler(void)
{
    // Deve requisitar propriedade de sess�o se suportar esta requisi��o.
}

// =================================================================================================
// Fun��o:     _USBCBCheckOtherReq
// Descri��o:  Quando pacotes SETUP chegam do host, alguns firmwares processam asome
// 					       requisi��o e respondem apropriadamente para cumprir a requisi��o.
//					        Alguns dos pacotes SETUP seguem o padr�o USB "cap�tulo 9" (cumprindo o
//                          cap�tulo 9 das especifica��es oficiais USB) de requisi��es, enquanto
//					        outros podem ser espec�ficos para a classe de dispositivo USB que est�
//					        sendo implementado. Por exemplo, um dispositivo de classe HID precisa ser
//					        capaz de responder requisi��es do tipo "GET REPORT". Isto n�o � uma requisi��o
//					        padr�o do cap�tulo 9, e ent�o n�o � tratada pelo arquivo usb_device.c.
//					        Ao inv�s disto, esta requisi��o deve ser tratada pelo firmware espec�fico
//					        de classe, como aquele contido no arquivo usb_function_hid.c.
// Par�metros: nenhum
// Retorno:    nenhum
// =================================================================================================
void USBCBCheckOtherReq(void)
{
    USBCheckCDCRequest();
}

// =================================================================================================
// Fun��o:     _USBCB_SOF_Handler
// Descri��o:  O host USB envia um pacote SOF a dispositivos full-speed a cada 1 ms.
//             Esta interrup��o pode ser �til para pipes isochronous. Desenvolvedores
//             devem implementar uma rotina de callback se necess�rio.
// Par�metros: nenhum
// Retorno:    nenhum
// =================================================================================================
void USBCB_SOF_Handler(void)
{
    // N�o h� necessidade de definir como 0 o UIRbits.SOFIF aqui.
    // A fun��o que chama este callback j� faz isto.
}

// =================================================================================================
// Fun��o:     _USBCBSuspend
// Descri��o:  Este callback � chamado quando um USB suspend � detectado.
// Par�metros: nenhum
// Retorno:    nenhum
// =================================================================================================
void USBCBSuspend(void)
{
    // Fazer os ajustes apropriados aqui para que o dispositivo entre em modo de baixo consumo.
}

// =================================================================================================
// Fun��o:     _USBCBWakeFromSuspend
// Descri��o:  O host pode colocar perif�ricos USB em modo de baixo consumo (ao "enviar" mais de
//					        3ms de idle). Assim que estiver no modo suspend, o host poder� acordar o dispositivo
//	  			            ao enviar uma sinaliza��o de estado n�o-idle.
//				            Esta fun��o de callback � chamada quando um "wakeup from USB suspend" �
//					        detectado.
// Par�metros: nenhum
// Retorno:    nenhum
// =================================================================================================
void USBCBWakeFromSuspend(void)
{
    // Aqui deve-se reverter o modo de baixo consumo de energia.
}

// =================================================================================================
// Fun��o:     _USBCBErrorHandler
// Descri��o:  O objetivo desta fun��o de callback � principalmente para depura��o
//             durante a fase de desenvolvimento. Verifique UEIR para ver qual erro
//             causou a interrup��o.
// Par�metros: nenhum
// Retorno:    nenhum
// =================================================================================================
void USBCBErrorHandler(void)
{
}

// =================================================================================================
// Fun��o:     _USER_USB_CALLBACK_EVENT_HANDLER
// Descri��o:  Esta fun��o � chamada pelo stack USB para notificar
//             a aplica��o do usu�rio que um evento de USB ocorreu.
//             Esta fun��o de callback est� no contexto da interrup��o
//             quando a op��o USB_INTERRUPT � selecionada.
// Par�metros: USB_EVENT event: o tipo de evento
//             void *pdata: ponteiro para os dados do evento
//             WORD size: tamanho dos dados do evento
// Retorno:    nenhum
// =================================================================================================
BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
    switch(event)
    {
        case EVENT_CONFIGURED: USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR: USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST: USBCBCheckOtherReq();
            break;
        case EVENT_SOF: USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND: USBCBSuspend();
            break;
        case EVENT_RESUME: USBCBWakeFromSuspend();
            break;
        case EVENT_BUS_ERROR: USBCBErrorHandler();
            break;
        case EVENT_TRANSFER: Nop();
            break;
        default: break;
    }      
    return TRUE; 
}

// =================================================================================================
// Fun��o:     _wrEEPROM
// Descri��o:  Escreve um byte na eeprom
// Par�metros: unsigned char ucEnd: endere�o a ser escrito.
//             unsigned char ucDado: dado a ser escrito.
// Retorno:    nenhum
// =================================================================================================
void wrEEPROM(unsigned char ucEnd, unsigned char ucDado)
{
    EEADR = ucEnd;
    EEDATA = ucDado;
    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;
    EECON1bits.WREN = 1;
    INTCONbits.GIE = 0;
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;
    while(EECON1bits.WR);
    INTCONbits.GIE = 1;
    PIR2bits.EEIF = 0;
    EECON1bits.WREN = 0;
}

// =================================================================================================
// Fun��o:     _rdEEPROM
// Descri��o:  L� um byte na eeprom
// Par�metros: unsigned char ucEnd: endere�o a ser lido
// Retorno:    char: caracter lido
// =================================================================================================
char rdEEPROM(unsigned char ucEnd)
{
    EECON1=0;
    EEADR = ucEnd;
    EECON1bits.RD = 1;
    return(EEDATA);
}

// =================================================================================================
// Fun��o:     _acionaBomba
// Descri��o:  Faz o acionamento de uma bomba selecionada
// Par�metros: unsigned char ucBomba: ID da bomba (1 a 4)
//             unsigned char estado: 1-ligado, 0-desligado
// Retorno:    nenhum
// =================================================================================================
void acionaBomba(unsigned char ucBomba, unsigned char estado)
{
    estado = (estado != 0) ? 1 : 0;    // Para evitar qualquer par�metro errado.
    switch(ucBomba)
    {
        case 0:
            ACIONA_BOMBA1 = estado;
            break;
        case 1:
            ACIONA_BOMBA2 = estado;
            break;
        case 2:
            ACIONA_BOMBA3 = estado;
            break;
        case 3:
            ACIONA_BOMBA4 = estado;
            break;
        default: break;
    }
}

// =================================================================================================
// Fun��o:     _leSensor
// Descri��o:  Faz a leitura de um sensor selecionado para um buffer
// Par�metros: unsigned char *buffer: buffer para receber a leitura
//             unsigned char ucSensor: sensor selecionado (1 a 4)
// Retorno:    1: leitura realizada
//             0: sensor selecionado n�o existe
// =================================================================================================
unsigned char leSensor(unsigned char *buffer, unsigned char ucSensor)
{
    unsigned int val;
    
    if(ucSensor < 4)
    {
        val = guiSensorVal[ucSensor];
        buffer[0] = ((unsigned char *)(&val))[0];
        buffer[1] = ((unsigned char *)(&val))[1];
        return(1);
    }
    else
        return(0);
}
// =================================================================================================
// Fun��o:     _trataComando
// Descri��o:  Faz o tratamento de dados recebidos pela porta USB.
// Par�metros: nenhum
// Retorno:    nenhum
// =================================================================================================
void trataComando(void)
{
    LED1 = ~LED1;
    switch(gucUSBReceiveBuffer[0])
    {
        case CMD_ACIONAR_BOMBA:
            acionaBomba(gucUSBReceiveBuffer[1], gucUSBReceiveBuffer[2]);
            break;
        case CMD_READ_SENSOR:
            ucTxBuffer[0] = gucUSBReceiveBuffer[0];
            ucTxBuffer[1] = gucUSBReceiveBuffer[1];
            if(leSensor((&ucTxBuffer[2]), gucUSBReceiveBuffer[1]))
            {
                if(USBUSARTIsTxTrfReady())
                    putUSBUSART((char *)ucTxBuffer, 4);
            }
        default:
            break;
    }
}

//==================================================================================================
// Fun��o: _main
// Descri��o: Fun��o principal do programa.
//==================================================================================================
void main(void)
{
    unsigned int uiADCSelect=0;
    unsigned int uiTemp;
    unsigned int uiAux;
     
    INTCON = 0x00;
    RCON = 0x80;         // Habilita prioridades para interrup��es
    ADCON1 = 0x0B;       // PORTA s� como entradas e sa�das digitais.
    ADCON2 = 0x92;
    CMCON = 0x07;        // Comparadores desligados.
    PIR2 = 0;
    TRISA = 0xFF;
    TRISB = 0x00;
    PORTB = 0xFF;
    TRISC = 0x01;
    PORTC = 0x00;
    TRISD = 0x00;
    PORTD = 0x00;
    TRISE = 0x00;
    PORTE = 0x07;
    INTCON2 = 0x00;
    PR2 = 249;           // Contagem de 1ms.
    T2CON = 0x13;        // Timer2 com 16x de prescaler e 3x de postcaler.
    IPR1 = 0x00;         // Prioridade baixa para o Timer 2;
    PIE1 = 0x02;         // Habilita interrup��o de Timer 2;
    INTCON2 = 0x00;
    INTCON = 0xC0;       // Liga interrup��o de Timer 2.
 
    // Requisita a primeira convers�o do A/D para que a leitura
    // no loop reflita a realidade no pino.
    ADCON0 = 0;
    ADCON0 |= 1;
    ADCON0 |= 2;
    USBDeviceInit();

    for(;;)
    {
        USBDeviceTasks();
        if(USBGetDeviceState() < CONFIGURED_STATE)
        {
            continue;
        }
        else
        {
            uiTemp = getsUSBUSART((char *)gucUSBReceiveBuffer, 3);
            if(uiTemp != 0)
            {
                trataComando();
            }
            if(ADCON0bits.DONE == 0)
            {
                guiSensorVal[uiADCSelect] = ADRES;
                switch(uiADCSelect)
                {
                    case 0:
                        ADCON0 = 0x04;
                        uiADCSelect = 1;
                        break;
                    case 1:
                        ADCON0 = 0x08;
                        uiADCSelect = 2;
                        break;
                    case 2:
                        ADCON0 = 0x0C;
                        uiADCSelect = 3;
                        break;
                    case 3:
                        ADCON0 = 0x00;
                        uiADCSelect = 0;
                        break;
                }
                ADCON0 |= 0x01;
                ADCON0 |= 0x02;
            }
            CDCTxService();
        }
 }
}
//**************************************************************************************************
