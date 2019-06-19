//**************************************************************************************************
//                                     CONTROLE DE IRRIGAÇÃO
//
// Data:         26/05/2019
// Autor:        Gustavo Adolpho Souteras Barbosa
// Descrição:    O presente programa faz o controle básico de bombas, baseando-se em
//               leituras dos sensores de umidade de solo instalados nela. O controle
//               é feito através da USB, mas também pode ser feito automaticamente através
//               do firmware
//
//**************************************************************************************************
#include "HardwareProfile.h"

//**************************************************************************************************
// FUSÍVEIS
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
// PROTÓTIPOS DE FUNÇÕES
//**************************************************************************************************
void execEvent(void);
void LowISR(void);
void HighISR(void);

//**************************************************************************************************
// ESTRUTURAS DE DADOS
//**************************************************************************************************

//**************************************************************************************************
// VARIÁVEIS GLOBAIS
//**************************************************************************************************
unsigned char gucUSBReceiveBuffer[7];
unsigned char ucTxBuffer[7];
unsigned int guiSensorVal[4];

//**************************************************************************************************
// INTERRUPÇÕES
//**************************************************************************************************
//==================================================================================================
// Tratamento dos vetores de interrupção do PIC18
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
#pragma code // Volta para a seção padrão de código

//**************************************************************************************************
// FUNÇÕES
//**************************************************************************************************
// =================================================================================================
// Função:     _USBCBInitEP
// Descrição:  Esta função é chamada quando o dispositivo se torna inicializado,
//             que ocorre depois do host enviar uma requisição SET_CONFIGURATION
// 					       (com wValue diferente de 0). Esta função de callback deve inicializar 
//					        os endpoints para o uso do dispositivo, de acordo com a configuração
//					        atual.
// Parâmetros: nenhum
// Retorno:    nenhum
// =================================================================================================
void USBCBInitEP(void)
{
    CDCInitEP();
}

// =================================================================================================
// Função:     _USBCBStdSetDscHandler
// Descrição:  A função de callback USBCBStdSetDscHandler() é chamada
//					        quando uma requisição SETUP, bRequest: SET_DESCRIPTOR
//					        é recebida. Tipicamente requisições SET_DESCRIPTOR não
//					        são usadas na maioria das aplicações, e é opcional dar
//					        suporte a este tipo de requisição.
// Parâmetros: nenhum
// Retorno:    nenhum
// =================================================================================================
void USBCBStdSetDscHandler(void)
{
    // Deve requisitar propriedade de sessão se suportar esta requisição.
}

// =================================================================================================
// Função:     _USBCBCheckOtherReq
// Descrição:  Quando pacotes SETUP chegam do host, alguns firmwares processam asome
// 					       requisição e respondem apropriadamente para cumprir a requisição.
//					        Alguns dos pacotes SETUP seguem o padrão USB "capítulo 9" (cumprindo o
//                          capítulo 9 das especificações oficiais USB) de requisições, enquanto
//					        outros podem ser específicos para a classe de dispositivo USB que está
//					        sendo implementado. Por exemplo, um dispositivo de classe HID precisa ser
//					        capaz de responder requisições do tipo "GET REPORT". Isto não é uma requisição
//					        padrão do capítulo 9, e então não é tratada pelo arquivo usb_device.c.
//					        Ao invés disto, esta requisição deve ser tratada pelo firmware específico
//					        de classe, como aquele contido no arquivo usb_function_hid.c.
// Parâmetros: nenhum
// Retorno:    nenhum
// =================================================================================================
void USBCBCheckOtherReq(void)
{
    USBCheckCDCRequest();
}

// =================================================================================================
// Função:     _USBCB_SOF_Handler
// Descrição:  O host USB envia um pacote SOF a dispositivos full-speed a cada 1 ms.
//             Esta interrupção pode ser útil para pipes isochronous. Desenvolvedores
//             devem implementar uma rotina de callback se necessário.
// Parâmetros: nenhum
// Retorno:    nenhum
// =================================================================================================
void USBCB_SOF_Handler(void)
{
    // Não há necessidade de definir como 0 o UIRbits.SOFIF aqui.
    // A função que chama este callback já faz isto.
}

// =================================================================================================
// Função:     _USBCBSuspend
// Descrição:  Este callback é chamado quando um USB suspend é detectado.
// Parâmetros: nenhum
// Retorno:    nenhum
// =================================================================================================
void USBCBSuspend(void)
{
    // Fazer os ajustes apropriados aqui para que o dispositivo entre em modo de baixo consumo.
}

// =================================================================================================
// Função:     _USBCBWakeFromSuspend
// Descrição:  O host pode colocar periféricos USB em modo de baixo consumo (ao "enviar" mais de
//					        3ms de idle). Assim que estiver no modo suspend, o host poderá acordar o dispositivo
//	  			            ao enviar uma sinalização de estado não-idle.
//				            Esta função de callback é chamada quando um "wakeup from USB suspend" é
//					        detectado.
// Parâmetros: nenhum
// Retorno:    nenhum
// =================================================================================================
void USBCBWakeFromSuspend(void)
{
    // Aqui deve-se reverter o modo de baixo consumo de energia.
}

// =================================================================================================
// Função:     _USBCBErrorHandler
// Descrição:  O objetivo desta função de callback é principalmente para depuração
//             durante a fase de desenvolvimento. Verifique UEIR para ver qual erro
//             causou a interrupção.
// Parâmetros: nenhum
// Retorno:    nenhum
// =================================================================================================
void USBCBErrorHandler(void)
{
}

// =================================================================================================
// Função:     _USER_USB_CALLBACK_EVENT_HANDLER
// Descrição:  Esta função é chamada pelo stack USB para notificar
//             a aplicação do usuário que um evento de USB ocorreu.
//             Esta função de callback está no contexto da interrupção
//             quando a opção USB_INTERRUPT é selecionada.
// Parâmetros: USB_EVENT event: o tipo de evento
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
// Função:     _wrEEPROM
// Descrição:  Escreve um byte na eeprom
// Parâmetros: unsigned char ucEnd: endereço a ser escrito.
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
// Função:     _rdEEPROM
// Descrição:  Lê um byte na eeprom
// Parâmetros: unsigned char ucEnd: endereço a ser lido
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
// Função:     _acionaBomba
// Descrição:  Faz o acionamento de uma bomba selecionada
// Parâmetros: unsigned char ucBomba: ID da bomba (1 a 4)
//             unsigned char estado: 1-ligado, 0-desligado
// Retorno:    nenhum
// =================================================================================================
void acionaBomba(unsigned char ucBomba, unsigned char estado)
{
    estado = (estado != 0) ? 1 : 0;    // Para evitar qualquer parâmetro errado.
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
// Função:     _leSensor
// Descrição:  Faz a leitura de um sensor selecionado para um buffer
// Parâmetros: unsigned char *buffer: buffer para receber a leitura
//             unsigned char ucSensor: sensor selecionado (1 a 4)
// Retorno:    1: leitura realizada
//             0: sensor selecionado não existe
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
// Função:     _trataComando
// Descrição:  Faz o tratamento de dados recebidos pela porta USB.
// Parâmetros: nenhum
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
// Função: _main
// Descrição: Função principal do programa.
//==================================================================================================
void main(void)
{
    unsigned int uiADCSelect=0;
    unsigned int uiTemp;
    unsigned int uiAux;
     
    INTCON = 0x00;
    RCON = 0x80;         // Habilita prioridades para interrupções
    ADCON1 = 0x0B;       // PORTA só como entradas e saídas digitais.
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
    PIE1 = 0x02;         // Habilita interrupção de Timer 2;
    INTCON2 = 0x00;
    INTCON = 0xC0;       // Liga interrupção de Timer 2.
 
    // Requisita a primeira conversão do A/D para que a leitura
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
