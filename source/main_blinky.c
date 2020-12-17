/*
 * Trabalho de ENGD33: Microcontrolador em Tempo Real para o Robô Husky
 * Por Gustavo Mota, Joao Carneiro, Osmar Oliveira
 *
 * Nosso projeto consiste na troca de mensagens entre microcontrolador e a
 * plataforma do Robô Husky por interface serial UART. Os pacotes recebidos e a
 * enviar são salvos em filas por tasks responsáveis por montar e desmontar
 * mensagens completas, salvas em outras filas. Outras tasks específicas para
 * cada tipo de mensagem recebem as mensagens completas destinadas a elas ou
 * enviam mensagens a serem encaminhadas para o serial.
 *
 * Foram implementadas essas quatro tasks e filas, alem de uma fila exclusiva
 * para as mensagens de Ack da plataforma. Alem disso, um firmware para tratar
 * do pressionamento de botões também foi implementado.
 */

/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* demo application includes. */
#include "partest.h"
#include "blocktim.h"
#include "flash_timer.h"
#include "semtest.h"
#include "genqtest.h"
#include "qpeek.h"
#include "lcd.h"
#include "timertest.h"
#include "intqueue.h"

/* Includes do projeto */
#include "Message.h"
#include "uart_isr.S"

/* Priorities at which the tasks are created. */
#define mainQUEUE_RECV_TASK_PRIORITY		( tskIDLE_PRIORITY )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY )
#define	mainUART_RECV_TASK_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define	mainUART_SEND_TASK_PRIORITY			( tskIDLE_PRIORITY + 1 )
#define	mainUART_RECV_TASK_PRIORITY			( tskIDLE_PRIORITY + 3 )

/* Prioridades das interrupcoes UART. */
#define mainUART_INTERRUPT_PRIORITY			( configKERNEL_INTERRUPT_PRIORITY + 1 )

/* Frequencia de acesso a filas, caso estejam vazias. O valor de 10ms e
convertido para ticks utilizando a constante portTICK_PERIOD_MS. */
#define mainQUEUE_ACCESS_FREQUENCY_MS		( 10 / portTICK_PERIOD_MS )

/* Numero de itens que podem ser guardados na fila. 1024 foi um numero escolhido
arbitrariamente para garantir certa margem de seguranca. */
#define mainQUEUE_LENGTH					( 1024 )

/* Taxa de baud desejada para comunicação UART. */
#define mainDESIRED_BAUD_RATE				( 115200 )

/* Mascara para retornar primeiro bit de um dado. */
#define mainFIRST_BIT_MASK					0x00000001

/*-----------------------------------------------------------*/

/*
 * Acessa fila de mensagens a enviar, pega uma mensagem, a divide em pacotes
 * UART para envio serial e os envia para a fila de pacotes a enviar. Aguarda
 * recebimento de ACK antes de enviar proxima mensagem.
 */
static void prvMessageSender( void *pvParameters );

/*
 * Acessa fila de pacotes UART e monta mensagem completa, então analisa seu
 * conteúdo e a encaminha para a tarefa responsavel pela acao necessaria.
 */
static void prvMessageReceiver( void *pvParameters );

/*
 * Configura registradores para o modulo UART funcionar como 115200 baud, 8
 * bits de data, sem paridade e 1 bit se parada.
 */
static void prvUartInit( void *pvParameters );

static void prvPooling( void *pvParameters );

/*
 * Handler de interrupcao do UART, que le da fila de chars a enviar e escreve
 * no buffer de envio, e le do buffer de recebimento para a fila de chars
 * recebidos.
 * Chama a funcao de receber mensagem quando detectar fim da
 * mensagem recebida para construir mensagem completa.
 */
void __attribute__( (interrupt(mainUART_INTERRUPT_PRIORITY), vector(_UART2_VECTOR))) vU2InterruptWrapper( void );

/*
 * Called by main() to create the simply blinky style application if
 * mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is set to 1.
 */
void main_husky( void );

/*-----------------------------------------------------------*/

/* Inicializa handles para as estruturas criadas a seguir. */
static QueueHandle_t xSendMessage = NULL;
static QueueHandle_t xRcvdMessage = NULL;
static QueueHandle_t xSendUART = NULL;
static QueueHandle_t xRcvdUART = NULL;
static QueueHandle_t xAck = NULL;

/*-----------------------------------------------------------*/

void main_husky( void )
{
	/* Cria as filas para armazenamento das mensagens a ler e enviar. */
	xSendMessage = xQueueCreate( mainQUEUE_LENGTH, sizeof( Message ) );
	xRcvdMessage = xQueueCreate( mainQUEUE_LENGTH, sizeof( Message ) );
	xSendUART = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint8_t ) );
	xRcvdUART = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint8_t ) );
	xAck = xQueueCreate( 1, sizeof( uint16_t ))

	/* Cria mutexes de acesso às filas. */
	xMutexSendMsg = xSemaphoreCreateMutex();
	xMutexRcvdMsg = xSemaphoreCreateMutex();
	xMutexSendUART = xSemaphoreCreateMutex();
	xMutexRcvdUART = xSemaphoreCreateMutex();
	xMutexAck = xSemaphoreCreateMutex();

	/* Configuracao inicial dos registradores UART. */
	prvUartInit();

	/* Cria task do receptor serial que salva mensagens recebidas em fila. */
	xTaskCreate( prvMessageReceiver,					/* The function that implements the task. */
				"Rx_Msg", 								/* The text name assigned to the task - for debug only as it is not used by the kernel. */
				configMINIMAL_STACK_SIZE,				/* The size of the stack to allocate to the task. */
				NULL,									/* The parameter passed to the task. */
				mainQUEUE_RECV_TASK_PRIORITY, 			/* The priority assigned to the task. */
				NULL );									/* The task handle is not required, so NULL is passed. */

	/* Cria task do transmissor serial que salva mensagens a enviar em fila. */
	xTaskCreate( prvMessageSender, "Tx_Msg", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL );

	/* Cria task do receptor UART. */
	xTaskCreate( prvUartReceiver, "Rx_UART", configMINIMAL_STACK_SIZE, NULL, mainUART_RECV_TASK_PRIORITY, NULL );

	/* Cria task do receptor UART. */
	xTaskCreate( prvUartSender, "Tx_UART", configMINIMAL_STACK_SIZE, NULL, mainUART_SEND_TASK_PRIORITY, NULL );
	
	/* Cria task do pooling dos botões */
	xTaskCreate( prvPooling, "Pooling", configMINIMAL_STACK_SIZE, NULL, mainUART_SEND_TASK_PRIORITY, NULL );


	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following
	line will never be reached.  If the following line does execute, then
	there was insufficient FreeRTOS heap memory available for the idle and/or
	timer tasks	to be created.  See the memory management section on the
	FreeRTOS web site for more details. */
	for( ;; );
}
/*-----------------------------------------------------------*/

static void prvPooling( void *pvParameters ) {
	Message xToSend;
	uint8_t data[64];
	
	
	for ( ;; ) {
	
		if (getXBitFromPortF(0)){
			// RF0 Vou colocar virar pra direita
			
			setMessage(&xToSend, 
		}
		if (getXBitFromPortF(1)){
			// RF1 Vou colocar virar pra esquerda
			
			
		}
		if (getXBitFromPortF(2)){
			// RF2
		}
		if (getXBitFromPortF(3)){
			// RF3
		}
		if (getXBitFromPortF(4)){
			// RA4
		}
		if (getXBitFromPortF(5)){
			// RA5
		}
		if (getXBitFromPortF(8)){
			// RA6
		}
	}
}



static void prvMessageSender( void *pvParameters )
{
uint8_t ucBadChecksum = 0;
uint16_t usAckResponse = NULL;
Message xToSend = NULL;

	for( ;; )
	{
		xToSend = NULL;

		while( xToSend == NULL)
		{
			/* Pega mutex para ler da pilha de mensagem a enviar */
			xSemaphoreTake( xMutexSendMsg, portMAX_DELAY );
			{
				xQueueReceive( xSendMessage, &( xToSend ), 0 );
			}
			xSemaphoreGive( xMutexSendMsg );
		}

		/* Rotina para enviar o dado */
		ucBadChecksum = 1;
		while(ucBadChecksum){
			/* Serializa conteudo da mensagem em uma array. */
			uint8_t ucMessageData[ 14 + xToSend.length ]:
			MessageSerialize( ucMessageData, xToSend );

			/* TODO: Fragmentação e envio da mensagem em protocolos UART */

			/* Pega mutex para acessar a pilha de ack e confirmar recebimento sem erro */
			while( usAckResponse == NULL ) {
				xSemaphoreTake( xMutexAck, portMAX_DELAY );
				{
					xQueueReceive( xAck, &( usAckResponse ), 0 );
				}
				xSemaphoreGive( xMutexAck );
			}

			/* Decodificao ack */
			/* TODO: Fazer ação para cada um desses caras */
			if(usAckResponse && mainFIRST_BIT_MASK) {
				/* Bad checksum -- Reenvia o dado */
				ucBadChecksum = 1;
			} else {
				ucBadChecksum = 0;
			}
			if((usAckResponse >> 1) && mainFIRST_BIT_MASK) {
				/* Type not supported -- Verificar o que foi enviado */
			}
			if((usAckResponse >> 2) && mainFIRST_BIT_MASK) {
				/* Bad format -- Verificar o que foi enviado */
			}
			if((usAckResponse >> 3) && mainFIRST_BIT_MASK) {
				/* Out of Range */
			}
			if((usAckResponse >> 4) && mainFIRST_BIT_MASK) {
				/* No bandwidth */
			}
			if((usAckResponse >> 5) && mainFIRST_BIT_MASK) {
				/* Frequency too high */
			}
			if((usAckResponse >> 6) && mainFIRST_BIT_MASK) {
				/* Too many message types */
			}
		}
	}
}
/*-----------------------------------------------------------*/

static void prvMessageReceiver( void *pvParameters )
{
uint8_t ucInvalidMessage;
uint8_t ucMsgLen;
uint8_t *ucRcvdData;
uint16_t usTesteChecksum;
uint16_t usMessageType;
uint32_t ulReceivedValue;
Message xMessageReceived;

	for( ;; )
	{
		/* Adquire mutex para ler da fila de bytes recebidos. */
		xSemaphoreTake( xMutexRcvdUART, portMAX_DELAY );
		{
			/* Itera pela mensagem ate preenchar o STX (tamanho fixo). */
			for( int i = 0; i < 12; i++ )
			{
				xQueueReceive( xRcvdUART, &( ucRcvdData[i] ), 0 );
			}

			/* Itera pelo comprimento restante da mensagem preenchendo o payload. */
			ucMsgLen = ucRcvdData[1];
			for( int i = 12; i < ucMsgLen ; i++ )
			{
				xQueueReceive( xRcvdUART, &( ucRcvdData[i] ), 0 );
			}
			xQueueReceive( xRcvdUART, &( ucRcvdData[ucMsgLen] ), 0 );

			xQueueReceive( xRcvdUART, &( ucRcvdData[ucMsgLen + 1] ), 0 );
		}
		xSemaphoreGive( xMutexRcvdUART );

		/* Monta mensagem a partir dos dados coletados. */
		xMessageReceived.soh = ucRcvdData[0];
		xMessageReceived.length = ucRcvdData[1];
		xMessageReceived.lengthCompliment = ucRcvdData[2];
		xMessageReceived.version = ucRcvdData[3];
		(uint8_t)(xMessageReceived.timeStamp) = ucRcvdData[4];
		(uint8_t)(xMessageReceived.timeStamp >> 8) = ucRcvdData[5];
		(uint8_t)(xMessageReceived.timeStamp >> 16) = ucRcvdData[6];
		(uint8_t)(xMessageReceived.timeStamp >> 24) = ucRcvdData[7];
		xMessageReceived.flags = ucRcvdData[8];
		(uint8_t)(xMessageReceived.timeStamp) = ucRcvdData[9];
		(uint8_t)(xMessageReceived.timeStamp >> 8) = ucRcvdData[10];
		xMessageReceived.stx = ucRcvdData[11];

		for( int i = 12; i < ucMsgLen ; i++ )
		{
			xMessageReceived.payload[i-12] = ucRcvdData[i];
		}

		(uint8_t)(xMessageReceived.checksum) = ucRcvdData[ucMsgLen];
		(uint8_t)(xMessageReceived.checksum >> 8) = ucRcvdData[ucMsgLen];


		/* Confere integridade da mensagem. */
		usTesteChecksum = getChecksum(xMessageReceived);
		if( xMessageReceived.soh != 0xAA )
			ucInvalidMessage = 1;
		else if( xMessageReceived.lengthCompliment != 0xFF - xMessageReceived.length )
			ucInvalidMessage = 1;
		else if( xMessageReceived.version != 0x01 )
			ucInvalidMessage = 1;
		else if( xMessageReceived.stx != 0x55 )
			ucInvalidMessage = 1;
		else if( xMessageReceived.checksum != usTesteChecksum )
			ucInvalidMessage = 1;
		else
			ucInvalidMessage = 0;

		/* TODO: Age sobre recebimento de mensagem inválida enviada pela plataforma. */
		if( ucInvalidMessage )
		{
		}

		/* TODO: Apura tipo da mensagem recebida e trata ou encaminha para a task necessária. */
		usMessageType = xMessageReceived.messageType;

		/* Se a mensagem não for de tipo Data, tem que ser um Ack em resposta a
		Requisições ou Comandos. */
		if( usMessageType < 0x8000)
		{
			/* Adquire mutex para escrever na fila de Ack */
			xSemaphoreTake( xMutexAck, portMAX_DELAY );
			{
				/* Envia pra fila de Ack */
				xQueueSend(xAck, (void) &xMessageReceived, 0 );
			}
			xSemaphoreGive( xMutexAck );
		}

		/* Caso seja um Echo Data. */
		if( usMessageType == 0x8000 )
		{
			/* TODO */
		}

		/* Caso seja Platform info. */
		if( usMessageType == 0x8001 )
			/* TODO */
		}

		/* TODO: tratar todos os tipos de mensagens de Data. */
	}
}
/*-----------------------------------------------------------*/

static void prvUartInit( void *pvParameters )
{
uint16_t usBRG = NULL;								/* Valor baud UART a ser passado ao registrador. */
uint32_t ulWantedBaud = NULL;						/* Valor baud desejado. */

	/* Calcula taxa a ser passada ao registrador U1BRG. */
	ulWantedBaud = mainDESIRED_BAUD_RATE;			/* Indica taxa baud desejada. */
	usBRG = (unsigned short)(( (float)configPERIPHERAL_CLOCK_HZ / ( (float)16 * (float)ulWantedBaud ) ) - (float)0.5);
	U2BRGbits.BRG = usBRG;							/* Inicializa taxa de baud apropriada. */

	U2MODEbits.PDSEL = 0;							/* 8 bits de dados, 0 de paridade. */
	U2MODEbits.STSEL = 0;							/* 1 bit de parada. */

	IEC1bits.U2TXIE = 1;							/* Habilita interrupcao do transmissor. */
	U2STAbits.UTXISEL0 = 0							/* Interrupcao gerada quando ha espaco no buffer. */

	IEC1bits.U2RXIE = 1;							/* Habilita interrupcao do receptor. */
	U2STAbits.URXISEL = 0							/* Interrupcao gerada quando chega byte no buffer. */

	IPC8bits.U2IP = mainUART_INTERRUPT_PRIORITY;	/* Define prioridade de interrupcao. */
	IPC8bits.U2IS = mainUART_INTERRUPT_PRIORITY;	/* Define subprioridade de interrupcao. */

	U2MODEbits.ON = 1;								/* Habilita modulo UART. */
	U2STAbits.URXEN = 1;							/* Habilita pino U1RX para leitura. */
	U2STAbits.UTXEN = 1;							/* Habilita pino U1TX para escrita. */
}
/*-----------------------------------------------------------*/

void vU2InterruptHandler( void )
{
/* Declared static to minimise stack use. */
static char cChar;
static uint8_t ucCharCount = 0;
static uint8_t ucMsgEnd = 255;
static portBASE_TYPE xHigherPriorityTaskWoken;

	xHigherPriorityTaskWoken = pdFALSE;

	/* Are any Rx interrupts pending? */
	if( IFS1bits.U2RXIF == 1)
	{
		while( U2STAbits.RXDA )
		{
			/* Retrieve the received character and place it in the queue of
			received characters. */
			cChar = U2RXREG;
			xQueueSendFromISR( xRcvdUART, &cChar, &xHigherPriorityTaskWoken );

			/* Detecta se foi transmitido o começo de uma mensagem. */
			if( cChar == 0xAA )
			{
				ucCharCount = 0;
			}

			/* Calcula fim da mensagem a partir do comprimento. */
			if( ucCharCount == 1 )
			{
				ucMsgEnd = 12 + cChar;
			}

			/* Se tiver chegado ao fim da mensagem, ativa tarefa para montar ela. */
			if( ucCharCount == ucMsgEnd )
			{
				prvMessageReceiver();
				ucMsgEnd = 255;
			}

			/* Incrementa contador de bytes. */
			ucCharCount += 1;
		}

		IFS1bits.U2RXIF = 0;
	}

	/* Are any Tx interrupts pending? */
	if( IFS1bits.U2TXIF == 1 )
	{
		while( ( U2STAbits.UTXBF ) == 0 )
		{
			if( xQueueReceiveFromISR( xSendUART, &cChar, &xHigherPriorityTaskWoken ) == pdTRUE )
			{
				/* Send the next character queued for Tx. */
				U2TXREG = cChar;
			}
			else
			{
				/* Queue empty, nothing to send. */
				xTxHasEnded = pdTRUE;
				break;
			}
		}
		IFS1bits.U2TXIF = 0;
	}

	/* If sending or receiving necessitates a context switch, then switch now. */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}