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

/* Priorities at which the tasks are created. */
#define mainSEM_TEST_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY )

/* Frequencia de acesso a filas, caso estejam vazias. O valor de 10ms e
convertido para ticks utilizando a constante portTICK_PERIOD_MS. */
#define mainQUEUE_ACCESS_FREQUENCY_MS		( 10 / portTICK_PERIOD_MS )

/* Numero de itens que podem ser guardados na fila. 1000 foi um numero escolhido
arbitrariamente para garantir certa margem de seguranca. */
#define mainQUEUE_LENGTH					( 1000 )

/* Misc. */
#define mainDONT_BLOCK						( 0 )

/* Mascara para retornar primeiro bit de um dado. */
#define mainFIRST_BIT_MASK					0x00000001

/*-----------------------------------------------------------*/

/*
 * Acessa fila de mensagens a enviar, pega uma mensagem, a divide em pacotes
 * UART para envio serial e os envia para a fila de pacotes a enviar. Aguarda
 * recebimento de ACK antes de enviar proxima mensagem.
 */
static void prvSerialSender( void *pvParameters );

/*
 * Acessa fila de pacotes UART e monta mensagem completa, então analisa seu
 * conteúdo e a encaminha para a tarefa responsavel pela acao necessaria.
 */
static void prvSerialReceiver( void *pvParameters );

/*
 * Called by main() to create the simply blinky style application if
 * mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is set to 1.
 */
void main_husky( void );

/*-----------------------------------------------------------*/

/* Inicializa handles para as estruturas criadas a seguir. */
static QueueHandle_t xSendMessage = NULL;
static QueueHandle_t xRcvdMessage = NULL;
static QueueHandle_t xAck = NULL;

/*-----------------------------------------------------------*/

void main_husky( void )
{
	TimerHandle_t xTimer;

	/* Cria as estruturas:
	 * Fila para mensagens a enviar;
	 * Fila para mensagens recebidas. */
	xSendMessage = xQueueCreate( mainQUEUE_LENGTH, sizeof( Message ) );
	xRcvdMessage = xQueueCreate( mainQUEUE_LENGTH, sizeof( Message ) );
	xAck = xQueueCreate( 1, sizeof( uint16_t ))

	/* Cria mutexes de acesso às estruturas. */
	xMutexSend = xSemaphoreCreateMutex();
	xMutexRcvd = xSemaphoreCreateMutex();
	xMutexAck = xSemaphoreCreateMutex();

	if( xSendMessage != NULL )
	{
		/* Cria task do receptor serial que salva mensagens recebidas em fila. */
		xTaskCreate( prvSerialReceiver,						/* The function that implements the task. */
					"Rx", 									/* The text name assigned to the task - for debug only as it is not used by the kernel. */
					configMINIMAL_STACK_SIZE,				/* The size of the stack to allocate to the task. */
					NULL,									/* The parameter passed to the task. */
					mainQUEUE_RECEIVE_TASK_PRIORITY, 		/* The priority assigned to the task. */
					NULL );									/* The task handle is not required, so NULL is passed. */

		/* Cria task do transmissor serial que salva mensagens a enviar em fila. */
		xTaskCreate( prvSerialSender, "Tx", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL );

		/* Start the tasks and timer running. */
		vTaskStartScheduler();
	}

	/* If all is well, the scheduler will now be running, and the following
	line will never be reached.  If the following line does execute, then
	there was insufficient FreeRTOS heap memory available for the idle and/or
	timer tasks	to be created.  See the memory management section on the
	FreeRTOS web site for more details. */
	for( ;; );
}
/*-----------------------------------------------------------*/

static void prvSerialSender( void *pvParameters )
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
			xSemaphoreTake( xMutexSend, portMAX_DELAY );
			{
				xQueueReceive( xSendMessage, &( xToSend ), 0 );
			}
			xSemaphoreGive( xMutexSend );
		}

		/* Rotina para enviar o dado */
		ucBadChecksum = 1;
		while(ucBadChecksum){
			/* Serializa conteudo da mensagem em uma array. */
			uint8_t ucMessageData[ 12 + xToSend.length ]:
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

static void prvSerialReceiver( void *pvParameters )
{
uint8_t ucInvalidMessage;
uint16_t usTesteChecksum;
uint32_t ulReceivedValue;
Message xMessageReceived;

	for( ;; )
	{
		/* TODO: Coleta pacotes UART na fila e junta os fragmentos da mensagem. */
		/* xSerialGetChar */

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

		/* TODO: Apura tipo da mensagem recebida e trata ou encaminha para a task necessária. */
		switch( xMessageReceived.messageType )
		{
			/* Caso seja um Ack, por exemplo: */
			case xMessageReceived.messageType < 0x8000:
				/* Coloca na fila de ack. */
				xSemaphoreTake( xMutexAck, portMAX_DELAY );
				{
					/* Envia pra fila de Ack */
					xQueueSend(xAck, (void) &xMessageReceived, 0 );
				}
				xSemaphoreGive( xMutexAck );

				/* Chama tarefa específica para o tipo de mensagem. */
				/* TODO */

			default:
			}
		}
	}
}