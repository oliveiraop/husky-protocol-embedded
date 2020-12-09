/*
 * FreeRTOS V202011.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/******************************************************************************
 * NOTE 1:  This project provides two demo applications.  A simple blinky style
 * project, and a more comprehensive test and demo application.  The
 * mainCREATE_SIMPLE_BLINKY_DEMO_ONLY setting in main.c is used to select
 * between the two.  See the notes on using mainCREATE_SIMPLE_BLINKY_DEMO_ONLY
 * in main.c.  This file implements the simply blinky style version.
 *
 * NOTE 2:  This file only contains the source code that is specific to the
 * basic demo.  Generic functions, such FreeRTOS hook functions, and functions
 * required to configure the hardware, are defined in main.c.
 ******************************************************************************
 *
 * main_blinky() creates one queue, two tasks, and one software timer.  It then
 * starts the scheduler.
 *
 * The Blinky Software Timer:
 * This demonstrates an auto-reload software timer.  The timer callback function
 * does nothing but toggle an LED.
 *
 * The Queue Send Task:
 * The queue send task is implemented by the prvQueueSendTask() function in
 * this file.  prvQueueSendTask() sits in a loop that causes it to repeatedly
 * block for 200 milliseconds, before sending the value 100 to the queue that
 * was created within main_blinky().  Once the value is sent, the task loops
 * back around to block for another 200 milliseconds.
 *
 * The Queue Receive Task:
 * The queue receive task is implemented by the prvQueueReceiveTask() function
 * in this file.  prvQueueReceiveTask() sits in a loop where it repeatedly
 * blocks on attempts to read data from the queue that was created within
 * main_blinky().  When data is received, the task checks the value of the
 * data, and if the value equals the expected 100, toggles the LED.  The 'block
 * time' parameter passed to the queue receive function specifies that the
 * task should be held in the Blocked state indefinitely to wait for data to
 * be available on the queue.  The queue receive task will only leave the
 * Blocked state when the queue send task writes to the queue.  As the queue
 * send task writes to the queue every 200 milliseconds, the queue receive
 * task leaves the Blocked state every 200 milliseconds, and therefore toggles
 * the LED every 200 milliseconds.
 */

/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Standard demo includes. */
#include "partest.h"

/* Includes do projeto */
#include "Message.h"

/* Priorities at which the tasks are created. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

/* The rate at which data is sent to the queue.  The 200ms value is converted
to ticks using the portTICK_PERIOD_MS constant. */
#define mainQUEUE_SEND_FREQUENCY_MS			( 200 / portTICK_PERIOD_MS )

/* Numero de itens que podem ser guardados na fila. 10 foi um numero escolhido
 * arbitrariamente para garantir certa margem de seguranca. */
#define mainQUEUE_LENGTH					( 10 )

/* The period of the blinky software timer.  The period is specified in ms and
converted to ticks using the portTICK_PERIOD_MS constant. */
#define mainBLINKY_TIMER_PERIOD				( 50 / portTICK_PERIOD_MS )

/* The LED used by the communicating tasks and the blinky timer respectively. */
#define mainTASKS_LED						( 0 )
#define mainTIMER_LED						( 1 )

/* Misc. */
#define mainDONT_BLOCK						( 0 )

/* Mascara para retornar primeiro bit de um dado. */
#define mainFIRST_BIT_MASK					0x00000001

/*-----------------------------------------------------------*/

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvQueueReceiveTask( void *pvParameters );
static void prvQueueSendTask( void *pvParameters );

/*
 * The callback function for the blinky software timer, as described at the top
 * of this file.
 */
static void prvBlinkyTimerCallback( TimerHandle_t xTimer );

/*
 * Called by main() to create the simply blinky style application if
 * mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is set to 1.
 */
void main_blinky( void );

/*-----------------------------------------------------------*/

/* Inicializa handles para as estruturas criadas a seguir. */
static QueueHandle_t xSendMessage = NULL;
static QueueHandle_t xRcvdMessage = NULL;

/*-----------------------------------------------------------*/

void main_blinky( void )
{
	TimerHandle_t xTimer;

	/* Criar as estruturas:
	 * Fila para mensagens a enviar; 
	 * Fila para mensagens recebidas. */
	xSendMessage = xQueueCreate( mainQUEUE_LENGTH, sizeof( Message ) );
	xRcvdMessage = xQueueCreate( mainQUEUE_LENGTH, sizeof( Message ) );

	if( xSendMessage != NULL )
	{
		/* Cria task do receptor serial que salva mensagens recebidas em fila. */
		xTaskCreate( prvSerialReceiver,						/* The function that implements the task. */
					"Rx", 									/* The text name assigned to the task - for debug only as it is not used by the kernel. */
					configMINIMAL_STACK_SIZE, 				/* The size of the stack to allocate to the task. */
					NULL									/* The parameter passed to the task. */
					mainQUEUE_RECEIVE_TASK_PRIORITY, 		/* The priority assigned to the task. */
					NULL );									/* The task handle is not required, so NULL is passed. */

		/* Cria task do transmissor serial que salva mensagens a enviar em fila. */
		xTaskCreate( prvSerialSender, "TX", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL );

		/* create the blinky software timer as described at the top of this
		file. */
		xtimer = xtimercreate( 	"blinky",					/* a text name, purely to help debugging. */
								( mainblinky_timer_period ),/* the timer period. */
								pdtrue,						/* this is an auto-reload timer, so xautoreload is set to pdtrue. */
								( void * ) 0,				/* the id is not used, so can be set to anything. */
								prvblinkytimercallback		/* the callback function that inspects the status of all the other tasks. */
							);

		if( xtimer != null )
		{
			xtimerstart( xtimer, maindont_block );
		}


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
TickType_t xNextWakeTime;
Message xToSend = NULL;
uint16_t = usAckResponse;


	/* Initialise xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();

	for( ;; )
	{
		/* Place this task in the blocked state until it is time to run again.
		The block time is specified in ticks, the constant used converts ticks
		to ms.  While in the Blocked state this task will not consume any CPU
		time. */
		vTaskDelayUntil( &xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS );

		xToSend = NULL;
		xQueueReceive(xSendMessage, &( xToSend ), 0);
		
		if (xToSend != NULL) {
			// rotina para enviar o dado
			// PEGA MUTEX SE NECESSARIO AQUI
			
			// Adicionar os valores nessa mesma ordem dentro das fun��es de envio para porta serial
			xToSend.soh;
			xToSend.length;
			xToSend.lenghtCompliment;
			xToSend.version;
			(uint8_t)xToSend.timeStamp;	
			(uint8_t)(xToSend.timeStamp >> 8);
			(uint8_t)(xToSend.timeStamp >> 16);
			(uint8_t)(xToSend.timeStamp >> 24);
			xToSend.flags;
			(uint8_t)xToSend.messageType;
			(uint8_t)(xToSend.messageType >> 8);
			xToSend.stx;
		
			for (int i = 0; i < xToSend.length ; i++) {
				xToSend.payload[i];
			}
			(uint8_t)(xToSend.checksum >> 8);
			(uint8_t)xToSend.checksum;
			
			// Entrega mutex de volta, mensagem enviada
			// Aguarda ack
			
			xQueueReceive(xAck, &( xAckResponse ), 30);
			
			// decodifica ack
			// Fazer a��o para cada um desses caras 
			//(podemos dar um jeito de fazer um log pra poder verificar se isso est� funcionando, ou ligar leds)
			if (xAckResponse && mainFIRST_BIT_MASK) {
				// Bad checksum -- Reenvia o dado
			}
			if ((xAckResponse >> 1) && mainFIRST_BIT_MASK) {
				//Type not supported -- Verificar o que foi enviado
			}
			if ((xAckResponse >> 2) && mainFIRST_BIT_MASK) {
				// Bad format -- Verificar o que foi enviado
			}
			if ((xAckResponse >> 3) && mainFIRST_BIT_MASK) {
				// Out of Range
			}
			if ((xAckResponse >> 4) && mainFIRST_BIT_MASK) {
				// No bandwidth
			}
			if ((xAckResponse >> 5) && mainFIRST_BIT_MASK) {
				// Frequency too high
			}
			if ((xAckResponse >> 6) && mainFIRST_BIT_MASK) {
				// Too many message types
			}
		}
	}
}
/*-----------------------------------------------------------*/

static void prvSerialReceiver( void *pvParameters )
{
unsigned long ulReceivedValue;
Message xMessageReceived;
uint8_t xCharReceived;
uint16_t xTesteChecksum;

	for( ;; )
	{
		//xSerialGetChar
		//Receber o char
		
		if (xMessageReceived.soh == xCharReceived) {
			//xSerialGetChar
			xMessageReceived.length = xCharReceived;
			xMessageReceived.lengthCompliment = xCharReceived;
			// Testa o complemento
			if (xMessageReceived.length + xMessageReceived.lengthCompliment = 0xFFFF){
				//xSerialGetChar
				if (xMessageReceived.version == xCharReceived) {
					// timestamp
					//flags
					//messageType (talvez pode ser colocado aqui o que vai ser feito por causa do ack, ou ficar de tarefa pra outra task)
					// stx
					if(xMessageReceived.stx == xCharReceived){
						//receber payload definido na length (um for mesmo)
						// Depois receber checksum e usar fun��o para testar com os dados que j� tem
						xTesteChecksum = getChecksum(xMessageReceived);
						if (xTesteChecksum == xMessageReceived.checksum) {
							// Finaliza parser e envia a mensagem para ser utilizada em outra fila de task
							if (xMessageReceived.messageType < 0x8000) {
								// isso � um ack, colocar na fila de ack
								xQueueSend(xAck, (void) &xMessageReceived, 0 ); // enviar pra fila de Ack
							} else {
								//Se n�o for vai ser um dado que pode ser colocado em uma fila para ser utilizado por outra task
							}
							
						} // checksum deu falha aqui;
					} // else aqui caso o stx venha diferente do esperado
				} // else aqui de erro caso a versao esteja incorreta
			} // else aqui caso o tamanho esteja incorreto
		}
	}
}
/*-----------------------------------------------------------*/

static void prvBlinkyTimerCallback( TimerHandle_t xTimer )
{
	/* This function is called when the blinky software time expires.  All the
	function does is toggle the LED.  LED mainTIMER_LED should therefore toggle
	with the period set by mainBLINKY_TIMER_PERIOD. */
	vParTestToggleLED( mainTIMER_LED );
}

