/**
  ******************************************************************************
  * @file    main.c
  * @author  Moe2Code
  * @version V1.0
  * @date    19-Dec-2020
  * @brief   This is an embedded application running on top of FreeRTOS. Nucleo
  * 		 STM32F446RE is the board of choice here. This application will run
  * 		 a main menu which will prompt the user over a serial monitor to select
  * 		 to:
  * 		 - Display and change time and date; set a daily alarm if needed
  * 		 - Play guess-a-number game
  * 		 - Run an integers calculator
  * 		 - Toggle an LED on the Nucleo board
  * 		 - Run a temperature monitor in the background to track current,
  * 		   highest, and lowest ambient temperatures
  * 		 - Put the application to sleep and wait for a user interrupt
  ******************************************************************************
*/

// INCLUDES

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

// CONSTANTS

// Invalid Number
#define INVALID_NUM 				(int32_t)0xFFFFFFFF

// Applications to choose from
#define RUN_CLOCK					1
#define RUN_GAME					2
#define RUN_CALCULATOR 				3
#define MONITOR_TEMP				4
#define TOGGLE_LED					5
#define SLEEP						6

// APPLICATION GLOBALS

// Task handles
TaskHandle_t xUartWriteTaskHandle = NULL;
TaskHandle_t xMainMenuTaskHandle = NULL;
TaskHandle_t xClockTaskHandle = NULL;
TaskHandle_t xGameTaskHandle = NULL;
TaskHandle_t xCalculatorTaskHandle = NULL;
TaskHandle_t xTempMonitorTaskHandle = NULL;

// Timer handle to toggle LED
TimerHandle_t pxLedToggleTimer = NULL;

// Queue Handles
// Queue to write to UART
QueueHandle_t xUartWriteQueue = NULL;

// ADC init struct used to initialize ADC1 for the
// purpose of measuring the internal temp sensor
// For some reason it needs to be in global space
ADC_InitTypeDef xAdcInit;

// Flag to go to sleep
BaseType_t xGoToSleep = pdFALSE;

// Flag to show temp stats if requested by the user
BaseType_t xShowTemps = pdFALSE;

// Flag set by the user to run temp monitoring
BaseType_t xRunTempMonitor = pdFALSE;

// Menu to display to user of application
char* pcMenu = "\
\r\n===============================================\
\r\nThis is a general FreeRTOS Application\
\r\nPress the letter Q (or q) and the return key\
\r\nto return to the main menu below any time\
\r\n=================MAIN MENU=====================\
\r\nSelect one of the sub-applications below to run\
\r\nTime and Alarms					----> 1\
\r\nGuess-A-Number Game		                ----> 2\
\r\nCalculator				        ----> 3\
\r\nMonitor temperature				----> 4\
\r\nToggle LED				        ----> 5\
\r\nSleep and Wait for Interrupt			----> 6\
\r\nType your option: ";

// FUNCTION PROTOTYPES

// Task handler prototypes
void vUartWriteTaskFunction(void *pvParam);
void vMainMenuTaskFunction(void *pvParam);
void vClockTaskFunction(void *pvParam);
void vGameTaskFunction(void *pvParam);
void vCalculatorTaskFunction(void *pvParam);
void vTempMonitorTaskFunction(void *pvPram);

// To toggle the green LED on Nucleo board
void vLedToggle(TimerHandle_t xTimer);

// To setup the MCU
static void vSetupHardware(void);

// To setup UART communication
static void vUartSetup(void);

// To setup RTC peripheral
static void vRtcSetup(void);

// To setup GPIO pins for LED
static void vGpioSetup(void);

// To setup the ADC to use for analog temperature measurement
static void vAdcSetup(void);

// To send UART messages to a terminal
static void vSendUartMsg(char *msg);

// To receive UART messages from a terminal
static BaseType_t xReceiveUartMsg(char* pcMsgBuffer, BaseType_t* pxQuitCurrentApp);

// To send error messages to UART terminal
static void vPostMsgToUartQueue(char* pcUartMsg);

// To acquire from RTC the current date and time and send them to UART terminal
static void vReadRtcDateTime(void);

// To convert the number received via UART to INT32 number
static int32_t lUartMsgtoInt32(char* pcUartMsg);

// To manage toggling the green LED of the Nucleo board
static void vManageLedToggle(void);

// To enable toggling the green LED on the Nucleo board
static void vLedToggleEnable(uint32_t ulToggleDuration);

// To disable toggling the green LED on the Nucleo board
static void vLedToggleDisable(void);

// To set an alarm by using the RTC peripheral
static void vSetAlarm( BaseType_t* xQuitCurrentApp );

// To configure the user desired date and time in the RTC peripheral
static void vSetDateAndTime( BaseType_t* pxQuitCurrentApp );

// To enter and leave sleep mode for this application
static void vManageAppSleep(void);

// To measure actual VDDA using VRefInt in order to have better temp readings
static float fMeasureVDDA(void);

// To measure the temperature using the internal temperature sensor on Nucleo board
static float fMeasureTemp(void);

// To manage user selections for temp monitor
static void vManageTempMonitor(void);
/*******************************************************************************
*   Procedure: main
*
*   Description: This is the main function for this general application. It
*   			- Calls various peripheral initialization functions
*   			- Performs Segger SystemView initialization to be able to get a
*   			  trace of the application
*   			- Creates a queue in order to serialize message transmission via
*   			  UART2
*   			- Creates the application tasks
*   			- Initializes random seed into rand()
*
*   Notes: None
*
*   Parameters: None
*
*   Return: None - It should never return
*
*******************************************************************************/
int main(void)
{
	// Cycle Count is needed to record the time stamp of a trace
	// Enable CYCCNT (Cycle Count) in DWT Control Register of ARM Cortex M4 processor
	// DWT stands for Data Watch and Trace
	DWT->CTRL |= (1 << 0);

	// FreeRTOS automatically configures the MCU to run at 180 MHz via PLL engine
	// That is not necessary for this application
	// Reset the RCC clock configuration to the default reset state
	// The default reset state will make the MCU run using HSI at 16 MHz
	// By looking at the clock tree you will see the System clock and CPU clock will be 16 MHz
	RCC_DeInit();

	// Update the SystemCoreClock variable
	// Each time the core clock (HCLK) changes, the below function must be called
	// to update SystemCoreClock variable value. Otherwise, any configuration based
	// on this variable will be incorrect.
	SystemCoreClockUpdate();

	// Set up various peripherals used in this RTA
	vSetupHardware();

	// Start recording trace to analyze via SEGGER SystemView
	SEGGER_SYSVIEW_Conf();
	// SEGGER SystemView events recording starts only when the below API is called
	SEGGER_SYSVIEW_Start();

	// Create queue of 10 elements to write to UART
	xUartWriteQueue = xQueueCreate(10, sizeof(char*));

	if( xUartWriteQueue != NULL )
	{
		// Create the application tasks
		// 0 is idle priority. Anything higher than that (e.g. 1) is non-idle-priority task
		// FreeRTOS APIs will now be used from the task handlers. Thus they will consume more
		// task memory. Therefore it's better to increase the task's private stack to 500 words
		xTaskCreate( vUartWriteTaskFunction, "UART_WRITE_TASK", 500, NULL, 2, &xUartWriteTaskHandle );
		xTaskCreate( vMainMenuTaskFunction, "MAIN_MENU_TASK", 500, NULL, 1, &xMainMenuTaskHandle );
		xTaskCreate( vClockTaskFunction, "CLOCK_TASK", 500, NULL, 1, &xClockTaskHandle);
		xTaskCreate( vGameTaskFunction, "GAME_TASK", 500, NULL, 1, &xGameTaskHandle );
		xTaskCreate( vCalculatorTaskFunction, "CALCULATOR_TASK", 500, NULL, 1, &xCalculatorTaskHandle);
		xTaskCreate( vTempMonitorTaskFunction, "TEMP_MONITOR_TASK", 500, NULL, 1, &xTempMonitorTaskHandle);

		// Start the scheduler in order to run the tasks
		vTaskStartScheduler();
	}
	else
	{
		vSendUartMsg("Queue creation failed\r\n");
	}

	// You will never return here unless there is an error
	for(;;);
}
/*******************************************************************************
*   Procedure: vUartWriteTaskFunction
*
*   Description: This is the task function for the UART Write Task. It supports other
*   			 tasks by serializing message transmission to UART2. This helps avoid
*   			 race condition to use UART2. A message will first be posted to the
*   			 xUartWriteQueue. The UART Write Task will then be moved from blocked
*   			 state to ready state and eventually it will run. Afterwards, this task
*   			 function will run and the posted message will be de-queued and transmitted
*   			 via UART2.
*
*   Notes: None
*
*   Parameters: pvParam - A pointer to data passed during task creation. It is not used
*   			in this task function
*
*   Return: None
*
*******************************************************************************/
void vUartWriteTaskFunction(void *pvParam)
{
	char* pcData = NULL;		// To hold the address of the data received

	// Task handler should always be executing
	while(1)
	{
		// Receive an item from the UART write queue
		// The task will block waiting indefinitely till an item becomes available on the queue to receive
		xQueueReceive( xUartWriteQueue, &pcData, portMAX_DELAY );

		// Print the data pointed to on terminal window using UART
		vSendUartMsg(pcData);
	}
}
/*******************************************************************************
*   Procedure: vMainMenuTaskFunction
*
*   Description: This is the task function for the Main Menu task. The Main Menu task
*                is the first application task to run and notifies other tasks such
*                as Clock task, Game task, Calculator task, and Temperature Monitor task
*                if the user selects to run one of them when prompted for an input. The
*                Main Menu task will notify one of these tasks to run and wait for a
*                notification back from the unblocked task. This does not apply to the
*                temperature monitor task which will run in the background once notified.
*                This will allow the user to run more tasks to run via Main Menu task.
*                The Main Menu task also allows the user to toggle the green LED on board
*                or send the application to normal sleep mode. Toggling the LED is handled
*                by the Timer Service task which runs in the background.
*
*                If the user does not provide his/her input when prompted within 30 seconds
*                then the whole operation will re-start by prompting the user to select one
*                of the main menu options. Similarly if the user does not provide a valid
*                input or presses the letter q/Q followed by the return key then the above
*                sequence will repeat.
*
*   Notes: None
*
*   Parameters: pvParam - A pointer to data passed during task creation. It is not used
*   			in this task function
*
*   Return: None
*
*******************************************************************************/
void vMainMenuTaskFunction(void *pvParam)
{
	char cUartMsg[50] = {0};			   // Buffer to hold the message received via UART
	uint8_t ucAppSelected = 0;			   // To hold the application selected by the user
	BaseType_t xQuitCurrentApp = pdFALSE;  // Flag to indicate if the user requested to quit the application
	BaseType_t xReadSuccess = pdFALSE;	   // Flag to indicate if reading user's input via UART was successful

	while(1)
	{
		// Reset the flags
		xQuitCurrentApp = pdFALSE;
		xReadSuccess = pdFALSE;

		// Print the Main Menu on the UART window
		// Push a pointer to the data (i.e. main menu string) into the UART write queue
		// The task will block waiting indefinitely till space becomes available on the queue
		xQueueSend( xUartWriteQueue, &pcMenu, portMAX_DELAY );

		// Zeroing the message buffer
		memset(&cUartMsg, 0, sizeof(cUartMsg));
		// Receive user's input for the selected app to run
		xReadSuccess = xReceiveUartMsg(cUartMsg, &xQuitCurrentApp);

		// If the UART read was successful and the user did not request to quit the application
		if( xReadSuccess == pdTRUE && xQuitCurrentApp == pdFALSE)
		{
			// Return the first byte from the buffer, which will contain the user's selection
			// 48 is subtracted to convert from ASCII to a number
			ucAppSelected = cUartMsg[0] - 48;

			switch( ucAppSelected )
			{
				case RUN_CLOCK:

					// The user has requested to run the clock sub-application
					// Notify the clock task to run it
					xTaskNotify( xClockTaskHandle, 0, eNoAction );

					// Wait in blocked state indefinitely till a notification is received
					xTaskNotifyWait( 0, 0, NULL, portMAX_DELAY);
					break;

				case RUN_GAME:

					// The user has requested to run the game sub-application
					// Notify the game task to run it
					xTaskNotify( xGameTaskHandle, 0, eNoAction );

					// Wait in blocked state indefinitely till a notification is received
					xTaskNotifyWait( 0, 0, NULL, portMAX_DELAY);
					break;

				case RUN_CALCULATOR:

					// The user has requested to run the calculator sub-application
					// Notify the calculator task to run it
					xTaskNotify( xCalculatorTaskHandle, 0, eNoAction );

					// Wait in blocked state indefinitely till a notification is received
					xTaskNotifyWait( 0, 0, NULL, portMAX_DELAY);
					break;

				case MONITOR_TEMP:

					// The user has requested to manage the temp monitor
					vManageTempMonitor();
					break;

				case TOGGLE_LED:

					// The user has requested to manage toggling the LED on Nucleo board
					vManageLedToggle();
					break;

				case SLEEP:

					// The user has requested to manage the sleep mode for the application
					vManageAppSleep();
					break;

				default:

					// Post a message to the UART write queue indicating that the option
					// selected is not recognized
					vPostMsgToUartQueue("\r\nError: Unrecognized option selected\r\n");
			}
		}
	}
}
/*******************************************************************************
*   Procedure: vClockTaskFunction
*
*   Description: This is the task function for the clock task. It allows the user
*                to display or update the date and time. It also allows the user to set
*                an alarm to trigger at a certain time in the day. If the user does not
*                provide his/her input when prompted within 30 seconds then the whole
*                operation will re-start by prompting the user to select whether to
*                display or update the date and time, set an alarm, or quit. Similarly if
*                the user does not provide a valid input then the above sequence will
*				 repeat. If the user quits by pressing the letter q/Q followed by the
*				 return key when prompted for an input then the Main Menu task will be
*				 notified to run and the clock task will wait for a notification in
*				 blocked state.
*
*   Notes: None
*
*   Parameters: pvParam - A pointer to data passed during task creation. It is not used
*   			in this task function
*
*   Return: None
*
*******************************************************************************/
void vClockTaskFunction(void *pvParam)
{
	char* pcData = NULL;		// To hold the address of the message to post to UART write queue
	char cUartMsg[150] = {0};   // Buffer to send and receive messages via UART
	BaseType_t xReadSuccess = pdFALSE;     // Flag to indicate if reading user's input via UART was successful
	BaseType_t xQuitCurrentApp = pdFALSE;  // Flag to indicate if the user requested to quit the application
	uint8_t ucOptionSelected = 0;  // To hold the user's selected option

	// Wait in blocked state indefinitely till a notification is received
	xTaskNotifyWait( 0, 0, NULL, portMAX_DELAY);

	while(1)
	{
		// Prompt the user to select one of the below options
		pcData = "\r\n\nThis is a clock sub-application\
				  \r\nDisplay date and time   ------> 1\
				  \r\nSet date and time	------> 2\
				  \r\nSet an alarm      	------> 3\
				  \r\nQuit application  	------> 4\
				  \r\nEnter your option here: ";

		xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );

		// Clear message buffer in order to use to receive a new message
		memset(&cUartMsg, 0, sizeof(cUartMsg));

		// Receive user's selected option
		xReadSuccess = xReceiveUartMsg(cUartMsg, &xQuitCurrentApp);

		// If the UART read was successful and the user did not request to quit the application
		if( xReadSuccess == pdTRUE && xQuitCurrentApp == pdFALSE )
		{
			// Return the first byte from the buffer, which will contain the user's selection
			// 48 is subtracted to convert from ASCII to a number
			ucOptionSelected = cUartMsg[0] - 48;

			switch( ucOptionSelected )
			{
				case 1:

					// Acquire current date and time and post them to UART write queue
					vReadRtcDateTime();
					break;

				case 2:

					// Set date and time and check constantly if user requested to quit the sub-application
					vSetDateAndTime( &xQuitCurrentApp );
					break;

				case 3:

					// Set alarm and check constantly if user requested to quit the sub-application
					vSetAlarm( &xQuitCurrentApp );
					break;

				case 4:

					// The user requested to quit the sub-application
					xQuitCurrentApp = pdTRUE;
					break;

				default:

					// Post a message to the UART write queue indicating that the option
					// selected is not recognized
					vPostMsgToUartQueue("\r\nError: Unrecognized option selected\r\n");
					break;
			}
		}

		// If the user requested to quit the sub-application
		if( xQuitCurrentApp == pdTRUE )
		{
			// Notify the Main Menu task to run it
			xTaskNotify( xMainMenuTaskHandle, 0, eNoAction );

			// Wait in blocked state indefinitely till a notification is received
			xTaskNotifyWait( 0, 0, NULL, portMAX_DELAY);

			// A notification to run is received so reset xQuitCurrentApp
			xQuitCurrentApp = pdFALSE;
		}
	}
}
/*******************************************************************************
*   Procedure: vGameTaskFunction
*
*   Description: This is the task function for the game task. It prompts the user
*   		     to guess a number between 0-25 and continues to do so until the
*   		     user guesses the correct number selected by the task function. If
*   		     the user does not provide his/her input when prompted within 30
*   		     seconds then the game will restart. Similarly if the user does not
*   		     provide a valid guess (i.e. number) then the game will restart as well.
*				 If the user quits by pressing the letter q/Q followed by the return
*				 key when prompted for input then the Main Menu task will be notified
*				 to run and the game task will wait for a notification in blocked
*				 state.
*   Notes: None
*
*   Parameters: pvParam - A pointer to data passed during task creation. It is not used
*   			in this task function
*
*   Return: None
*
*******************************************************************************/
void vGameTaskFunction(void *pvParam)
{
	char* pcData = NULL;  			      // To hold the address of the message to post to UART write queue
	char cUartMsg[150] = {0};             // Buffer to receive messages via UART
	BaseType_t xReadSuccess = pdFALSE;    // Flag to indicate if reading user's input via UART was successful
	BaseType_t xQuitCurrentApp = pdFALSE; // Flag to indicate if the user requested to quit the application
	uint8_t lUserGuess = 0xFF;			  // To hold user's guess; initialized to invalid number
	uint8_t ucSelectedNum = 0xFF; 		  // To hold a random integer from 0 to 25; initialized to invalid number
	uint32_t ulNumOfGuesses;			  // To hold the number of attempts or guesses by the user

	// Wait in blocked state indefinitely till a notification is received
	xTaskNotifyWait( 0, 0, NULL, portMAX_DELAY);

	while(1)
	{
		// Generate a new random number from 0 to 25
		ucSelectedNum = rand() % 26;

		// Reset the guess counter
		ulNumOfGuesses = 0;

		// Post a message to the UART write queue asking the user to guess a number
		pcData = "\r\n\nThis is a game sub-application\
				  \r\nGuess a number between 0 to 25: ";

		// Push a pointer to the data into the UART write queue
		// The task will block waiting indefinitely till space becomes available on the queue
		xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );

		// Receive user's guess
		memset(&cUartMsg, 0, sizeof(cUartMsg));
		xReadSuccess = xReceiveUartMsg(cUartMsg, &xQuitCurrentApp);
		lUserGuess = lUartMsgtoInt32(cUartMsg);

		// Increment the guess counter
		ulNumOfGuesses++;

		// Keep prompting the user to guess as long as the guess is not correct, the UART read was successful
		// and the user did not request to quit the sub-application
		while( lUserGuess!= ucSelectedNum && xReadSuccess == pdTRUE && xQuitCurrentApp == pdFALSE )
		{
			if(lUserGuess > ucSelectedNum)
			{
				pcData = "\r\n\nYou guessed too high\r\n";
				xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );
			}
			else
			{
				pcData = "\r\n\nYou guessed too low\r\n";
				xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );
			}

			// Prompt the user to guess again
			pcData = "\r\nGuess a number between 0 to 25: ";
			xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );

			// Receive user's guess
			memset(&cUartMsg, 0, sizeof(cUartMsg));
			xReadSuccess = xReceiveUartMsg(cUartMsg, &xQuitCurrentApp);
			lUserGuess = lUartMsgtoInt32(cUartMsg);

			// Increment the guess counter
			ulNumOfGuesses++;
		}

		// If the user has guessed the correct number
		if( lUserGuess == ucSelectedNum )
		{
			memset(&cUartMsg, 0, sizeof(cUartMsg));
			sprintf( cUartMsg, "\r\n\nYou guessed the correct number!\
					            \r\nIt took you %ld attempt(s) to guess the number!", ulNumOfGuesses );
			pcData = cUartMsg;
			xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );
		}

		// If the user requested to quit the sub-application
		if( xQuitCurrentApp == pdTRUE )
		{
			// Notify the Main Menu task to run it
			xTaskNotify( xMainMenuTaskHandle, 0, eNoAction );

			// Wait in blocked state indefinitely till a notification is received
			xTaskNotifyWait( 0, 0, NULL, portMAX_DELAY);

			// A notification to run is received so reset xQuitCurrentApp
			xQuitCurrentApp = pdFALSE;
		}
	}
}
/*******************************************************************************
*   Procedure: vCalculatorTaskFunction
*
*   Description: This is the task function for the calculator task. It prompts
*   			 the user to provide two numbers and select a mathematical operation
*   			 (+ - * /). If the user does not provide his/her input when prompted
*   			 within 30 seconds then the calculation fails and the operation will
*   			 be restarted. Similarly if a valid number or a valid mathematical
*   			 operation is not selected then the user input will be ignored and user
*   			 will be prompted for inputs from the start again. If the user quits
*   			 by pressing the letter q/Q followed by the return key when prompted
*   			 for input then the Main Menu task will be notified to run and the
*   			 calculator task will wait for a notification in blocked state.
*
*   Notes: None
*
*   Parameters: pvParam - A pointer to data passed during task creation. It is not used
*   			in this task function
*
*   Return: None
*
*******************************************************************************/
void vCalculatorTaskFunction(void *pvParam)
{
	char cUartMsg[50] = {0};              // Buffer used to send and receive messages via UART
	BaseType_t xQuitCurrentApp = pdFALSE;  // Flag to indicate if the user requested to quit the application
	BaseType_t xReadSuccess = pdFALSE;     // Flag to indicate if reading user's input via UART was successful
	int32_t lFirstNum;                     // To hold the first number of the calculation
	int32_t lSecondNum;					   // To hold the second number of the calculation
	int32_t lCalcNum;                      // To hold the result of the calculation

	// Wait in blocked state indefinitely till a notification is received
	xTaskNotifyWait( 0, 0, NULL, portMAX_DELAY);

	while(1)
	{
		// Set the calculator variables to an invalid number
		lFirstNum = INVALID_NUM;
		lSecondNum = INVALID_NUM;
		lCalcNum = INVALID_NUM;

		// Post a message to the UART write queue prompting the user to enter the first number
		// of the calculation
		char* pcData = "\r\n\nThis is a calculator sub-application\
						\r\nEnter the first integer = ";

		// Push the address of a pointer to the data into the UART write queue
		// The task will block waiting indefinitely till space becomes available on the queue
		xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );

		// Receive the first number of the calculation from the user
		memset(&cUartMsg, 0, sizeof(cUartMsg));
		xReadSuccess = xReceiveUartMsg(cUartMsg, &xQuitCurrentApp);
		lFirstNum = lUartMsgtoInt32(cUartMsg);

		// If the UART read was successful, the user did not request to quit the sub-application,
		// and the first number is not an invalid number then continue to prompt the user to
		// enter the second number
		if( xReadSuccess == pdTRUE && xQuitCurrentApp == pdFALSE && lFirstNum != INVALID_NUM )
		{
			pcData = "\r\n\nEnter the second integer = ";
			xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );

			memset(&cUartMsg, 0, sizeof(cUartMsg));
			xReadSuccess = xReceiveUartMsg(cUartMsg, &xQuitCurrentApp);
			lSecondNum = lUartMsgtoInt32(cUartMsg);

			// If the UART read was successful, the user did not request to quit the sub-application,
			// and the second number is not an invalid number then continue to prompt the user to
			// enter the operation for the calculation
			if( xReadSuccess == pdTRUE && xQuitCurrentApp == pdFALSE && lSecondNum != INVALID_NUM )
			{
				pcData = "\r\n\nEnter the operator (+ - * /) = ";
				xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );

				memset(&cUartMsg, 0, sizeof(cUartMsg));
				xReadSuccess = xReceiveUartMsg(cUartMsg, &xQuitCurrentApp);

				// If the UART read was successful, the user did not request to quit the sub-application
				// then conduct the appropriate mathematical operation based on the operator selected
				if( xReadSuccess == pdTRUE && xQuitCurrentApp == pdFALSE )
				{
					switch( cUartMsg[0] )
					{
						case '+':

							lCalcNum = lFirstNum + lSecondNum;
							sprintf( cUartMsg, "\r\n\nThe calculated integer is %ld", lCalcNum );
							pcData = cUartMsg;
							xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );
							break;

						case '-':

							lCalcNum = lFirstNum - lSecondNum;
							sprintf( cUartMsg, "\r\n\nThe calculated integer is %ld", lCalcNum );
							pcData = cUartMsg;
							xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );
							break;

						case '*':

							lCalcNum = lFirstNum * lSecondNum;
							sprintf( cUartMsg, "\r\n\nThe calculated integer is %ld", lCalcNum );
							pcData = cUartMsg;
							xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );
							break;

						case '/':

							lCalcNum = lFirstNum / lSecondNum;
							sprintf( cUartMsg, "\r\n\nThe calculated integer is %ld", lCalcNum);
							pcData = cUartMsg;
							xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );
							break;

						default:

							// Post a message to the UART write queue indicating that the operator
							// selected is not recognized
							vPostMsgToUartQueue("\r\nError: Unrecognized mathematical operator selected\r\n");
							break;
					}
				}
			}
		}

		// If the user has requested to quit the sub-application
		if( xQuitCurrentApp == pdTRUE )
		{
			// Notify the Main Menu task to run it
			xTaskNotify( xMainMenuTaskHandle, 0, eNoAction );

			// Wait in blocked state indefinitely till a notification is received
			xTaskNotifyWait( 0, 0, NULL, portMAX_DELAY);

			// A notification to run is received so reset xQuitCurrentApp
			xQuitCurrentApp = pdFALSE;
		}
	}
}
/*******************************************************************************
*   Procedure: vTempMonitorTaskFunction
*
*   Description: This is the task function for the temperature monitor task. It
*   			 keeps track of the current, highest, and lowest temperatures.
*   			 It displays these temperatures when the user requests them.
*   Notes: None
*
*   Parameters: pvParam - A pointer to data passed during task creation. It is not used
*   			in this task function
*
*   Return: None
*******************************************************************************/
void vTempMonitorTaskFunction(void *pvPram)
{
	float fHighestTemp = 0.0;			// To hold the highest temperature measured
	float fLowestTemp = 100.0;			// To hold the lowest temperature measured
	float fCurrentTemp;					// To hold the current temperature measured
	char cTempStatsMsg[100] = {0};      // Buffer to hold the message to post to the UART write queue
	char* pcDateTime = cTempStatsMsg;   // Pointer to the start of the message buffer
	RTC_DateTypeDef xDateForHTemp;      // To hold the recorded date of highest temp
	RTC_TimeTypeDef xTimeForHTemp;      // To hold the recorded time of highest temp
	RTC_DateTypeDef xDateForLTemp;      // To hold the recorded date of lowest temp
	RTC_TimeTypeDef xTimeForLTemp;      // To hold the recorded time of lowest temp
	RTC_DateTypeDef xCurrentDate;       // To hold the current date
	RTC_TimeTypeDef xCurrentTime;       // To hold the current time

	while(1)
	{
		// If the user has requested to stop temp monitoring
		// Or the user has not requested to start temp monitoring
		if( xRunTempMonitor == pdFALSE )
		{
			// Reset temperature stats
			fHighestTemp = 0.0;
			fLowestTemp = 100.0;
			memset(&xDateForHTemp, 0, sizeof(xDateForHTemp));
			memset(&xTimeForHTemp, 0, sizeof(xTimeForHTemp));
			memset(&xDateForLTemp, 0, sizeof(xDateForLTemp));
			memset(&xTimeForLTemp, 0, sizeof(xTimeForLTemp));

			// Wait in blocked state indefinitely till a notification is received
			xTaskNotifyWait( 0, 0, NULL, portMAX_DELAY);

			// A notification to run is received so set xRunTempMonitor flag
			xRunTempMonitor = pdTRUE;
		}

		// Acquire current time
		RTC_GetTime( RTC_Format_BIN, &xCurrentTime );

		// Acquire current date
		// RTC_GetDate() needs to be called twice to get the updated date
		RTC_GetDate( RTC_Format_BIN, &xCurrentDate );
		RTC_GetDate( RTC_Format_BIN, &xCurrentDate );

		// Acquire current temp from sensor
		fCurrentTemp = fMeasureTemp();

		// Check to see if our lowest or highest temps have changed
		// If so, record the new temps and the time and date for those temps
		if( fCurrentTemp > fHighestTemp )
		{
			fHighestTemp = fCurrentTemp;

			xDateForHTemp = xCurrentDate;
			xTimeForHTemp = xCurrentTime;
		}
		else if( fCurrentTemp < fLowestTemp )
		{
			fLowestTemp = fCurrentTemp;

			xDateForLTemp = xCurrentDate;
			xTimeForLTemp = xCurrentTime;
		}

		vTaskDelay(pdMS_TO_TICKS(500));

		if( xShowTemps == pdTRUE )
		{
			sprintf(cTempStatsMsg, "\r\n\n%02d-%02d-%02d %02d:%02d:%02d Current Temp Recorded = %0.2f C",\
					xCurrentDate.RTC_Date, xCurrentDate.RTC_Month, xCurrentDate.RTC_Year,\
					xCurrentTime.RTC_Hours, xCurrentTime.RTC_Minutes, xCurrentTime.RTC_Seconds, fCurrentTemp);

			// Post the address of the message to the UART write queue
			xQueueSend( xUartWriteQueue, &pcDateTime, portMAX_DELAY );

			sprintf(cTempStatsMsg, "\r\n\n%02d-%02d-%02d %02d:%02d:%02d Highest Temp Recorded = %0.2f C",\
				    xDateForHTemp.RTC_Date, xDateForHTemp.RTC_Month, xDateForHTemp.RTC_Year,\
					xTimeForHTemp.RTC_Hours, xTimeForHTemp.RTC_Minutes, xTimeForHTemp.RTC_Seconds, fHighestTemp);

			// Post the address of the message to the UART write queue
			xQueueSend( xUartWriteQueue, &pcDateTime, portMAX_DELAY );

			sprintf(cTempStatsMsg, "\r\n\n%02d-%02d-%02d %02d:%02d:%02d Lowest Temp Recorded = %0.2f C\r\n",\
					xDateForLTemp.RTC_Date, xDateForLTemp.RTC_Month, xDateForLTemp.RTC_Year,\
					xTimeForLTemp.RTC_Hours, xTimeForLTemp.RTC_Minutes, xTimeForLTemp.RTC_Seconds, fLowestTemp);

			// Post the address of the message to the UART write queue
			xQueueSend( xUartWriteQueue, &pcDateTime, portMAX_DELAY );

			// Reset xShowTemps flag
			xShowTemps = pdFALSE;
		}
	}
}
/*******************************************************************************
*   Procedure: vUartSetup
*
*   Description: This function configures and enables UART2 to allow message
*   			 transmission and reception
*
*   Notes: None
*
*   Parameters: None
*
*   Return: None
*
*******************************************************************************/
static void vUartSetup(void)
{
	GPIO_InitTypeDef xGpioUartPins;	// To hold the configurations for the GPIO UART pins to be initialized
	USART_InitTypeDef xUart2Init;       // To hold the configurations for the UART peripheral to be initialized

	// Enable UART2 peripheral clock and GPIOA peripheral clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// Zeroing each struct member
	memset(&xGpioUartPins, 0, sizeof(xGpioUartPins));

	// Alternate function configuration of MCU pins to behave as UART2 TX and RX
	// PA2 is UART2_TX and PA3 is UART2_RX
	xGpioUartPins.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	xGpioUartPins.GPIO_Mode = GPIO_Mode_AF;
	xGpioUartPins.GPIO_PuPd = GPIO_PuPd_UP;  // UART frame is high (logic 1) when idle
	GPIO_Init(GPIOA, &xGpioUartPins);

	// AF mode settings for the pins
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); 	// Configure AF mode for PA2 as UART2_TX
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); 	// Configure AF mode for PA3 as UART2_RX

	// Zeroing each struct member
	memset(&xUart2Init, 0, sizeof(xUart2Init));

	// UART parameter initializations
	xUart2Init.USART_BaudRate = 115200;
	xUart2Init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	xUart2Init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	xUart2Init.USART_Parity = USART_Parity_No;
	xUart2Init.USART_StopBits = USART_StopBits_1;
	xUart2Init.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &xUart2Init);

	// Enable UART2 peripheral
	USART_Cmd(USART2, ENABLE);
}
/*******************************************************************************
*   Procedure: vGpioSetup
*
*   Description: This function configures GPIO A Pin 5 which is connected to the
*   			 the green LED on Nucleo board. GPIO A Pin 5 is configured to output
*   			 mode to allow toggling the LED when the user chooses to.
*   Notes: None
*
*   Parameters: None
*
*   Return: None
*
*******************************************************************************/
static void vGpioSetup(void)
{
	GPIO_InitTypeDef xLedInit; // To hold the configurations for the GPIO pin to be initialized for the LED

	// Zeroing each struct member to avoid random values causing abnormal behavior
	memset(&xLedInit, 0, sizeof(xLedInit));

	// Turn on the clock for GPIOA where the LED is hanging
	// GPIOA is hanging on AHB1 bus
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// LED is handing on PA5 on Nucleo board
	xLedInit.GPIO_Mode = GPIO_Mode_OUT;
	xLedInit.GPIO_OType = GPIO_OType_PP;
	xLedInit.GPIO_Pin = GPIO_Pin_5;
	xLedInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
	xLedInit.GPIO_Speed = GPIO_Low_Speed;

	// Initialize PA5 with the configurations above
	GPIO_Init(GPIOA, &xLedInit);
}
/*******************************************************************************
*   Procedure: vSetupHardware
*
*   Description: This function calls GPIO, UART2, and RTC initialization functions.
*
*   Notes: None
*
*   Parameters: None
*
*   Return: None
*
*******************************************************************************/
static void vSetupHardware(void)
{
	//To setup the green LED on Nucleo board
	vGpioSetup();

	// To setup UART2 for message transmission and reception
	vUartSetup();

	// To setup the RTC to track date, time, and set up an alarm
	vRtcSetup();

	// To setup the ADC to use for analog temperature measurement
	vAdcSetup();
}
/*******************************************************************************
*   Procedure: vSendUartMsg
*
*   Description: This function transmits data byte by byte via UART2 peripheral.
*
*   Notes: The user should avoid using this function to transmit UART messages.
*   	   Rather the user should post messages to the UART write queue in order
*   	   to serialize message transmission and avoid race condition for UART2
*   	   peripheral.
*
*   Parameters: pcMsg - A pointer to a message buffer of type char
*
*   Return: None
*
*******************************************************************************/
static void vSendUartMsg(char* pcMsg)
{
	// Continue to loop while there are still bytes in the buffer to send
	for(int i = 0; i < strlen(pcMsg); i++)
	{
		// Loop until the transmit data register for UART2 is empty
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) != SET);

		// Send one byte at a time
		USART_SendData(USART2, pcMsg[i]);
	}
}
/*******************************************************************************
*   Procedure: xReceiveUartMsg
*
*   Description: This function receives a message from the user via the UART window
*   			 whenever the user is prompted to provide an input. It will time out
*   			 if the user input is not received within 30 seconds It will also
*   			 flag that the user has requested to quit the current task if the
*   			 the user presses the letter q/Q followed by the return key.
*
*   Notes: None
*
*   Parameters: - pucMsgBuffer - A pointer to a message buffer to hold the received
*   			  message from the user
*
*   			- pxQuitCurrentApp - A pointer to BaseType_t location that will hold
*   		      pdTRUE if the user has requested to quit the application, otherwise
*   		      it will hold pdFALSE.
*
*   Return: BaseType_t - pdTRUE is returned if the user provided his/her input in less
*   		than 30 seconds, otherwise pdFALSE is returned.
*
*******************************************************************************/
static BaseType_t xReceiveUartMsg(char* pcMsgBuffer, BaseType_t* pxQuitCurrentApp)
{
	uint8_t ucDataByte = 0;			// To hold current data byte received
	uint8_t ucPrvDataByte = 0;		// Previous data byte to check if the user quit the current app
	uint16_t usMsgLen = 0;			// To index the bytes received
	TickType_t xCurrentTickCount= xTaskGetTickCount();   // To hold the tick count at the start of the call
	char* pcData = NULL;   // To hold the address of the message to post to UART write queue

	// Wait for user's input no more than 30 seconds
	while( xTaskGetTickCount() < ( xCurrentTickCount + pdMS_TO_TICKS(30000) ) )
	{
		// Wait till data is received at the data register of USART2
		while ( USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != SET );

		ucPrvDataByte = ucDataByte; 	// Hold the previous byte to check if the user quit the current app

		// Retrieve a byte of data
		ucDataByte = (uint8_t)( USART_ReceiveData( USART2 ) & 0xFF );

		// If the return key is pressed by the user then exit
		if( ucDataByte != '\r' )
		{
			// Push the byte received into the message buffer provided
			pcMsgBuffer[usMsgLen++] = ucDataByte;
		}
		else
		{
			if( ucPrvDataByte == 'q' || ucPrvDataByte == 'Q' )
			{
				// User wants to quit current app and go back to Main Menu app
				*pxQuitCurrentApp = pdTRUE;
			}

			break;
		}
	}

	// Message is successfully received if the message was typed before the 30 seconds limit
	if( xTaskGetTickCount() < ( xCurrentTickCount + pdMS_TO_TICKS(30000) ))
	{
		// Return true if the message is received
		return(pdTRUE);
	}
	else
	{
		pcData = "\r\nUser input timeout...\r\n";
		xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );

		// Return false if the message is not received in time
		return(pdFALSE);
	}
}
/*******************************************************************************
*   Procedure: vPostMsgToUartQueue
*
*   Description: This function posts an error message to the UART write queue so
*   		     it can be printed on the UART window for the user to see.
*
*   Notes: None
*
*   Parameters: pcErrorMsg- A pointer to a location holding the error message
*
*   Return: None
*
*******************************************************************************/
static void vPostMsgToUartQueue(char* pcUartMsg)
{
	// Post error message to the UART write queue
	xQueueSend( xUartWriteQueue, &pcUartMsg, portMAX_DELAY );
}
/*******************************************************************************
*   Procedure: vRtcSetup
*
*   Description: This function configures and enables the RTC peripheral to track
*   		     date and time. It also enables Alarm A so it can be configured
*   		     by the user using the clock task to trigger an alarm.
*
*   Notes: None
*
*   Parameters: None
*
*   Return: None
*
*******************************************************************************/
static void vRtcSetup(void)
{
	RTC_InitTypeDef xRtcInitStruct;		// To hold the configurations for the RTC peripheral to be initialized with
	RTC_DateTypeDef xDateToSet;         // A place holder date for the RTC peripheral to go with
	RTC_TimeTypeDef xTimeToSet;         // A place holder time for the RTC peripheral to go with
	EXTI_InitTypeDef xAlarmExtiInit;	// Configure EXTI line 17 since the RTC Alarm A and B are connected to it

	// As the RTC clock configuration bits are in the Backup domain and write
	// access is denied to this domain after reset, you have to enable write
	// access using PWR_BackupAccessCmd(ENABLE) function before to configure
	// the RTC clock source (to be done once after reset)
	PWR_BackupAccessCmd( ENABLE );

	// Turn on the LSE clock
	RCC_LSEConfig( RCC_LSE_ON );

	// Select clock source for RTC to be LSE
	// If the LSE or LSI is used as RTC clock source, the RTC continues to
	// work in STOP and STANDBY modes, and can be used as wakeup source.
	RCC_RTCCLKConfig( RCC_RTCCLKSource_LSE );

	// Enables the RTC clock. This function must be used only after the RTC clock source was
	// selected using the RCC_RTCCLKConfig function.
	RCC_RTCCLKCmd( ENABLE );

	// Configure the parameters of the RTC peripheral
	xRtcInitStruct.RTC_HourFormat = RTC_HourFormat_24;
	xRtcInitStruct.RTC_AsynchPrediv = 0x7F;		//127
	xRtcInitStruct.RTC_SynchPrediv = 0xFF;		//255

	// Initialize RTC peripheral
	RTC_Init(&xRtcInitStruct);

	// Configure the time. Random time is chosen
	xTimeToSet.RTC_Hours = 17;
	xTimeToSet.RTC_Minutes = 0;
	xTimeToSet.RTC_Seconds = 0;

	// Set time
	RTC_SetTime( RTC_Format_BIN, &xTimeToSet);

	// Configure the date. Random date is chosen
	xDateToSet.RTC_Date = 3;
	xDateToSet.RTC_Month = 12;
	xDateToSet.RTC_Year = 20;	// 20 for 2020
	xDateToSet.RTC_WeekDay = RTC_Weekday_Thursday;

	// Set date
	RTC_SetDate( RTC_Format_BIN, &xDateToSet );

	// Interrupt configuration for Alarm A of RTC

	// Zeroing each struct member to avoid random values causing abnormal behaviors
	memset(&xAlarmExtiInit, 0, sizeof(xAlarmExtiInit));

	xAlarmExtiInit.EXTI_Line = EXTI_Line17;				// Select EXTI line 17
	xAlarmExtiInit.EXTI_LineCmd = ENABLE;				// Select the desired state for the EXTI line
	xAlarmExtiInit.EXTI_Mode = EXTI_Mode_Interrupt;		// Select the mode for the EXTI line (e.g. interrupt or event)
	xAlarmExtiInit.EXTI_Trigger = EXTI_Trigger_Rising;	// For RTC alarm a rising edge trigger is needed

	// Initialize the EXTI line configured
	EXTI_Init( &xAlarmExtiInit );

	// Turn on interrupt for Alarm A
	RTC_ITConfig( RTC_IT_ALRA, ENABLE);

	// Set priority for Alarm A/B interrupt in NVIC
	// By default the priority of newly enabled interrupt will be 0
	// The priority cannot be less than 5 as per configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
	NVIC_SetPriority( RTC_Alarm_IRQn, 5 );

	// Enable Alarm A/B interrupt reception at the NVIC
	NVIC_EnableIRQ(RTC_Alarm_IRQn);
}
/*******************************************************************************
*   Procedure: RTC_Alarm_IRQHandler
*
*   Description: Non-weak implementation of the interrupt handler for RTC Alarm
*   			 A and B. If an alarm is configured by the user using the clock
*   			 task then this handler will get executed once the RTC alarm is
*   			 triggered. Generally, the handler will print a message on the UART
*   			 window notifying the user that the alarm has been triggered.
*
*   Notes: None
*
*   Parameters: None
*
*   Return: None
*
*******************************************************************************/
void RTC_Alarm_IRQHandler(void)
{
	// We will only be here if an Alarm A has occurred

	// Alarm A and B are connected to EXTI line 17
	// To avoid the interrupt handler being executed continuously
	// clear the interrupt pending bit of EXTI line 17
	EXTI_ClearITPendingBit( EXTI_Line17 );

	// Alert the user that the alarm was triggered
	vSendUartMsg("\r\nThe alarm was triggered\r\n");

	if( xGoToSleep == pdTRUE)
	{
		vSendUartMsg("\r\nStill in sleep mode\
				      \r\nPress any keyboard letter/number to wake up\r\n");
	}
}
/*******************************************************************************
*   Procedure: vReadRtcDateTime
*
*   Description: This function reads the current date and time and post them to
*   			 the UART write queue in order to display them on the UART window.
*
*   Notes: None
*
*   Parameters: None
*
*   Return: None
*
*******************************************************************************/
static void vReadRtcDateTime(void)
{
	char cDateTimeMsg[100] = {0};     // Buffer to hold the message to post to the UART write queue
	char* pcDateTime = cDateTimeMsg;  // Pointer to the start of the message buffer
	RTC_DateTypeDef xCurrentDate;     // To hold the current date parameters
	RTC_TimeTypeDef xCurrentTime;     // To hold the current time parameters

	// Zeroing each struct member
	memset(&xCurrentDate, 0, sizeof(xCurrentDate));
	memset(&xCurrentTime, 0, sizeof(xCurrentTime));

	// Acquire time info
	RTC_GetTime( RTC_Format_BIN, &xCurrentTime );

	// Acquire date info
	// RTC_GetDate() needs to be called twice to get the updated date
	RTC_GetDate( RTC_Format_BIN, &xCurrentDate );
	RTC_GetDate( RTC_Format_BIN, &xCurrentDate );


	sprintf(cDateTimeMsg, "\r\n\nTime: %02d:%02d:%02d\r\nDate: %02d-%02d-%02d",\
			xCurrentTime.RTC_Hours, xCurrentTime.RTC_Minutes,xCurrentTime.RTC_Seconds,\
			xCurrentDate.RTC_Date, xCurrentDate.RTC_Month, xCurrentDate.RTC_Year);

	// Post the address of the message to the UART write queue
	xQueueSend( xUartWriteQueue, &pcDateTime, portMAX_DELAY );
}
/*******************************************************************************
*   Procedure: lUartMsgtoInt32
*
*   Description: This function converts the ASCII input received from the user via
*   			 UART to an INT32 number.
*
*   Notes: None
*
*   Parameters: pucUartMsg- A pointer to a location holding the UART message
*
*   Return: INT32 number
*
*******************************************************************************/
static int32_t lUartMsgtoInt32(char* pcUartMsg)
{
	uint32_t i = 0;  		// Iteration index
	int32_t ulNum = 0;		// To hold the constructed number

	// Continue to construct the number as long as the byte in hand
	// is a number within 0 to 9
	while ( pcUartMsg[i] >= '0' && pcUartMsg[i] <= '9' )
	{
		// Convert the digit in hand from ASCII to a number, shift the current digits
		// to the left and combine the new digit
		ulNum = (ulNum * 10) + (pcUartMsg[i] - 48);
		i++;
	}

	if( i > 0 )
	{
		// We have a valid number
		return( ulNum );
	}
	else
	{
		// We do not have a valid number
		ulNum = INVALID_NUM;
		return ( ulNum );
	}
}
/*******************************************************************************
*   Procedure: vLedToggleEnable
*
*   Description: This functions enables toggling the green LED on the Nucleo board.
*   			 It relies on a software timer provided the FreeRTOS layer. This
*   			 timer expires at ulToggleDuration and calls vLedToggle() which in
*   			 turn toggles the LED.
*
*   Notes: None
*
*   Parameters: ulToggleDuration- A UINT32 variable which specifies the duration at
*   		    which the timer expires.
*
*   Return: None
*
*******************************************************************************/
static void vLedToggleEnable(uint32_t ulToggleDuration)
{
	// To avoid calling xTimerCreate() repeatedly, the if and else if guards are used
	if( pxLedToggleTimer == NULL )
	{
		// Create a software timer that repeatedly expires at ulToggleDuration and calls vLedToggle()
		pxLedToggleTimer = xTimerCreate( "LED-TIMER", ulToggleDuration, pdTRUE, NULL, vLedToggle );

		// Start the software timer
		// The calling task will be held in blocked state indefinitely waiting for the start command
		// to be successfully sent to the timer command queue
		xTimerStart( pxLedToggleTimer, portMAX_DELAY );
	}
	else
	{
		// xTimerCreate() has been called already, so do not call it again
		// Start the software timer
		xTimerStart( pxLedToggleTimer, portMAX_DELAY );
	}
}
/*******************************************************************************
*   Procedure: vLedToggle
*
*   Description: This function toggles the green LED connected to GPIO A Pin 5.
*
*   Notes: None
*
*   Parameters: None
*
*   Return: None
*
*******************************************************************************/
void vLedToggle(TimerHandle_t xTimer)
{
	GPIO_ToggleBits( GPIOA, GPIO_Pin_5 );
}
/*******************************************************************************
*   Procedure: vLedToggleDisable
*
*   Description: This function disables the LED toggle by disabling the timer that
*   			 calls vLedToggle(). It will also switch off the LED in case we
*   			 stopped toggling while the LED on
*
*   Notes: None
*
*   Parameters: None
*
*   Return: None
*
*******************************************************************************/
static void vLedToggleDisable(void)
{
	if( pxLedToggleTimer != NULL)
	{
		// Stop the software timer. The timer will not be deleted.
		// The calling task will be held in blocked state indefinitely waiting for the
		// stop command to be successfully sent to the timer command queue
		xTimerStop( pxLedToggleTimer, portMAX_DELAY );

		// Switch off the LED in case we stopped toggling while the LED on
		GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
	}
}
/*******************************************************************************
*   Procedure: vApplicationIdleHook
*
*   Description: The idle hook function will execute if the idle task is running.
*   			 If the xGoToSleep flag is set then the WFI (Wait for Interrupt)
*   			 thumb instruction will execute which puts the Nucleo board in
*   			 normal sleep mode. Interrupts from the likes of Systick will
*   			 wake the system up. However since all tasks are blocked the idle
*   			 task will run again which will make this hook function execute
*   			 and put the system to sleep again.
*
*   Notes: None
*
*   Parameters: None
*
*   Return: None
*
*******************************************************************************/
void vApplicationIdleHook(void)
{
	// Only go to sleep if the xGoToSleep flag is set
	if( xGoToSleep == pdTRUE)
	{
		// Send the CPU to normal sleep (i.e. CPU clock will be turned off)
		// Wait For Interrupt thumb instruction is used here
		// Interrupts from the likes of Systick will make us exit the WFI instruction
		__WFI();
	}
}
/*******************************************************************************
*   Procedure: USART2_IRQHandler
*
*   Description: Non-weak implementation of USART2 exception handler. This
*   			 handler is executed if the application is in sleep mode and the
*   			 user presses any button in the UART window.  This handler clears
*   			 the xGoToSleep flag in order to stop executing the WFI (Wait
*   			 For Interrupt) instruction in the idle hook function. It also
*   			 notifies the Main Menu task in order to run it and go back to
*   			 normal operation.
*
*   Notes: None
*
*   Parameters: None
*
*   Return: None
*
*******************************************************************************/
void USART2_IRQHandler(void)
{
	BaseType_t *pxIsMainMenuTaskHigherPriority = NULL;  // A pointer to a flag set if a higher priority task is woken due to task notification

	// Clear the interrupt bit for UART2 RXNE to prevent the interrupt handler
	// from continuously running
	USART_ClearITPendingBit( USART2, USART_IT_RXNE);

	// Reset the xGoToSleep flag in order to stop using the WFI instruction
	xGoToSleep = pdFALSE;

	// Notify the Main Menu task to run it and go back to normal operation
	xTaskNotifyFromISR( xMainMenuTaskHandle, 0, eNoAction, pxIsMainMenuTaskHigherPriority );

	if( *pxIsMainMenuTaskHigherPriority == pdTRUE )
	{
		// The notification caused the task to which the notification was sent to leave the Blocked
		// state, and the unblocked task has a priority higher than the currently running task.
		// Thus, yield the current task
		taskYIELD();
	}
}
/*******************************************************************************
*   Procedure: vSetAlarm
*
*   Description: This function executes under the Main Menu task function once
*   			 the user has chosen to set an Alarm. It will walk the user through
*   			 the required inputs and eventually sets the alarm. This function
*   			 will not set an alarm for any of the following reasons:
*   			 - the user does not provide an appropriate input,
*   			 - the user quits by pressing q/Q and the return key, or
*   			 - the user does not provide an input within 30 seconds
*
*   Notes: None
*
*   Parameters: pxQuitCurrentApp - A pointer to BaseType_t location that will hold
*   		    pdTRUE if the user has requested to quit the application, otherwise
*   		    it will hold pdFALSE.
*
*   Return: None
*
*******************************************************************************/
static void vSetAlarm( BaseType_t* pxQuitCurrentApp )
{
	char* pcData = NULL;        // To hold the address of the message to post to UART write queue
	char cUartMsg[50] = {0};    // Buffer to receive messages via UART
	BaseType_t xReadSuccess = pdFALSE;       // Flag to indicate if reading user's input via UART was successful
	int32_t lUsersInput = INVALID_NUM;	     // To hold user's entry; initialized to an invalid number
	RTC_AlarmTypeDef xAlarmAConfig;			 // To hold the configurations to initialize Alarm A with

	// Zeroing each struct member
	memset(&xAlarmAConfig, 0, sizeof(xAlarmAConfig));

	// Configure the alarm to occur daily (i.e. ignore/mask the date and week day)
	xAlarmAConfig.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay;

	// Post a UART message to prompt the user to enter the selected hour of the alarm
	pcData = "\r\nEnter the hour of the Alarm\r\n";
	xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );

	// Receive user's input for the hour of the alarm
	memset(&cUartMsg, 0, sizeof(cUartMsg));
	xReadSuccess = xReceiveUartMsg(cUartMsg, pxQuitCurrentApp);
	lUsersInput = lUartMsgtoInt32(cUartMsg);

	// If the UART read was successful, the user did not quit the sub-application, and user's input is within 0-23
	// then go ahead and prompt the user to enter the minute of the alarm
	if( xReadSuccess == pdTRUE && *pxQuitCurrentApp == pdFALSE && lUsersInput >= 0 && lUsersInput <= 23 )
	{
		// Set the hour of the alarm
		xAlarmAConfig.RTC_AlarmTime.RTC_Hours = lUsersInput;

		// Post a UART message to prompt the user to enter the selected minute of the alarm
		pcData = "\r\nEnter the minute of the Alarm\r\n";
		xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );

		// Receive user's input for the minute of the alarm
		memset(&cUartMsg, 0, sizeof(cUartMsg));
		xReadSuccess = xReceiveUartMsg(cUartMsg, pxQuitCurrentApp);
		lUsersInput = lUartMsgtoInt32(cUartMsg);

		// If the UART read was successful, the user did not quit the sub-application, and user's input is within 0-59
		// then go ahead and prompt the user to enter the second of the alarm
		if ( xReadSuccess == pdTRUE && *pxQuitCurrentApp == pdFALSE && lUsersInput >= 0 && lUsersInput <= 59 )
		{
			// Set the minute of the alarm
			xAlarmAConfig.RTC_AlarmTime.RTC_Minutes = lUsersInput;

			// Post a UART message to prompt the user to enter the selected second of the alarm
			pcData = "\r\nEnter the second of the Alarm\r\n";
			xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );

			// Receive user's input for the second of the alarm
			memset(&cUartMsg, 0, sizeof(cUartMsg));
			xReadSuccess = xReceiveUartMsg(cUartMsg, pxQuitCurrentApp);
			lUsersInput = lUartMsgtoInt32(cUartMsg);

			// If the UART read was successful, the user did not quit the sub-application, and user's input is within 0-59
			// then set and enable the alarm
			if ( xReadSuccess == pdTRUE && *pxQuitCurrentApp == pdFALSE && lUsersInput >= 0 && lUsersInput <= 59 )
			{
				// Set the seconds of the alarm
				xAlarmAConfig.RTC_AlarmTime.RTC_Seconds = lUsersInput;

				// The Alarm register can only be written when the corresponding Alarm is disabled
				RTC_AlarmCmd(RTC_Alarm_A, DISABLE);

				// Set Alarm A
				RTC_SetAlarm( RTC_Format_BIN, RTC_Alarm_A, &xAlarmAConfig);

				// Enable Alarm A
				RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
			}
		}
	}
}
/*******************************************************************************
*   Procedure: vSetDateAndTime
*
*   Description: This function executes under the Main Menu task function once
*   			 the user has chosen to change the current date and time. It
*   			 will walk the user through the required inputs and eventually
*   			 sets the the new date and time. This function will not set the
*   			 date or time for any of the following reasons:
*   			 - the user does not provide an appropriate input,
*   			 - the user quits by pressing q/Q and the return key, or
*   			 - the user does not provide an input within 30 seconds
*
*   Notes: None
*
*   Parameters: pxQuitCurrentApp- A pointer to BaseType_t location that will hold
*               pdTRUE if the user has requested to quit the application, otherwise
*               it will hold pdFALSE.
*
*   Return: None
*
*******************************************************************************/
static void vSetDateAndTime( BaseType_t* pxQuitCurrentApp )
{
	char* pcData = NULL;        // To hold the address of the message to post to UART write queue
	char cUartMsg[50] = {0};	// Buffer to receive messages via UART
	BaseType_t xReadSuccess = pdFALSE;    // Flag to indicate if reading user's input via UART was successful
	int32_t lUsersInput = INVALID_NUM;	  // To hold user's entry; initialized to an invalid number

	RTC_TimeTypeDef	xTimeConfig;
	RTC_DateTypeDef xDateConfig;

	// Zeroing each struct member
	memset(&xTimeConfig, 0, sizeof(xTimeConfig));
	memset(&xDateConfig, 0, sizeof(xDateConfig));

	// Post a UART message to prompt the user to enter the selected hour for the time
	pcData = "\r\n\nConfiguring the time\
			  \r\nEnter the hour in 24 hour format\r\n";
	xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );

	// Receive user's input
	memset(&cUartMsg, 0, sizeof(cUartMsg));
	xReadSuccess = xReceiveUartMsg(cUartMsg, pxQuitCurrentApp);
	lUsersInput = lUartMsgtoInt32(cUartMsg);

	// If the UART read was successful, the user did not quit the sub-application, and user's input is within 0-23
	// then go ahead and prompt the user to enter the minute for the time
	if( xReadSuccess == pdTRUE && *pxQuitCurrentApp == pdFALSE && lUsersInput >= 0 && lUsersInput <= 23 )
	{
		// Set the hour
		xTimeConfig.RTC_Hours = lUsersInput;

		// Post a UART message to prompt the user to enter the minute for the time
		pcData = "\r\n\nEnter the minute\r\n";
		xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );

		// Receive user's input
		memset(&cUartMsg, 0, sizeof(cUartMsg));
		xReadSuccess = xReceiveUartMsg(cUartMsg, pxQuitCurrentApp);
		lUsersInput = lUartMsgtoInt32(cUartMsg);

		// If the UART read was successful, the user did not quit the sub-application, and user's input is within 0-59
		// then go ahead and prompt the user to enter the second for the time
		if ( xReadSuccess == pdTRUE && *pxQuitCurrentApp == pdFALSE && lUsersInput >= 0 && lUsersInput <= 59 )
		{
			// Set the minute
			xTimeConfig.RTC_Minutes = lUsersInput;

			// Post a UART message to prompt the user to enter the second for the time
			pcData = "\r\n\nEnter the second\r\n";
			xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );

			// Receive user's input
			memset(&cUartMsg, 0, sizeof(cUartMsg));
			xReadSuccess = xReceiveUartMsg(cUartMsg, pxQuitCurrentApp);
			lUsersInput = lUartMsgtoInt32(cUartMsg);

			// If the UART read was successful, the user did not quit the sub-application, and user's input is within 0-59
			// then set the RTC peripheral with the configured hour, minute, and second
			if ( xReadSuccess == pdTRUE && *pxQuitCurrentApp == pdFALSE && lUsersInput >= 0 && lUsersInput <= 59 )
			{
				// Set the seconds
				xTimeConfig.RTC_Seconds = lUsersInput;

				// Apply the new time configured
				RTC_SetTime( RTC_Format_BIN, &xTimeConfig);
			}
		}
	}

	// If the user did not request to quit the app then continue to set up the date as well
	if (*pxQuitCurrentApp == pdFALSE )
	{
		// Post a UART message to prompt the user to enter the selected day of the month
		pcData = "\r\n\nConfiguring the date\
				  \r\nEnter the day of the month\r\n";
		xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );

		// Receive user's input
		memset(&cUartMsg, 0, sizeof(cUartMsg));
		xReadSuccess = xReceiveUartMsg(cUartMsg, pxQuitCurrentApp);
		lUsersInput = lUartMsgtoInt32(cUartMsg);

		// If the UART read was successful, the user did not quit the sub-application, and the user entered
		// a valid day then go ahead and prompt the user to enter the selected month
		if( xReadSuccess == pdTRUE && *pxQuitCurrentApp == pdFALSE && IS_RTC_DATE(lUsersInput) == pdTRUE )
		{
			// Set the day
			xDateConfig.RTC_Date= lUsersInput;

			// Post a UART message to prompt the user to enter the selected month
			pcData = "\r\n\nEnter the month\r\n";
			xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );

			// Receive user's input
			memset(&cUartMsg, 0, sizeof(cUartMsg));
			xReadSuccess = xReceiveUartMsg(cUartMsg, pxQuitCurrentApp);
			lUsersInput = lUartMsgtoInt32(cUartMsg);

			// If the UART read was successful, the user did not quit the sub-application, and the user entered
			// a valid month then go ahead and prompt the user to enter the selected year
			if ( xReadSuccess == pdTRUE && *pxQuitCurrentApp == pdFALSE && IS_RTC_MONTH(lUsersInput) == pdTRUE )
			{
				// Set the month
				xDateConfig.RTC_Month = lUsersInput;

				// Post a UART message to prompt the user to enter the selected year
				pcData = "\r\n\nEnter the year\
						  \r\nEnter 20 for 2020\r\n";
				xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );

				// Receive user's input
				memset(&cUartMsg, 0, sizeof(cUartMsg));
				xReadSuccess = xReceiveUartMsg(cUartMsg, pxQuitCurrentApp);
				lUsersInput = lUartMsgtoInt32(cUartMsg);

				// If the UART read was successful, the user did not quit the sub-application, and the user entered
				// a valid year then go ahead and prompt the user to select the day of the week
				if ( xReadSuccess == pdTRUE && *pxQuitCurrentApp == pdFALSE && lUsersInput >= 0 && lUsersInput <= 99 )
				{
					// Set the year
					xDateConfig.RTC_Year = lUsersInput;

					// Post a UART message to prompt the user to select the day of the week
					pcData = "\r\n\nEnter the day of the week\
							  \r\nEnter 1 for Monday\
							  \r\nEnter 2 for Tuesday\
							  \r\nEnter 3 for Wednesday\
							  \r\nEnter 4 for Thursday\
							  \r\nEnter 5 for Friday\
							  \r\nEnter 6 for Saturday\
							  \r\nEnter 7 for Sunday\r\n";

					xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );

					//Receive user's input
					memset(&cUartMsg, 0, sizeof(cUartMsg));
					xReadSuccess = xReceiveUartMsg(cUartMsg, pxQuitCurrentApp);
					lUsersInput = lUartMsgtoInt32(cUartMsg);

					// If the UART read was successful, the user did not quit the sub-application, and the user entered
					// a valid week day then set the RTC peripheral with the configured date
					if ( xReadSuccess == pdTRUE && *pxQuitCurrentApp == pdFALSE && IS_RTC_WEEKDAY(lUsersInput) == pdTRUE )
					{
						// Set the day of the week
						xDateConfig.RTC_WeekDay = lUsersInput;

						// Apply the new date configured
						if( RTC_SetDate( RTC_Format_BIN, &xDateConfig) != SUCCESS)
						{
							pcData = "\r\n\nRTC set date error\r\n";
							xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );
						}
					}
				}
			}
		}
	}
}
/*******************************************************************************
*   Procedure: vManageAppSleep
*
*   Description: This function executes under the Main Menu task function once
*   			 the user has chosen to put the application to sleep. It first
*   			 stops any LED toggle. Then it enables interrupt reception through
*   			 UART2 in order to allow the user to exit the sleep mode.
*   			 xTaskNotifyWait() is then called which puts the Main Menu task
*   			 in blocked state and allows the idle task to run. In the idle
*   			 task hook function, a WFI (Wait For Interrupt) thumb instruction
*   			 is called to put the application to sleep. Once the user presses
*   			 any button in the UART window an interrupt is generated and the
*   			 sleep mode is existed.
*
*   Notes:	None
*
*   Parameters: None
*
*   Return:	None
*
*******************************************************************************/
static void vManageAppSleep(void)
{
	char* pcData = NULL;   // To hold the address of the message to post to UART write queue

	// If the LED toggle is configured then delete the timer in order to block the timer service task
	if( pxLedToggleTimer != NULL)
	{
		// Stop LED toggle if it's on and switch off the LED
		vLedToggleDisable();

		// The calling task will be held in blocked state indefinitely waiting for the delete command
		// to be successfully sent to the timer command queue
		xTimerDelete( pxLedToggleTimer, portMAX_DELAY );
	}

	// Set the xRunTempMonitor flag to false to stop the temp monitor if running.
	// This will put the task in blocked state waiting a for notification to re-start
	xRunTempMonitor = pdFALSE;

	// Setup UART Rx Interrupt in order to use as a wake up method
	// Turn on interrupt for Receive Buffer Not Empty (RXNE) flag
	USART_ITConfig( USART2, USART_IT_RXNE, ENABLE );

	// Set priority for USART2 interrupt in NVIC
	// By default the priority of newly enabled interrupt will be 0
	// The priority cannot be less than 5 as per configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
	NVIC_SetPriority( USART2_IRQn, 5 );

	// Enable UART2 interrupt reception at the NVIC
	NVIC_EnableIRQ(USART2_IRQn);

	// Set the xGoToSleep flag to true so that the idle hook function will run the WFI instruction
	xGoToSleep = pdTRUE;

	pcData = "\r\n\nWent to sleep\
			  \r\nPress any keyboard letter/number to wake up\r\n";
	xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );

	// Wait in blocked state indefinitely till a notification is received
	xTaskNotifyWait( 0, 0, NULL, portMAX_DELAY);

	// To resume from here once a task notification is received
	// Print a message that we woke up
	pcData = "\r\nWoke up from sleep mode\r\n";
	xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );

	// On exit from normal sleep mode, disable UART Rx Interrupt
	// This is to prevent it from running during UART Rx in blocking (non-interrupt) mode
	USART_ITConfig( USART2, USART_IT_RXNE, DISABLE );
	NVIC_DisableIRQ(USART2_IRQn);
}
/*******************************************************************************
*   Procedure: vManageLedToggle
*
*   Description: This function is executed under the Main Menu task function. It
*   			 prompts the user to choose between start or stop toggling the
*   			 LED. It executes vLedToggleEnable() or vLedToggleDisable()
*   			 according to the user's input
*
*   Notes:	None
*
*   Parameters: None
*
*   Return:	None
*
*******************************************************************************/
static void vManageLedToggle(void)
{
	char* pcData = NULL;					// To hold the address of the message to post to UART write queue
	char cUartMsg[50] = {0};				// Buffer to receive messages via UART
	BaseType_t xQuitCurrentApp = pdFALSE;   // Flag to indicate if the user requested to quit the application
	BaseType_t xReadSuccess = pdFALSE;      // Flag to indicate if reading user's input via UART was successful

	// Post a UART message to prompt the user to select to toggle or stop toggling the LED
	pcData = "\r\nToggle the LED?\
	          \r\nTo start toggling the LED press ---> y/Y\
	          \r\nTo stop toggling the LED press  ---> n/N\r\n";

	xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );

	// Receive user's input
	memset(&cUartMsg, 0, sizeof(cUartMsg));
	xReadSuccess = xReceiveUartMsg(cUartMsg, &xQuitCurrentApp);

	// If the UART read was successful and the user did not request to quit the application
	// then enable or disable toggling the LED according to user's selection
	if( xReadSuccess == pdTRUE && xQuitCurrentApp == pdFALSE)
	{
		if( cUartMsg[0] == 'y' || cUartMsg[0] == 'Y' )
		{
			// Start toggling the LED at 500 msec
			vLedToggleEnable( pdMS_TO_TICKS(500) );
		}
		else if( cUartMsg[0] == 'n' || cUartMsg[0] == 'N' )
		{
			// Stop toggling the LED
			vLedToggleDisable();
		}
	}
}
/*******************************************************************************
*   Procedure: vAdcSetup
*
*   Description: Configure the ADC to use for analog temperature measurement
*
*   Notes:None
*
*   Parameters: None
*
*   Return:	None
*
*******************************************************************************/
static void vAdcSetup(void)
{
	//Enable the ADC interface clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	// Fills each xAdcInit member with its default value.
	ADC_StructInit( &xAdcInit );

	// Initializes ADC1 according to the specified parameters in the xAdcInit
	ADC_Init(ADC1, &xAdcInit);

	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);

	fMeasureTemp();
}
/*******************************************************************************
*   Procedure: fMeasureTemp
*
*   Description: This function measures and returns the temperature acquired from
*   			 the internal temperature sensor on Nucleo board.
*
*   Notes: None
*
*   Parameters: None
*
*   Return:	float - The measured temperature
*
*******************************************************************************/
static float fMeasureTemp(void)
{
#define TS_CAL_30C_ADDR		 (uint16_t*)(0x1FFF7A2C) 		// Temp sensor calibration data @ 3.3V, 30C
#define TS_CAL_110C_ADDR	 (uint16_t*)(0x1FFF7A2E)		// Temp sensor calibration data @ 3.3V, 110C
#define TS_CAL_REF_VOLTAGE	 (float)3.3						// Reference voltage used at time of calibration

	float fRefVoltage;				// Current reference voltage
	float fCal30CScaled;			// Scaled temp sensor 30C calibration data
	float fCal110CScaled;			// Scaled temp sensor 110C calibration data
	uint16_t usAdc1Data;			// Raw temp reading acquired from ADC1
	float fTemp;					// Calculated temp in C

	// Disable the VBAT (Voltage Battery) channel so we can measure the temp sensor channel
	ADC_VBATCmd(DISABLE);

	// Enable the temperature sensor channel
	ADC_TempSensorVrefintCmd(ENABLE);

	// Measure the actual VDDA using VRefInt
	// VDDA is the internal reference voltage for our analog to digital conversion
	fRefVoltage = fMeasureVDDA();

	// Measure the temperature now

	// Configure for the TS channel its rank in the sequencer and its sample time
	// The minimum time needed to sample the temp sensor is 10 usec.
	// Thus the ADC Sample Time parameter must be set accordingly.
	// The ADC Sample Time is dependent on the ADCCLK.
	// The selected default config for the ADCCLK is as follows: ADCCLK = APB2CLK/2.
	// Since we call RCC_DeInit() at the beginning of main(), we know that
	// our system clock is HSI (16 MHz) and the prescalar for AHB and APB2
	// busses is 1. Also, APB2CLK = AHBCLK/prescalar and AHBCLK= SYSCLK/prescalar
	// Thus, AHBCLK = 16/1 = 16 MHz, APB2CLK = 16/1 = 16 MHz, and ADCCLK = 16/2 = 8 MHz
	// The ADC Sample Time Needed = (10u/(1/8M)) = 80 cycles of ADCCLK
	ADC_RegularChannelConfig(ADC1, ADC_Channel_18, 1, ADC_SampleTime_84Cycles);

	// Scale the temp sensor calibration data to the current reference voltage
	fCal30CScaled = ((float)(*TS_CAL_30C_ADDR))*(fRefVoltage/TS_CAL_REF_VOLTAGE);
	fCal110CScaled = ((float)(*TS_CAL_110C_ADDR))*(fRefVoltage/TS_CAL_REF_VOLTAGE);

	// Start data conversion from analog to digital for regular channels
	ADC_SoftwareStartConv(ADC1);

	// Clear the start of conversion flag for regular channels
	ADC_ClearFlag( ADC1, ADC_FLAG_STRT );

	// Wait while the ADC is not finished with the conversion
	while( ADC_GetFlagStatus( ADC1, ADC_FLAG_EOC) != SET );

	// Reading the data converted from analog to digital
	// Reading from the ADC DR also clears the EOC flag for regular channels
	usAdc1Data = ADC_GetConversionValue(ADC1);

	// Compute the temperature
	fTemp = (((float)usAdc1Data - fCal30CScaled)/(fCal110CScaled-fCal30CScaled)*(110.0-30.0))+30.0;

	// Return the measured temperature
	return ( fTemp );
}
/*******************************************************************************
*   Procedure: fMeasureVDDA
*
*   Description: This function measures and averages the value for VDDA - the
*   			 voltage used as internal reference voltage in analog to digital
*   			 conversions. The actual VDDA is measured using VRefInt. Having
*   			 a more accurate VDDA measurement allows to have better temperature
*   			 measurements.
*
*   Notes: None
*
*   Parameters: None
*
*   Return:	float - The measured VDDA averaged over 20 values
*
*******************************************************************************/
static float fMeasureVDDA(void)
{
#define VREFINT_CAL_ADDR	(uint16_t*)(0x1FFF7A2A) 	// Internal reference voltage calibration value @ 3.3V, 30C
	uint8_t ucIndex = 0;								// Index used to get 21 VDDA readings to average
	uint16_t usAdc1Data;								// Raw temp reading acquired from ADC1
	float fVDDA = 0;									// The actual VDDA calculated

	// Configure for the VREFINT channel its rank in the sequencer and its sample time
	// ADCCLK = 8 MHz. The ADC Sample Time Needed = (10u/(1/8M)) = 80 cycles of ADCCLK
	ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 1, ADC_SampleTime_84Cycles);

	// Acquire 21 VDDA measurements and ignore the first because it is an outlier
	for(ucIndex = 0; ucIndex < 21; ucIndex++)
	{
		// Start data conversion from analog to digital for regular channels
		ADC_SoftwareStartConv(ADC1);

		// Clear the start of conversion flag for regular channels
		ADC_ClearFlag( ADC1, ADC_FLAG_STRT );

		// Wait while the ADC is not finished with the conversion
		while( ADC_GetFlagStatus( ADC1, ADC_FLAG_EOC) != SET );

		// Reading the data converted from analog to digital
		// Reading from the ADC DR also clears the EOC flag for regular channels
		usAdc1Data = ADC_GetConversionValue(ADC1);

		// Ignore the first measurement because it looks like an outlier
		if( ucIndex != 0)
		{
			// Compute VDDA and add it to a sum of up to 20 readings
			fVDDA += (((float)(*VREFINT_CAL_ADDR))/((float)usAdc1Data))*3.3;
		}
	}

	// Compute the average of those 20 readings
	fVDDA /= 20.0;

	return ( fVDDA );
}
/*******************************************************************************
*   Procedure: vManageTempMonitor
*
*   Description: This function is executed under the Main Menu task function. It
*   			 prompts the user to choose from
*   			 - Start temperature monitoring
*   			 - Show temperature statistics (current, highest, and lowest
*   			   temperatures recorded)
*   			 - Quit temperature monitoring.
*   			 If temperature monitoring is selected then the Temperature
*   			 Monitor task will be notified to run. It will then run in the
*   			 background. No temperature statistics can be displayed if the
*   			 temperature monitor is not started yet
*   Notes: None
*
*   Parameters: None
*
*   Return:	None
*
*******************************************************************************/
static void vManageTempMonitor(void)
{
	char* pcData = NULL;				   // To hold the address of the message to post to UART write queue
	char cUartMsg[150] = {0};   		   // Buffer to send and receive messages via UART
	BaseType_t xReadSuccess = pdFALSE;     // Flag to indicate if reading user's input via UART was successful
	BaseType_t xQuitCurrentApp = pdFALSE;  // Flag to indicate if the user requested to quit
	uint8_t ucOptionSelected = 0;  		   // To hold the user's selected option

	memset(&cUartMsg, 0, sizeof(cUartMsg));

	// Prompt the user to select one of the below options
	pcData = "\r\n\nThis is a temperature monitoring sub-application\
			  \r\nStart temperature monitoring	 ------> 1\
			  \r\nDisplay temperature statistics   ------> 2\
			  \r\nStop temperature monitoring  	 ------> 3\
			  \r\nEnter your option here: ";

	xQueueSend( xUartWriteQueue, &pcData, portMAX_DELAY );

	memset(&cUartMsg, 0, sizeof(cUartMsg));

	// Receive user's selected option
	xReadSuccess = xReceiveUartMsg(cUartMsg, &xQuitCurrentApp);

	// If the UART read was successful and the user did not request to quit the application
	if( xReadSuccess == pdTRUE && xQuitCurrentApp == pdFALSE )
	{
		// Return the first byte from the buffer, which will contain the user's selection
		// 48 is subtracted to convert from ASCII to a number
		ucOptionSelected = cUartMsg[0] - 48;

		switch( ucOptionSelected )
		{
			case 1:

				// The user has selected to run temp monitor sub-application
				// Notify the temp monitor task to run it
				xTaskNotify( xTempMonitorTaskHandle, 0, eNoAction );

				// Post a UART message to the user indicating that the temp monitor
				// sub-application has been started
				vPostMsgToUartQueue("\r\n\nTemperature monitor started\r\n");

				// Apply a delay in order to allow the above UART message to be sent
				// to UART write queue first before the Main Menu options
				vTaskDelay(pdMS_TO_TICKS(500));
				break;

			case 2:

				// Check if the temp monitor is running already
				// If not then no temp stats exist or can be displayed
				if( xRunTempMonitor == pdFALSE)
				{
					vPostMsgToUartQueue("\r\n\nTemperature monitor has not been started yet\
							             \r\nNo temperature statistics exist\r\n");
				}
				else
				{
					// The user has selected to display temp monitor stats
					xShowTemps = pdTRUE;

					// Apply a delay in order to allow temp monitor stats to be sent
					// to UART write queue first before the Main Menu options
					vTaskDelay(pdMS_TO_TICKS(500));
				}

				break;

			case 3:

				// Set the xRunTempMonitor flag to false to stop temp monitoring
				xRunTempMonitor = pdFALSE;

				// Post a UART message to the user indicating that the temp monitor
				// sub-application has been stopped
				vPostMsgToUartQueue("\r\n\nTemperature monitor stopped\r\n");
				break;

			default:

				// Post a message to the UART write queue indicating that the option
				// selected is not recognized
				vPostMsgToUartQueue("\r\n\nError: Unrecognized option selected\r\n");
				break;
		}
	}
}
