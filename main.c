/**
  ******************************************************************************
  * @file    Multi/Examples/MotionControl/IHM04A1_ExampleFor1BiDirMotor/Src/main.c
  * @author  IPC Rennes
  * @version V1.0.0
  * @date    January 06, 2015
  * @brief   This example shows how to use 1 IHM04A1 expansion board with
  * 1 bidirectionnal Brush DC motor.
  * The motor has one lead connected to the bridge A, the other lead to bridge B.
  * Bridge input 1A has to be parralelised with bridge input 2A.
  * Bridge input 1B has to be parralelised with bridge input 2B.
  * The demo sequence starts when the user button is pressed.
  * Each time, the user button is pressed, the demo step is changed
  ******************************************************************************
*  Oct 2021 Paul Snyder
*  Merged Project 8 with Project 12.
*  Goal: Read analog input signal from pot, control motor speed
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_nucleo.h"
#include "stm32f4xx_it.h"

#include "x_nucleo_ihmxx.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include <stdlib.h> // For malloc/free
#include <string.h> // For memset


/** @defgroup IHM04A1_Example_for_1_Bidirectionnal_motor
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define for Motor Control ------------------------------------------------------------*/
#define MAX_STEPS (9)

/* Private define for ADC ------------------------------------------------------------*/

/* Definitions of ADC hardware constraints delays */
/* Note: Only ADC IP HW delays are defined in ADC LL driver driver,           */
/*       not timeout values:                                                  */
/*       Timeout values for ADC operations are dependent to device clock      */
/*       configuration (system clock versus ADC clock),                       */
/*       and therefore must be defined in user application.                   */
/*       Refer to @ref ADC_LL_EC_HW_DELAYS for description of ADC timeout     */
/*       values definition.                                                   */

  /* Timeout values for ADC operations. */
  /* (enable settling time, disable settling time, ...)                       */
  /* Values defined to be higher than worst cases: low clock frequency,       */
  /* maximum prescalers.                                                      */
  /* Example of profile very low frequency : ADC clock frequency 36MHz        */
  /* prescaler 2, sampling time 56 ADC clock cycles, resolution 12 bits.      */
  /*  - ADC enable time: maximum delay is 3 us                                */
  /*    (refer to device datasheet, parameter "tSTAB")                        */
  /*  - ADC disable time: maximum delay should be a few ADC clock cycles      */
  /*  - ADC stop conversion time: maximum delay should be a few ADC clock     */
  /*    cycles                                                                */
  /*  - ADC conversion time: with this hypothesis of clock settings, maximum  */
  /*    delay will be 99us.                                                   */
  /*    (refer to device reference manual, section "Timing")                  */
  /* Unit: ms                                                                 */
  #define ADC_CALIBRATION_TIMEOUT_MS       ((uint32_t)   1)
  #define ADC_ENABLE_TIMEOUT_MS            ((uint32_t)   1)
  #define ADC_DISABLE_TIMEOUT_MS           ((uint32_t)   1)
  #define ADC_STOP_CONVERSION_TIMEOUT_MS   ((uint32_t)   1)
  #define ADC_CONVERSION_TIMEOUT_MS        ((uint32_t)   2)

/* Definitions of environment analog values */
  /* Value of analog reference voltage (Vref+), connected to analog voltage   */
  /* supply Vdda (unit: mV).                                                  */
  #define VDDA_APPLI                       ((uint32_t)3300)


/* Definitions of data related to this example */
  /* Definition of ADCx conversions data table size */
  /* Size of array set to ADC sequencer number of ranks converted,            */
  /* to have a rank in each array address.                                    */
  #define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)   3)

/* Private macro -------------------------------------------------------------*/
/* Private variables - Motor Control ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
 static volatile uint16_t gLastError;
 static volatile bool gButtonPressed = FALSE;
 static volatile uint8_t gStep = MAX_STEPS;


 float motorcommand; /* command to motor */

 deviceParams_t initDeviceParameters =
 {
		 L6206_CONF_PARAM_PARALLE_BRIDGES,
		 {L6206_CONF_PARAM_FREQ_PWM1A, L6206_CONF_PARAM_FREQ_PWM2A, L6206_CONF_PARAM_FREQ_PWM1B, L6206_CONF_PARAM_FREQ_PWM2B},
		 {100,100,100,100},
		 {FORWARD,FORWARD,BACKWARD,FORWARD}, /* initial FORWARD,FORWARD,BACkWARD,FORWARD - Paul*/
		 {INACTIVE,INACTIVE,INACTIVE,INACTIVE},
		 {FALSE,FALSE}
 };

/* PRIVATE VARIABLES - executive and ADC */

 /* executive variables */
 static uint32_t intrpt_numloops1; /* set the number of times the loop 1 interrupt will fire before action taken */
 static uint32_t intrpt_numloops2; /* set the number of times the loop 2 interrupt will fire before action taken */

 static uint32_t intrpt_loopcount1; /* current count of number of loops of Loop 1 */
 static uint32_t intrpt_loopcount2; /* current count of number of loops of Loop 2 */

 /* filter variables */
 float inputsample; /* value of analog signal read from ADC */
 double inputsample_old; /* value of analog signal read from ADC */
 float filterout;  /* computed value of signal after filter */
 double filterout_old1;  /* previous value of signal after filter */
 double filterout_old2;  /* oldest value of signal after filter */

 /* Variables for ADC conversion data */
 __IO uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]; /* ADC group regular conversion data */

 /* Variable to report status of DMA transfer of ADC group regular conversions */
 /*  0: DMA transfer is not completed                                          */
 /*  1: DMA transfer is completed                                              */
 /*  2: DMA transfer has not been started yet (initial state)                  */
 __IO uint8_t ubDmaTransferStatus = 2; /* Variable set into DMA interruption callback */

 /* Variable to report status of ADC group regular sequence conversions:       */
 /*  0: ADC group regular sequence conversions are not completed               */
 /*  1: ADC group regular sequence conversions are completed                   */
 __IO uint8_t ubAdcGrpRegularSequenceConvStatus = 0; /* Variable set into ADC interruption callback */

 /* Variable to report number of ADC group regular sequence completed          */
 static uint32_t ubAdcGrpRegularSequenceConvCount = 0; /* Variable set into ADC interruption callback */

 /* Variables for ADC conversion data computation to physical values */
 __IO uint16_t uhADCxConvertedData_VoltageGPIO_mVolt = 0;        /* Value of voltage on GPIO pin (on which is mapped ADC channel) calculated from ADC conversion data (unit: mV) */
 __IO uint16_t uhADCxConvertedData_VrefInt_mVolt = 0;            /* Value of internal voltage reference VrefInt calculated from ADC conversion data (unit: mV) */
 __IO  int16_t hADCxConvertedData_Temperature_DegreeCelsius = 0; /* Value of temperature calculated from ADC conversion data (unit: degree Celcius) */
 __IO uint16_t uhADCxConvertedData_VrefAnalog_mVolt = 0;         /* Value of analog reference voltage (Vref+), connected to analog voltage supply Vdda, calculated from ADC conversion data (unit: mV) */

 /* Private variables - Discrete Output ---------------------------------------------------------*/
 static GPIO_InitTypeDef  GPIO_InitStruct;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

static void     SystemClock_Config(void); /* hopefully this will work as it did in Project 4 even tho not static */

void     Configure_DMA(void);
void     Configure_ADC(void);
void     Activate_ADC(void);
void     LED_Init(void);
void     LED_On(void);
void     LED_Off(void);
void     LED_Blinking(uint32_t Period);
void     UserButton_Init(void);


static void MyFlagInterruptHandler(void);

void ButtonHandler(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32xx HAL library initialization */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

    /* Configure KEY Button */

  /* Set Systick Interrupt to the highest priority to have HAL_Delay working*/
  /* under the user button handler */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0x0, 0x0);

  //----- Init of the Motor control library
  /* Set the L6208 library to use 1 device */
  BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_L6206, 1);
  /* When BSP_MotorControl_Init is called with NULL pointer,                  */
  /* the L6206 parameters are set with the predefined values from file        */
  /* l6206_target_config.h, otherwise the parameters are set using the        */
  /* initDeviceParameters structure values.                                   */
  BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6206, &initDeviceParameters);

  /* Select the configuration with paralleling of brigde input IN1A with IN2A,
  and with paralleling of brigde input IN1B with IN2B with the use of one
  bidirectionnal motor */
  BSP_MotorControl_SetDualFullBridgeConfig(PARALLELING_IN1A_IN2A__IN1B_IN2B__1_BIDIR_MOTOR);

  /* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
  BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);

  /* Attach the function Error_Handler (defined below) to the error Handler*/
  BSP_MotorControl_AttachErrorHandler(Error_Handler);

  /* Set PWM Frequency of bridge A inputs to 10000 Hz */
  BSP_MotorControl_SetBridgeInputPwmFreq(0,20000);

  /* Set PWM Frequency of bridge B inputs to 10000 Hz */
  BSP_MotorControl_SetBridgeInputPwmFreq(1,20000);

 /* Switch Initialization */
	GPIO_InitTypeDef  GPIO_InitStruct;

	  /*##-1- Enable GPIOA Clock (to be able to program the configuration registers) */
	  __HAL_RCC_GPIOA_CLK_ENABLE();

	  /*##-2- Configure PA8 IO in output push-pull mode to drive scope ###*/
	  GPIO_InitStruct.Pin = GPIO_PIN_8;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);




  /* Set Acceleration to low value pps^2 in order to limit current demand - Paul */
  /* BSP_MotorControl_SetAcceleration(0,77);  */


  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();

  /* Configure the system clock to 100 MHz */
SystemClock_Config();

/* Initialize LED2 */
LED_Init();

/* Initialize button in EXTI mode */
UserButton_Init();

/* Configure DMA for data transfer from ADC */
Configure_DMA();

/* Configure ADC */
/* Note: This function configures the ADC but does not enable it.           */
/*       To enable it, use function "Activate_ADC()".                       */
/*       This is intended to optimize power consumption:                    */
/*       1. ADC configuration can be done once at the beginning             */
/*          (ADC disabled, minimal power consumption)                       */
/*       2. ADC enable (higher power consumption) can be done just before   */
/*          ADC conversions needed.                                         */
/*          Then, possible to perform successive "Activate_ADC()",          */
/*          "Deactivate_ADC()", ..., without having to set again            */
/*          ADC configuration.                                              */
Configure_ADC();

/* Activate ADC */
/* Perform ADC activation procedure to make it ready to convert. */
Activate_ADC();


/* Executive initialization */

intrpt_numloops1 = 10; /* Frame 1 freq = 1000 div intrpt_numloops1 */
intrpt_numloops2 = 10; /* Frame 2 freq = 1000 div intrpt_numloops2 */
intrpt_loopcount1 = 1;
intrpt_loopcount2 = 1;

  /* Infinite loop */
  while(1)
  {
	  /****  Frame 1 begin - Frequency = (1 div intrpt_numloops1) ******/
	    while ( (uwTick-intrpt_loopcount1) > intrpt_numloops1 )
	    {
	    	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8); /* set reference signal to o-scope */
	        /****  Frame 1 - Task 1.1 - Read Discrete Inputs ******/

	        /****  Frame 1 - Task 1.2 - Compute Mode Logic ******/

	        /****  Frame 1 - Task 1.3 - Read Discrete Inputs ******/

	        /****  Frame 1 - Task 1.4 - Read Analog Inputs ******/
	    	    /* Note: ADC group regular conversion start is done into push button      */
	    	    /*       IRQ handler, refer to function "UserButton_Callback()".          */

	    	    /* Note: LED state depending on DMA transfer status is set into DMA       */
	    	    /*       IRQ handler, refer to function "DmaTransferComplete()".          */

	    	    /* Note: For this example purpose, number of ADC group regular sequences  */
	    	    /*       completed are stored into variable                               */
	    	    /*       "ubAdcGrpRegularSequenceConvCount"                               */
	    	    /*       (for debug: see variable content into watch window)              */

	    	    /* Note: ADC conversions data are stored into array "aADCxConvertedData"  */
	    	    /*       (for debug: see variable content into watch window).             */
	    	    /*       Each rank of the sequence is at an address of the array:         */
	    	    /*       - aADCxConvertedData[0]: ADC channel set on rank1                */
	    	    /*                                (ADC1 channel 4)                        */
	    	    /*       - aADCxConvertedData[1]: ADC channel set on rank2                */
	    	    /*                                (ADC1 internal channel VrefInt)         */
	    	    /*       - aADCxConvertedData[2]: ADC channel set on rank3                */
	    	    /*                                (ADC1 internal channel temperature sensor)*/

	    	    /* Start ADC Conversion - this does not require a button to be pressed */
	    	    UserButton_Callback();

	    	    /* Wait for ADC conversion and DMA transfer completion to process data */
	    	    while(ubDmaTransferStatus != 1)
	    	    {
	    	    }

	        /****  Frame 1 - Task 2 - implement handmade filter - 2 tap FIR ******/

	        /****  Frame 1 - Task 3 - implement Modeller filter 1 - low pass - commented out because filter 2 was last used  ******/

	        /****  Frame 1 - Task 4 - implement Modeller filter 2 - low pass  ******/
	     	inputsample = uhADCxConvertedData_VoltageGPIO_mVolt;

		    /* Computation of ADC conversions raw data to physical values             */
		    /* using LL ADC driver helper macro.                                      */
		    uhADCxConvertedData_VoltageGPIO_mVolt        = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, aADCxConvertedData[0], LL_ADC_RESOLUTION_12B);
		    uhADCxConvertedData_VrefInt_mVolt            = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, aADCxConvertedData[1], LL_ADC_RESOLUTION_12B);
		    hADCxConvertedData_Temperature_DegreeCelsius = __LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(INTERNAL_TEMPSENSOR_AVGSLOPE,
			                                                                                    INTERNAL_TEMPSENSOR_V25,
		                                                                                        INTERNAL_TEMPSENSOR_V25_TEMP,
		                                                                                        VDDA_APPLI,
		                                                                                        aADCxConvertedData[2],
		                                                                                        LL_ADC_RESOLUTION_12B);	        /****  Frame 1 - Task 8 - Set Discrete Outputs ******/

	        /****  Frame 1 - Task 9 - Toggle PA05 IO ******/

	        /****  Frame 1 - Task 10 - Compute Control Law For Motor ******/
		    /* motorcommand is scaled to 100 max */
		    motorcommand = (100*inputsample)/3300;

		    /* the input pot is only a positive voltage, so arbitrarily set the
		     * zero command point at half max
		     */
            if (motorcommand > (53)) /* command is greater than 3% fullscale positive */
            {
            	BSP_MotorControl_Run(0, FORWARD);
            	BSP_MotorControl_SetMaxSpeed(0,2*(motorcommand-53)+6);
            }
            else if (motorcommand < (47)) /* command is greater than 3% fullscale negative */
            {
            	BSP_MotorControl_Run(0, BACKWARD);
            	BSP_MotorControl_SetMaxSpeed(0,2*(47-motorcommand)+6);
            }
            else /* command is within a 1% deadband around zero */
            {
            }


	    	/**** End of all Tasks in Frame 1  ****/
	        intrpt_loopcount1 = uwTick; /** reset loopcount1 for Frame 1 restart **/

	    	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8); /* REset reference signal to o-scope */
	    }
	    /****  Frame 1 end *********************************************/

	    /****  Frame 2 begin - Frequency = (1 div intrpt_numloops2) ******/
	    while ( (uwTick-intrpt_loopcount2) > intrpt_numloops2 )
	    {
	        /****  Frame 2 - Task 1 ******/

	        /****  Frame 2 - Task 2 ******/

	        /****  Frame 2 - Task 3 ******/

	        /****  Frame 2 - Task 4 ******/

	        intrpt_loopcount2 = uwTick; /** reset loopcount2 for frame restart **/
	    } /****  Frame 2 end *********************************************/

	  } /****** Executive End ******/

  } /* main end */

void Configure_DMA(void)
{
  /*## Configuration of NVIC #################################################*/
  /* Configure NVIC to enable DMA interruptions */
  NVIC_SetPriority(DMA2_Stream0_IRQn, 1);  /* DMA IRQ lower priority than ADC IRQ */
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);

  /*## Configuration of DMA ##################################################*/
  /* Enable the peripheral clock of DMA */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

  /* Configure the DMA transfer */
  /*  - DMA transfer in circular mode to match with ADC configuration:        */
  /*    DMA unlimited requests.                                               */
  /*  - DMA transfer from ADC without address increment.                      */
  /*  - DMA transfer to memory with address increment.                        */
  /*  - DMA transfer from ADC by half-word to match with ADC configuration:   */
  /*    ADC resolution 12 bits.                                               */
  /*  - DMA transfer to memory by half-word to match with ADC conversion data */
  /*    buffer variable type: half-word.                                      */
    LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_0, LL_DMA_CHANNEL_0);
  LL_DMA_ConfigTransfer(DMA2,
                        LL_DMA_STREAM_0,
                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
                        LL_DMA_MODE_CIRCULAR              |
                        LL_DMA_PERIPH_NOINCREMENT         |
                        LL_DMA_MEMORY_INCREMENT           |
                        LL_DMA_PDATAALIGN_HALFWORD        |
                        LL_DMA_MDATAALIGN_HALFWORD        |
                        LL_DMA_PRIORITY_HIGH               );

 /* Set DMA transfer addresses of source and destination */
  LL_DMA_ConfigAddresses(DMA2,
                        LL_DMA_STREAM_0,
                         LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
                         (uint32_t)&aADCxConvertedData,
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  /* Set DMA transfer size */
  LL_DMA_SetDataLength(DMA2,
                        LL_DMA_STREAM_0,
                       ADC_CONVERTED_DATA_BUFFER_SIZE);

  /* Enable DMA transfer interruption: transfer complete */
  LL_DMA_EnableIT_TC(DMA2,
                        LL_DMA_STREAM_0);

  /* Enable DMA transfer interruption: transfer error */
  LL_DMA_EnableIT_TE(DMA2,
                        LL_DMA_STREAM_0);

  /*## Activation of DMA #####################################################*/
  /* Enable the DMA transfer */
    LL_DMA_EnableStream(DMA2,LL_DMA_STREAM_0);
}

/**
  * @brief  Configure ADC (ADC instance: ADC1) and GPIO used by ADC channels.
  * @note   In case re-use of this function outside of this example:
  *         This function includes checks of ADC hardware constraints before
  *         executing some configuration functions.
  *         - In this example, all these checks are not necessary but are
  *           implemented anyway to show the best practice usages
  *           corresponding to reference manual procedure.
  *           (On some STM32 series, setting of ADC features are not
  *           conditioned to ADC state. However, in order to be compliant with
  *           other STM32 series and to show the best practice usages,
  *           ADC state is checked anyway with same constraints).
  *           Software can be optimized by removing some of these checks,
  *           if they are not relevant considering previous settings and actions
  *           in user application.
  *         - If ADC is not in the appropriate state to modify some parameters,
  *           the setting of these parameters is bypassed without error
  *           reporting:
  *           it can be the expected behavior in case of recall of this
  *           function to update only a few parameters (which update fullfills
  *           the ADC state).
  *           Otherwise, it is up to the user to set the appropriate error
  *           reporting in user application.
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
void Configure_ADC(void)
{
  __IO uint32_t wait_loop_index = 0;

  /*## Configuration of GPIO used by ADC channels ############################*/

  /* Note: On this STM32 device, ADC1 channel 4 is mapped on GPIO pin PA.04 */

  /* Enable GPIO Clock */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  /* Configure GPIO in analog mode to be used as ADC input */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);

  /*## Configuration of NVIC #################################################*/
  /* Configure NVIC to enable ADC1 interruptions */
  NVIC_SetPriority(ADC_IRQn, 0); /* ADC IRQ greater priority than DMA IRQ */
  NVIC_EnableIRQ(ADC_IRQn);

  /*## Configuration of ADC ##################################################*/

  /*## Configuration of ADC hierarchical scope: common to several ADC ########*/

  /* Enable ADC clock (core clock) */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features are not             */
  /*       conditioned to ADC state.                                          */
  /*       However, in order to be compliant with other STM32 series          */
  /*       and to show the best practice usages, ADC state is checked.        */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  if(__LL_ADC_IS_ENABLED_ALL_COMMON_INSTANCE() == 0)
  {
    /* Note: Call of the functions below are commented because they are       */
    /*       useless in this example:                                         */
    /*       setting corresponding to default configuration from reset state. */

    /* Set ADC clock (conversion clock) common to several ADC instances */
    LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_SYNC_PCLK_DIV2);

    /* Set ADC measurement path to internal channels */
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), (LL_ADC_PATH_INTERNAL_VREFINT | LL_ADC_PATH_INTERNAL_TEMPSENSOR));

    /* Delay for ADC temperature sensor stabilization time.                   */
    /* Compute number of CPU cycles to wait for, from delay in us.            */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles (depends on compilation optimization).     */
    /* Note: If system core clock frequency is below 200kHz, wait time        */
    /*       is only a few CPU processing cycles.                             */
    /* Note: This delay is implemented here for the purpose in this example.  */
    /*       It can be optimized if merged with other delays                  */
    /*       during ADC activation or if other actions are performed          */
    /*       in the meantime.                                                 */
    wait_loop_index = ((LL_ADC_DELAY_TEMPSENSOR_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
    while(wait_loop_index != 0)
    {
      wait_loop_index--;
    }

  /*## Configuration of ADC hierarchical scope: multimode ####################*/

    /* Note: ADC multimode is not available on this device:                   */
    /*       only 1 ADC instance is present.                                  */
    /* Set ADC multimode configuration */
    // LL_ADC_SetMultimode(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_MULTI_INDEPENDENT);

    /* Set ADC multimode DMA transfer */
    // LL_ADC_SetMultiDMATransfer(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_MULTI_REG_DMA_EACH_ADC);

    /* Set ADC multimode: delay between 2 sampling phases */
    // LL_ADC_SetMultiTwoSamplingDelay(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_MULTI_TWOSMP_DELAY_1CYCLE);

  }


  /*## Configuration of ADC hierarchical scope: ADC instance #################*/

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features are not             */
  /*       conditioned to ADC state.                                          */
  /*       However, ADC state is checked anyway with standard requirements    */
  /*       (refer to description of this function).                           */
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Note: Call of the functions below are commented because they are       */
    /*       useless in this example:                                         */
    /*       setting corresponding to default configuration from reset state. */

    /* Set ADC data resolution */
    // LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);

    /* Set ADC conversion data alignment */
    // LL_ADC_SetResolution(ADC1, LL_ADC_DATA_ALIGN_RIGHT);

    /* Set Set ADC sequencers scan mode, for all ADC groups                   */
    /* (group regular, group injected).                                       */
    LL_ADC_SetSequencersScanMode(ADC1, LL_ADC_SEQ_SCAN_ENABLE);

  }


  /*## Configuration of ADC hierarchical scope: ADC group regular ############*/

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features are not             */
  /*       conditioned to ADC state.                                          */
  /*       However, ADC state is checked anyway with standard requirements    */
  /*       (refer to description of this function).                           */
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Set ADC group regular trigger source */
    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);

    /* Set ADC group regular trigger polarity */
    // LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);

    /* Set ADC group regular continuous mode */
    LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);

    /* Set ADC group regular conversion data transfer */
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);

    /* Specify which ADC flag between EOC (end of unitary conversion)         */
    /* or EOS (end of sequence conversions) is used to indicate               */
    /* the end of conversion.                                                 */
    // LL_ADC_REG_SetFlagEndOfConversion(ADC1, LL_ADC_REG_FLAG_EOC_SEQUENCE_CONV);

    /* Set ADC group regular sequencer */
    /* Note: On this STM32 serie, ADC group regular sequencer is              */
    /*       fully configurable: sequencer length and each rank               */
    /*       affectation to a channel are configurable.                       */
    /*       Refer to description of function                                 */
    /*       "LL_ADC_REG_SetSequencerLength()".                               */

    /* Set ADC group regular sequencer length and scan direction */
    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS);

    /* Set ADC group regular sequencer discontinuous mode */
    // LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);

    /* Set ADC group regular sequence: channel on the selected sequence rank. */
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_VREFINT);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_TEMPSENSOR);
  }


  /*## Configuration of ADC hierarchical scope: ADC group injected ###########*/

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features are not             */
  /*       conditioned to ADC state.                                          */
  /*       However, ADC state is checked anyway with standard requirements    */
  /*       (refer to description of this function).                           */
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Note: Call of the functions below are commented because they are       */
    /*       useless in this example:                                         */
    /*       setting corresponding to default configuration from reset state. */

    /* Set ADC group injected trigger source */
    // LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_SOFTWARE);

    /* Set ADC group injected trigger polarity */
    // LL_ADC_INJ_SetTriggerEdge(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);

    /* Set ADC group injected conversion trigger  */
    // LL_ADC_INJ_SetTrigAuto(ADC1, LL_ADC_INJ_TRIG_INDEPENDENT);

    /* Set ADC group injected sequencer */
    /* Note: On this STM32 serie, ADC group injected sequencer is             */
    /*       fully configurable: sequencer length and each rank               */
    /*       affectation to a channel are configurable.                       */
    /*       Refer to description of function                                 */
    /*       "LL_ADC_INJ_SetSequencerLength()".                               */

    /* Set ADC group injected sequencer length and scan direction */
    // LL_ADC_INJ_SetSequencerLength(ADC1, LL_ADC_INJ_SEQ_SCAN_DISABLE);

    /* Set ADC group injected sequencer discontinuous mode */
    // LL_ADC_INJ_SetSequencerDiscont(ADC1, LL_ADC_INJ_SEQ_DISCONT_DISABLE);

    /* Set ADC group injected sequence: channel on the selected sequence rank. */
    // LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_4);
  }


  /*## Configuration of ADC hierarchical scope: channels #####################*/

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features are not             */
  /*       conditioned to ADC state.                                          */
  /*       However, in order to be compliant with other STM32 series          */
  /*       and to show the best practice usages, ADC state is checked.        */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Set ADC channels sampling time */
    /* Note: Considering interruption occurring after each ADC group          */
    /*       regular sequence conversions                                     */
    /*       (IT from DMA transfer complete),                                 */
    /*       select sampling time and ADC clock with sufficient               */
    /*       duration to not create an overhead situation in IRQHandler.      */
    /* Note: Set long sampling time due to internal channels (VrefInt,        */
    /*       temperature sensor) constraints.                                 */
    /*       Refer to description of function                                 */
    /*       "LL_ADC_SetChannelSamplingTime()".                               */
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_56CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_480CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SAMPLINGTIME_480CYCLES);

  }


  /*## Configuration of ADC transversal scope: analog watchdog ###############*/

  /* Note: On this STM32 serie, there is only 1 analog watchdog available.    */

  /* Set ADC analog watchdog: channels to be monitored */
  // LL_ADC_SetAnalogWDMonitChannels(ADC1, LL_ADC_AWD_DISABLE);

  /* Set ADC analog watchdog: thresholds */
  // LL_ADC_SetAnalogWDThresholds(ADC1, LL_ADC_AWD_THRESHOLD_HIGH, __LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B));
  // LL_ADC_SetAnalogWDThresholds(ADC1, LL_ADC_AWD_THRESHOLD_LOW, 0x000);


  /*## Configuration of ADC transversal scope: oversampling ##################*/

  /* Note: Feature not available on this STM32 serie */


  /*## Configuration of ADC interruptions ####################################*/
  /* Enable interruption ADC group regular end of unitary conversion          */
  /* or end of sequence conversions.                                          */
  /* Note: On this STM32 serie, ADC group regular end of conversion           */
  /*       must be selected among end of unitary conversion                   */
  /*       or end of sequence conversions.                                    */
  /*       Refer to function "LL_ADC_REG_SetFlagEndOfConversion()".           */
  LL_ADC_EnableIT_EOCS(ADC1);

  /* Enable interruption ADC group regular overrun */
  LL_ADC_EnableIT_OVR(ADC1);

  /* Note: in this example, ADC group regular end of conversions              */
  /*       (number of ADC conversions defined by DMA buffer size)             */
  /*       are notified by DMA transfer interruptions).                       */
  /*       ADC interruptions of end of conversion are enabled optionally,     */
  /*       as demonstration purpose in this example.                          */

}

/**
  * @brief  Perform ADC activation procedure to make it ready to convert
  *         (ADC instance: ADC1).
  * @note   Operations:
  *         - ADC instance
  *           - Enable ADC
  *         - ADC group regular
  *           none: ADC conversion start-stop to be performed
  *                 after this function
  *         - ADC group injected
  *           none: ADC conversion start-stop to be performed
  *                 after this function
  * @param  None
  * @retval None
  */
void Activate_ADC(void)
{
  #if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0; /* Variable used for timeout management */
  #endif /* USE_TIMEOUT */

  /*## Operation on ADC hierarchical scope: ADC instance #####################*/

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features are not             */
  /*       conditioned to ADC state.                                          */
  /*       However, in order to be compliant with other STM32 series          */
  /*       and to show the best practice usages, ADC state is checked.        */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Enable ADC */
    LL_ADC_Enable(ADC1);

  }

  /*## Operation on ADC hierarchical scope: ADC group regular ################*/
  /* Note: No operation on ADC group regular performed here.                  */
  /*       ADC group regular conversions to be performed after this function  */
  /*       using function:                                                    */
  /*       "LL_ADC_REG_StartConversion();"                                    */

  /*## Operation on ADC hierarchical scope: ADC group injected ###############*/
  /* Note: No operation on ADC group injected performed here.                 */
  /*       ADC group injected conversions to be performed after this function */
  /*       using function:                                                    */
  /*       "LL_ADC_INJ_StartConversion();"                                    */

}

/**
  * @brief  Initialize LED2.
  * @param  None
  * @retval None
  */
void LED_Init(void)
{
  /* Enable the LED2 Clock */
  LED2_GPIO_CLK_ENABLE();

  /* Configure IO in output push-pull mode to drive external LED2 */
  LL_GPIO_SetPinMode(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_MODE_OUTPUT);
  /* Reset value is LL_GPIO_OUTPUT_PUSHPULL */
  //LL_GPIO_SetPinOutputType(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  /* Reset value is LL_GPIO_SPEED_FREQ_LOW */
  //LL_GPIO_SetPinSpeed(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_SPEED_FREQ_LOW);
  /* Reset value is LL_GPIO_PULL_NO */
  //LL_GPIO_SetPinPull(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_PULL_NO);
}

/**
  * @brief  Turn-on LED2.
  * @param  None
  * @retval None
  */
void LED_On(void)
{
  /* Turn LED2 on */
  LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN);
}

/**
  * @brief  Turn-off LED2.
  * @param  None
  * @retval None
  */
void LED_Off(void)
{
  /* Turn LED2 off */
  LL_GPIO_ResetOutputPin(LED2_GPIO_PORT, LED2_PIN);
}

/**
  * @brief  Set LED2 to Blinking mode for an infinite loop (toggle period based on value provided as input parameter).
  * @param  Period : Period of time (in ms) between each toggling of LED
  *   This parameter can be user defined values. Pre-defined values used in that example are :
  *     @arg LED_BLINK_FAST : Fast Blinking
  *     @arg LED_BLINK_SLOW : Slow Blinking
  *     @arg LED_BLINK_ERROR : Error specific Blinking
  * @retval None
  */
void LED_Blinking(uint32_t Period)
{
  /* Turn LED2 on */
  LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN);

  /* Toggle IO in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
    LL_mDelay(Period);
  }
}

/**
  * @brief  Configures User push-button in EXTI Line Mode.
  * @param  None
  * @retval None
  */
void UserButton_Init(void)
{
  /* Enable the BUTTON Clock */
  USER_BUTTON_GPIO_CLK_ENABLE();

  /* Configure GPIO for BUTTON */
  LL_GPIO_SetPinMode(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_PULL_NO);

  /* if(Button_Mode == BUTTON_MODE_EXTI) */
  {
    /* Connect External Line to the GPIO */
    USER_BUTTON_SYSCFG_SET_EXTI();

    /* Enable a rising trigger EXTI line 13 Interrupt */
    USER_BUTTON_EXTI_LINE_ENABLE();
    USER_BUTTON_EXTI_FALLING_TRIG_ENABLE();

    /* Configure NVIC for USER_BUTTON_EXTI_IRQn */
    NVIC_EnableIRQ(USER_BUTTON_EXTI_IRQn);
    NVIC_SetPriority(USER_BUTTON_EXTI_IRQn,0x03);
  }
}


/**
  * @brief  This function is the User handler for the flag interrupt
  * @param  None
  * @retval None
  */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


    /* Enable HSE oscillator */
    LL_RCC_HSE_EnableBypass();
    LL_RCC_HSE_Enable();
    while(LL_RCC_HSE_IsReady() != 1)
    {
    };

    /* Set FLASH latency */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_3);

    /* Main PLL configuration and activation */
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 400, LL_RCC_PLLP_DIV_4);
    LL_RCC_PLL_Enable();
    while(LL_RCC_PLL_IsReady() != 1)
    {
    };

    /* Sysclk activation on the main PLL */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    {
    };

    /* Set APB1 & APB2 prescaler */
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

    /* Set systick to 1ms */
    SysTick_Config(100000000 / 1000);

    /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
    SystemCoreClock = 100000000;

    HAL_GetTick();

}

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/

/**
  * @brief  Function to manage IRQ Handler
  * @param  None
  * @retval None
  */
void UserButton_Callback(void)
{
  /* Turn LED off before performing a new ADC conversion start */
  /* (conversion of ranks set in ADC group regular sequencer).                */
  LED_Off();

  /* Reset status variable of DMA transfer before performing a new ADC        */
  /* conversion start of a sequence (in case of previous DMA transfer         */
  /* completed).                                                              */
  /* Note: Optionally, for this example purpose, check DMA transfer           */
  /*       status before starting another ADC conversion.                     */
  if (ubDmaTransferStatus != 0)
  {
    ubDmaTransferStatus = 0;
  }
  else
  {
    /* Error: Previous action (ADC conversion or DMA transfer) not yet        */
    /* completed.                                                             */
    LED_Blinking(LED_BLINK_ERROR);
  }

  /* Start ADC group regular conversion */
  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features are not             */
  /*       conditioned to ADC state.                                          */
  /*       However, in order to be compliant with other STM32 series          */
  /*       and to show the best practice usages, ADC state is checked.        */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  if (LL_ADC_IsEnabled(ADC1) == 1)
  {
    LL_ADC_REG_StartConversionSWStart(ADC1);
  }
  else
  {
    /* Error: ADC conversion start could not be performed */
    LED_Blinking(LED_BLINK_ERROR);
  }
}

/**
  * @brief  DMA transfer complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
void AdcDmaTransferComplete_Callback()
{
  /* Update status variable of DMA transfer */
  ubDmaTransferStatus = 1;

  /* Set LED depending on DMA transfer status */
  /* - Turn-on if DMA transfer is completed */
  /* - Turn-off if DMA transfer is not completed */
  LED_On();

  /* For this example purpose, check that DMA transfer status is matching     */
  /* ADC group regular sequence status:                                       */
  if (ubAdcGrpRegularSequenceConvStatus != 1)
  {
    AdcDmaTransferError_Callback();
  }

  /* Reset status variable of ADC group regular sequence */
  ubAdcGrpRegularSequenceConvStatus = 0;
}

/**
  * @brief  DMA transfer error callback
  * @note   This function is executed when the transfer error interrupt
  *         is generated during DMA transfer
  * @retval None
  */
void AdcDmaTransferError_Callback()
{
  /* Error detected during DMA transfer */
  LED_Blinking(LED_BLINK_ERROR);
}

/**
  * @brief  ADC group regular end of sequence conversions interruption callback
  * @note   This function is executed when the ADC group regular
  *         sequencer has converted all ranks of the sequence.
  * @retval None
  */
void AdcGrpRegularSequenceConvComplete_Callback()
{
  /* Update status variable of ADC group regular sequence */
  ubAdcGrpRegularSequenceConvStatus = 1;
  ubAdcGrpRegularSequenceConvCount++;
}

/**
  * @brief  ADC group regular overrun interruption callback
  * @note   This function is executed when ADC group regular
  *         overrun error occurs.
  * @retval None
  */
void AdcGrpRegularOverrunError_Callback(void)
{
  /* Note: Disable ADC interruption that caused this error before entering in */
  /*       infinite loop below.                                               */

  /* Disable ADC group regular overrun interruption */
  LL_ADC_DisableIT_OVR(ADC1);

  /* Error from ADC */
  LED_Blinking(LED_BLINK_ERROR);
}



void MyFlagInterruptHandler(void)
{
  /* Code to be customised */
  /************************/
  /* Get the state of bridge A */
  uint16_t bridgeState  = BSP_MotorControl_CmdGetStatus(0);

  if (bridgeState == 0)
  {
    if (BSP_MotorControl_GetDeviceState(0) != INACTIVE)
    {
      /* Bridge A was disabling due to overcurrent or over temperature */
      /* When  motor was running */
        Error_Handler(0XBAD0);
    }
  }
 }

/**
  * @brief  This function is executed when the Nucleo User button is pressed
  * @param  error number of the error
  * @retval None
  */
void ButtonHandler(void)
{
  gButtonPressed = TRUE;

  /* Let 300 ms before clearing the IT for key debouncing */
  HAL_Delay(300);
  __HAL_GPIO_EXTI_CLEAR_IT(KEY_BUTTON_PIN);
  HAL_NVIC_ClearPendingIRQ(KEY_BUTTON_EXTI_IRQn);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  error number of the error
  * @retval None
  */
void Error_Handler(uint16_t error)
{
  /* Backup error number */
  gLastError = error;

  /* Infinite loop */
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
