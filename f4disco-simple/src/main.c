/**
 * File: main.c in f4disco-simple
 * @author Chris Robinson and Tom Erbe
 * @date December 11, 2017
 * @version 1.0
 *
 * Description:  This is a final project for MUS177.
 * This code creates a tremolo effect by multiplying the signal,
 * which is in this case a sawtooth wave, by a low frequency osc.
 * The speed of the tremolo is automatically set to 500ms, but can
 * be changed by pressing the button twice at the desired speed.
 *
 */


// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <stm32f4_discovery_audio.h>
#include "diag/Trace.h"

#include "Timer.h"
#include "sinetable.h"
#include "freqtable.h"
#include "f0c.h"

// ----------------------------------------------------------------------------
//
// Standalone STM32F4 led blink sample (trace via ITM).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// Then demonstrates how to blink a led with 1 Hz, using a
// continuous loop and SysTick delays.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the ITM output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//


// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


// pins connected on discovery board
// A0 4 5 6 7 9 10 11 12
// B6 9 10
// C0 3 4 7 10 12 14 15
// D4 5 12 13 14 15
// E0 1 3
// DAC B0, B1,
// SWITCH D0 1 2 3
// Timer, LED and Switch globals
TIM_HandleTypeDef TimHandle;
float millisecs;
int32_t ledselected;
int32_t time, button_time[16];

// Switch globals
int32_t switchon[8];
int32_t switchoff[8];
// ADC globals
ADC_HandleTypeDef AdcHandle;
__IO uint16_t adcBuffer[8];
uint32_t smoothadc[8], avgtotal[8];
uint32_t adc0average[64];
uint32_t adc1average[64];
int16_t avgindex;

// DAC globals
DAC_HandleTypeDef    DacHandle;
uint16_t dacBuffer[32]; // 32 samples X 1 channels

// I2S globals
extern I2S_HandleTypeDef       hAudioOutI2s;
extern I2S_HandleTypeDef       hAudioInI2s;
int16_t codecBuffer[64];	// 32 samples X 2 channels
int bspAudioState;

// Sound globals
void audioBlock(float *input, float *output, int32_t samples);
float inBuffer[1024], outBuffer[1024]; // interleaved - LRLRLRLRLRLRLRLRLRLRLR - inBuffer[frame << 1] inBuffer[(frame << 1) + 1]

float wavetable[8192];

// Synthesis types
typedef struct rando
{
	uint32_t seed;
};
typedef struct osco
{
	float pi;	// phase increment
	float phs;	// phase
	float *tab;	// table address
	uint32_t ts;	// table size
	uint32_t tm;	// table mask aka table size minus one
};

typedef struct lino
{
	float val;		// current value
	float dst;		// value to move to
	float preinc;
	float inc;		// increment per sample to get from val to dst
	long elapsed;
};

typedef struct filter0
{
	float stt;
} filter0;
typedef struct filter4
{
	float c;
	float stt[4];
} filter4;

// Synthesis globals
struct rando r;
struct osco phasor[1];
struct osco trem[1];
float amp;
float timer_ms;
int32_t samplecount, samplenotecount, tapcount;
int32_t taptime[8];
int32_t switchontime[8];
int32_t switchstate[8];


void
led_init()
{
  // Enable GPIO Peripheral clock
  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStructure;

  // Configure pin in output push/pull mode
  GPIO_InitStructure.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

}

void
switch_init()
{

	int32_t i;

	// Enable GPIO Peripheral clock
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStructure;

  // Configure pin for polling
  GPIO_InitStructure.Pin = GPIO_PIN_0;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

  // to time the switch down time
  for(i = 0; i < 8; i++)
  {
	  switchon[i] = 0;
	  switchoff[i] = 1;
  }

}

void
timer3_init()
{
	  __HAL_RCC_TIM3_CLK_ENABLE();

	  TimHandle.Instance = TIM3;
	  TimHandle.Init.Period = 8399; // Clock = 168000000/2 = 84000000/(8399+1) = 10000
	  TimHandle.Init.Prescaler = 0;
	  TimHandle.Init.ClockDivision = 0;
	  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	  HAL_TIM_Base_Init(&TimHandle) ;
	  HAL_TIM_Base_Start_IT(&TimHandle);

	  /*##-2- Configure the NVIC for TIMx ########################################*/
	  /* Set Interrupt Group Priority */
	  HAL_NVIC_SetPriority(TIM3_IRQn, 4, 0);
	  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

void timer6_init(void)
{
  static TIM_HandleTypeDef  htim;
  TIM_MasterConfigTypeDef sMasterConfig;

  /*##-1- Configure the TIM peripheral #######################################*/
  /* Time base configuration */
  __HAL_RCC_TIM6_CLK_ENABLE();
  htim.Instance = TIM6;

//  htim.Init.Period = 874;	// 96000 clock - suitable for 2 channel 48000
  htim.Init.Period = 1749;	// 48000 clock - suitable for 1 channel 48000
  htim.Init.Prescaler = 0;
  htim.Init.ClockDivision = 0;
  htim.Init.CounterMode = TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&htim);

  /* TIM6 TRGO selection */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig);

  /*##-2- Enable TIM peripheral counter ######################################*/
  HAL_TIM_Base_Start(&htim);
}

void adc1_init(void)
{
	int i;

	for(i = 0; i < 64; i++)
		adc0average[i] = adc1average[i] = 0.0f;
    avgindex = 0;
    avgtotal[0] = avgtotal[1] = 0;

    ADC_ChannelConfTypeDef sConfig;
	  AdcHandle.Instance = ADC1;

	  AdcHandle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
	  AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
	  AdcHandle.Init.ScanConvMode = ENABLE;	// to scan multiple channels
	  AdcHandle.Init.ContinuousConvMode = ENABLE;
	  AdcHandle.Init.DiscontinuousConvMode = DISABLE;
	  AdcHandle.Init.NbrOfDiscConversion = 0;
	  AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	  AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	  AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  AdcHandle.Init.NbrOfConversion = 2;
	  AdcHandle.Init.DMAContinuousRequests = ENABLE;
	  AdcHandle.Init.EOCSelection = DISABLE;

	  if(HAL_ADC_Init(&AdcHandle) != HAL_OK)
	  {
	    /* Initialization Error */
	    ;
	  }

	  /*##-2- Configure ADC regular channel ######################################*/
	  /* Note: Considering IT occurring after each number of size of              */
	  /*       "uhADCxConvertedValue"  ADC conversions (IT by DMA end             */
	  /*       of transfer), select sampling time and ADC clock with sufficient   */
	  /*       duration to not create an overhead situation in IRQHandler.        */
	  sConfig.Channel = ADC_CHANNEL_9;	// Port B, Pin 1
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	  sConfig.Offset = 0;

	  HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);


	  /*##-3- Start the conversion process and enable interrupt ##################*/
	  /* Note: Considering IT occurring after each number of ADC conversions      */
	  /*       (IT by DMA end of transfer), select sampling time and ADC clock    */
	  /*       with sufficient duration to not create an overhead situation in    */
	  /*        IRQHandler. */
	  HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)&adcBuffer[0], 1);
}

void dac1_init(void)
{
	static DAC_ChannelConfTypeDef sConfig;
	DacHandle.Instance = DAC;

    HAL_DAC_Init(&DacHandle);
    sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

    HAL_DAC_ConfigChannel(&DacHandle, &sConfig, DAC_CHANNEL_1);

    /*##-2- Enable DAC Channel1 and associated DMA #############################*/
    HAL_DAC_Start_DMA(&DacHandle, DAC_CHANNEL_1, (uint32_t*)dacBuffer, 32, DAC_ALIGN_12B_R);

}

void audio_init()
{
	int32_t i;
	// set up a sawtooth waveform
		for(i = 0; i<8192; i++)
		{
			wavetable[i] = sinetable[i];
			wavetable[i] += sinetable[(i*2) & 8191];
			wavetable[i] += sinetable[(i*3) & 8191];
			wavetable[i] += sinetable[(i*4) & 8191];
			wavetable[i] += sinetable[(i*5) & 8191];
			wavetable[i] += sinetable[(i*7) & 8191] ;
			wavetable[i] += sinetable[(i*9) & 8191];
			wavetable[i] += sinetable[(i*11) & 8191];
			wavetable[i] += sinetable[(i*13) & 8191];
			wavetable[i] += sinetable[(i*15) & 8191] ;
			wavetable[i] += sinetable[(i*17) & 8191];
			wavetable[i] += sinetable[(i*19) & 8191] ;
			wavetable[i] += sinetable[(i*21) & 8191];
			wavetable[i] /= 13.0f;
		}
		// synth init
		for (i = 0; i<1; i++)
		{
			phasor[i].phs = 0.0f;
			phasor[i].pi = 0.01f;
			phasor[i].tab = wavetable;
			phasor[i].ts = 8192;
			phasor[i].tm = 8191;
		}
		for(i = 0; i<1; i++){
			trem[i].phs = 0.0f;
			trem[i].pi = 0.01f;
			trem[i].tab = wavetable;
			trem[i].ts = 8192;
			trem[i].tm = 8191;
		}
		for(i = 0; i<6;i++){
			taptime[i] = 0;
		}
		timer_ms = 500.0f;
		samplecount = tapcount = 0;
}

int
main(int argc, char* argv[])
{
	int i;
	timer_start();

	audio_init();
	led_init();
	switch_init();
	timer3_init();
//	timer6_init();
	adc1_init();
//	dac1_init();
	// output device, volume, sample rate
	BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, 90, 48000);

	// initialize globals
	time = 240;
	button_time[0] = button_time[1] = button_time[2] = button_time[3] = button_time[4] = 0;
	millisecs = 0.0f;
	ledselected = 0;
	bspAudioState = 0;

	// set up first buffer (in bytes)
	BSP_AUDIO_OUT_Play((uint16_t *)codecBuffer, 128);

	// Infinite loop
	while (1)
	{
		  if(bspAudioState == 1)
		  		{
		  			bspAudioState = 0;
		  			audioBlock(inBuffer, outBuffer, 16);
		  			for(i = 0; i < 32; i+=2)
		  			{
		  				codecBuffer[i+0] = (int16_t)((outBuffer[i]) * 32767.0f);
		  				codecBuffer[i+1] = (int16_t)((outBuffer[i+1]) * 32767.0f);
		  			}

		  		}
		  		else if(bspAudioState == 2)
		  		{
		  			bspAudioState = 0;

		  			audioBlock(inBuffer, outBuffer, 16);
		  			for(i = 0; i < 32; i+=2)
		  			{
		  				codecBuffer[i+32] = (int16_t)((outBuffer[i]) * 32767.0f);
		  				codecBuffer[i+33] = (int16_t)((outBuffer[i+1]) * 32767.0f);
		  			}
		  			// set up next buffer (in samples)
		  		}

	}
  // Infinite loop, never return.
}
float sat3(float sample)
{
	if(sample > 1.0f)
		sample = 1.0f;
	if(sample < -1.0f)
		sample = -1.0f;

	sample = sample * 1.5f - sample * sample * sample * 0.5f;
	return(sample);
}

float sat2(float sample)
{
	if(sample > 1.0f)
		sample = 1.0f;
	if(sample < -1.0f)
		sample = -1.0f;
	if(sample > 0.0f)
		sample = sample * 2.0 - sample * sample;
	else
		sample = sample * 2.0f + sample * sample;
	return(sample);
}

float lp0(float sample, float c, float state)
{
	return(sample + (state - sample)  * c);
}
// simple 4 pole
float lp4(float in, filter4 *f)
{
	float samp;
	int32_t i;

	samp = in - f->stt[3] * 3.85f;
	if(samp > 1.0f)
		samp = 1.0f;
	if(samp < -1.0f)
		samp = -1.0f;
	for(i = 0; i < 4; i++)
		samp = f->stt[i] = samp + (f->stt[i] - samp)  * f->c;
	return(samp);
}

void audioBlock(float *input, float *output, int32_t samples)
{
	int32_t i;
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1)
			switchontime[0]++;
		else
		{
			switchontime[0] = 0;
			switchstate[0] = 0;
		}

		if(switchontime[0] > 3 && switchstate[0] == 0)
		{
			switchstate[0] = 1;
			tapcount++;
			taptime[tapcount&7] = samplecount;
		}
		if((tapcount & 7) == 2)
		{
			timer_ms = (taptime[2]-taptime[1])/48;
			tapcount = 0;
			samplecount = 0;
		}

	for(i = 0; i < samples; i++, samplecount++)
	{
		//phase increment set to 200(Hz)/48000(samples/sec)
		phasor[0].pi = 0.004166666666667;
		// add this phase increment to phase every sample
		phasor[0].phs = phasor[0].phs + phasor[0].pi;
		// when we go over 1.0, subtract 1.0 to get position on "circle"
		while(phasor[0].phs >= 1.0f)
			phasor[0].phs -= 1.0f;
		// similarly if it goes below zero (would only happen if phase increment is modulated)
		while(phasor[0].phs < 0.0f)
			phasor[0].phs += 1.0f;
		// now our phasor is complete, it will go from 0 to 1 200 times in 48000 samples

		//tremolo
		//phase increment is ( 500 / clicked time msec ) Hz / 48000 samples
			    trem[0].pi=(500/timer_ms)/48000;
				//trem[0].pi=timer_ms/48000;
				trem[0].phs += trem[0].pi;
				while(trem[0].phs >= 1.0f)
					trem[0].phs -= 1.0f;
				while(trem[0].phs < 0.0f)
					trem[0].phs += 1.0f;

				amp = sinetable[ (int32_t)(trem[0].phs * 8192.0f) & 8191 ];


		if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){
			//the samples of the output are multiplied by the value of the trem osc
			//and halved to minimize clipping
			output[(i<<1)] = output[(i<<1)+1] = sinetable[ (int32_t)(phasor[0].phs * 8192.0f) & 8191 ]*amp*0.5f;
		}
		else{
			//while button is pressed, the sound is turned off
			output[(i<<1)] = output[(i<<1)+1] = 0.0f;
		}

		if(output[i<<1] > 1.0f) output[i<<1] = 1.0f;
		if(output[i<<1] < -1.0f) output[i<<1] = -1.0f;
		if(output[(i<<1) + 1] > 1.0f) output[(i<<1) + 1] = 1.0f;
		if(output[(i<<1) + 1] < -1.0f) output[(i<<1) + 1] = -1.0f;
	}
}

void EXTI0_IRQHandler(void)
{
	 HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}
// interrupt from switch - pin D1
void EXTI1_IRQHandler(void)
{
	 HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}
// interrupt from switch - pin D2
void EXTI2_IRQHandler(void)
{
	 HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}
// interrupt from switch - pin D3
void EXTI3_IRQHandler(void)
{
	 HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

// callback from switch interrupt function HAL_GPIO_EXTI_IRQHandler()
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
// debouncing
	int32_t time_elapsed, i;
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
	{
		time_elapsed = HAL_GetTick() - button_time[0];
		if(time_elapsed > 10)
		{
			// register new button down
			button_time[0] = HAL_GetTick();

		}
	}
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == 0)
	{
		time_elapsed = HAL_GetTick() - button_time[1];
		if(time_elapsed > 10)
		{
			// register new button down
			button_time[1] = HAL_GetTick();
		}
	}
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == 0)
	{
		time_elapsed = HAL_GetTick() - button_time[2];
		if(time_elapsed > 100)
		{
			// register new button down
			button_time[2] = HAL_GetTick();
		    r.seed = (r.seed * 196314165) + 907633515;
		}
	}
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == 0)
	{
		time_elapsed = HAL_GetTick() - button_time[3];
		if(time_elapsed > 100)
		{
			// register new button down
			button_time[3] = HAL_GetTick();
		    r.seed = (r.seed * 196314165) + 907633515;
		}
	}
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == 0)
	{
		time_elapsed = HAL_GetTick() - button_time[4];
		if(time_elapsed > 100)
		{
			// register new button down
			button_time[4] = HAL_GetTick();
		    r.seed = (r.seed * 196314165) + 907633515;
		}
	}
}

// interrupt from timer 3
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimHandle);
}

// callback from timer interrupt function HAL_TIM_IRQHandler()
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	millisecs += 0.1f;
	if(millisecs > time)
	{
		millisecs = 0.0f;
		switch(ledselected)
		{
		case 0:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
			break;
		}
		ledselected += 1;
		ledselected &= 3;
	}
}

// interrupt from DMA 2 - Stream 0 (ADC1)
void DMA2_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(AdcHandle.DMA_Handle);
}

// callback from DMA interrupt function HAL_DMA_IRQHandler()
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
  time = (adcBuffer[0] * adcBuffer[0]) >> 12;

  avgtotal[0] -= adc0average[avgindex];
  adc0average[avgindex] = adcBuffer[0];
  avgtotal[0] += adc0average[avgindex];
  smoothadc[0] = avgtotal[0] >> 6;

  avgtotal[1] -= adc1average[avgindex];
  adc1average[avgindex] = adcBuffer[1];
  avgtotal[1] += adc1average[avgindex];
  smoothadc[1] = avgtotal[1] >> 6;
  avgindex++;
  if(avgindex > 63) avgindex = 0;
}

// interrupt from DMA 1 - Stream 5 (DAC Ch 1)
void DMA1_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(DacHandle.DMA_Handle1);
}

// callback from DMA interrupt function HAL_DMA_IRQHandler()

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* DacHandle)
{
	int i,j;


	audioBlock(inBuffer, outBuffer, 16);
	for(i = 0, j = 0; i < 16; i++, j+=2)
	{
		dacBuffer[i] = (int16_t)((outBuffer[j] + outBuffer[j+1] + 2.0f) * 1023.0f);
	}
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* DacHandle)
{
	int i;

	audioBlock(inBuffer, outBuffer, 16);
	for(i = 0; i < 16; i++)
	{
		dacBuffer[i+16] = (int16_t)((outBuffer[(i * 2)] + outBuffer[(i * 2)+1] + 2.0f) * 1023.0f);
	}
}
/**
  * @brief  This function handles main I2S interrupt.
  * @param  None
  * @retval 0 if correct communication, else wrong communication
  */
void I2S3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hAudioOutI2s.hdmatx);
}

/**
  * @brief  This function handles DMA Stream interrupt request.
  * @param  None
  * @retval None
  */
void I2S2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hAudioInI2s.hdmarx);
}

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{
	bspAudioState = 1;

}

/**
* @brief  Calculates the remaining file size and new position of the pointer.
* @param  None
* @retval None
*/
void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{

	int i;
	bspAudioState = 2;
	BSP_AUDIO_OUT_ChangeBuffer((uint16_t *)codecBuffer, 64);
}

void BSP_AUDIO_OUT_Error_CallBack(void)
{
  /* Stop the program with an infinite loop */
  while (1)
  {}

  /* Could also generate a system reset to recover from the error */
  /* .... */
}

/**
* @brief  This function handles DMA interrupt request.
* @param  None
* @retval None
*/
void DACx_DMA_IRQHandler2(void)
{
  HAL_DMA_IRQHandler(DacHandle.DMA_Handle2);
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
