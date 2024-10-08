/*
 * pal.c
 *
 *  Created on: Aug 21, 2024
 *      Author: mirco
 */

#include "pal.h"

uint8_t pal_line_buffers[PAL_LINE_LEN * 5];

void PAL_CopyBuffer(PAL_t *hpal, uint32_t n, uint8_t second);
void PAL_CopyLine(PAL_t *hpal, uint32_t n, uint8_t second);
void PAL_SetBuffer(PAL_t *hpal, uint8_t second);

void PAL_Init(PAL_t *hpal)
{
	for (uint32_t i = 0; i < sizeof(hpal->dma_buffer); i++)
	{
		hpal->dma_buffer[i] = PAL_ZERO;
	}
	for (uint32_t i = 0; i < sizeof(pal_line_buffers); i++)
	{
		pal_line_buffers[i] = PAL_ZERO;
	}

	for (uint32_t i = 0; i < PAL_LINE_LEN; i++)
	{
		uint32_t i1 = i % ((PAL_LINE_LEN + 1) / 2);
		pal_line_buffers[i] = i1 <= PAL_LONG_PULSE ? PAL_OFF : PAL_ZERO;
	}
	for (uint32_t i = 0; i < PAL_LINE_LEN; i++)
	{
		uint32_t i1 = i % ((PAL_LINE_LEN + 1) / 2);
		if (i < (PAL_LINE_LEN + 1) / 2)
		{
			pal_line_buffers[1 * PAL_LINE_LEN + i] = i1 <= PAL_LONG_PULSE ? PAL_OFF : PAL_ZERO;
		}
		else
		{
			pal_line_buffers[1 * PAL_LINE_LEN + i] = i1 <= PAL_SHORT_PULSE ? PAL_OFF : PAL_ZERO;
		}
	}
	for (uint32_t i = 0; i < PAL_LINE_LEN; i++)
	{
		uint32_t i1 = i % ((PAL_LINE_LEN + 1) / 2);
		pal_line_buffers[2 * PAL_LINE_LEN + i] = i1 <= PAL_SHORT_PULSE ? PAL_OFF : PAL_ZERO;
	}
	for (uint32_t i = 0; i < PAL_LINE_LEN; i++)
	{
		uint32_t i1 = i % ((PAL_LINE_LEN + 1) / 2);
		if (i < (PAL_LINE_LEN + 1) / 2)
		{
			pal_line_buffers[3 * PAL_LINE_LEN + i] = i1 <= PAL_SHORT_PULSE ? PAL_OFF : PAL_ZERO;
		}
		else
		{
			pal_line_buffers[3 * PAL_LINE_LEN + i] = i1 <= PAL_LONG_PULSE ? PAL_OFF : PAL_ZERO;
		}
	}
	for (uint32_t i = 0; i < PAL_LINE_LEN; i++)
	{
		if (i < PAL_LINE_SYNC)
		{
			pal_line_buffers[4 * PAL_LINE_LEN + i] = PAL_OFF;
		}
		else if (i < PAL_LINE_SYNC + PAL_BACK_PORCH)
		{
			pal_line_buffers[4 * PAL_LINE_LEN + i] = PAL_ZERO;
		}
		else if (i < PAL_LINE_LEN - PAL_ONE_US - PAL_FRONT_PORCH)
		{
			// frame area
			pal_line_buffers[4 * PAL_LINE_LEN + i] = PAL_ZERO;
		}
		else if (i < PAL_LINE_LEN - PAL_FRONT_PORCH)
		{
			pal_line_buffers[4 * PAL_LINE_LEN + i] = PAL_ZERO; // PAL_BLACK
		}
		else
		{
			pal_line_buffers[4 * PAL_LINE_LEN + i] = PAL_ZERO;
		}
	}

	hpal->line_counter = 0;
	PAL_COPYBUFFER_DEF_1(hpal, 0);
	PAL_COPYBUFFER_DEF_2(hpal, 0);
}

void PAL_Start(PAL_t *hpal)
{
	HAL_DAC_Start_DMA(hpal->hdac, hpal->dac_channel, (uint32_t*)hpal->dma_buffer, PAL_LINE_LEN, DAC_ALIGN_8B_R);
	HAL_TIM_Base_Start(hpal->htim);
}

void PAL_IntHalfCplt(PAL_t *hpal)
{
	uint32_t line = (hpal->line_counter + 1) % PAL_LINE_COUNT;
	if (line == 0)
	{
		PAL_COPYBUFFER_DEF_1(hpal, 0);
	}
	else if (line == 2)
	{
		PAL_COPYBUFFER_DEF_1(hpal, 1);
	}
	else if (line == 3)
	{
		PAL_COPYBUFFER_DEF_1(hpal, 2);
	}
	else if (line == 5)
	{
		PAL_COPYBUFFER_DEF_1(hpal, 4);
	}
	else if (line >= 5 + PAL_BLANKING_LINES && line < 310)
	{
		// TODO: constrain
		PAL_COPYLINE_DEF_1(hpal, line - 22);
	}
	else if (line == 310)
	{
		PAL_COPYBUFFER_DEF_1(hpal, 2);
	}
	else if (line == 312)
	{
		PAL_COPYBUFFER_DEF_1(hpal, 3);
	}
	else if (line == 313)
	{
		PAL_COPYBUFFER_DEF_1(hpal, 0);
	}
	else if (line == 315)
	{
		PAL_COPYBUFFER_DEF_1(hpal, 2);
	}
	else if (line == 317)
	{
		PAL_COPYBUFFER_DEF_1(hpal, 4);
	}
	else if (line >= 317 + PAL_BLANKING_LINES && line < 622)
	{
		// TODO: constrain
		PAL_COPYLINE_DEF_1(hpal, line - 334);
	}
	else if (line == 622)
	{
		PAL_COPYBUFFER_DEF_1(hpal, 2);
	}
}

void PAL_IntCplt(PAL_t *hpal)
{
	uint32_t line = (hpal->line_counter + 1) % PAL_LINE_COUNT;
	hpal->line_counter = line;
	if (line == 0)
	{
		PAL_COPYBUFFER_DEF_2(hpal, 0);
	}
	else if (line == 2)
	{
		PAL_COPYBUFFER_DEF_2(hpal, 1);
	}
	else if (line == 3)
	{
		PAL_COPYBUFFER_DEF_2(hpal, 2);
	}
	else if (line == 5)
	{
		PAL_COPYBUFFER_DEF_2(hpal, 4);
	}
	else if (line >= 5 + PAL_BLANKING_LINES && line < 310)
	{
		// TODO: constrain
		PAL_COPYLINE_DEF_2(hpal, line - 22);
	}
	else if (line == 310)
	{
		PAL_COPYBUFFER_DEF_2(hpal, 2);
	}
	else if (line == 312)
	{
		PAL_COPYBUFFER_DEF_2(hpal, 3);
	}
	else if (line == 313)
	{
		PAL_COPYBUFFER_DEF_2(hpal, 0);
	}
	else if (line == 315)
	{
		PAL_COPYBUFFER_DEF_2(hpal, 2);
	}
	else if (line == 317)
	{
		PAL_COPYBUFFER_DEF_2(hpal, 4);
	}
	else if (line >= 317 + PAL_BLANKING_LINES && line < 622)
	{
		// TODO: constrain
		PAL_COPYLINE_DEF_2(hpal, line - 334);
	}
	else if (line == 622)
	{
		PAL_COPYBUFFER_DEF_2(hpal, 2);
	}
}
