/*
 * pal.h
 *
 *  Created on: Aug 21, 2024
 *      Author: mirco
 */

#ifndef INC_PAL_H_
#define INC_PAL_H_

#include <string.h>

#include "stm32l4xx_hal.h"

#define PAL_ARR 21
// 0.954
#define PAL_TIME_CORRECTION 0.954
#define PAL_OFF 0
#define PAL_ZERO 23
// 23, 39
#define PAL_BLACK 23
#define PAL_WHITE 77
// 16, 17, 25
#define PAL_BLANKING_LINES 16
#define PAL_LINE_COUNT 625
#define PAL_FRAME_LINE_COUNT ((610 / 2) - PAL_BLANKING_LINES)

#define PAL_SA(u) ((uint32_t)((double)u * 80.0 / PAL_ARR * PAL_TIME_CORRECTION + 0.5))
#define PAL_LINE_LEN PAL_SA(64.0)
// 0.0, 1.0
#define PAL_ONE_US PAL_SA(0.0)
#define PAL_LINE_SYNC PAL_SA(4.7)
// 4.7, 5.8, 6.5
#define PAL_BACK_PORCH PAL_SA(6.5)
// 1.50, 1.55, 1.65
#define PAL_FRONT_PORCH PAL_SA(1.55)
#define PAL_SHORT_PULSE PAL_SA(2.35)
#define PAL_LONG_PULSE PAL_SA(27.3)
#define PAL_FRAME_LINE_LEN (PAL_LINE_LEN - PAL_ONE_US - PAL_FRONT_PORCH - 2 * PAL_LINE_SYNC)
#define PAL_BUFFER_LEN1 (PAL_LINE_LEN / 2)
#define PAL_BUFFER_LEN2 ((PAL_LINE_LEN + 1) / 2)
#define PAL_LINE_OFF1 (PAL_LINE_SYNC + PAL_BACK_PORCH)
#define PAL_LINE_LEN1 (PAL_LINE_LEN / 2 - PAL_LINE_OFF1)
#define PAL_LINE_OFF2 (PAL_LINE_LEN / 2)
#define PAL_LINE_LEN2 ((PAL_LINE_LEN + 1) / 2 - PAL_ONE_US - PAL_FRONT_PORCH)
#define PAL_COPYBUFFER_DEF_1(hpal, n) memcpy(hpal->dma_buffer, pal_line_buffers + PAL_LINE_LEN * (n), PAL_BUFFER_LEN1);
#define PAL_COPYBUFFER_DEF_2(hpal, n) memcpy(hpal->dma_buffer + PAL_BUFFER_LEN1, pal_line_buffers + PAL_LINE_LEN * (n) + PAL_BUFFER_LEN1, PAL_BUFFER_LEN2);
#define PAL_COPYLINE_DEF_1(hpal, n) memcpy(hpal->dma_buffer + PAL_LINE_OFF1, hpal->frame_buffer + PAL_FRAME_LINE_LEN * (n), PAL_LINE_LEN1);
#define PAL_COPYLINE_DEF_2(hpal, n) memcpy(hpal->dma_buffer + PAL_LINE_OFF2, hpal->frame_buffer + PAL_FRAME_LINE_LEN * (n) + PAL_LINE_LEN1, PAL_LINE_LEN2);

typedef struct
{
	DAC_HandleTypeDef *hdac;
	uint32_t dac_channel;
	TIM_HandleTypeDef *htim;
	uint32_t line_counter;
	uint8_t dma_buffer[PAL_LINE_LEN];
	uint8_t frame_buffer[PAL_FRAME_LINE_LEN * PAL_FRAME_LINE_COUNT];
} PAL_t;

extern uint8_t pal_line_buffers[];

void PAL_Init(PAL_t *hpal);
void PAL_Start(PAL_t *hpal);
void PAL_IntHalfCplt(PAL_t *hpal);
void PAL_IntCplt(PAL_t *hpal);

#endif /* INC_PAL_H_ */
