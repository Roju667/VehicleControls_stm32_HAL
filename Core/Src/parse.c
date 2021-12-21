/*
 * parse.c
 *
 *  Created on: 24 cze 2021
 *      Author: ROJEK
 */

#include "main.h"
#include "string.h"
#include "ringbuffer.h"
#include "usart.h"
#include "tim.h"
#include "stdlib.h"
#include "stdio.h"
#include "parse.h"


/*
 * @ function parse message and start command procedures
 */
uint8_t Parser_Parse(uint8_t *ParseBuffer, uint16_t ServoX,uint16_t ServoY)
{
	uint8_t *ParsePointer;

	ParsePointer = (uint8_t*)(strtok((char*)ParseBuffer, ";"));

	if(strcmp((char*)ParsePointer,"ACK") == 0)
	{
		ParsePointer = (uint8_t*)(strtok(NULL, ";"));
		ServoX = atoi((const char *)ParsePointer);

		ParsePointer = (uint8_t*)(strtok(NULL, ";"));
		ServoY = atoi((const char *)ParsePointer);
	}

	return 0;
}
