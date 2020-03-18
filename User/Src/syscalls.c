/*
******************************************************************************
File:     syscalls.c
Info:     Generated by Atollic TrueSTUDIO(R) 9.3.0   2019-10-16

The MIT License (MIT)
Copyright (c) 2019 STMicroelectronics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

******************************************************************************
*/

/* Includes */
#include <stdint.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>
#include "usart.h"
#include "tftlcd.h"

/* Variables */
#undef errno
extern int errno;

uint8_t *__env[1] = { 0 };
uint8_t **environ = __env;


/* Functions */

int __io_putchar(int ch)
{
	static uint8_t tmp00='\r';
	static uint8_t tmp01='\n';
	if(ch=='\n'){
#if (UART_PRINTF&&LCD_PRINTF)
		HAL_UART_Transmit(&huart1,&tmp00,1,HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1,&tmp01,1,HAL_MAX_DELAY);
		__lcd_x=0;
		__lcd_y+=FONT_SIZE;
#elif (UART_PRINTF)
		HAL_UART_Transmit(&huart1,&tmp00,1,HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1,&tmp01,1,HAL_MAX_DELAY);
#elif (LCD_PRINTF)
		__lcd_x=0;
		__lcd_y+=FONT_SIZE;
#endif
		return ch;
	}
	if(ch=='\r'){
		return ch;
	}
#if (UART_PRINTF&&LCD_PRINTF)
	if(__lcd_x==SSD_VER_RESOLUTION){
		__lcd_x=0;
		__lcd_y+=FONT_SIZE;
	}
	if(__lcd_y>SSD_HOR_RESOLUTION){
		LCD_Clear(BACK_COLOR);
		__lcd_x=0;
		__lcd_y=0;
	}
	HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,HAL_MAX_DELAY);
	LCD_ShowChar(__lcd_x,__lcd_y,ch,FONT_SIZE,0);
	__lcd_x+=(FONT_SIZE/2);
#elif (UART_PRINTF)
	HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,HAL_MAX_DELAY);
#elif (LCD_PRINTF)
	if(__lcd_x==SSD_VER_RESOLUTION){
		__lcd_x=0;
		__lcd_y+=FONT_SIZE;
	}
	if(__lcd_y>SSD_HOR_RESOLUTION){
		LCD_Clear(BACK_COLOR);
		__lcd_x=0;
		__lcd_y=0;
	}
	LCD_ShowChar(__lcd_x,__lcd_y,ch,FONT_SIZE,0);
	__lcd_x+=(FONT_SIZE/2);
#endif
	return ch;
}

void initialise_monitor_handles()
{
}

int _getpid(void)
{
	return 1;
}

int _kill(int32_t pid, int32_t sig)
{
	errno = EINVAL;
	return -1;
}

void _exit (int32_t status)
{
	_kill(status, -1);
	while (1) {}    /* Make sure we hang here */
}

int _write(int32_t file, uint8_t *ptr, int32_t len)
{
	/* Implement your write code here, this is used by puts and printf for example */
	int DataIdx;
	for (DataIdx = 0; DataIdx < len;DataIdx++){
				__io_putchar(*ptr++);
	}
	return len;
}

caddr_t _sbrk(int32_t incr)
{
	extern uint32_t _Min_Heap_Size; /* _Min_Heap_Size symbol defined in the linker script. */
	extern uint8_t end asm("end");
	const uint8_t *max_heap = (uint8_t*)((uint32_t)&end + (uint32_t)&_Min_Heap_Size);
	static uint8_t *heap_end;
	uint8_t *prev_heap_end;

	if (heap_end == 0)
		heap_end = &end;

	prev_heap_end = heap_end;
	if (heap_end + incr > max_heap)
	{
//    write(1, "Heap and stack collision\n", 25);
//    abort();
		errno = ENOMEM;
		return (caddr_t) -1;
	}

	heap_end += incr;

	return (caddr_t) prev_heap_end;
}

int _close(int32_t file)
{
	return -1;
}


int _fstat(int32_t file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int32_t file)
{
	return 1;
}

int _lseek(int32_t file, int32_t ptr, int32_t dir)
{
	return 0;
}

int _read(int32_t file, uint8_t *ptr, int32_t len)
{
	return 0;
}

int _open(uint8_t *path, int32_t flags, int32_t mode)
{
	/* Pretend like we always fail */
	return -1;
}

int _wait(int32_t *status)
{
	errno = ECHILD;
	return -1;
}

int _unlink(const uint8_t *name)
{
	errno = ENOENT;
	return -1;
}

int _times(struct tms *buf)
{
	return -1;
}

int _stat(const uint8_t *file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _link(const uint8_t *old, const uint8_t *new)
{
	errno = EMLINK;
	return -1;
}

int _fork(void)
{
	errno = EAGAIN;
	return -1;
}

int _execve(const uint8_t *name, uint8_t * const *argv, uint8_t * const *env)
{
	errno = ENOMEM;
	return -1;
}
