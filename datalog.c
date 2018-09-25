
// FTS datalog / black box
// Records all data comming from the mavlink connection
// Flash storage should be performed only in case of parachute deploy

#include "datalog.h"
#include "stm32f10x_flash.h"
#include <kernel/panic.h>

// Page size: 2KB
// 32KB area, size controlled by assert
#define FLASH_RESERVE      (32768)
#define FLASH_DATALOG_ADDR (0x8040000 - FLASH_RESERVE) //0x8003C00


#define DATALOG_BLEN (14)
#define DATALOG_LEN  (1 << DATALOG_BLEN)
#define DATALOG_MASK (DATALOG_LEN - 1)

// Average mavlink package is ~30 bytes
#define MAX_ENTRIES_BLEN   (7)
#define MAX_ENTRIES        (1 << MAX_ENTRIES_BLEN)
#define MAX_ENTRIES_MASK   (MAX_ENTRIES - 1)

typedef struct
{
	int start;
	unsigned length;
} datalog_entry_t;

static int _init = 0;

typedef struct 
{
	char mark[4];	// 'F','T','S',version
	unsigned cause; // fired_cause_t

	datalog_entry_t entries[MAX_ENTRIES];
	int curr_entry;

	unsigned char datalog[DATALOG_LEN];
	int curr_datalog_idx;
} datalog_t;

static datalog_t _dl;

void _flash_init(void) 
{
	// Next commands may be used in SysClock initialization function
	// In this case using of FLASH_Init is not obligatorily 
	// Enable Prefetch Buffer 
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
	// Flash 2 wait state 
	FLASH_SetLatency(FLASH_Latency_2);
}

static void _flash_data_raw_save (unsigned* data, int bytes)
{
	int i, w;
	int pages = (bytes + 2047) / 2048;
	ASSERT(FLASH_RESERVE > (pages * 2048), KERNEL_ERROR_KERNEL_PANIC);
	//NVIC_DisableIRQ(CAN_IRQn);
	FLASH_Unlock ();
	//FLASH_ClearFlag (FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
	for (i = 0; i < pages; i++)
	{
		unsigned *src = i * 512 + data;
		unsigned dst = i * 2048 + FLASH_DATALOG_ADDR;
		// Erase the entire page before you can write
		FLASH_ErasePage(dst);      

		for (w = 0; w < 512; w++)
			FLASH_ProgramWord(dst + w * 4, src[w]);
	}
	FLASH_Lock();
	//NVIC_EnableIRQ(CAN_IRQn);
}

void _datalog_init()
{
	_dl.mark[0] = 'F', _dl.mark[1] = 'T', _dl.mark[2] = 'S';
	_dl.curr_entry = 0;
	_dl.curr_datalog_idx = 0;

	int i;
	for (i = 0; i < MAX_ENTRIES; i++)
		_dl.entries[i].start = -1;

	_flash_init();

	_init = 1;
}

void datalog_flash(fired_cause_t cause)
{
	if (!_init)
		_datalog_init ();

	_dl.cause = (unsigned)cause;

	// This is a very slow operation, it takes about 1 second
	_flash_data_raw_save ((unsigned *)&_dl, sizeof(_dl));
}

void datalog_add_event(unsigned char* data, unsigned length)
{
	int i;
	if (!_init)
		_datalog_init ();

	_dl.entries [_dl.curr_entry].start = _dl.curr_datalog_idx; 
	_dl.entries [_dl.curr_entry].length = length; 
	_dl.curr_entry = (_dl.curr_entry + 1) & MAX_ENTRIES_MASK;

	for (i = 0; i < length; i++)
	{
		_dl.datalog [_dl.curr_datalog_idx] = data[i];
		_dl.curr_datalog_idx = (_dl.curr_datalog_idx + 1) & DATALOG_MASK; 
	}
}
