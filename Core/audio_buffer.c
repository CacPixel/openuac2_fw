#include "audio_buffer.h"
#include "usbd_audio.h"
#include "main.h"
#include "ak4490r.h"

#define USE_DMA_COPY

#ifdef USE_DMA_COPY
#define DMA_COPY_HANDLE hdma_memtomem_dma2_stream0
#define DMA_FILL_HANDLE hdma_memtomem_dma2_stream1
extern DMA_HandleTypeDef DMA_COPY_HANDLE;
extern DMA_HandleTypeDef DMA_FILL_HANDLE;
#endif

__attribute__((aligned(4))) static uint8_t s_AudBufMem[AUDIO_BUF_SIZE + 1024];
uint8_t s_BufZeros[6400] = { 0 };

AudioBuffer ab_instance;
AudioBuffer ab_empty;

static void MemCopy(uint32_t* pDst, uint32_t* pSrc, uint32_t count)
{
#ifdef USE_DMA_COPY
	HAL_DMA_Start_IT(&DMA_COPY_HANDLE, (uint32_t)pSrc, (uint32_t)pDst, count);
#else
	for (uint32_t i = 0; i < count; ++i)
		*pDst++ = *pSrc++;
#endif
}

// TODO: Use another stream
static void MemFill(uint32_t* pDst, uint32_t val, uint32_t count)
{
#ifdef USE_DMA_COPY
	static uint32_t x;
	x = val;
	HAL_DMA_Start_IT(&DMA_FILL_HANDLE, (uint32_t)&x, (uint32_t)pDst, count);
#else
	for (uint32_t i = 0; i < count; ++i)
		*pDst++ = val;
#endif
}

AudioBuffer* AudioBuffer_Instance()
{
	return &ab_instance;
}

void AudioBuffer_Reset(AudioBuffer* ab, uint32_t capacity)
{
	ab->state = AB_OK;
	ab->size = 0;
	ab->wr_ptr = 0;
	ab->rd_ptr = 0;
	ab->capacity = capacity;
}

void AudioBuffer_Init(AudioBuffer* ab, uint32_t capacity)
{
	AudioBuffer_Init_WithBuf(ab, capacity, s_AudBufMem);
}

void AudioBuffer_Init_WithBuf(AudioBuffer* ab, uint32_t capacity, void* buf)
{
	ab->mem = buf;
	AudioBuffer_Reset(ab, capacity);
}

void AudioBuffer_Recieve(AudioBuffer* ab, uint32_t rxSize)
{
	if (ab->size + rxSize <= ab->capacity)
	{
		ab->size += rxSize;
		ab->wr_ptr += rxSize;

		if (ab->wr_ptr >= ab->capacity)
		{
			ab->wr_ptr -= ab->capacity;
			if (ab->wr_ptr > 0)
			{
				MemCopy((uint32_t*)ab->mem, (uint32_t*)&ab->mem[ab->capacity], ab->wr_ptr >> 2);
			}
		}

		ab->state = AB_OK;
	}
	else
	{
		ab->size = ab->capacity;
		ab->state = AB_OVFL;
	}
	// printf("AudioBuffer_Recieve: \tab->capacity = %d, ab->size = %d, ab->wr_ptr = %d, ab->rd_ptr = %d, ab->state = %s\r\n",
	// ab->capacity,
	// ab->size,
	// ab->wr_ptr,
	// ab->rd_ptr,
	// ab->state == AB_OK ? "AB_OK" : ab->state == AB_OVFL ? "AB_OVFL" : "AB_UDFL");

}

void AudioBuffer_Sync(AudioBuffer* ab, uint32_t txSize)
{
	if (ab->size >= txSize)
	{
		// MemFill((uint32_t*)&ab->mem[ab->rd_ptr], 0, txSize >> 2);
		ab->size -= txSize;
		ab->rd_ptr += txSize;

		if (ab->rd_ptr >= ab->capacity)
		{
			ab->rd_ptr -= ab->capacity;
		}

		ab->state = AB_OK;
	}
	else
	{
		ab->size = 0;
		ab->state = AB_UDFL;
	}


	// while ((USART1->SR & 0x40) == 0) { ; }
	// USART1->DR = (ab->size % 1000) / 100 + '0';
	// while ((USART1->SR & 0x40) == 0) { ; }
	// USART1->DR = (ab->size % 100) / 10 + '0';
	// while ((USART1->SR & 0x40) == 0) { ; }
	// USART1->DR = (ab->size % 10) / 1 + '0';
	// while ((USART1->SR & 0x40) == 0) { ; }
	// USART1->DR = '\n';
		// printf("AudioBuffer_Sync: \tab->capacity = %d, ab->size = %d, ab->wr_ptr = %d, ab->rd_ptr = %d, ab->state = %s\r\n",
		// ab->capacity,
		// ab->size,
		// ab->wr_ptr,
		// ab->rd_ptr,
		// ab->state == AB_OK ? "AB_OK" : ab->state == AB_OVFL ? "AB_OVFL" : "AB_UDFL");
}

void AudioBuffer_Switch(I2S_HandleTypeDef* hi2s, AudioBuffer* ab)
{



}



