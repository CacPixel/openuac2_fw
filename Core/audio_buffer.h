#ifndef _AUDIO_BUFFER_H_
#define _AUDIO_BUFFER_H_

#include <stdint.h>
#include "stm32f4xx.h"

typedef enum
{
	AB_OK,
	AB_OVFL,
	AB_UDFL
} AudioBufferState;

typedef struct
{
	uint8_t* mem;
	uint32_t capacity;
	uint32_t size;
	uint32_t wr_ptr;
	uint32_t rd_ptr;
	uint8_t state;
} AudioBuffer;

extern AudioBuffer ab_instance;
extern AudioBuffer ab_empty;

//public:
AudioBuffer* AudioBuffer_Instance();
void AudioBuffer_I2S_InitFirst(void);
void I2S_DMA_MultiBufferStart(I2S_HandleTypeDef* hi2s, AudioBuffer* ab);
void AudioBuffer_Init(AudioBuffer* ab, uint32_t);
void AudioBuffer_Init_WithBuf(AudioBuffer* ab, uint32_t capacity, void* buf);
void AudioBuffer_Reset(AudioBuffer* ab, uint32_t);
void AudioBuffer_Recieve(AudioBuffer* ab, uint32_t);
void AudioBuffer_Sync(AudioBuffer* ab, uint32_t);
void AudioBuffer_Switch(I2S_HandleTypeDef* hi2s, AudioBuffer* ab);

#define AudioBuffer_WrPtr(ab) (&(ab)->mem[(ab)->wr_ptr])

#endif // _AUDIO_BUFFER_H_
