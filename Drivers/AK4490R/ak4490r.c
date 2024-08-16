#include "ak4490r.h"
#include "usbd_audio_if.h"
#include "main.h"

#define AT24C02_ADDR    0xA0

extern I2C_HandleTypeDef AK4490R_I2C_HANDLE;
extern I2S_HandleTypeDef AUDIO_I2S_MSTR_HANDLE;

static uint8_t play;
static uint8_t reset;
static uint8_t stop;
static uint8_t initialized;

AUDIO_CodecTypeDef ak4490r_instance =
{
    AK4490R_Init,
    NULL,
    AK4490R_Play,
    AK4490R_SetFormat,
    AK4490R_Stop,
    NULL,
    AK4490R_SetMute,
    AK4490R_SetVolume
};

static AK4490R_RegisterTypeDef reg;

uint8_t AK4490R_Init()
{
    if (initialized)
    {
        return -1;
    }
    initialized = 1;

    reg.control1 = AK4490R_ACKS | AK4490R_DIF2 | AK4490R_DIF1 | AK4490R_DIF0 | AK4490R_RSTN;
    reg.control2 = AK4490R_SD | AK4490R_DEM0;
    reg.control3 = 0;
    reg.lch_att = 0;
    reg.rch_att = 0;
    reg.control4 = 0;
    reg.control5 = 0;
    reg.control6 = 0;
    reg.control7 = 0;
    reg.control8 = 0;

    HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_CONTROL1_ADDR, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&reg, sizeof(reg), 1000);
    reset = 1;
    return 0;
}

uint8_t AK4490R_SetVolume(uint8_t vol)
{
    if (AK4490R_I2C_HANDLE.State == HAL_I2C_STATE_READY)
    {
        vol = (vol > 0) ? (vol + 155) : 0;
        reg.lch_att = reg.rch_att = vol;
        HAL_I2C_Mem_Write_IT(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_LCH_ATT_ADDR, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&reg.lch_att, 2);
    }

    return 0;
}

uint8_t AK4490R_SetMute(uint8_t mute)
{
   if (AK4490R_I2C_HANDLE.State == HAL_I2C_STATE_READY)
   {
       if (mute)
       {
           reg.control2 |= AK4490R_SMUTE;
       }
       else
       {
           reg.control2 &= ~AK4490R_SMUTE;
       }

       HAL_I2C_Mem_Write_IT(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_CONTROL2_ADDR, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&reg.control2, 1);
   }

    return 0;
}

uint8_t AK4490R_SetFormat(uint8_t format)
{
    //	if (AK4490R_I2C_HANDLE.State == HAL_I2C_STATE_READY)
    //	{
    //		if (format == AUDIO_FORMAT_DSD)
    //		{
    //			reg.control3 |= AK4490R_DP;
    //		}
    //		else
    //		{
    //			reg.control3 &= ~AK4490R_DP;
    //		}

    //		HAL_I2C_Mem_Write_IT(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_CONTROL3_ADDR, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&reg.control3, 1);
    //	}
    return 0;
}

uint8_t AK4490R_Play()
{
    uint8_t ret;
    play = 1;
    // ret = AK4490R_SetMute(0);
    return ret;
}

uint8_t AK4490R_Stop()
{
    uint8_t ret;
    stop = 1;
    // ret = AK4490R_SetMute(0);
    return ret;
}

void AK4490R_ProcessEvents()
{
//    for (int i = 0; i < sizeof(reg); i++)
//    {
//        printf1("reg[%d] = %#x\r\n", i, ((uint8_t*)&reg)[i]);
//    }
//    printf1("------------------\r\n");
//    LL_mDelay(10);
    if (reset)
    {
        AK4490R_Reset();
        reset = 0;
    }
    if (stop)
    {
        while (AK4490R_I2C_HANDLE.State != HAL_I2C_STATE_READY) {}
        HAL_I2C_Mem_Write(&hi2c1, AT24C02_ADDR, 0x0000, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&reg.lch_att, 1, 1000);
        stop = 0;
    }
    if (play)
    {
        while (hi2c1.State != HAL_I2C_STATE_READY) {}
        HAL_I2C_Mem_Read(&hi2c1, AT24C02_ADDR, 0x0000, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&reg.lch_att, 1, 1000);
        reg.rch_att = reg.lch_att;
        while (AK4490R_I2C_HANDLE.State != HAL_I2C_STATE_READY) {}
        HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_LCH_ATT_ADDR, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&reg.lch_att, 2, 1000);
        play = 0;
    }
}

void AK4490R_Reset(void)
{
    while (AK4490R_I2C_HANDLE.State != HAL_I2C_STATE_READY) {}
    reg.control1 &= ~AK4490R_RSTN;
    HAL_I2C_Mem_Write_IT(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_CONTROL1_ADDR, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&reg.control1, 1);
    while (AK4490R_I2C_HANDLE.State != HAL_I2C_STATE_READY) {}
    LL_mDelay(50);
    reg.control1 |= AK4490R_RSTN;
    HAL_I2C_Mem_Write_IT(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_CONTROL1_ADDR, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&reg.control1, 1);
}

void AK4490R_SaveConfigToEEPROM(void)
{
    while (HAL_I2C_Mem_Write(&hi2c1, AT24C02_ADDR, 0x0000, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&reg.lch_att, 1, 1000) != HAL_OK);
}