#include "sys.h"
#include "stdio.h"

#if !defined(__MICROLIB)

//#pragma import(__use_no_semihosting)
__asm (".global __use_no_semihosting\n\t");
void _sys_exit(int x) //
{
    x = x;
}
//__use_no_semihosting was requested, but _ttywrch was 
void _ttywrch(int ch) {
    ch = ch;
}
#ifdef __ICCARM__
struct __FILE
{
  int handle;
};
#endif
FILE __stdout;

#endif

#if defined ( __GNUC__ ) && !defined (__clang__) 
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#elif defined(__ICCARM__)
#define PUTCHAR_PROTOTYPE 
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif 

#ifndef __ICCARM__
PUTCHAR_PROTOTYPE
{
    //serial_write(&serial1, (uint8_t)ch); //
    //HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 1000);

    while ((USART1->SR & 0x40) == 0) { ; }
    USART1->DR = *(uint8_t*)&ch;
    return ch;

}
#endif


#if defined(__ICCARM__)
/*******************
 *
 * Copyright 1998-2017 IAR Systems AB.
 *
 * This is a template implementation of the "__write" function used by
 * the standard library.  Replace it with a system-specific
 * implementation.
 *
 * The "__write" function should output "size" number of bytes from
 * "buffer" in some application-specific way.  It should return the
 * number of characters written, or _LLIO_ERROR on failure.
 *
 * If "buffer" is zero then __write should perform flushing of
 * internal buffers, if any.  In this case "handle" can be -1 to
 * indicate that all handles should be flushed.
 *
 * The template implementation below assumes that the application
 * provides the function "MyLowLevelPutchar".  It should return the
 * character written, or -1 on failure.
 *
 ********************/

#include <LowLevelIOInterface.h>

#pragma module_name = "?__write"

int MyLowLevelPutchar(int ch)
{
    while ((USART1->SR & 0x40) == 0) { ; }
    USART1->DR = *(uint8_t*)&ch;
    return ch;
 
}

/*
 * If the __write implementation uses internal buffering, uncomment
 * the following line to ensure that we are called with "buffer" as 0
 * (i.e. flush) when the application terminates.
 */
//unsigned char const* __write(int handle, const unsigned char *ch, size_t size)
size_t __write(int handle, const unsigned char * buffer, size_t size)
{
  /* Remove the #if #endif pair to enable the implementation */
#if 1

  size_t nChars = 0;

  if (buffer == 0)
  {
    /*
     * This means that we should flush internal buffers.  Since we
     * don't we just return.  (Remember, "handle" == -1 means that all
     * handles should be flushed.)
     */
    return 0;
  }

  /* This template only writes to "standard out" and "standard err",
   * for all other file handles it returns failure. */
  if (handle != _LLIO_STDOUT && handle != _LLIO_STDERR)
  {
    return _LLIO_ERROR;
  }

  for (/* Empty */; size != 0; --size)
  {
    if (MyLowLevelPutchar(*buffer++) < 0)
    {
      return _LLIO_ERROR;
    }

    ++nChars;
  }

  return nChars;

#else

  /* Always return error code when implementation is disabled. */
  return _LLIO_ERROR;

#endif

}

#endif

////THUMB
////WFI  
//__asm void WFI_SET(void)
//{
//	WFI;		  
//}
////(faultNMI)
//__asm void INTX_DISABLE(void)
//{
//	CPSID   I
//	BX      LR	  
//}
////
//__asm void INTX_ENABLE(void)
//{
//	CPSIE   I
//	BX      LR  
//}
////
////addr:
//__asm void MSR_MSP(u32 addr) 
//{
//	MSR MSP, r0 			//set Main Stack value
//	BX r14
//}
