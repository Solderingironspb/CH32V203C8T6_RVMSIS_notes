#include "main.h"
#include "string.h"
#include "stdio.h"

#define DEBUG_USE_USART_NUM             3 //§ª§ã§á§à§Ý§î§Ù§à§Ó§Ñ§ä§î USART3 §Õ§Ý§ñ §à§ä§Ý§Ñ§Õ§Ü§Ú

#define PRINT(format, ...)    printf(format, ##__VA_ARGS__)

__attribute__((used))
    int _write(int fd, char *buf, int size)
    {
        int i;

        for(i = 0; i < size; i++){
    #if(DEBUG_USE_USART_NUM == 1)
            while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
            USART_SendData(USART1, *buf++);
    #elif(DEBUG_USE_USART_NUM == 2)
            while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
            USART_SendData(USART2, *buf++);
    #elif(DEBUG_USE_USART_NUM == 3)
            while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
            USART_SendData(USART3, *buf++);
    #endif
        }

        return size;
    }
