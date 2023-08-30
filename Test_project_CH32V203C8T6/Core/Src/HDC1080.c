/*
 * HDC1080.c
 *
 *  Created on: Aug 23, 2023
 *      Author: Solderingiron
 */


#include "HDC1080.h"

uint8_t I2C1_tx_buffer[4] = { 0, };  //������է��ڧ� �ҧ��֧�
uint8_t I2C1_rx_buffer[10] = { 0, };  //�����է��ڧ� �ҧ��֧�
float Temperature, Humidity;

void HDC1080_init(void){
    /*-------------------������ާ��� �ߧѧ����֧� �էݧ� �ާ�է�ݧ� HDC1080-------------------*/
    I2C1_tx_buffer[0] = 0x02;  //����ާѧߧէ� �ܧ�ߧ�ڧԧ��ѧ�ڧ�
    I2C1_tx_buffer[1] = 0x10;
    I2C1_tx_buffer[2] = 0x00;
    //���ܧѧا֧� �ާ�է�ݧ�, ���� ����ڧ� �٧ѧ�ڧ�ѧ�� �էѧߧߧ�� �ߧ� �ѧէ�֧� "Configuration" ��� �٧ߧѧ�֧ߧڧ֧� 0x0000
    RVMSIS_I2C_Data_Transmit(I2C1, HDC1080ADDR, I2C1_tx_buffer, 3, 100);
    Delay_ms(20);
    //���ܧѧا֧� �ާ�է�ݧ�, ���� ����ڧ� �����ާ���֧�� �էѧߧߧ�� �� �ѧէ�֧�� "Configuration"
    RVMSIS_I2C_Data_Transmit(I2C1, HDC1080ADDR, I2C1_tx_buffer, 1, 100);
    Delay_ms(20);
    //������ѧߧڧ� �ܧ�ߧ�ڧԧ��ѧ�ڧ� �� ��֧�֧ާ֧ߧߧ��
    RVMSIS_I2C_Data_Receive(I2C1, HDC1080ADDR, I2C1_rx_buffer, 2, 100);
    /*-------------------������ާ��� �ߧѧ����֧� �էݧ� �ާ�է�ݧ� HDC1080-------------------*/
}
