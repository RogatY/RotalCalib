/********************************************************************************************************
 *
 *   nRF24L01 Test code - this code has not been fully ported or tested
 *
 *   SPI subsystem has not been implemented in main code
 *
 *   Code extracted from arduino example and moved to CooCox
 *
 *******************************************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "nRF24L01.h"

unsigned char TX_ADDRESS[TX_ADR_WIDTH]  = {0x34,0x43,0x10,0x10,0x01}; // Define a static TX address
unsigned char RX_ADDRESS[RX_ADR_WIDTH]  = {0x34,0x43,0x10,0x10,0x01}; // Define a static RX address


static unsigned char  sta;
#define	  RX_DR	    (sta&0x40)
#define   TX_DS	    (sta&0x20)
#define   MAX_RT    (sta&0x10)

//#define nRF24L01_RX_Debug 1

extern void delay_us(unsigned long ult);


void digitalWrite(unsigned char pin, int state)
{

}

unsigned char digitalRead(unsigned char pin)
{

}

void pinMode(unsigned char, int32_t)
{

}

void SPI_Init(void)
{
    pinMode(nRF24L01_SCK_PIN, OUTPUT); pinMode(nRF24L01_CE_PIN, OUTPUT); pinMode(nRF24L01_CSN_PIN, OUTPUT);
    pinMode(nRF24L01_MOSI_PIN, OUTPUT); pinMode(nRF24L01_MISO_PIN, INPUT); pinMode(nRF24L01_IRQ_PIN, INPUT);

    // Init SPI subsystem
}

void nRF24L01_Init(void)
{ 
    SPI_Init();
    
    nRF24L01_CE_CLR;   // chip enable
    nRF24L01_CSN_SET;   // Spi disable 
    nRF24L01_SCK_CLR;   // Spi clock line init high
    
    SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);
    SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, RX_ADR_WIDTH);
    
    SPI_RW_Reg(WRITE_REG + CONFIG, 0xc); // 16-CRC  
#ifdef nRF24L01_TX_Debug
    //SPI_RW_Reg(WRITE_REG + EN_AA, 0x00);
    //SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x00);
    //SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x00);
#elif (defined nRF24L01_RX_Debug)
    //SPI_RW_Reg(WRITE_REG + EN_AA, 0x00);
    //SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);
#else
    SPI_RW_Reg(WRITE_REG + EN_AA, 0x00);
    SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);
    SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x0a);
#endif
    
    SPI_RW_Reg(WRITE_REG + SETUP_AW, 0x03);     // Setup address width=5 bytes
    SPI_RW_Reg(WRITE_REG + RF_CH, 0);
    SPI_RW_Reg(WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);
    SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);
   
    // Disable dynamic payloads, to match dynamic_payloads_enabled setting
    SPI_RW_Reg(WRITE_REG + DYNPD, 0x0); 
    
    SPI_RW_Reg(FLUSH_TX,0xff);
    SPI_RW_Reg(WRITE_REG + STATUS,0xff);
    SPI_RW_Reg(FLUSH_RX,0xff);
}

/**************************************************
  Function: SPI_RW();
  Description:
  Writes one byte to nRF24L01, and return the byte read
  from nRF24L01 during write, according to SPI protocol
**************************************************/
unsigned char SPI_RW(unsigned char byte)
{
    unsigned char i;
    for(i=0;i<8;i++)
    {
        if(byte&0x80)   nRF24L01_MOSI_SET;
        else            nRF24L01_MOSI_CLR;
        byte<<=1; 
        nRF24L01_SCK_SET;  
        if(nRF24L01_MISO_ReadDat)   byte |= 1;
        else  byte &= 0xFE;
        nRF24L01_SCK_CLR;
    }
    return byte;
}
/**************************************************/

/**************************************************
  Function: SPI_RW_Reg();
  Description:
  Writes value 'value' to register 'reg'
**************************************************/
unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value)
{
    unsigned char status;
    
    nRF24L01_CSN_CLR;                   // CSN low, init SPI transaction
    status = SPI_RW(reg);               // select register
    SPI_RW(value);                      // ..and write value to it..
    nRF24L01_CSN_SET;                   // CSN high again
    
    return(status);                     // return nRF24L01 status byte
}
/**************************************************/

/**************************************************
  Function: SPI_Read();
  Description:
  Read one byte from nRF24L01 register, 'reg'
**************************************************/
unsigned char SPI_Read(unsigned char reg)
{
    unsigned char reg_val;
    
    nRF24L01_CSN_CLR;                // CSN low, initialize SPI communication...
    SPI_RW(reg);                     // Select register to read from..
    reg_val = SPI_RW(0);             // ..then read registervalue
    nRF24L01_CSN_SET;                // CSN high, terminate SPI communication
    
    return(reg_val);                 // return register value
}
/**************************************************/

/**************************************************
  Function: SPI_Read_Buf();
  Description:
**************************************************/
unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
    unsigned char status,byte_ctr;
    
    nRF24L01_CSN_CLR;                    	  // Set CSN low, init SPI tranaction
    status = SPI_RW(reg);       		  // Select register to write to and read status byte
    
    for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
        pBuf[byte_ctr] = SPI_RW(0);           // 
    
    nRF24L01_CSN_SET;                           
    
    return(status);                               // return nRF24L01 status byte
}
/**************************************************/

/**************************************************
  Function: SPI_Write_Buf();
  Description:
  Writes contents of buffer '*pBuf' to nRF24L01
  Typically used to write TX payload, Rx/Tx address
**************************************************/
unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
    unsigned char status,byte_ctr;
    
    nRF24L01_CSN_CLR;                   
    status = SPI_RW(reg);   
    for(byte_ctr=0; byte_ctr<bytes; byte_ctr++) 
        SPI_RW(*pBuf++);
    nRF24L01_CSN_SET;                 
    return(status);                   
}
/**************************************************/

/**************************************************
  Function: RX_Mode();
  Description:
**************************************************/
void SetRX_Mode(void)
{
    nRF24L01_CE_CLR;
    SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);   
    SPI_RW_Reg(WRITE_REG + STATUS, 0x70);
    nRF24L01_CE_SET; 
    delay_us(400);
}
/**************************************************/
unsigned char nRF24L01_RxPacket(unsigned char* rx_buf)
{
    unsigned char revale=0;

    sta=SPI_Read(STATUS);	// read register STATUS's value
    if(RX_DR)			// if receive data ready (RX_DR) interrupt
    {
        nRF24L01_CE_CLR; 
        SPI_Read_Buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
        revale =1;//we have receive data
    }
    SPI_RW_Reg(WRITE_REG+STATUS,sta);// clear RX_DR or TX_DS or MAX_RT interrupt flag
    
    return revale;
}

/**************************************************
  Function: nRF24L01_TxPacket();

  Description:
  This function initializes one nRF24L01 device to
  TX mode, set TX address, set RX address for auto.ack,
  fill TX payload, select RF channel, datarate & TX pwr.
  PWR_UP is set, CRC(2 bytes) is enabled, & PRIM:TX.
  
	ToDo: One high pulse(>10us) on CE will now send this
	packet and expext an acknowledgment from the RX device.
**************************************************/
void nRF24L01_TxPacket(unsigned char * tx_buf)
{
    nRF24L01_CE_CLR;//StandBy Iģʽ
    SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);
    SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);
    SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);
    nRF24L01_CE_SET;
    delay_us(100);
}
/**************************************************/

/************************************************** 
 Check_ACK()
      clear TX FIFO
TX_Mode(TX_BUF);
Check_ACK(1);
delay_ms(250);  

*************************************************/  
unsigned char Check_ACK(unsigned char clear)  
{  
    while(nRF24L01_IRQ_ReadDat);  
    sta = SPI_RW(NOP);
    if(MAX_RT)  
        if(clear)
            SPI_RW(FLUSH_TX);  
    SPI_RW_Reg(WRITE_REG + STATUS, sta);
    if(TX_DS)  
        return(0x00);  
    else  
        return(0xff);  
}

uint8_t flush_tx(void)
{
  uint8_t status;

  nRF24L01_CSN_CLR;
  status = SPI_RW(FLUSH_TX);
  nRF24L01_CSN_SET;

  return status;
}

uint8_t flush_rx(void)
{
  uint8_t status;

  nRF24L01_CSN_CLR;
  status = SPI_RW(FLUSH_RX);
  nRF24L01_CSN_SET;

  return status;
}

unsigned char nRF24L01_debug(void)
{
#ifdef nRF24L01_RX_Debug
    unsigned char RxBuf[RX_PLOAD_WIDTH]={0};
    SetRX_Mode();
    while(1){
        SetRX_Mode();
        if(nRF24L01_RxPacket(RxBuf)){ 
            //add your code
            return 1;
        }
    } 
#endif
    
#ifdef nRF24L01_TX_Debug   
    unsigned char sta=0; 
    unsigned char TxBuf[TX_PLOAD_WIDTH]={0};
    nRF24L01_TxPacket(TxBuf);
    while(1){ 
        nRF24L01_TxPacket(TxBuf);
        sta = SPI_Read(STATUS);
        if(sta == 0x2e){
            //add your code
            return 1;
        }
        SPI_RW_Reg(WRITE_REG+STATUS, 0xff);
        xSysCtlDelay(10000);
    }
#endif
}

void print_status(uint8_t status)
{
    printf("STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n",
           status,
           (status & RX_DR)?1:0,
           (status & TX_DS)?1:0,
           (status & MAX_RT)?1:0,
           ((status >> 1) & 0x7),
           (status & 0)?1:0
          );
}

void print_observe_tx(uint8_t value)
{
    printf("OBSERVE_TX=%02x: POLS_CNT=%x ARC_CNT=%x\r\n",
           value,
           (value >> 4) & 0x0f,
           (value >> 0) & 0x0f
          );
}
