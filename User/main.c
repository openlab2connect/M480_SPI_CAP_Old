/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Implement SPI Master loop back transfer. This sample code needs to
 *           connect MISO_0 pin and MOSI_0 pin together. It will compare the
 *           received data with transmitted data.
 *
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdbool.h>
#include "NuMicro.h"

#define TEST_COUNT  64
#define PLL_CLOCK   192000000

uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];

void SYS_Init(void)
{
    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Select PCLK0 as the clock source of SPI1 */
    CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_PCLK0, MODULE_NoMsk);

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable SPI1 peripheral clock */
    CLK_EnableModuleClock(SPI1_MODULE);

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Setup SPI0 multi-function pins */
//    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA0MFP_SPI0_MOSI | SYS_GPA_MFPL_PA1MFP_SPI0_MISO | SYS_GPA_MFPL_PA2MFP_SPI0_CLK | SYS_GPA_MFPL_PA3MFP_SPI0_SS;
    /* Setup SPI1 multi-function pins */
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB4MFP_SPI1_MOSI | SYS_GPB_MFPL_PB5MFP_SPI1_MISO | SYS_GPB_MFPL_PB3MFP_SPI1_CLK | SYS_GPB_MFPL_PB2MFP_SPI1_SS;

    /* Enable SPI0 clock pin (PA2) schmitt trigger */
//    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;
    /* Enable SPI1 clock pin (PB3) schmitt trigger */
//    PB->SMTEN |= GPIO_SMTEN_SMTEN3_Msk;

    /* Enable SPI0 I/O high slew rate */
//    GPIO_SetSlewCtl(PA, 0xF, GPIO_SLEWCTL_HIGH);
    /* Enable SPI1 I/O high slew rate */
//    GPIO_SetSlewCtl(PB, 0xF, GPIO_SLEWCTL_HIGH);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();
}

uint8_t SPI_Read(char * byteArray, uint8_t byteSize, uint8_t byteStop) {
	uint8_t i;
	uint8_t rb;
	uint8_t read = 0x7f;
	bool stop = false;

	// The register value read start from send the second MOSI 0x7f
	// index i = 3
	for (i=0; i < (byteSize + 2); i++) {
		if (i >= 2)
		  SPI_WRITE_TX(SPI1, (uint32_t)read);
		else
		  SPI_WRITE_TX(SPI1, (uint32_t)byteArray[i]);

		/* Check SPI1 busy status */
		while(SPI_IS_BUSY(SPI1));
		rb = SPI_READ_RX(SPI1);
		printf("SPI:read 0x%x MISO:0x%x\n", (i >= 2) ?read :byteArray[i], rb);

		if (byteStop != 0) {
			if (byteStop == i) {
				stop = true;
				break;
			}
		}
	}
	return (stop) ?rb :0;
}

//void SPI_Read(uint8_t * byteArray, uint8_t byteSize) {
//	uint8_t i;
//	uint8_t rb;
//	uint8_t read = 0x7f;
//
//	for (i=0; i < byteSize+2; i++) {
//		if (i >= 2)
//		  SPI_WRITE_TX(SPI1, (uint32_t)read);
//		else
//		  SPI_WRITE_TX(SPI1, (uint32_t)byteArray[i]);
//		/* Check SPI1 busy status */
//		while(SPI_IS_BUSY(SPI1));
//		rb = SPI_READ_RX(SPI1);
//		printf("SPI:read 0x%x MISO:0x%x\n", (i >= 2) ?read :byteArray[i], rb);
//	}
//}

void SPI_Write(uint8_t * byteArray, uint8_t byteSize) {
	uint8_t i;
	uint8_t rb;
	uint8_t write = 0x7e;

	for (i=0; i < byteSize; i++) {
		if (i == 2)
		  SPI_WRITE_TX(SPI1, (uint32_t)write);
		else
		  SPI_WRITE_TX(SPI1, (uint32_t)byteArray[i]);
		/* Check SPI1 busy status */
		while(SPI_IS_BUSY(SPI1));
		rb = SPI_READ_RX(SPI1);
		printf("SPI:write 0x%x MISO:0x%x\n", (i != 2) ?byteArray[i] :write, rb);
	}
}
//void SPI_Read(char * byte_array, uint8_t maxbyte) {
//	uint8_t i;
//	uint8_t rb;
//
//	for (i=0; i < maxbyte; i++) {
//		SPI_WRITE_TX(SPI1, (uint32_t)byte_array[i]);
//		/* Check SPI1 busy status */
//		while(SPI_IS_BUSY(SPI1));
//		rb = SPI_READ_RX(SPI1);
//		printf("SPI:read 0x%x MISO:0x%x\n", byte_array[i], rb);
//	}
//}
//
//void SPI_Write(char * byte_array, uint8_t maxbyte) {
//	uint8_t i;
//	uint8_t rb;
//
//	for (i=0; i < maxbyte; i++) {
//		SPI_WRITE_TX(SPI1, (uint32_t)byte_array[i]);
//		/* Check SPI1 busy status */
//		while(SPI_IS_BUSY(SPI1));
//		rb = SPI_READ_RX(SPI1);
//		printf("SPI:write 0x%x MISO:0x%x\n", byte_array[i], rb);
//	}
//}

void SPI_Reset(void) {
	uint8_t i;
	uint8_t rb;
	uint8_t reset = 0x7a;

	for (i=0; i<2; i++) {
	  SPI_WRITE_TX(SPI1, (uint32_t)reset);
	  /* Check SPI1 busy status */
	  while(SPI_IS_BUSY(SPI1));
	  rb = SPI_READ_RX(SPI1);
	  printf("SPI:reset MISO:0x%x\n", rb);
	}
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2MHz */
    //SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 32, 2000000);

    /* 8-bit transaction */
	SPI_Open(SPI1, SPI_MASTER, SPI_MODE_3, 8, 2000000);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    //SPI_EnableAutoSS(SPI0, SPI_SS, SPI_SS_ACTIVE_LOW);

    SPI_EnableAutoSS(SPI1, SPI_SS, SPI_SS_ACTIVE_LOW);

}

void delay_us(int usec)
{
    /*
     *  Configure Timer0, clock source from XTL_12M. Prescale 12
     */
    /* TIMER0 clock from HXT */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR0SEL_Msk)) | CLK_CLKSEL1_TMR0SEL_HXT;
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk;
    TIMER0->CTL = 0;        /* disable timer */
    TIMER0->INTSTS = (TIMER_INTSTS_TIF_Msk | TIMER_INTSTS_TWKF_Msk);   /* write 1 to clear for safety */
    TIMER0->CMP = usec;
    TIMER0->CTL = (11 << TIMER_CTL_PSC_Pos) | TIMER_ONESHOT_MODE | TIMER_CTL_CNTEN_Msk;

    while (!TIMER0->INTSTS);
}

/**
 * @brief       GPIO PB IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PB default IRQ, declared in startup_M480.s.
 */
void GPA_IRQHandler(void)
{
	uint8_t active_low;
	uint8_t main_ctrl[2] = {0x7d, 0x00};
	uint8_t main_ctrl_0[4] = {0x7d, 0x0, 0x0, 0x00};

    /* To check if PA.12 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PA, BIT12))
    {
        active_low = SPI_Read(main_ctrl, sizeof(main_ctrl), 3);
        SPI_Write(main_ctrl_0, sizeof(main_ctrl_0));
        GPIO_CLR_INT_FLAG(PA, BIT12);
        printf("PA.12 INT occurred\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PB interrupts */
        PA->INTSRC = PA->INTSRC;
        printf("Un-expected interrupts.\n");
    }
}

int main(void)
{
    uint8_t led_reg72[4] = {0x7d, 0x72, 0x0, 0x3f};
    uint8_t led_reg71[2] = {0x7d, 0x71};
    uint8_t product_id[2] = {0x7d, 0xfd};
    uint8_t input_10[2] = {0x7d, 0x10};
    uint8_t input_1f[4] = {0x7d, 0x1f, 0x0, 0x0f};
    uint8_t cs_recal[4] = {0x7d, 0x26, 0x00, 0x3f};

    uint8_t main_ctrl_0[4] = {0x7d, 0x0, 0x0, 0x00};
    uint8_t main_ctrl[2] = {0x7d, 0x00};

    uint8_t active_low = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Init SPI */
    SPI_Init();

    printf("\n\n");
    printf("+--------------------------------------------------------------------+\n");
    printf("|                   M480 SPI1 CAP Test Code                          |\n");
    printf("+--------------------------------------------------------------------+\n");
    printf("\n");

    SPI_Reset();

    SPI_Write(led_reg72, sizeof(led_reg72));
    //SPI_Read(led_reg71, sizeof(led_reg71)+3);

    //SPI_Write(input_1f, sizeof(input_1f));
    SPI_Write(cs_recal, sizeof(cs_recal));
    delay_us(600000);

    SPI_Write(main_ctrl_0, sizeof(main_ctrl_0));

    GPIO_SetMode(PA, BIT12, GPIO_MODE_INPUT);
    GPIO_EnableInt(PA, 12, GPIO_INT_LOW);
    NVIC_EnableIRQ(GPA_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PA, BIT12);

    while(1) {
      SPI_Read(input_10, sizeof(input_10)+5, 0);
//      active_low = SPI_Read(main_ctrl, sizeof(main_ctrl), 3);
//      printf("--------------------------\n");
//      if (active_low & 0x01) {
//    	  printf("cap int active low\n");
//    	  SPI_Write(main_ctrl_c0, sizeof(main_ctrl_c0));
//      }

      delay_us(1000000);
      //printf("%d %d %d\n", PA12, PC0, PC1);
    }

    /* Close SPI1 */
    SPI_Close(SPI1);

    while(1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
