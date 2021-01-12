//
// Smpl_Timer_SR04 : using one SR04 Ultrasound Sensors
//
// Timer Capture :
// GPB2 / RTS0 / T2EX (NUC140 pin34)
// GPB4 / RX1         (NUC140 pin19)

// SR04 Ultrasound Sensor 
// pin1 Vcc : to Vcc
// pin2 Trig: to GPB4      (NUC140VE3xN pin19)
// pin3 ECHO: to GPB2/T2EX (NUC140VE3xN pin34)
// pin4 Gnd : to GND



// SRF04 range : 2cm to 450cm
// jummper select: ON = UART TX/RX, OFF = Level Trig/Echo
// pin1 : Vcc +5V
// pin2 : Trig/TX  to TX0/GPB1 (NUC140-pin32)
// pin3 : Echo/RX  to RX0/GPB0 (NUC140-pin33)
// pin4 : Gnd
// pin5 : Gnd


// HC05 Bluetooth module
// pin1 : KEY   N.C
// pin2 : VCC   to Vcc +5V
// pin3 : GND   to GND
// pin4 : TXD   to NUC140 UART0-RX (GPD14)
// pin5 : RXD   to NUC140 UART0-TX (GPD15)
// pin6 : STATE N.C.

//
// Buzzer : E_GPB11
//
//	Switch : E_GPA6
//

#include <stdio.h>
#include <stdbool.h>
#include "NUC1xx.h"
#include "Driver\DrvUART.h"
#include "DrvGPIO.h"
#include "DrvSYS.h"
#include "LCD_Driver.h"
#include "scankey.h"

// Global definition
#define	_SR04A_ECHO		   (GPB_2)			//NUC140VE3xN, Pin19
#define	_SR04A_TRIG		   (GPB_4)			//NUC140VE3xN, Pin34
#define	_SR04A_TRIG_Low	 (GPB_4=0)
#define	_SR04A_TRIG_High (GPB_4=1)

// Global variables
volatile uint8_t comRbuf[9];
volatile uint8_t comRbytes = 0;
volatile uint32_t SR04A_Echo_Width = 0;
volatile uint32_t SR04A_Echo_Flag  = FALSE;
	char     TEXT2[17] = "SR04a: ";
	uint32_t distance_mm;
	uint16_t MailCount = 0;
	uint8_t DATASIZE = 8;
	uint32_t DistanceFromMail;
	uint8_t  write_byte[1];
	uint8_t  read_byte[2];
	uint8_t num=0;
	uint8_t i;
	char  write_buf[16] ;
	char InitBuffer[16];
	char NewLine[8] = 0x0a;
	char StartLine[8] = 0x0d;
	uint8_t password[4] = {1,2,3,4};
	bool Flag = true;
	bool NoMailFlag = true;
	bool NumFlag = true;
	bool OpenBoxFlag = false;
	bool MessageAlarm = true;
	char TEXT[16] = "                ";
	STR_UART_T sParam;
//
// Initial TMR2
//
// Timer Clock:	12 MHz
// Prescale:		11
// Compare:		0xffffff
// Mode:			One-Shot mode
// Capture:		Enable, Capture with Falling Edge
void Init_TMR2(void)
{	
	//Step 1: T2EX pin Enable (PB.2, Pin34)
	SYS->GPBMFP.UART0_nRTS_nWRL = 1;	
	SYS->ALTMFP.PB2_T2EX = 1;
	
  //Step 2: Timer Controller Reset and Setting Clock Source
	SYS->IPRSTC2.TMR2_RST = 1;          //Timer Controller: Reset
	SYS->IPRSTC2.TMR2_RST = 0;          //Timer Controller: Normal
	SYSCLK->CLKSEL1.TMR2_S = 0;	        //Timer Clock = 12 MHz
	SYSCLK->APBCLK.TMR2_EN = 1;         //Timer C lock Enable

	//Step 3: Timer Controller Setting
	//  TMR0_T = (12 MHz / (11+1) / 1000000)^-1 = 1.000 Second
	TIMER2->TCMPR = 0xffffff;           //Timer Compare Value:  [0~16777215]
	TIMER2->TCSR.PRESCALE = 11;         //Timer Prescale:       [0~255]
	TIMER2->TCSR.MODE = 0;              //Timer Operation Mode: One-Shot

	//Step 4: External Capture Mode Setting
	TIMER2->TEXCON.TEXEN = 1;	          //External Capture Function Enable
	TIMER2->TEXCON.RSTCAPSEL = 0;	      //Capture Mode Select: Capture Mode
	TIMER2->TEXCON.TEX_EDGE = 2;	      //Capture Edge: Rising & Falling

	//Step 5: Timer Interrupt Setting
//	TIMER2->TCSR.IE = 1;				      //Timeout Interrupt Enable
//	TIMER2->u32TISR |= 0x01;		      //Clear Timeout Flag (TIF)
	TIMER2->TEXCON.TEXIEN = 1;		      //Capture Interrupt Enable
	TIMER2->u32TEXISR |= 0x01;		      //Clear Capture Flag (TEXIF)
	NVIC_EnableIRQ(TMR2_IRQn);		      //Timer NVIC IRQ Enable

	//Step 6: Start Timer Capture (Set by Ultrasonic_Trigger() Function)
// 	TIMER2->TCSR.CRST = 1;			      //Timer Counter Reset
// 	TIMER2->TCSR.CEN = 1;				      //Timer Start
}

// TMR2 Interrupt Handler
void TMR2_IRQHandler(void)
{
	TIMER2->TEXCON.RSTCAPSEL = 0;       // set back for falling edge to capture
	TIMER2->TCSR.CEN = 1;					      //Timer Start

	if(TIMER2->TEXISR.TEXIF == 1)	      //Capture Flag (TEXIF)
	{
	 	TIMER2->u32TEXISR |= 0x01;				//Clear Capture Flag (TEXIF)
		SR04A_Echo_Width = TIMER2->TCAP;	//Load Capture Value (Unit: us)
		SR04A_Echo_Flag  = TRUE;
	}
}

// Ultrasonic Trigger
void SR04_Trigger(void)
{
	//Trigger of Ultrasonic Sensor
	_SR04A_TRIG_High;
	DrvSYS_Delay(10);							// 10us for TRIG width
	_SR04A_TRIG_Low;
	
  TIMER2->TEXCON.RSTCAPSEL = 1; // set for rising edge trigger to reset counter
}

void Init_GPIO_SR04(void)
{
	//Ultrasonic I/O Pins Initial
	GPIOB->PMD.PMD2 = 0;							//_SR04_ECHO pin, Input						
	GPIOB->PMD.PMD4 = 1;              //_SR04_TRIG pin, Output
  _SR04A_TRIG_Low;                  // set Trig output to Low
}

void Dist_Ultra_down(void)
{
	 	write_byte[0]=0x55;						// trigger SRF04
		DrvUART_Write(UART_PORT0,write_byte,1);	// write to SRF04
		DrvUART_Read(UART_PORT0,read_byte,2); 	// read two bytes from SRF04
		DistanceFromMail = read_byte[0]*256 + read_byte[1];// distance = byte[0] *256 + byte[1];
// 		sprintf(TEXT+0,"Distance: %dmm",DistanceFromMail);
// 		print_lcd(1, TEXT);
		//return  DistanceFromMail;
}


void UltraCounter(void)
{
    SR04_Trigger();                 // Trigger Ultrasound Sensor for 10us   		
		DrvSYS_Delay(40000);            // Wait 40ms for Echo to trigger interrupt
	
	
			if(SR04A_Echo_Flag==TRUE)
		{
			SR04A_Echo_Flag = FALSE;			

			distance_mm = SR04A_Echo_Width * (340/2) / 1000;
	  }   
}

bool CheckIfInsideMail(uint32_t distance)
{	

	if  ((distance < 40) && (Flag == true))
	{

		Flag = false;
		return true;
	}
	else
	{
		Flag = true;
		return false;
	}

}

void CountMail(void)
{
		if (CheckIfInsideMail(distance_mm))
		{
			MailCount++;
			sprintf(write_buf, "%d Mail! ", MailCount);	
			DrvUART_Write(UART_PORT2,NewLine,DATASIZE);
			DrvUART_Write(UART_PORT2,StartLine,DATASIZE);
			DrvUART_Write(UART_PORT2,write_buf,DATASIZE);
			DrvSYS_Delay(1000000);  
			while(distance_mm < 40)
			{
				DrvSYS_Delay(3000000);  
				UltraCounter();
			}
			Flag = true;
			
		}

}


void ResetCounter(void)
{
	if (DistanceFromMail > 25 && NoMailFlag==true )
	{
		MailCount =0;
		DrvUART_Write(UART_PORT2,NewLine,DATASIZE);
		DrvUART_Write(UART_PORT2,StartLine,DATASIZE);
 		DrvUART_Write(UART_PORT2,"No Mail!",DATASIZE);
		NoMailFlag = false;
	}
	if(MailCount != 0 )
	{
		NoMailFlag = true ;
	}
}


void UART_INT_HANDLE(void)
{
	while(UART0->ISR.RDA_IF==1) 
	{
		comRbuf[comRbytes]=UART0->DATA;
		comRbytes++;		
		if (comRbytes==8) {	
			sprintf(TEXT,"%s",comRbuf);
			print_lcd(1,TEXT);			
		  comRbytes=0;
		}
	}
}


void Buzzer(void) // Initialize GPIO pins
{

	
		if (DrvGPIO_GetBit(E_GPA,6) == 1 && OpenBoxFlag == false )
		{
			if (MessageAlarm == true)
			{
				MessageAlarm = false;
				DrvUART_Write(UART_PORT2,NewLine,DATASIZE);
				DrvUART_Write(UART_PORT2,StartLine,DATASIZE);
				DrvUART_Write(UART_PORT2,"Box open",DATASIZE);
				DrvGPIO_ClrBit(E_GPB,11); 
			}

		}
		else
		{
			MessageAlarm = true;
			DrvGPIO_SetBit(E_GPB,11); 
	  }
}



bool CheckPassword()
{
	NumFlag = true;
	for (i=0 ; i<4 ;i++)
	{
	  print_lcd(0, "Smart MailBox");			
		print_lcd(1, "Enter password:");
		num = Scankey();
		DrvSYS_Delay(500000); 
		while ( num == 0)
		{
			num = Scankey();
			DrvSYS_Delay(500000); 
		}

		if(num != password[i])
		{
				clr_all_panel();   
			
				NumFlag= false;
		}
		
	}
	if ( NumFlag == false ) 
		{

			return false;
		}
		else
		{
			return true;
		}

}
	


void CheckPressNum(void)
{
	num = Scankey();
	DrvSYS_Delay(500000); 
	if(num != 0 ) 
	{
		clr_all_panel();               
		print_lcd(0, "Smart MailBox");	   
		print_lcd(1, "Enter password:");
		if (CheckPassword())
		{
			num = Scankey();			
			while ( DrvGPIO_GetBit(E_GPA,6) == 0) 
			{
				print_lcd(0, "Smart MailBox");	   
				print_lcd(1, "Open Mailbox");	
			}
			while( DrvGPIO_GetBit(E_GPA,6) == 1 )
				{
					OpenBoxFlag= true;
					DrvSYS_Delay(1000000); 
					clr_all_panel();               
					print_lcd(0, "Smart MailBox");	   
					print_lcd(1, "Close Mailbox");
					if ( DrvGPIO_GetBit(E_GPA,6) == 0)
					{
						OpenBoxFlag=false;
						break;
					}
				}
				DrvSYS_Delay(500000); 
			
		}
		else
		{
			clr_all_panel();               			
			print_lcd(0, "Smart MailBox");	
			print_lcd(1, "Wrong password!");
		for(i = 0 ; i<7 ; i++)
			{			
				DrvSYS_Delay(1000000); 
			}
		}
	}
}






//------------------------------
// MAIN function
//------------------------------
int main (void)
{
	
	STR_UART_T sParam;
	//System Clock Initial
	UNLOCKREG();
	DrvSYS_SetOscCtrl(E_SYS_XTL12M, ENABLE);
	while(DrvSYS_GetChipClockSourceStatus(E_SYS_XTL12M) == 0);
	DrvSYS_Open(50000000);
	LOCKREG();
	
	OpenKeyPad();
	
	DrvGPIO_Open(E_GPA, 6, E_IO_QUASI); // Door alaram          
	DrvGPIO_Open(E_GPB, 11, E_IO_OUTPUT); // initial GPIO pin GPB11 for controlling Buzzer
	
	
	/* Set UART Pin */
	DrvGPIO_InitFunction(E_FUNC_UART0);
	
		/* Set UART Pin */
	DrvGPIO_InitFunction(E_FUNC_UART2);
	
	
	/* UART Setting */
    sParam.u32BaudRate 		= 9600;
    sParam.u8cDataBits 		= DRVUART_DATABITS_8;
    sParam.u8cStopBits 		= DRVUART_STOPBITS_1;
    sParam.u8cParity 		= DRVUART_PARITY_NONE;
    sParam.u8cRxTriggerLevel= DRVUART_FIFO_1BYTES;
	
			/* Set UART0 Configuration */
 	if(DrvUART_Open(UART_PORT0,&sParam) != E_SUCCESS);

			/* Set UART1 Configuration */
 	if(DrvUART_Open(UART_PORT2,&sParam) != E_SUCCESS);

	//DrvUART_EnableInt(UART_PORT2, DRVUART_RDAINT, UART_INT_HANDLE);
	
	
	Initial_panel();                  // initialize LCD
	clr_all_panel();                  // clear LCD display
	print_lcd(0, "Smart MailBox");	   
	print_lcd(1, "Press any key");
	        
	
	Init_TMR2();                      // initialize Timer2 Capture
	



	while(1) 
	{
		Buzzer();
		clr_all_panel();                  // clear LCD display
		print_lcd(0, "Smart MailBox");	   
		print_lcd(1, "Press any key");
		UltraCounter();
		CountMail();
		Dist_Ultra_down();
		ResetCounter();
		CheckPressNum();
		DrvSYS_Delay(10000);           // 10ms from Echo to next Trigger
	}


}
