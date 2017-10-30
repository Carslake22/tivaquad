#include "registers.h"
#include "macros.c"
#include "math.h"
//#define test
void calibrateESC(void);
void Delay(unsigned int);
void setupPortF(void);
void setupTimers(void);
void setupSystickTimer(void);
void setupI2c(void);
void setupUart0(void);
void setupUart1 (void);
void setupMpu6050(void);
void getRawSensorData(void);
short mpu6050ReceiveTwoByte(unsigned int);
void printSensorData(void);
float convertGyroValue(short);
void getGyroCal(void);
void convertAcceltoAngles(float * , float *,float * );
void convertGyrotoAngles(float *, float *, float *, float*, float*, float*);
void getController(void);
unsigned short readUartByte(void);
void printControllerValue(void);
void assignControls(void);
void printControls (void);
void convertControls (void);
void printControls(void);
float convertToFloat(int data);
void applyControls(void);
void printAdjust(void);
int getDt(void);
void printS(char *string);
int serialRead(void);
void incValue(void);




extern short accelDataX;
extern short accelDataY;
extern short accelDataZ;
extern short gyroDataX;
extern short gyroDataY;
extern short gyroDataZ;	
extern float gyroRoll;
extern float gyroPitch;
extern float gyroYaw;
extern float gyroCalX, gyroCalY, gyroCalZ, accelCalX, accelCalY, accelCalZ;
extern float accelRoll;
extern float accelPitch;
extern float accelYaw;
extern float angle;
extern float gyroRollAngle;
extern float gyroPitchAngle;
extern float gyroYawAngle;
extern float *paccelRoll, *paccelPitch, *paccelYaw;
extern float *pgyroRoll, *pgyroPitch, *pgyroYaw, *pgyroRollAngle, *pgyroPitchAngle, *pgyroYawAngle;
extern float roll, *proll;
extern float pitch, *ppitch;
extern unsigned short rawThrottleControl, rawRollControl, rawPitchControl, rawButtonControl, *pRawThrottleControl, *pRawRollControl, *pRawPitchControl;
extern unsigned short  *pRawButtonControl;
extern int uartRxFrame[25];
extern float throttleC, rollC, pitchC, *pthrottleC, *prollC, *ppitchC, floatRoll, floatPitch, *pfloatRoll, *pfloatPitch;
extern short R1, L1, X , TRI;
extern float rollcorr, pitchcorr, rollerror, pitcherror, yawrate, yawcorr, yawerror, rollAccum, pitchAccum;
extern int timeBefore, dt;
extern float PitchBefore, RollBefore;
extern int newDataFlag;
extern float pitchP, pitchI, pitchD, rollP,rollI, rollD;
extern int test;
//extern float Kp,Ki,Kd;

void putChar(char);
void out2Dec(int, int);
void putNum(int,int);
void putNum1(int);
void putFloat(float num, int decimalPoints);
void mpu6050TransmitSingleByte(unsigned int, unsigned int);
short mpu6050ReceiveSingleByte(unsigned int);



void Delay(unsigned int numLoops)
{
volatile unsigned int lp;
volatile unsigned int i;

for(lp=0; lp<numLoops; lp++)
	for (i=0; i<=0xFFFF; i++) ;
}

void calibrateESC(void){
	
	setupPortF();
	SYSCTL_RCGCTIMER |= 0x7;                 // enable clock for timer 0 and 1
	while ((SYSCTL_PRTIMER &0x7) != 0x7);    // wait for stable clock
	TIMER0_CTL &= ~0x101;                     // disable timer 0
	TIMER1_CTL &= ~0x101;                     // disable timer 1
	TIMER2_CTL &= ~0x101;                     // disable timer 1
	TIMER0_CTL |= 0x4040;                    // invert output timer 0 a and b
	TIMER1_CTL |= 0x4040;                    // invert output timer 1 a and b
	TIMER2_CTL |= 0x2;                      // freeze counting in debug
	TIMER0_CFG |= 0x4;                       // set to 16 bit timer
	TIMER1_CFG |= 0x4;                       // set to 16 bit timer
	TIMER2_CFG &= ~0x1;                       // set to 32 bit timer
	TIMER0_TAMR |= 0xA;                      // set PWM mode and periodic mode
	TIMER0_TBMR |= 0xA;                      // set PWM mode and periodic mode
	TIMER1_TAMR |= 0xA;                      // set PWM mode and periodic mode
	TIMER1_TBMR |= 0xA;                      // set PWM mode and periodic mode
	TIMER2_TAMR |= 0x2;                      // set to periodic mode
	TIMER0_TAILR = FREQ350;                   // set preload value to 2.86ms
	TIMER0_TBILR = FREQ350;                    // set preload value to 2.86ms
  TIMER1_TAILR = FREQ350;                   // set preload value to 2.86ms
	TIMER1_TBILR = FREQ350;                   // set preload value to 2.86ms
	TIMER0_TAMATCH = FULL;                       // set match value to 1 ms
	TIMER0_TBMATCH = FULL;                       // set match value to 1 ms
	TIMER1_TAMATCH = FULL;                        // set match value to 1 ms
	TIMER1_TBMATCH = FULL;                        // set match value to 1 ms
	TIMER0_CTL |= 0x101;                       // enable timer 0
	TIMER1_CTL |= 0x101;                       // enable timer 1
	TIMER2_CTL |= 0x101;                       // enable timer 1
	
	setupUart0();
	
	while (1)
	{
		incValue();
		putNum(TIMER0_TAMATCH ,3);
	}
}
	
	
	





void setupPortF(void){
	
 SYSCTL_RCGCGPIO |= 0x20;                  // enable clock for port E
 while ((SYSCTL_PRGPIO & 0x20) != 0x20);   // check for stable clock
 GPIO_PORTF_LOCK = 0x4C4F434B;           // enable writing to gpio cr register
 GPIOF_CR = 0xFF;                          // enable port zero to be changed
 GPIOF_DIR |= 0x1F;
 GPIOF_AFSEL |= 0x1F;          // select alternative function
 GPIOF_PCTL &= ~0xFFFFF;                    // and not, clear port control values
 GPIOF_PCTL |= 0X77777;                     // SET PORT CONTROL TO TIMER FUNCTIONS	
 GPIOF_DEN |= 0x1F;           // SET TO DIGITAL

}

void setupSystickTimer(void){
	
	NVIC_ST_CTRL &= ~0x1;                // disable timer for setup
	NVIC_ST_CURRENT |= 0x1;              // clear current counter
	NVIC_ST_CTRL |= 0x6;                  // enable interrupts and use system clock
	NVIC_ST_RELOAD = threehundred;              // set to 300Hz
	NVIC_SYS_PRI3 &= ~(0x1<<29);             // set interrupt priority to zero
}



void setupTimers(void){
	
	setupPortF();
	SYSCTL_RCGCTIMER |= 0x7;                 // enable clock for timer 0,1 AND 2
	while ((SYSCTL_PRTIMER &0x7) != 0x7);    // wait for stable clock
	TIMER0_CTL &= ~0x101;                     // disable timer 0
	TIMER1_CTL &= ~0x101;                     // disable timer 1
	TIMER2_CTL &= ~0x1;                     // disable timer 2
	TIMER0_CTL |= 0x4040;                    // invert output timer 0 a and b
	TIMER1_CTL |= 0x4040;                    // invert output timer 1 a and b
	TIMER0_CFG |= 0x4;                       // set to 16 bit timer
	TIMER1_CFG |= 0x4;                       // set to 16 bit timer
	TIMER2_CFG &= ~0x1;                       // set to 32 bit timer
	TIMER0_TAMR |= 0xA;                      // set PWM mode and periodic mode
	TIMER0_TBMR |= 0xA;                      // set PWM mode and periodic mode
	TIMER1_TAMR |= 0xA;                      // set PWM mode and periodic mode
	TIMER1_TBMR |= 0xA;                      // set PWM mode and periodic mode
	TIMER2_TAMR |= 0x12;                      // set to periodic mode, count up
	TIMER0_TAILR = FREQ350;                   // set preload value to 2.86ms
	TIMER0_TBILR = FREQ350;                    // set preload value to 2.86ms
  TIMER1_TAILR = FREQ350;                   // set preload value to 2.86ms
	TIMER1_TBILR = FREQ350;                   // set preload value to 2.86ms
 	TIMER2_TAILR = fourhundredms;                   // set preload value to 2.86ms
	TIMER0_TAMATCH = OFF;                       // set match value to 1 ms
	TIMER0_TBMATCH = OFF;                       // set match value to 1 ms
	TIMER1_TAMATCH = OFF;                        // set match value to 1 ms
	TIMER1_TBMATCH = OFF;                        // set match value to 1 ms
	TIMER0_CTL |= 0x101;                       // enable timer 0
	TIMER1_CTL |= 0x101;                       // enable timer 1
	TIMER2_CTL |= 0x1;                       // enable timer 1
}

void setupI2c(void){

	RCGI2C |= 0X1;
	SYSCTL_RCGCGPIO |= 0X2;
	while ((SYSCTL_PRGPIO & 0x2) !=0x2);
	GPIOB_AFSEL |= 0xC;                       // enable alternate function
	GPIOB_DEN |= 0xC;                         // set sda and scl as digital
	GPIOB_DIR |= 0XC;                         // SET AS OUTPUT
	GPIOB_ODR |= 0x8;                         // set sda pin as open drain
	GPIOB_ODR &= ~0x4;                       // 
	GPIOB_PCTL &= ~0xFF00;                   // clear port control
	GPIOB_PCTL |= 0x3300;                    // set as i2c function
	

	I2CMCR |= 0X10;                          // INITALIZE AS MASTER
	I2CMCR &= ~0x80;                         // set tpr to apply to low speed 
	I2CMTPR |= 0x2;                          // SET TPR TO for 100kbps 0x7 = 90kHz
	I2CCLKOCNT |= ~0XFF;                     // set the timeout clock to maximum
	
}

void setupUart0(void){
	
	SYSCTL_RCGUART |= 0x1;
	while ((SYSCTL_PRUART &= 0x1) != 0x1){};
  SYSCTL_RCGCGPIO |= 0x1;
	while ((SYSCTL_PRGPIO &= 0x1) != 0x1){};
	GPIOA_AFSEL |= 0x3;
	GPIOA_PCTL &= ~0xFF;
	GPIOA_PCTL |=0x11;
	GPIOA_DEN |= 0x3;
	UART0_CTL &= ~0x1;                    // disable uart0
	UART0_IBRD = 104;
	UART0_FBRD = 11;
  UART0_LCRH |= 0x10;                   // enable fifo buffer
	UART0_LCRH |= 0x64;                  // set 8 bit word length, even parity
	UART0_LCRH &= ~0x8;                  // 1 stop bit
	UART0_LCRH &= ~0x2;                   //  no parity
	UART0_CTL &= ~0x20;                  // set prescaler of 16
	
	UART0_CTL |= 0x301;                  // enable rx, tx and uart tranismission
	
}

void setupUart1 (void){
	
	SYSCTL_RCGUART |= 0x2;
	while ((SYSCTL_PRUART &= 0x2) != 0x2){};            // enable clock to uart1
	SYSCTL_RCGCGPIO |= 0X2;
	while ((SYSCTL_PRGPIO & 0x2) !=0x2);
	GPIOB_AFSEL |= 0x3;
	GPIOB_PCTL &= ~0xFF;
	GPIOB_PCTL |=0x11;
	GPIOB_DEN |= 0x3;
	UART1_CTL &= ~0x1;                    // disable uart1
	UART1_IBRD =  0X1A;
	UART1_FBRD = 0X3;
	UART1_LCRH |= 0x60;                  // set 8 bit word LENGTH
	UART1_LCRH &= ~0x8;                  // 1 stop bit
	UART1_LCRH &= ~0x2;                   // NO PARITY
	UART1_LCRH |= 0x10;                   // enable fifo buffer
	UART1_CTL &= ~0x20;                  // set prescaler of 16
	UART1_IM |= 0x10;                   // enable RX interrupts 
	NVIC_EN0 |= 0x40;                   // enable interrupts for uart 1
	NVIC_PRI1 &= ~(0xF<<21);            // clear priority field
	NVIC_PRI1 |= (0x1<<21);             // set priority to 1
	UART1_CTL &= ~0x100;                // disable transmit
	UART1_FLS &=  ~0x3F;                // interrupt at 1/8 full on RX
	//UART1_FLS |=  (0x1<<5);
		
	UART1_CTL |= 0x1;                  // enable uart tranismission
	
}


/*************************************************************************************************************************************************/
                                                                                                                                                 //
void setupMpu6050(void){                                                                                                                         //
	                                                                                                                                               //
	short data = mpu6050ReceiveSingleByte(whoAmI);                                                                                                 //
	                                                                                                                                               //
	mpu6050TransmitSingleByte(PWR_MGMT_1,0x01);         // turn on clock to mpu and set clock ref to gyro x reference                              //
	                                                                                                                                               //
 	mpu6050TransmitSingleByte(PWR_MGMT_1,0x08);       //DISABLE TEMP SENSOR                                                                        //
	                                                                                                                                               //
	mpu6050TransmitSingleByte(SMPRT_DIV,0x07);      // SET SAMPLE RATE TO 1KHz                                                                     //
                                                                                                                                                 //
	mpu6050TransmitSingleByte(CONFIG,0x05);         // DISABLED FSYNC,  0x0 260Hz, 0x1 = 185Hz,,0x2 = 95 Hz, 0x3 = 44 Hz, 0x4 =21 Hz, 0x5 = 10 Hz  //
 	                                                                                                                                               //
	mpu6050TransmitSingleByte(GYRO_CONFG,0x08);     // 0X00 = 250DEG/S, 0X08 = 500DEG/S, 0X10 = 1000DEG/S, 0X18 = 2000DEG/S                        //
	                                                                                                                                               //
	mpu6050TransmitSingleByte(ACCEL_CONFIG,0x08);    // 0X00 = 2G, 0X08 = 4G, 0X10 = 8G                                                            //
	                                                                                                                                               //
	mpu6050TransmitSingleByte(MOT_THR,0X00);          // SET MOTION DETETECTION THRESHOLD                                                          //
	                                                                                                                                               //
	mpu6050TransmitSingleByte(FIFO_EN,0X00);          // NO DATA IS LOADED INTO BUFFER                                                             //
	                                                                                                                                               //
	mpu6050TransmitSingleByte(MOT_DETECT_CTRL,0X00);    // NO ACCEL ON DELAY                                                                       //
	                                                                                                                                               //
}                                                                                                                                                //
                                                                                                                                                 //
/*************************************************************************************************************************************************/



void out2Dec(int number, int minprint){
	
	int divider = 100000000;
	int current;
	int hasprinted = 0;
	while ((divider / 10) != 0){
		if((current = number / divider) != 0 || hasprinted){
			putChar((char)(current |= 0x30));
		hasprinted = 1;
		}
	else if( minprint > 0){
		putChar(' ');
		minprint--;
	}
	number = number - (current &= ~0x30) * divider;
}
	}

void putChar(char output){
	while((UART0_FR & 0x20) == 0x20){}
		UART0_DR = output;
	
}

void testUART0()
{
	char inputChar;
	
	while(1)
	{
		//Get a character from the terminal
		
		while((UART0_FR & 0x10) != 0x0);
		inputChar = (UART0_DR &= 0xFF);
		
		if(inputChar == 'p') //Finished if character was 'p'
			return;
		
		//Echo charater to terminal
		
		while((UART0_FR & 0x20) != 0x0);
		UART0_DR |= inputChar;
	}	
}

void putNum(int number, int minPrint)
{
	int divider = 100000000;
	int current;
	int hasPrinted = 0;
	
	if (number < 0){putChar('-');}
	
	while(((divider /=10.0 ) != 0))
	{
		if((current = number / divider) != 0 || hasPrinted)
		{
		 putChar((char)(current |= 0x30));
		 hasPrinted = 1;
		}
		
		else if(minPrint > 0)
		{
			putChar((char)(' '));
			minPrint--;			
		}
		
		number = number - (current &= ~0x30) * divider;
	}
}



void putFloat(float num, int decimalPoints)
{
	int divider = 100000;
	int current;
	int intiger;
	int i;
	
	if(num < 0)
 {
	 num *= -1;
	 putChar('-'); 
 }
 
 intiger = (int)num;
 
	while((divider /=10 ) != 0)
	{
		current = intiger / divider;
		current |= 0x30;
		while((UART0_FR & 0x20) != 0x0);
		UART0_DR = current;		
		intiger = intiger - (current &= ~0x30) * divider;
	}
	
	while((UART0_FR & 0x20) != 0x0){};
		UART0_DR = '.';	
	
	divider = 10;
	
	for(i= 1;i<decimalPoints;i++)
		divider *=10;
	
	num = num-(int)num;
	num *=divider;
	num = (int)num;
	
	while((divider /=10 ) != 0)
	{
		current = num / divider;
		current |= 0x30;
		while((UART0_FR & 0x20) != 0x0);
		UART0_DR = current;		
		num = num - (current &= ~0x30) * divider;
	}	
}


/**************************************************************************************************/

void mpu6050TransmitSingleByte(unsigned int writeAddress, unsigned int data){
	int check;
	int retry = 0;
	do{
	I2CMSA = mpu6050Tx;
	I2CMDR = writeAddress;
	I2CMCS = (masterStart | masterRun);
	while ((I2CMCS & i2cBusy) == i2cBusy){};
		if ((I2CMCS & i2cError) == i2cError)
		{ I2CMCS = masterStop;
			putChar ('e');  
			putChar ('r');  
			putChar ('r');  
			putChar ('o');  
			putChar ('r');  
			putChar ('\n');
		}
	I2CMDR = 0x0;
	I2CMDR = data;
	I2CMCS = (masterStop | masterRun);
	while ((I2CMCS & i2cBusy) == i2cBusy){};
		if ((I2CMCS & i2cError) == i2cError)
		{ I2CMCS = masterStop;
			putChar ('e');  
			putChar ('r');  
			putChar ('r');  
			putChar ('o');  
			putChar ('r');  
			putChar ('\n');
		}
		retry ++;
	}while (((I2CMCS&( i2cAdrack | i2cError )) != 0x0) && (retry < 10));
	
	Delay(0xA);
	check = mpu6050ReceiveSingleByte(writeAddress);
	if ( check == data )
	{
		putChar('o');
		putChar('k');
		putChar((char)(0x0D));
		putChar((char)(0x0A));
		
		}
	else 
	{
		putChar('w');
		putChar('r');
		putChar('i');
		putChar('t');
		putChar('e');
		putChar(' ');
		putChar('e');
		putChar('r');
		putChar('r');
		putChar('o');
		putChar('r');
		putChar('\n');
		putChar((char)0x0D);
	}
}
	
/************************************************************************************************/

short mpu6050ReceiveSingleByte(unsigned int regAddress){
	
	int retry = 0;
	int data;
	do{
	I2CMSA = mpu6050Tx;
	I2CMDR = regAddress;
	I2CMCS = (masterStart | masterRun);
	
	while ((I2CMCS & i2cBusy) == i2cBusy){};
		if ((I2CMCS & i2cError) == i2cError)
		{ I2CMCS = masterStop;
			putChar ('e');  
			putChar ('r');  
			putChar ('r');  
			putChar ('o'); 
			putChar ('r');  
			putChar ('\n');
		}
		I2CMDR = 0x0;
		I2CMSA |= 0x1;
		I2CMCS = (masterStart | masterStop | masterRun);
	
	while ((I2CMCS & i2cBusy) == i2cBusy){};
		if ((I2CMCS & i2cError) == i2cError)
		{ I2CMCS = masterStop;
			putChar ('e');  
			putChar ('r');  
			putChar ('r');  
			putChar ('o');  
			putChar ('r');  
			putChar ('\n');
		}
	retry ++;
	data = (I2CMDR & 0xFF);
	} while (((I2CMCS&( i2cAdrack | i2cError )) != 0x0) && (retry < 10));
	

	return data;
}

 short mpu6050ReceiveTwoByte(unsigned int regAddress){
	
	
	short data;
	int retry;
	do{
	I2CMSA = mpu6050Tx;
	I2CMDR = regAddress;
	I2CMCS = (masterStart | masterRun);
		
		while ((I2CMCS & i2cBusy) == i2cBusy){};
		if ((I2CMCS & i2cError) == i2cError)
		{ I2CMCS = masterStop;
			putChar ('e');  
			putChar ('r');  
			putChar ('r');  
			putChar ('o');  
			putChar ('r');  
			putChar ('\n');
		}
		I2CMDR = 0x0;
		I2CMSA |= 0x1;
		I2CMCS = (masterStart| masterAck | masterRun );
		
	while ((I2CMCS & i2cBusy) == i2cBusy){};
		if ((I2CMCS & i2cError) == i2cError)
		{ I2CMCS = masterStop;
			putChar ('e');  
			putChar ('r');  
			putChar ('r');  
			putChar ('o');  
			putChar ('r');  
			putChar ('\n');
		}
	
	data = (I2CMDR & 0xFF)<<8;
		
		I2CMDR = 0x0;
		I2CMSA |= 0x1;
		I2CMCS = ( masterStop | masterRun);
		
	while ((I2CMCS & i2cBusy) == i2cBusy){};
		if ((I2CMCS & i2cError) == i2cError)
		{ I2CMCS = masterStop;
			putChar ('e');  
			putChar ('r');  
			putChar ('r');  
			putChar ('o');  
			putChar ('r');  
			putChar ('\n');
		}
		
	data = data | (I2CMDR & 0xFF);
		
	}while (((I2CMCS&( i2cAdrack | i2cError )) != 0x0) && (retry < 10));
	
	return	data;
}
 



void getRawSensorData(void){
	
	accelDataX = mpu6050ReceiveTwoByte(accelHighX);//- accelCalX;
	accelDataY = mpu6050ReceiveTwoByte(accelHighY);//- accelCalY;
	accelDataZ = mpu6050ReceiveTwoByte(accelHighZ);//- accelCalZ;
	gyroDataX = mpu6050ReceiveTwoByte(gyroHighX) - gyroCalX;
	gyroDataY = mpu6050ReceiveTwoByte(gyroHighY)- gyroCalY;
	gyroDataZ = mpu6050ReceiveTwoByte(gyroHighZ) - gyroCalZ;
}


void printSensorData(void){
	
	putChar(' ');
	putChar('G');
	putChar('y');
	putChar('r');
	putChar('o');
	putChar('\t');
	// gyro Data
	putFloat(gyroRollAngle,2);
	putChar('\t');
	putFloat(gyroPitchAngle,2);
	putChar('\t');
	putFloat(gyroYawAngle,2);
	putChar('\t');
	putChar(' ');
	putChar('A');
	putChar('c');
	putChar('c');
	putChar('e');
	putChar('l');
	putChar('\t');
	// Accell Data
	putFloat(accelRoll,2);
	putChar('\t');
	putFloat(accelPitch,2);
	putChar('\t');
	putFloat(accelYaw,2);
	
	
		putChar('\t');
	putChar(' ');
	putChar('C');
	putChar('o');
	putChar('m');
	putChar('p');
	putChar('\t');
	// Filtered
	putFloat(roll,2);
	putChar('\t');
	putFloat(pitch,2);
	//putChar('\t');
	//putFloat(accelYaw,2);
	putChar(carriageR);
	
	

}

float convertGyroValue(short data){
	
	float conversion;
	

		if(data < 0)
		{
		 data *= -1;
		 conversion = (float)data /(float)(65.5*300);
		 conversion *= -1;
	  }
		else
		{
			conversion = (float)data/(float)(65.5*300);			       // calculate gain of raneg setting and period samping rate
		}

	return	conversion;
}
	
float convertGyroYawValue(short data){
	
	float conversion;
	

		if(data < 0)
		{
		 data *= -1;
		 conversion = (float)data /(float)(65.5*300);
		 conversion *= -1;
	  }
		else
		{
			conversion = (float)data/(float)(65.5*300);			       // calculate gain of raneg setting and period samping rate
		}

	return	conversion;
}


float convertAccelValue(short data){
	
	float conversion;
		if(data < 0)
		{
		 data *= -1;
		 conversion = (float)data;
		 conversion *= -1;
	  }
		else
		{
			conversion = (float)data;	
		}
	
	return conversion;
	}

	
void convertAcceltoAngles(float *roll, float *pitch, float *yaw){
	
	float vectorTotal1;
	float vectorTotal2;
	float rollcopy = *roll;
	float pitchcopy = *pitch;
	float yawcopy = *yaw;
	
	vectorTotal1 = (float)pow(pitchcopy,2)+(float)pow(yawcopy,2);
	vectorTotal1 = (float)sqrt(vectorTotal1);
	
	vectorTotal2 = (float)pow(rollcopy,2)+(float)pow(yawcopy,2);
	vectorTotal2 = (float)sqrt(vectorTotal2);
	

	*roll = atan(rollcopy/vectorTotal1)*(180.0/PI)*-1 +3;                      // convert x data to angle in degrees
	*pitch = atan(pitchcopy/vectorTotal2)*(180.0/PI)+1.5;                    // onvert y data to angle in degrees
	
 }
void convertGyrotoAngles(float *roll,float *pitch,float *yaw,float *rollangle, float *pitchangle, float *yawangle){

	  float yawcopy = *yaw;
		*rollangle += *roll;
		*pitchangle += *pitch;
		     	                                          // integrate values to obtain angle from power on value
	
		*rollangle -= *pitchangle * sin(yawcopy*(PI/180.0));              /* transfer yaw to roll and pitch */
		*pitchangle += *rollangle * sin(yawcopy*(PI/180.0));
	
	/*	if (*paccelPitch >= -0.5 && *paccelPitch  <= 0.5)
		{
			*pitchangle = *ppitch;
		}
		if (*paccelRoll >= -0.5 && *paccelRoll<= 0.5)
		{
			*rollangle = *proll;
		}
	*/
}

void compFilter(float *gyroRoll,float *gyroPitch,float* accelRoll, float *accelPitch){
	
	
	*proll = ((float)0.95*(*gyroRoll))+((float)0.05*(*accelRoll));
	*ppitch = ((float)0.95*(*gyroPitch))+((float)0.05*(*accelPitch));
	
}
	
void getGyroCal(){
	
	int i = 0;
	while (i < 2000){
		
	gyroCalX += mpu6050ReceiveTwoByte(gyroHighX);

	gyroCalY += mpu6050ReceiveTwoByte(gyroHighY);

	gyroCalZ += mpu6050ReceiveTwoByte(gyroHighZ);
	/*	
	accelCalX += mpu6050ReceiveTwoByte(accelHighX);
		
	accelCalY += mpu6050ReceiveTwoByte(accelHighY);
		
	accelCalZ += mpu6050ReceiveTwoByte(accelHighZ);
		*/
	i++;
	
}
	gyroCalX /= 2000;
	gyroCalY /= 2000;
	gyroCalZ /= 2000;
/*
	accelCalX /= 5000;
	accelCalY /= 5000;
	accelCalZ /= 5000;
*/

	putChar(' ');
	putChar('G');
	putChar('y');
	putChar('r');
	putChar('o');
	putChar(' ');
	putChar('o');
	putChar('k');
	putChar((char)(0x0D));
	putChar((char)(0x0A));
	
}

void handlePeriodicData(void){
	
	getRawSensorData();
	gyroPitch = convertGyroValue(gyroDataX);
	gyroRoll = convertGyroValue(gyroDataY);
	gyroYaw = convertGyroYawValue(gyroDataZ);
	accelRoll = convertAccelValue(accelDataX);
	accelPitch = convertAccelValue(accelDataY);		
	accelYaw = convertAccelValue(accelDataZ);
	convertAcceltoAngles(paccelRoll,paccelPitch,paccelYaw);
	convertGyrotoAngles(pgyroRoll,pgyroPitch,pgyroYaw,pgyroRollAngle,pgyroPitchAngle,pgyroYawAngle);
	compFilter(pgyroRollAngle,pgyroPitchAngle,paccelRoll,paccelPitch);
	
	newDataFlag = 1;
	
}


void getController(void){
	
		int i = 0;
		int j =1;
	
	if (( uartRxFrame[0] = (UART1_DR & 0xFF)) == 126)
	{
	while ( j < 20)
	{
		while ((UART1_FR & 0x10) == 0X10){};
	
		uartRxFrame[j] = readUartByte() & 0xFF;
		
		j++;
		}
	}
	else UART1_ICR |= 0x4;      /* clear interrupt flag */
		
	}

unsigned short readUartByte(void){
	
	unsigned short variable;
	int temp;
	temp = UART1_DR;
	
	if ( temp == delim )
	{
		variable = (UART1_DR^0x20);
	}
	
	else if ( temp == escape )
	{
		variable = (UART1_DR^0x20);
	}
	
	else variable = temp;
	
	return variable;
}

void printControllerValue(void){
	
	int i,j;
		for (i=0;i<20;i++)
	{
		putNum(uartRxFrame[i],2);
		putChar(' ');
	}
	putChar((char)(0x0A));
	putChar((char)(0x0D));
	
}
	
void assignControls(void){
	
	*pRawButtonControl = (uartRxFrame[11]<<8) | (uartRxFrame[12]);
	*pRawThrottleControl = (uartRxFrame[13]<<8) | (uartRxFrame[14]);
	*pRawRollControl = (uartRxFrame[15]<<8 | uartRxFrame[16]);
	*pRawPitchControl = (uartRxFrame[17]<<8 | uartRxFrame[18]);
	
}

void printRawControls (void){
	
	putNum(rawThrottleControl,1);
	putChar('\t');
	putNum(rawRollControl,1);
	putChar('\t');
	putNum(rawPitchControl,1);
	putChar('\t');
	putNum(rawButtonControl,1);
	
	putChar((char)(0x0A));
	putChar((char)(0x0D));
	
}

void convertControls (void){

	/* convert raw throttle value into an applied timer match cvalue */
	
	*pthrottleC = (*pRawThrottleControl * dutyCycleGain);
	
	 /* Convert raw roll value into a +- angle < 45 degrees */
	
	if (*pfloatRoll == 512.0) *prollC = (float)0.0;
	else if ( *pfloatRoll > 512 )
	{
		*pfloatRoll -= 512.0;
		*prollC = (float)(*pfloatRoll * angle20Gain);
	}
	else if ( *pfloatRoll < 512 )
	{
		*pfloatRoll= 512 - *pfloatRoll;
		*prollC = (float)(*pfloatRoll * angle20Gain * -1.0);
	}                                                              
	
	  /* Convert raw pitch value into a +- angle < 45 degrees */
	
		if (*pRawPitchControl == 512) *ppitchC = (float)0.0;
	else if ( *pRawPitchControl > 512 )
	{
		*pRawPitchControl -= 512;
		*ppitchC = (float)*pRawPitchControl * angle20Gain * -1.0;
	}
	else if ( *pRawPitchControl < 512 )
	{
		
		*pRawPitchControl = 512 - *pRawPitchControl;
		*ppitchC = (float)*pRawPitchControl * angle20Gain;
	}               
		/* Assign Boolean values to Buttons */
	if ((*pRawButtonControl & 0x8) != 0x8){L1 = 1;}
	else { L1 = 0;}
	if ((*pRawButtonControl & 0x40) != 0x40){R1 = 1;}
	else { R1 = 0;}
	if ((*pRawButtonControl & 0x80) != 0x80){X = 1;}
	else { X = 0;}
	if ((*pRawButtonControl & 0x10) != 0x10){TRI = 1;}
	else { TRI = 0;}
}


void printControls(void){
	
	putNum(throttleC,1);
	putChar('\t');
	putFloat(rollC,1);
	putChar('\t');
	putFloat(pitchC,1);
	putChar('\t');
	
	if (L1 == 1){
		putChar('L');
		putChar('1');
	}
	putChar('\t');
	if (R1 == 1){
		putChar('R');
		putChar('1');
	}
	
		
	
	
	putChar((char)(0x0A));
	putChar((char)(0x0D));
	
}


float convertToFloat(int data){
	
	float conversion;
		if(data < 0)
		{
		 data *= -1;
		 conversion = (float)data;
		 conversion *= -1;
	  }
		else
		{
			conversion = (float)data;	
		}
	
	return conversion;
	}

	
void applyControls(void){
	
	
	if (R1 == 1 && L1 != 1){yawrate = rate1*-1;}
	else {yawrate = 0;}
	
	if (L1 == 1 && R1 != 1){yawrate = rate1;}
	else {yawrate = 0;}

	
	/* Determine current error */
	rollerror = rollC - roll;
	pitcherror = pitchC - pitch;
	yawerror = gyroYaw;
	

	
	
	
	//if (rollerror > -0.5 && rollerror < 0.5){rollerror = 0;}                   /* delta error before adjustment of 1 degrees */
	//if (pitcherror > -0.5 && pitcherror < 0.5){pitcherror = 0;}
	
                   
	
	
	if (rawThrottleControl > 200)                                           /* restrain integral term to in flight */
	{
	
		rollAccum += rollerror;                                              /* accumulative integral of error */
	pitchAccum += pitcherror;
	
	if (rollAccum < -2000)
		rollAccum = -2000;                                /* restrain integral windup within bounds */
	if (rollAccum > 2000)
		rollAccum = 2000;
	if (pitchAccum < -2000)
		pitchAccum = -2000;
	if (pitchAccum > 2000)
		pitchAccum = 2000;
	
	}
	else 
	{
	 rollAccum = 0;
	 pitchAccum = 0;
	}
	                 
	
	/* Apply PID Control for Pitch */       
	pitchP = (pitcherror*Kp);
	pitchI = (pitchAccum*Ki);
	pitchD =(Kd*(PitchBefore-pitch));
		
	//pitchcorr = ( pitchP + pitchI + pitchD );
	
	
	/* Apply PID Control for Pitch */
	rollP = (rollerror*Kp);
	rollI = (rollAccum*Ki);
	rollD = (Kd*(RollBefore-roll));
		
	rollcorr = ( rollP + rollI + rollD );
	
	
	
	/* Apply PID for Yaw rate */
	//yawcorr = dutyCycleGain*(yawerror*YKp);
	
	/* Apply rate of Yaw */
	yawcorr=0;
	
	/* Save values for later use */
	PitchBefore = pitch;
	RollBefore = roll;
	
	
	/****************************************************/
	
	TIMER0_TAMATCH = OFF + throttleC + rollcorr + pitchcorr + yawrate + yawcorr;     /* foward left motor */
	TIMER0_TBMATCH = OFF + throttleC + rollcorr - pitchcorr - yawrate - yawcorr;     /* rear left motor */
	TIMER1_TAMATCH = OFF + throttleC - rollcorr - pitchcorr + yawrate + yawcorr;    /* rear right motor */
	//TIMER1_TBMATCH = OFF + throttleC - rollcorr + pitchcorr - yawrate - yawcorr;   /* front right motor */
	
	if (TIMER0_TAMATCH > FULL){TIMER0_TAMATCH = FULL;}
	if (TIMER0_TAMATCH > FULL){TIMER0_TAMATCH = FULL;}
	if (TIMER1_TAMATCH > FULL){TIMER1_TAMATCH = FULL;}
	if (TIMER1_TBMATCH > FULL){TIMER1_TBMATCH = FULL;}

}

void printAdjust(void){
	/*
	float motor1, motor2, motor3, motor4;

	motor1 = ((TIMER0_TAMATCH - OFF)/(float)OFF)*100;
	motor2 = ((TIMER0_TBMATCH - OFF)/(float)OFF)*100;
	motor3 = ((TIMER1_TAMATCH - OFF)/(float)OFF)*100;
	motor4 = ((TIMER1_TBMATCH - OFF)/(float)OFF)*100;
	
	if (motor1 < 0 ) motor1 = 0;
	if (motor2 < 0 ) motor2 = 0;
	if (motor3 < 0 ) motor3 = 0;
	if (motor4 < 0 ) motor4 = 0;
	/*
	putNum(motor1,4);
	putChar('\t');
	putFloat(motor2,4);
	putChar('\t');
	putFloat(motor3,4);
	putChar('\t');
	putFloat(motor4,4);
	putChar('\t');
	putFloat(rollAccum,6);
	putChar('\t');
	putFloat(pitchAccum,6);
	putChar('\t');
	putFloat(roll,2);
	putChar('\t');
	putFloat(pitch,2);
	putChar('\t');
	//putFloat(,2);
	putChar((char)(0x0D));
	*/
	//(int)(pitcherror);
	
	pitchP = pitchP/(float)PIDgain;
	pitchI = pitchI/(float)PIDgain;
	pitchD = pitchD/(float)PIDgain;
	pitchcorr = pitchcorr/(float)PIDgain;
	
	//putNum(motor1,2);
	putFloat(rollerror,2);
	putChar(',');
	putFloat(rollP,2);
	putChar(',');
	putFloat(rollI,2);
  putChar(',');
	putFloat(rollD,6);
	putChar(',');
	putFloat(rollcorr,6);
	putChar('\n');
	putChar((char)(0xD));
	
	
	
}



void printS(char *string){
	
	int i = 0;
	
	while( string[i] != '\0')
	{
		putChar(string[i]);
		i++;
	}
	
	putChar('\n');
	putChar((char)(0x0D));
	
}





int serialRead(void){
	
	int value;
	
	while(( UART0_FR & 0x10) == 0)
	{
		value = value<<8;
		value = (UART0_DR & 0xFF);
		
	
	}
	
	return value;
}



void incValue(void){
	
	if (serialRead() == 0x77){TIMER0_TAMATCH = TIMER0_TBMATCH = TIMER1_TAMATCH = TIMER1_TBMATCH = FULL;}       // w add to Kp
	
	if (serialRead() == 0x73){TIMER0_TAMATCH = TIMER0_TBMATCH = TIMER1_TAMATCH = TIMER1_TBMATCH = OFF; }       // s sub from Kp
	
	//if (serialRead() == 0x65){Ki += 0.001;}       // e add to Ki
	
	//if (serialRead() == 0x64){Ki -= 0.001;}       // d sub from Ki
	
	//if (serialRead() == 0x72){Kd += 0.001;}       // r add to Kd
	
	//if (serialRead() == 0x66){Kd -= 0.001;}       // f sub from kd
}