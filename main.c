#include "registers.h"
#include "macros.c"

unsigned int iAm;

extern void Delay(unsigned int);
extern void setupPortF(void);
extern void setupTimers(void);
extern void setupSystickTimer(void);
extern void setupI2c(void);
extern void out2Dec(int, int);
extern void putChar(char);
extern void putNum(int,int);
extern void setupUart0(void);
extern void setupMpu6050(void);
extern void testUART0(void);
extern void setupUart1 (void);
extern void putNum1(int);
extern void mpu6050TransmitSingleByte(unsigned int, unsigned int);
extern short mpu6050ReceiveSingleByte(unsigned int);
extern float mpu6050ReceiveTwoByte(unsigned int);
extern void getRawSensorData(void);
extern void printSensorData(void);
extern float convertGyroValue(short);
extern float convertAccelValue(short);
extern void convertAcceltoAngles(float *, float *,float *);
extern void convertGyrotoAngles(float *,float *,float *,float *,float *,float *);
void getGyroCal(void);
extern void getController(void);
extern void printControllerValue(void);
extern void assignControls(void);
extern void printRawControls (void);
extern void convertControls (void);
extern void printControls(void);
extern float convertToFloat(int data);
extern void applyControls(void);
extern void printAdjust(void);
extern void calibrateESC(void);
extern void printS(char *string);
extern int serialRead(void);
extern void putFloat(float,int);
extern void incValue(void);




short accelDataX;
short accelDataY;
short accelDataZ;
short gyroDataX;
short gyroDataY;
short gyroDataZ;	
unsigned short rawThrottleControl, rawRollControl, rawPitchControl, rawButtonControl, *pRawThrottleControl, *pRawRollControl, *pRawPitchControl;
unsigned short  *pRawButtonControl;
float rollC, pitchC, *prollC, *ppitchC, floatRoll, floatPitch, *pfloatRoll, *pfloatPitch;
int throttleC,*pthrottleC;
float gyroRoll, *pgyroRoll;
float gyroPitch, *pgyroPitch;
float gyroYaw, *pgyroYaw;
float accelRoll, *paccelRoll;
float accelPitch, *paccelPitch;
float accelYaw, *paccelYaw;
float gyroCalX, gyroCalY, gyroCalZ, accelCalX, accelCalY, accelCalZ;
float gyroRollAngle, *pgyroRollAngle;
float gyroPitchAngle, *pgyroPitchAngle;
float gyroYawAngle, *pgyroYawAngle;
float roll, pitch, *proll, *ppitch;
int uartRxFrame[25];
short R1, L1, X , TRI;
float rollcorr, pitchcorr, rollerror, pitcherror, yawrate, yawerror, yawcorr, rollAccum, pitchAccum;
int timeBefore, dt;
float PitchBefore;
float RollBefore;
int newDataFlag;

float pitchP, pitchI, pitchD, rollP, rollI, rollD;

int test;

//float Kp = 0.5;
//float Ki, Kd;



int main(void){

/************************************************ Initalize Variables **************************************/
                                                                                                           //
  paccelRoll = &accelRoll;                                                                                 //
	paccelPitch = &accelPitch;                                                                               //
	paccelYaw = &accelYaw;                                                                                   //
	pgyroRoll = &gyroRoll;                                                                                   //
	pgyroPitch =&gyroPitch;                                                                                  //
	pgyroYaw = &gyroYaw;                                                                                     //
	pgyroRollAngle = &gyroRollAngle;                                                                         //
	pgyroPitchAngle = &gyroPitchAngle;                                                                       //
	pgyroYawAngle = &gyroYawAngle;                                                                           //
	proll = &roll;                                                                                           //
	ppitch = &pitch;                                                                                         //
	pRawButtonControl = &rawButtonControl;                                                                   //
	pRawThrottleControl = &rawThrottleControl;                                                               //
	pRawRollControl = &rawRollControl;                                                                       //
	pRawPitchControl = &rawPitchControl;                                                                     //
	pthrottleC = &throttleC;                                                                                 //
	prollC = &rollC;                                                                                         //
	ppitchC = &pitchC;                                                                                       //
	pfloatRoll = &floatRoll;                                                                                 //
	pfloatPitch = &floatPitch;                                                                               //
	                                                                                                         //
/***********************************************************************************************************/
	
	
	
	
	
	
/********************************* ESC calibration *********************************************************/
	                                                                                                         //
	//calibrateESC();                                                                                        //
	                                                                                                         //
/******************************** Uncomment for Calibration ************************************************/
	
	
	
	
	
/********************************************** Set up Sub Systems *****************************************/
                                                                                                           //
  setupUart1();     	                                                        //
	setupSystickTimer();                                                                                         	//
	setupTimers();                                                                                           //
	setupI2c();                                                                                              //
	setupUart0();                                                                                            //                                                                                          //
	setupMpu6050();                                                                                          //
	                                                                                                         //
/***********************************************************************************************************/





	
/*********************************************** Calibrate sensors *****************************************/
	getGyroCal();                                                                                            //                                                                                                       //
	getRawSensorData();	                                                                                     //                                                                                                   //
	*pgyroPitchAngle = 0;                                                                                    //
	*pgyroRollAngle = 0;                                                                                     //
	*pgyroYawAngle = 0;                                                                                      //                                                                                                       //
	gyroPitch = convertGyroValue(gyroDataX);                                                                 //
	gyroRoll = convertGyroValue(gyroDataY);                                                                  // 
	gyroYaw = convertGyroValue(gyroDataZ);                                                                   //
	accelRoll = convertAccelValue(accelDataX);                                                               //
	accelPitch = convertAccelValue(accelDataY);		                                                           //
	accelYaw = convertAccelValue(accelDataZ);                                                                //
	convertAcceltoAngles(paccelRoll,paccelPitch,paccelYaw);                                                  //
	convertGyrotoAngles(pgyroRoll,pgyroPitch,pgyroYaw,pgyroRollAngle,pgyroPitchAngle,pgyroYawAngle); 
  accelRoll += 3;                                                                                       	//
	gyroPitchAngle += accelPitch;                                                                            //
	gyroRollAngle += accelRoll; 
                                                                                                           // 
	gyroYawAngle = 0;                                                                                        //
	                                                                                                         //
/***********************************************************************************************************/
	
	
/*
	char readSlider[] = "!Read(Slider)";
	char nameSlider[] = "!POBJ txtSlider=Proportional";
	char setSlider[] = "!POBJ Slider.Max=1000";
	
	printS(nameSlider);
	printS(setSlider);
	*/
	
/******************************************** Main Flight Loop *********************************************/
	                                                                                                         //
	NVIC_ST_CTRL |= 0x1;	                        // start systick timer                                     //
	                                                                                                         //
		while (1){                                                                                             //
	//printS(readSlider);                                                                                                      //
  //printSensorData();                                                                                       //
	                                                                                                         //
	assignControls();                                                                                        //
	                                                                                                         //
	floatRoll = convertToFloat(rawRollControl);                                                              //
	floatPitch = convertToFloat(rawPitchControl);                                                            //
	                                                                                                         //
	//printRawControls();                                                                                    //
	//printControls();	                                                                                     //
	convertControls();                                                                                       //
	//printAdjust();                                                                                         //
    

	if (rawThrottleControl > 200) 
	{
	if ( newDataFlag == 1 )                                                                                  //
	 {                                                                                                       //
	applyControls();                                                                                         //
	newDataFlag = 0;		                                                                                     //
	 }
 }
	else{
		TIMER0_TAMATCH = OFF;
		TIMER0_TBMATCH = OFF;
		TIMER1_TAMATCH = OFF;
		TIMER1_TBMATCH = OFF;
	}
	
	//printS(readSlider);


	 
  
	//putChar('\t');
	//putFloat(Ki,3);
	// putChar('\t');
//	putFloat(Kd,3);
  putChar((char)(0x0A));
	putChar((char)(0x0D));
	                                                                                                        //
 }                                                                                                         //
/***********************************************************************************************************/

}
