// PWM outputs
#define M1 0x1  
#define M2 0x2
#define M3 0x4
#define M4 0x8

//Uart
#define newline 0x0A
#define carriageR 0x0D
// Timers
#define FREQ350 0xB291 // 350Hz
#define OFF 15984      //  1ms 
#define HALF 23976     // 1.5ms
#define FULL 31968     // 2ms
#define twohundredms 319999   // 200ms
#define fourhundredms 640000  // 400ms
// I2C
#define mpu6050Add 0x68    // the address of the mpu6050
#define mpu6050Tx 0xD0
#define mpu6050Rx 0xD1
#define i2cBusy 0x1
#define i2cError 0x2
#define busBusy 0x40
#define i2cAdrack 0x4


// control parameters
#define masterRun 0x1
#define masterStart 0x2
#define masterStop 0x4
#define masterAck 0x8


//Delay times
#define wait 0x4
#define twohunded 0x11557
#define threehundred 0xD054




// mpu6050
#define whoAmI 0x75

#define accelHighX 0x3B
#define accelLowX 0x3C
#define accelHighY 0x3D
#define accelLowY 0x3E
#define accelHighZ 0x3F
#define accelLowZ 0x40

#define gyroHighX 0x43
#define gyroLowX 0x44
#define gyroHighY 0x45
#define gyroLowY 0x46
#define gyroHighZ 0x47
#define gyroLowZ 0x48


#define SMPRT_DIV 0x19
#define CONFIG 0X1A
#define GYRO_CONFG 0X1B
#define ACCEL_CONFIG 0X1C
#define MOT_THR 0X1F
#define FIFO_EN 0X23
#define PWR_MGMT_1 0X6B
#define PWR_MGMT_2 0X6C
#define MOT_DETECT_CTRL 0X69

#define PI 3.141
#define dutyCycleGain 15.6094
#define PIDgain 15.6094
#define angle45Gain 0.08789
#define angle30Gain 0.05860
#define angle20Gain 0.03906
#define Kp 150
#define Ki 2.5
#define Kd 140
#define YKp 1.0
#define rate1 1000.0

#define FRcal 0
#define FLcal 0
#define RRcal 0
#define RLcal 0

// uart

#define delim 0x7E
#define escape 0x7D
