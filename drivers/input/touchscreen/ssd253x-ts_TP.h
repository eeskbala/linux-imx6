#define DRIVENO	22
#define SENSENO	36
#define ENABLE_INT		2	// 0->Polling, 1->Interupt, 2->Hybrid
#define EdgeDisable		0	// if Edge Disable, set it to 1, else reset to 0
#define RunningAverageMode	2	//{0,8},{5,3},{6,2},{7,1}
#define RunningAverageDist	4	// Threshold Between two consecutive points
#define MicroTimeTInterupt	10000000// 100Hz - 10,000,000us
#define FINGERNO		10

#define IRQMask			((~((1<<FINGERNO)-1)<<4)|(0xE))
#define IRQMaskH		(IRQMask/256)&0xFF
#define IRQMaskL		(IRQMask%256)&0xFF
//#define SoftSWAPX
//#define SoftSWAPY
#define SCALE_FACTOR(target, pins) ((target * 0xFFF0)/((pins - 1) * 64))
#define X_SCALE SCALE_FACTOR(1280, SENSENO)
#define Y_SCALE SCALE_FACTOR(800, DRIVENO)

// SSD2533 Setting
#define Skw_Rate	4
 
struct ChipSetting ssd253xcfgTable[]={
{ 1, 0x06, DRIVENO-1, 0x00},		//Set drive line 0=1, 1F=
{ 1, 0x07, SENSENO, 0x00},		//Set sense line 0=1, 3F=
{ 2, 0x08, 0x00, 0x20 * Skw_Rate +  0},	//Set 1 drive line reg
{ 2, 0x09, 0x00, 0x20 * Skw_Rate +  1},	//Set 2 drive line reg
{ 2, 0x0A, 0x00, 0x20 * Skw_Rate +  2},	//Set 3 drive line reg
{ 2, 0x0B, 0x00, 0x20 * Skw_Rate +  3},	//Set 4 drive line reg
{ 2, 0x0C, 0x00, 0x20 * Skw_Rate +  4},	//Set 5 drive line reg
{ 2, 0x0D, 0x00, 0x20 * Skw_Rate +  5},	//Set 6 drive line reg
{ 2, 0x0E, 0x00, 0x20 * Skw_Rate +  6},	//Set 7 drive line reg
{ 2, 0x0F, 0x00, 0x20 * Skw_Rate +  7},	//Set 8 drive line reg
{ 2, 0x10, 0x00, 0x20 * Skw_Rate +  8},	//Set 9 drive line reg
{ 2, 0x11, 0x00, 0x20 * Skw_Rate +  9},	//Set 10 drive line reg
{ 2, 0x12, 0x00, 0x20 * Skw_Rate + 10},	//Set 11 drive line reg
{ 2, 0x13, 0x00, 0x20 * Skw_Rate + 11},	//Set 12 drive line reg
{ 2, 0x14, 0x00, 0x20 * Skw_Rate + 12},	//Set 13 drive line reg
{ 2, 0x15, 0x00, 0x20 * Skw_Rate + 13},	//Set 14 drive line reg
{ 2, 0x16, 0x00, 0x20 * Skw_Rate + 14},	//Set 15 drive line reg
{ 2, 0x17, 0x00, 0x20 * Skw_Rate + 15},	//Set 16 drive line reg
{ 2, 0x18, 0x00, 0x20 * Skw_Rate + 16},	//Set 17 drive line reg
{ 2, 0x19, 0x00, 0x20 * Skw_Rate + 17},	//Set 18 drive line reg
{ 2, 0x1A, 0x00, 0x20 * Skw_Rate + 18},	//Set 19 drive line reg
{ 2, 0x1B, 0x00, 0x20 * Skw_Rate + 19},	//Set 20 drive line reg
{ 2, 0x1C, 0x00, 0x20 * Skw_Rate + 20},	//Set 21 drive line reg
{ 2, 0x1D, 0x00, 0x20 * Skw_Rate + 21},	//Set 22 drive line reg
{ 1, 0xD5, 0x06, 0x00},			//Set Driving voltage 0(5.5V) to 7(9V)
{ 1, 0x2A, 0x07, 0x00},			//Set sub-frame default=1, range 0 to F
{ 1, 0x2C, 0x01, 0x00},			//Set Median Filter
{ 1, 0x2E, 0x0B, 0x00},			//Sub-frame Drive pulse number default=0x17
{ 1, 0x2F, 0x01, 0x00},			//Set integration gain
{ 1, 0x30, 0x03, 0x00},			//start integrate 125ns/div
{ 1, 0x31, 0x09, 0x00},			//stop integrate 125n/div
{ 1, 0xD7, 0x06, 0x00},			//ADC range default=4, 0 to 7
{ 1, 0xD8, 0x09, 0x00},			//Select Sense line biasing resistance
{ 1, 0xDB, 0x03, 0x00},			//Set integration cap default=0, 0 to 7
{ 2, 0x33, 0x00, 0x02},			//Set Min. Finger area
{ 2, 0x34, 0x00, 0x50},			//Set Min. Finger level
{ 2, 0x35, 0x00, 0xFF},			//Set Min. Finger weight
{ 2, 0x36, 0x00, 0x20},			//Set Max. Finger weight
{ 1, 0x37, 0x01, 0x00},			//Segmentation Depth
//{ 1, 0x39, 0x01, 0x00},			//CG_METHOD_REG set to Hybrid mode
{ 1, 0x3D, 0x01, 0x00},			//FILTER_SEL_REG
//{ 1, 0x3A, 0x00, 0x00},			//Hybrid mode select
{ 1, 0x40, 0xA0, 0x00},			//UNKNOWN!!!
{ 1, 0x44, 0x01, 0x00},			//UNKNOWN!!!
{ 1, 0x53, 0x10, 0x00},			//Event move tolerance
{ 2, 0x54, 0x00, 0x80},			//X tracking
{ 2, 0x55, 0x00, 0x80},			//Y tracking
{ 1, 0x56, 0x02, 0x00},			//Moving Average Filter 0:null, 1:5-3(Set), 2:6-2  3:7-1
{ 1, 0x58, 0x00, 0x00},			//Finger weight scaling
{ 1, 0x59, 0x01, 0x00},			//Enable Random walk
{ 1, 0x5A, 0x00, 0x00},			//Disable Missing frame
{ 1, 0x5B, 0x03, 0x00},			//Set Random walk window
{ 1, 0x65, 0x04, 0x00},			//XY Mapping
{ 2, 0x66, 0x92, 0x40},			//X_SCALING_REG
{ 2, 0x67, 0x98, 0x60},			//Y_SCALING_REG
{ 1, 0xAE, 0x00, 0x00},
{ 2, 0x7A, 0xFF, 0xFF},			//Event Mask - Enable Leave Event
{ 2, 0x7B, IRQMaskH, IRQMaskL},	//IRQ Mask - Mask off Unused Fingers interrupt except FIFO
{ 1, 0x8A, FINGERNO, 0x00},		//Max finger
//{ 1, 0x8B, 0x01, 0x00},			//Edge Remap
//{ 1, 0x8C, 0x90, 0x00},			//Edge Suppress
//{ 1, 0x2C, 0x00, 0x00},			//Median Filter 0:disable to 1:enable
//{ 1, 0x3D, 0x01, 0x00},			//2d filter
{ 1, 0x25, 0x10, 0x00},			//Set scan mode A
};

// For SSD2533 Bug Version Only //
//#define	SSD2533FIXEDCODE
struct ChipSetting ssd253xcfgTable1[]={
{ 1, 0xA4, 0x00, 0x00},			//MCU prescaler default=01
{ 1, 0xD4, 0x09, 0x00},			//Dummy Code
{ 1, 0xD4, 0x09, 0x00},			//Set Osc frequency default=8, range 0 to F
};

struct ChipSetting Reset[]={
{ 1, 0x04, 0x00, 0x00},	// SSD2533
{ 1, 0x23, 0x00, 0x00},	// SSD2531
};

struct ChipSetting Resume[]={
{ 1, 0x04, 0x00, 0x00},	// SSD2533
{ 1, 0x23, 0x00, 0x00},	// SSD2531
{ 1, 0x25, 0x10, 0x00}, // Set Operation Mode
};

struct ChipSetting Suspend[] ={
{ 1, 0x25, 0x00, 0x00}, // Set Operation Mode
{ 1, 0x05, 0x00, 0x00},	// SSD2533
{ 1, 0x24, 0x00, 0x00},	// SSD2531
};
