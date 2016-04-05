#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <asm/io.h>
#include <linux/slab.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif /* CONFIG_HAS_EARLYSUSPEND */
#include <linux/i2c/ssd253x-ts.h>

#define CONFIG_TOUCHSCREEN_SSL_DEBUG	
#undef  CONFIG_TOUCHSCREEN_SSL_DEBUG

#define DEVICE_ID_REG                 2
#define VERSION_ID_REG                3
#define EVENT_STATUS                  121
#define EVENT_MSK_REG                 122
#define IRQ_MSK_REG                   123
#define FINGER01_REG                  124
#define FINGER02_REG                  125
#define FINGER03_REG                  126
#define FINGER04_REG                  127
#define EVENT_STACK                   128
#define EVENT_FIFO_SCLR               135

struct ChipSetting {
	char No;
	char Reg;
	char Data1;
	char Data2;
};

//#include "ssd253x-ts_TP-zonbest.h"
#include "ssd253x-ts_TP.h"

void deviceReset(struct i2c_client *client, int gpio_reset);
void deviceResume(struct i2c_client *client);
void deviceSuspend(struct i2c_client *client);
void SSD253xdeviceInit1(struct i2c_client *client);
void SSD253xdeviceInit(struct i2c_client *client); 

static int ssd253x_ts_open(struct input_dev *dev);
static void ssd253x_ts_close(struct input_dev *dev);
static int ssd253x_ts_suspend(struct i2c_client *client, pm_message_t mesg);
static int ssd253x_ts_resume(struct i2c_client *client);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void ssd253x_ts_early_suspend(struct early_suspend *h);
static void ssd253x_ts_late_resume(struct early_suspend *h);
#endif /* CONFIG_HAS_EARLYSUSPEND */

static enum hrtimer_restart ssd253x_ts_timer(struct hrtimer *timer);
static irqreturn_t ssd253x_ts_isr(int irq, void *dev_id);
static struct workqueue_struct *ssd253x_wq;

struct ssl_ts_priv {
	struct i2c_client *client;
	struct input_dev *input;
	struct hrtimer timer;
	struct work_struct  ssl_work;
#ifdef	CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif 
	int irq_enabled;
	int irq;
	int use_irq;
        int reset_gpio;
        int int_gpio;
	int FingerNo;
	int FingerX[FINGERNO];
	int FingerY[FINGERNO];
	int FingerP[FINGERNO];

	int Resolution;
	int EventStatus;
	int FingerDetect;

	int sFingerX[FINGERNO];
	int sFingerY[FINGERNO];
	int pFingerX[FINGERNO];
	int pFingerY[FINGERNO];
};

int ReadRegister(struct i2c_client *client,uint8_t reg,int ByteNo)
{
	unsigned char buf[4];
	struct i2c_msg msg[2];
	int ret;

	memset(buf, 0xFF, sizeof(buf));
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = ByteNo;
	msg[1].buf = buf;

	ret = i2c_transfer(client->adapter, msg, 2);

	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	if(ret<0)	printk("		ReadRegister: i2c_transfer Error !\n");
	else		printk("		ReadRegister: i2c_transfer OK !\n");
	#endif

	if(ByteNo==1) return (int)((unsigned int)buf[0]<<0);
	if(ByteNo==2) return (int)((unsigned int)buf[1]<<0)|((unsigned int)buf[0]<<8);
	if(ByteNo==3) return (int)((unsigned int)buf[2]<<0)|((unsigned int)buf[1]<<8)|((unsigned int)buf[0]<<16);
	if(ByteNo==4) return (int)((unsigned int)buf[3]<<0)|((unsigned int)buf[2]<<8)|((unsigned int)buf[1]<<16)|(buf[0]<<24);
	return 0;
}

void WriteRegister(struct i2c_client *client,uint8_t Reg,unsigned char Data1,unsigned char Data2,int ByteNo)
{	
	struct i2c_msg msg;
	unsigned char buf[4];
	int ret;

	buf[0]=Reg;
	buf[1]=Data1;
	buf[2]=Data2;
	buf[3]=0;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = ByteNo+1;
	msg.buf = (char *)buf;
	ret = i2c_transfer(client->adapter, &msg, 1);

	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	if(ret<0)	printk("		WriteRegister: i2c_master_send Error !\n");
	else		printk("		WriteRegister: i2c_master_send OK !\n");
	#endif
}

void SSD253xdeviceInit1(struct i2c_client *client) 
{	
#ifdef	SSD2533FIXEDCODE
	int i;
	mdelay(600); //SSD2533 ESD2 EEPROM VERSION
	for(i=0;i<sizeof(ssd253xcfgTable1)/sizeof(ssd253xcfgTable1[0]);i++)
	{
		WriteRegister(	client,ssd253xcfgTable1[i].Reg,
				ssd253xcfgTable1[i].Data1,ssd253xcfgTable1[i].Data2,
				ssd253xcfgTable1[i].No);
	}
#endif
}

void SSD253xdeviceInit(struct i2c_client *client)
{	
	int i;
	for(i=0;i<sizeof(ssd253xcfgTable)/sizeof(ssd253xcfgTable[0]);i++)
	{
		WriteRegister(	client,ssd253xcfgTable[i].Reg,
				ssd253xcfgTable[i].Data1,ssd253xcfgTable[i].Data2,
				ssd253xcfgTable[i].No);
	}
}

void deviceReset(struct i2c_client *client, int gpio_reset)
{	
	int i;
	i = gpio_request(gpio_reset, "Touch reset");
	if (i == 0) {
		gpio_direction_output(gpio_reset, 0);
		msleep(1);
		gpio_set_value(gpio_reset, 1);
	}


	for(i=0;i<sizeof(Reset)/sizeof(Reset[0]);i++)
	{
		WriteRegister(	client,Reset[i].Reg,
				Reset[i].Data1,Reset[i].Data2,
				Reset[i].No);
	}
	mdelay(100);
	SSD253xdeviceInit1(client);
}

void deviceResume(struct i2c_client *client)
{	
	int i;
	int timeout=10;
	int status;
	for(i=0;i<sizeof(Resume)/sizeof(Resume[0]);i++)
	{
		WriteRegister(	client,Resume[i].Reg,
				Resume[i].Data1,Resume[i].Data2,
				Resume[i].No);
	}
	do {
		status=ReadRegister(client,0x26,1);
//		printk("		deviceResume: status : %d !\n",status);
		if(status==Resume[2].Data1) break;
		mdelay(1);
	}while(timeout--); // Check the status
}

void deviceSuspend(struct i2c_client *client)
{	
	int i;
	int timeout=10;
	int status;
	WriteRegister(	client,Suspend[0].Reg,
			Suspend[0].Data1,Suspend[0].Data2,
			Suspend[0].No);
	do {
		status=ReadRegister(client,0x26,1);
		if(status==Suspend[0].Data1) break;
		mdelay(1);				
	}while(timeout--);
	for(i=1;i<sizeof(Suspend)/sizeof(Suspend[0]);i++)
	{
		WriteRegister(	client,Suspend[i].Reg,
				Suspend[i].Data1,Suspend[i].Data2,
				Suspend[i].No);
	}
}

#define Mode RunningAverageMode
#define Dist RunningAverageDist
void RunningAverage(unsigned short *xpos,unsigned short *ypos,int No,struct ssl_ts_priv *ssl_priv)
{	
	int FilterMode[4][2]={{0,8},{5,3},{6,2},{7,1}};
	int dx,dy;
	int X,Y;

	X=*xpos;
	Y=*ypos;
	if((ssl_priv->pFingerX[No]!=0x0FFF)&&(X!=0x0FFF))
	{
		dx=abs(ssl_priv->pFingerX[No]-X);
		dy=abs(ssl_priv->pFingerY[No]-Y);
		if(dx+dy<Dist*64)
		{
			ssl_priv->pFingerX[No]=(FilterMode[Mode][0]*ssl_priv->pFingerX[No]+FilterMode[Mode][1]*X)/8;
			ssl_priv->pFingerY[No]=(FilterMode[Mode][0]*ssl_priv->pFingerY[No]+FilterMode[Mode][1]*Y)/8;
		}
		else
		{
			ssl_priv->pFingerX[No]=X;
			ssl_priv->pFingerY[No]=Y;
		}
	}
	else
	{
		ssl_priv->pFingerX[No]=X;
		ssl_priv->pFingerY[No]=Y;
	}
	*xpos=ssl_priv->pFingerX[No];
	*ypos=ssl_priv->pFingerY[No];
}

void FingerCheckSwap(int *FingerX,int *FingerY,int *FingerP,int FingerNo,int *sFingerX,int *sFingerY)
{
  	int i,j;
  	int index1,index2;
  	int Vx,Vy;
  	int Ux,Uy;
  	int R1x,R1y;
  	int R2x,R2y;
	for(i=0;i<FingerNo;i++)
  	{
 		index1=i;
	    	if( FingerX[index1]!=0xFFF)
		if(sFingerX[index1]!=0xFFF) 
		{
			for(j=i+1;j<FingerNo+3;j++)
			{
				index2=j%FingerNo;
	    			if( FingerX[index2]!=0xFFF)
				if(sFingerX[index2]!=0xFFF) 
		    		{
					Ux=sFingerX[index1]-sFingerX[index2];
					Uy=sFingerY[index1]-sFingerY[index2];      
					Vx= FingerX[index1]- FingerX[index2];
					Vy= FingerY[index1]- FingerY[index2];					

					R1x=Ux-Vx;
					R1y=Uy-Vy;
					R2x=Ux+Vx;
					R2y=Uy+Vy;
							
					R1x=R1x*R1x;
					R1y=R1y*R1y; 
					R2x=R2x*R2x;
					R2y=R2y*R2y;

					if(R1x+R1y>R2x+R2y)
				    	{
				    		Ux=FingerX[index1];
						Uy=FingerY[index1];
						Vx=FingerP[index1];
							          
						FingerX[index1]=FingerX[index2];
						FingerY[index1]=FingerY[index2];
						FingerP[index1]=FingerP[index2];
							
						FingerX[index2]=Ux;
						FingerY[index2]=Uy;
						FingerP[index2]=Vx;
					}
					break;
			    	}
			}
		}
  	}        
  	for(i=0;i<FingerNo;i++)
  	{
    		sFingerX[i]=FingerX[i];
    		sFingerY[i]=FingerY[i];
  	}
}

static void ssd253x_ts_work(struct work_struct *work)
{
	int i;
	unsigned short xpos=0, ypos=0, width=0;
	int FingerInfo;
	int EventStatus;
	int FingerX[FINGERNO];
	int FingerY[FINGERNO];
	int FingerP[FINGERNO];
	int clrFlag=0;
	

	struct ssl_ts_priv *ssl_priv = container_of(work,struct ssl_ts_priv,ssl_work);
	
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_work!                  |\n");
	printk("+-----------------------------------------+\n");
	#endif

	EventStatus = ReadRegister(ssl_priv->client,EVENT_STATUS,2)>>4;
	ssl_priv->FingerDetect=0;
	for(i=0;i<ssl_priv->FingerNo;i++)
	{
		if((EventStatus>>i)&0x1)
		{
			FingerInfo=ReadRegister(ssl_priv->client,FINGER01_REG+i,4);
			xpos = ((FingerInfo>>0)&0xF00)|((FingerInfo>>16)&0xFF);
			ypos = ((FingerInfo>>4)&0xF00)|((FingerInfo>>24)&0xFF);
			width= ((FingerInfo>>4)&0x00F);	

			if(xpos!=0xFFF)
			{
				ssl_priv->FingerDetect++;
				xpos=xpos-1;// Please check bias
				ypos=ypos-1;// Please check bias

				#ifdef SoftSWAPX
				xpos = (DRIVENO-EdgeDisable)*(ssl_priv->Resolution)-xpos;
				#endif
				#ifdef SoftSWAPY
				ypos = (SENSENO-EdgeDisable)*(ssl_priv->Resolution)-ypos;
				#endif
			}
			else 
			{
				// This part is to avoid asyn problem when the finger leaves
//				printk("		ssd253x_ts_work: Correct %x\n",EventStatus);
				EventStatus=EventStatus&~(1<<i);
				clrFlag=1;
			}
		}
		else
		{
			xpos=ypos=0xFFF;
			width=0;
			clrFlag=1;
		}
		FingerX[i]=xpos;
		FingerY[i]=ypos;
		FingerP[i]=width;
	}
	if(ssl_priv->use_irq==1 && ssl_priv->irq_enabled == 0) {
		enable_irq(ssl_priv->irq);
		ssl_priv->irq_enabled = 1;
	}
	if(ssl_priv->use_irq==2)
	{
		if(ssl_priv->FingerDetect==0 && ssl_priv->irq_enabled==0) {
			enable_irq(ssl_priv->irq);
			ssl_priv->irq_enabled = 1;
		}
		else hrtimer_start(&ssl_priv->timer, ktime_set(0, MicroTimeTInterupt), HRTIMER_MODE_REL);
	}
	if(clrFlag) WriteRegister(ssl_priv->client,EVENT_FIFO_SCLR,0x01,0x00,1);

	if(ssl_priv->input->id.product==0x2533)
		if(ssl_priv->input->id.version==0x0101) 
			FingerCheckSwap(FingerX,FingerY,FingerP,ssl_priv->FingerNo,ssl_priv->sFingerX,ssl_priv->sFingerY);

	for(i=0;i<ssl_priv->FingerNo;i++)
	{
		xpos=FingerX[i];
		ypos=FingerY[i];
		width=FingerP[i];
		if(ssl_priv->input->id.product==0x2533)
		{
			if(ssl_priv->input->id.version==0x0101) RunningAverage(&xpos,&ypos,i,ssl_priv);
			if(ssl_priv->input->id.version==0x0102) RunningAverage(&xpos,&ypos,i,ssl_priv);
		}

		if(xpos!=0xFFF)
		{
			if (i==0) {
				input_report_key(ssl_priv->input, BTN_TOUCH, 1);
				input_report_abs(ssl_priv->input, ABS_X, xpos);
				input_report_abs(ssl_priv->input, ABS_Y, ypos);
				input_report_abs(ssl_priv->input, ABS_PRESSURE, 1);
			}

			input_report_abs(ssl_priv->input, ABS_MT_TRACKING_ID, i);  
			input_report_abs(ssl_priv->input, ABS_MT_TOUCH_MAJOR, 1);
			input_report_abs(ssl_priv->input, ABS_MT_POSITION_X, xpos);
			input_report_abs(ssl_priv->input, ABS_MT_POSITION_Y, ypos);
			input_report_abs(ssl_priv->input, ABS_MT_WIDTH_MAJOR, width);
			input_mt_sync(ssl_priv->input);

			#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
			if(i==0) printk("		ssd253x_ts_work: X = 0x%x , Y = 0x%x, W = 0x%x\n",xpos,ypos,width);
			#endif
		}
		else if(ssl_priv->FingerX[i]!=0xFFF)
		{
			if (i==0) {
				input_report_key(ssl_priv->input, BTN_TOUCH, 0);
				input_report_abs(ssl_priv->input, ABS_PRESSURE, 0);
			}
			input_report_abs(ssl_priv->input, ABS_MT_TRACKING_ID, i);
			input_report_abs(ssl_priv->input, ABS_MT_TOUCH_MAJOR, 0);
			input_mt_sync(ssl_priv->input);
			#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
			if(i==0) printk("	release	ssd253x_ts_work: X = 0x%x , Y = 0x%x, W = 0x%x\n",xpos,ypos,width);
			#endif
		}
		ssl_priv->FingerX[i]=FingerX[i];
		ssl_priv->FingerY[i]=FingerY[i];
		ssl_priv->FingerP[i]=width;
	}		
	ssl_priv->EventStatus=EventStatus;	
	input_sync(ssl_priv->input);
}

static int ssd253x_ts_probe(struct i2c_client *client,const struct i2c_device_id *idp)
{
	struct ssl_ts_priv *ssl_priv;
	struct input_dev *ssl_input;
        struct device *dev = &client->dev;
        enum of_gpio_flags flags;
	int error, ret;
	int i;

#ifdef CONFIG_TOUCHSCREEN_SELECTOR
        extern char ts_select[];
        if (strcmp(ts_select, "ssd253x")) {
                printk("*** SSD253x not enabled on command line\n");
                return -ENODEV;
        }
#endif

	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_probe!                 |\n");
	printk("+-----------------------------------------+\n");
	#endif

        if (client->irq == 0) {
                printk("%s: missing IRQ\n", __FUNCTION__);
                return -ENODEV;
        }        

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: need I2C_FUNC_I2C\n");
		#endif
		return -ENODEV;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: i2c Check OK!\n");
		printk("		ssd253x_ts_probe: i2c_client name : %s\n",client->name);
		#endif
	}

	ssl_priv = kzalloc(sizeof(*ssl_priv), GFP_KERNEL);
	if (!ssl_priv)
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: kzalloc Error!\n");
		#endif
		error=-ENODEV;
		goto	err0;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: kzalloc OK!\n");
		#endif
	}
	dev_set_drvdata(&client->dev, ssl_priv);
	
	ssl_input = input_allocate_device();
	if (!ssl_input)
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: input_allocate_device Error\n");
		#endif
		error=-ENODEV;
		goto	err1;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: input_allocate_device OK\n");
		#endif
	}

	ssl_input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) | BIT_MASK(EV_SYN) ;
	ssl_input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) | BIT_MASK(BTN_2);
	set_bit(INPUT_PROP_DIRECT, ssl_input->propbit);
	ssl_input->name = client->name;
	ssl_input->id.bustype = BUS_I2C;
	ssl_input->id.vendor  = 0x2878; // Modify for Vendor ID
	ssl_input->dev.parent = &client->dev;
	ssl_input->open = ssd253x_ts_open;
	ssl_input->close = ssd253x_ts_close;

	input_set_drvdata(ssl_input, ssl_priv);
	ssl_priv->client = client;
	ssl_priv->input = ssl_input;
	ssl_priv->use_irq = ENABLE_INT;
	ssl_priv->FingerNo=FINGERNO;
	ssl_priv->Resolution=64;
	ssl_priv->irq = client->irq;

	for(i=0;i<ssl_priv->FingerNo;i++)
	{
		ssl_priv->FingerP[i]=0;
		// For Finger Check Swap
		ssl_priv->sFingerX[i]=0xFFF;
		ssl_priv->sFingerY[i]=0xFFF;

		// For Adaptive Running Average
		ssl_priv->pFingerX[i]=0xFFF;
		ssl_priv->pFingerY[i]=0xFFF;
	}

        ret = of_get_named_gpio_flags(dev->of_node, "reset-gpio", 0, &flags);
        if (!ret) {
                dev_info(dev, "%s: reset-gpio is missing?\n", __FUNCTION__);
                return -ENODEV;
        }
        ssl_priv->reset_gpio = ret;

        ret = of_get_named_gpio_flags(dev->of_node, "int-gpio", 0, &flags);
        if (!ret) {
                dev_info(dev, "%s: int-gpio is missing?\n", __FUNCTION__);
                return -ENODEV;
        }
        ssl_priv->int_gpio = ret;

	deviceReset(client, ssl_priv->reset_gpio);
	printk("SSL Touchscreen I2C Address: 0x%02X\n",client->addr);
	ssl_input->id.product = ReadRegister(client, DEVICE_ID_REG,2);
	ssl_input->id.version = ReadRegister(client,VERSION_ID_REG,2);
	ssl_input->id.product = ReadRegister(client, DEVICE_ID_REG,2);
	ssl_input->id.version = ReadRegister(client,VERSION_ID_REG,2);
	printk("SSL Touchscreen Device ID  : 0x%04X\n",ssl_input->id.product);
	printk("SSL Touchscreen Version ID : 0x%04X\n",ssl_input->id.version);

	SSD253xdeviceInit(client);
	WriteRegister(client,EVENT_FIFO_SCLR,0x01,0x00,1); // clear Event FiFo
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("		ssd253X_ts_probe: %04XdeviceInit OK!\n",ssl_input->id.product);
	#endif

	if(ssl_priv->input->id.product==0x2531)		ssl_priv->Resolution=32;
	else if(ssl_priv->input->id.product==0x2533)	ssl_priv->Resolution=64;
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: ssl_input->id.product Error\n");
		#endif
		error=-ENODEV;
                gpio_free(ssl_priv->reset_gpio);
		goto	err1;
	}

	input_set_abs_params(ssl_input, ABS_MT_TRACKING_ID, 0,16, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_WIDTH_MAJOR, 0, 8, 0, 0);
//	input_set_abs_params(ssl_input, ABS_MT_POSITION_X,  0,(DRIVENO-EdgeDisable)*(ssl_priv->Resolution), 0, 0);
//	input_set_abs_params(ssl_input, ABS_MT_POSITION_Y,  0,(SENSENO-EdgeDisable)*(ssl_priv->Resolution), 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_POSITION_X,  0,1280, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_POSITION_Y,  0,800, 0, 0);
	input_set_abs_params(ssl_input, ABS_X,  0,1280, 0, 0);
	input_set_abs_params(ssl_input, ABS_Y,  0,800, 0, 0);
	input_set_abs_params(ssl_input, ABS_PRESSURE, 0, 1, 0, 0);

	INIT_WORK(&ssl_priv->ssl_work, ssd253x_ts_work);
	error = input_register_device(ssl_input);
	if(error)
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: input_register_device input Error!\n");
		#endif
		error=-ENODEV;
		goto	err1;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: input_register_device input OK!\n");
		#endif
	}

	if((ssl_priv->use_irq==1)||(ssl_priv->use_irq==2))
	{
		ssl_priv->irq_enabled = 1;
		// Options for different interrupt system 
//		error = request_irq(ssl_priv->irq, ssd253x_ts_isr, IRQF_DISABLED|IRQF_TRIGGER_FALLING, client->name,ssl_priv);
//		error = request_irq(ssl_priv->irq, ssd253x_ts_isr, IRQF_TRIGGER_FALLING, client->name,ssl_priv);
//		error = request_irq(ssl_priv->irq, ssd253x_ts_isr, IRQF_TRIGGER_LOW, client->name,ssl_priv);
		error = request_irq(ssl_priv->irq, ssd253x_ts_isr, IRQF_SHARED, client->name,ssl_priv);
		if(error)
		{
			#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
			printk("		ssd253x_ts_probe: request_irq Error!\n");
			#endif
			error=-ENODEV;
			goto err2;
		}
		else
		{
			#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
			printk("		ssd253x_ts_probe: request_irq OK!\n");
			#endif
		}	
	}

	if((ssl_priv->use_irq==0)||(ssl_priv->use_irq==2))
	{
		hrtimer_init(&ssl_priv->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ssl_priv->timer.function = ssd253x_ts_timer;
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_probe: timer_init OK!\n");
		#endif
	}
#ifdef	CONFIG_HAS_EARLYSUSPEND
	ssl_priv->early_suspend.suspend = ssd253x_ts_early_suspend;
	ssl_priv->early_suspend.resume  = ssd253x_ts_late_resume;
	ssl_priv->early_suspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN+1;
	register_early_suspend(&ssl_priv->early_suspend);
#endif 
	return 0;

err2:	input_unregister_device(ssl_input);
err1:	input_free_device(ssl_input);
	kfree(ssl_priv);
err0:	dev_set_drvdata(&client->dev, NULL);
	return error;
}

static int ssd253x_ts_open(struct input_dev *dev)
{
	struct ssl_ts_priv *ssl_priv = input_get_drvdata(dev);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_open!                  |\n");
	printk("+-----------------------------------------+\n");
	#endif	
	deviceResume(ssl_priv->client);
	if(ssl_priv->use_irq && ssl_priv->irq_enabled==0) {
		enable_irq(ssl_priv->irq);
		ssl_priv->irq_enabled = 1;
	}
	else hrtimer_start(&ssl_priv->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	return 0;
}

static void ssd253x_ts_close(struct input_dev *dev)
{
	struct ssl_ts_priv *ssl_priv = input_get_drvdata(dev);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_close!                 |\n");
	printk("+-----------------------------------------+\n");
	#endif
	deviceSuspend(ssl_priv->client);
	if((ssl_priv->use_irq==0)||(ssl_priv->use_irq==2)) hrtimer_cancel(&ssl_priv->timer);
#if 0 // This doesn't seem right!
	if((ssl_priv->use_irq==1)||(ssl_priv->use_irq==2)) free_irq(ssl_priv->irq, ssl_priv);
#endif
}

static int ssd253x_ts_resume(struct i2c_client *client)
{
	struct ssl_ts_priv *ssl_priv = dev_get_drvdata(&client->dev);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_resume!                |\n");
	printk("+-----------------------------------------+\n");
	#endif
	deviceResume(client);
	if(ssl_priv->use_irq && ssl_priv->irq_enabled==0) {
		enable_irq(ssl_priv->irq);
		ssl_priv->irq_enabled = 1;
	}
	else hrtimer_start(&ssl_priv->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	return 0;
}

static int ssd253x_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ssl_ts_priv *ssl_priv = dev_get_drvdata(&client->dev);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_suspend!               |\n");
	printk("+-----------------------------------------+\n");
	#endif
	deviceSuspend(client);		
	if((ssl_priv->use_irq==0)||(ssl_priv->use_irq==2)) hrtimer_cancel(&ssl_priv->timer);
#if 0 // This doesn't seem right!
	if((ssl_priv->use_irq==1)||(ssl_priv->use_irq==2)) free_irq(ssl_priv->irq, ssl_priv);	
#endif
	return 0;
}

#ifdef	CONFIG_HAS_EARLYSUSPEND
static void ssd253x_ts_late_resume(struct early_suspend *h)
{
	struct ssl_ts_priv *ssl_priv = container_of(h, struct ssl_ts_priv, early_suspend);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_late_resume!           |\n");
	printk("+-----------------------------------------+\n");
	#endif
	ssd253x_ts_resume(ssl_priv->client);
}
static void ssd253x_ts_early_suspend(struct early_suspend *h)
{
	struct ssl_ts_priv *ssl_priv = container_of(h, struct ssl_ts_priv, early_suspend);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_early_suspend!         |\n");
	printk("+-----------------------------------------+\n");
	#endif
	ssd253x_ts_suspend(ssl_priv->client, PMSG_SUSPEND);
}
#endif

static int ssd253x_ts_remove(struct i2c_client *client)
{
	struct ssl_ts_priv *ssl_priv = dev_get_drvdata(&client->dev);	
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_remove !               |\n");
	printk("+-----------------------------------------+\n");
	#endif	
	if((ssl_priv->use_irq==0)||(ssl_priv->use_irq==2)) hrtimer_cancel(&ssl_priv->timer);
	if((ssl_priv->use_irq==1)||(ssl_priv->use_irq==2)) free_irq(ssl_priv->irq, ssl_priv);
	input_unregister_device(ssl_priv->input);
	input_free_device(ssl_priv->input);
	kfree(ssl_priv);
	dev_set_drvdata(&client->dev, NULL);
	return 0;
}

static irqreturn_t ssd253x_ts_isr(int irq, void *dev_id)
{
	struct ssl_ts_priv *ssl_priv = dev_id;
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_isr!                   |\n");
	printk("+-----------------------------------------+\n");
	#endif	
	disable_irq_nosync(ssl_priv->irq);
	ssl_priv->irq_enabled = 0;
	queue_work(ssd253x_wq, &ssl_priv->ssl_work);
	return IRQ_HANDLED;
}

static enum hrtimer_restart ssd253x_ts_timer(struct hrtimer *timer)
{
	struct ssl_ts_priv *ssl_priv = container_of(timer, struct ssl_ts_priv, timer);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_timer!                 |\n");
	printk("+-----------------------------------------+\n");
	#endif
	queue_work(ssd253x_wq, &ssl_priv->ssl_work);
	if(ssl_priv->use_irq==0) hrtimer_start(&ssl_priv->timer, ktime_set(0, MicroTimeTInterupt), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static const struct i2c_device_id ssd253x_ts_id[] = {
	{ "ssd253x-ts", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ssd253x_ts_id);

static struct i2c_driver ssd253x_ts_driver = {
	.driver = {
		.name = "ssd253x-ts",
	},
	.probe = ssd253x_ts_probe,
	.remove = ssd253x_ts_remove,
#ifndef	CONFIG_HAS_EARLYSUSPEND
	.suspend = ssd253x_ts_suspend,
	.resume = ssd253x_ts_resume,
#endif
	.id_table = ssd253x_ts_id,
};

static char banner[] __initdata = KERN_INFO "SSL Touchscreen driver, (c) 2011 Solomon Systech Ltd.\n";
static int __init ssd253x_ts_init(void)
{
	int ret;
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG	
	printk("+-----------------------------------------+\n");
	printk("|	SSL_ts_init!                      |\n");
	printk("+-----------------------------------------+\n");
	#endif
	printk(banner);	
	ssd253x_wq = create_singlethread_workqueue("ssd253x_wq");
	if (!ssd253x_wq)
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_init: create_singlethread_workqueue Error!\n");
		#endif
		return -ENOMEM;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd253x_ts_init: create_singlethread_workqueue OK!\n");
		#endif
	}
	ret=i2c_add_driver(&ssd253x_ts_driver);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	if(ret) printk("		ssd253x_ts_init: i2c_add_driver Error! \n");
	else    printk("		ssd253x_ts_init: i2c_add_driver OK! \n");
	#endif
	return ret;
}

static void __exit ssd253x_ts_exit(void)
{
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_exit!                  |\n");
	printk("+-----------------------------------------+\n");
	#endif
	i2c_del_driver(&ssd253x_ts_driver);
	if (ssd253x_wq) destroy_workqueue(ssd253x_wq);
}

module_init(ssd253x_ts_init);
module_exit(ssd253x_ts_exit);

MODULE_AUTHOR("Solomon Systech Ltd - Design Technology, Icarus Choi");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ssd253x Touchscreen Driver 1.3");
