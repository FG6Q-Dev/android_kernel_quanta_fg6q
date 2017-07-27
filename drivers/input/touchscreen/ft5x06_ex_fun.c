#include <linux/ft5x06_ex_fun.h>
#include <linux/ft5x06_ts.h>
#include <linux/mount.h>
#include <linux/netdevice.h>
//#include <mach/gpio.h>

//#include <mach/map.h>
//#include <mach/regs-clock.h>
//#include <mach/regs-gpio.h>
//#include <plat/gpio-cfg.h>
//#include <mach/gpio-bank-q.h>

extern void delay_qt_ms(unsigned long  w_ms);
extern int ft5x0x_i2c_Read(char * writebuf, int writelen, char *readbuf, int readlen);
extern int ft5x0x_i2c_Write(char *writebuf, int writelen);

#define    FTS_PACKET_LENGTH        128
#define FT5x0x_TX_NUM	28
#define FT5x0x_RX_NUM   16


//#define FT5606
#ifdef FT5606
#define UPGRADE_AA_DELAY 50
#define UPGRADE_55_DELAY 10
#define UPGRADE_ID_1	0x79
#define UPGRADE_ID_2	0x06
#define UPGRADE_READID_DELAY 100
#else
#define UPGRADE_AA_DELAY 50
#define UPGRADE_55_DELAY 30
#define UPGRADE_ID_1	0x79
#define UPGRADE_ID_2	0x03
#define UPGRADE_READID_DELAY 1
#endif

E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth);

static unsigned char CTPM_FW[]=
{
	//#include "ft_app.i"
};
static struct mutex device_mode_mutex;
#define CTP_I2C_SLAVEADDR 0x70
int ft5x0x_write_reg(unsigned char regaddr, unsigned char regvalue)
{
	unsigned char buf[2];
	buf[0] = regaddr;
	buf[1] = regvalue;
	return ft5x0x_i2c_Write(buf, sizeof(buf));
}

int ft5x0x_read_reg(unsigned char regaddr, unsigned char * regvalue)
{
	return ft5x0x_i2c_Read(&regaddr, 1, regvalue, 1);
}

int fts_ctpm_auto_clb(void)
{
	unsigned char uc_temp;
	unsigned char i ;

	printk("[FTS] start auto CLB.\n");
	msleep(200);
	ft5x0x_write_reg(0, 0x40);  
	delay_qt_ms(100);   //make sure already enter factory mode
	ft5x0x_write_reg(2, 0x4);  //write command to start calibration
	delay_qt_ms(300);
	for(i=0;i<100;i++)
	{
		ft5x0x_read_reg(0,&uc_temp);
		if ( ((uc_temp&0x70)>>4) == 0x0)  //return to normal mode, calibration finish
		{
		    break;
		}
		delay_qt_ms(200);
		printk("[FTS] waiting calibration %d\n",i);	    
	}
	printk("[FTS] calibration OK.\n");

	msleep(300);
	ft5x0x_write_reg(0, 0x40);  //goto factory mode
	delay_qt_ms(100);   //make sure already enter factory mode
	ft5x0x_write_reg(2, 0x5);  //store CLB result
	delay_qt_ms(300);
	ft5x0x_write_reg(0, 0x0); //return to normal mode 
	msleep(300);
	printk("[FTS] store CLB result OK.\n");
	return 0;
}

int fts_ctpm_fw_upgrade_with_i_file(void)
{
	FTS_BYTE*     pbt_buf = NULL;
	int i_ret;
	int fw_len = sizeof(CTPM_FW);
	if(fw_len<8 || fw_len>32*1024)
	{
		pr_err("FW length error\n");
		return -1;
	}
	if((CTPM_FW[fw_len-8]^CTPM_FW[fw_len-6])==0xFF
		&& (CTPM_FW[fw_len-7]^CTPM_FW[fw_len-5])==0xFF
		&& (CTPM_FW[fw_len-3]^CTPM_FW[fw_len-4])==0xFF)
	{
		//=========FW upgrade========================*/
		pbt_buf = CTPM_FW;
		/*call the upgrade function*/
		i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW));
		if (i_ret != 0)
		{
			printk("[FTS] upgrade failed i_ret = %d.\n", i_ret);
		}
		else
		{
			printk("[FTS] upgrade successfully.\n");
			fts_ctpm_auto_clb();  //start auto CLB
		}
	}
	else
	{
		pr_err("FW format error\n");
		return -1;
	}
	return i_ret;
}

unsigned char fts_ctpm_get_i_file_ver(void)
{
    unsigned int ui_sz;
    ui_sz = sizeof(CTPM_FW);
    if (ui_sz > 2)
    {
        return CTPM_FW[ui_sz - 2];
    }
    else
    {
        //TBD, error handling?
        return 0x00; //default value
    }
}

#define    FTS_SETTING_BUF_LEN        128

//update project setting
//only update these settings for COB project, or for some special case
int fts_ctpm_update_project_setting(void)
{
	unsigned char uc_i2c_addr;             //I2C slave address (8 bit address)
	unsigned char uc_io_voltage;           //IO Voltage 0---3.3v;	1----1.8v
	unsigned char uc_panel_factory_id;     //TP panel factory ID

	unsigned char buf[FTS_SETTING_BUF_LEN];
	FTS_BYTE reg_val[2] = {0};
	FTS_BYTE  auc_i2c_write_buf[10];
	FTS_BYTE  packet_buf[FTS_SETTING_BUF_LEN + 6];
	FTS_DWRD i = 0;
	int      i_ret;

	uc_i2c_addr = CTP_I2C_SLAVEADDR;
	uc_io_voltage = 0x0;
	uc_panel_factory_id = 0x5a;

	/*********Step 1:Reset  CTPM *****/
	/*write 0xaa to register 0xfc*/
	ft5x0x_write_reg(0xfc,0xaa);
	delay_qt_ms(50);
	 /*write 0x55 to register 0xfc*/
	ft5x0x_write_reg(0xfc,0x55);
	printk("[FTS] Step 1: Reset CTPM test\n");

	delay_qt_ms(30);   

	/*********Step 2:Enter upgrade mode *****/
	auc_i2c_write_buf[0] = 0x55;
	auc_i2c_write_buf[1] = 0xaa;
	do
	{
	    	i ++;
	    	//i_ret = ft5x0x_i2c_txdata(auc_i2c_write_buf, 2);
	    	i_ret = ft5x0x_i2c_Write(auc_i2c_write_buf, 2);
	    	delay_qt_ms(5);
	}while(i_ret <= 0 && i < 5 );

	/*********Step 3:check READ-ID***********************/
	auc_i2c_write_buf[0] = 0x90;
	auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
	ft5x0x_i2c_Read(auc_i2c_write_buf, 4, reg_val, 2);
	if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
	{
	    	printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
	}
	else
	{
	    	return ERR_READID;
	}
	auc_i2c_write_buf[0] = 0xcd;
	ft5x0x_i2c_Read(auc_i2c_write_buf, 1, reg_val, 1);
	printk("bootloader version = 0x%x\n", reg_val[0]);


	/* --------- read current project setting  ---------- */
	//set read start address
	buf[0] = 0x3;
	buf[1] = 0x0;
	buf[2] = 0x78;
	buf[3] = 0x0;
	//ft5x0x_i2c_Write(buf, 4);
	//byte_read(buf, FTS_SETTING_BUF_LEN);
	ft5x0x_i2c_Read(buf, 4, buf, FTS_SETTING_BUF_LEN);

	printk("[FTS] old setting: uc_i2c_addr = 0x%x, uc_io_voltage = %d, uc_panel_factory_id = 0x%x\n",
	    		buf[0],  buf[2], buf[4]);
	for (i = 0; i < FTS_SETTING_BUF_LEN; i++)
	{
	    	if (i % 16 == 0)     printk("\n");
	    	printk("0x%x, ", buf[i]);  
	}
	printk("\n");

	 /*--------- Step 4:erase project setting --------------*/
	auc_i2c_write_buf[0] = 0x63;
	ft5x0x_i2c_Write(auc_i2c_write_buf, 1);
	delay_qt_ms(100);

	/*----------  Set new settings ---------------*/
	buf[0] = uc_i2c_addr;
	buf[1] = ~uc_i2c_addr;
	buf[2] = uc_io_voltage;
	buf[3] = ~uc_io_voltage;
	buf[4] = uc_panel_factory_id;
	buf[5] = ~uc_panel_factory_id;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	packet_buf[2] = 0x78;
	packet_buf[3] = 0x0;
	packet_buf[4] = 0;
	packet_buf[5] = FTS_SETTING_BUF_LEN;
	for (i = 0; i < FTS_SETTING_BUF_LEN; i++)
	{
		packet_buf[6 + i] = buf[i];
		if (i % 16 == 0)     printk("\n");
		printk("0x%x, ", buf[i]);
	}
	printk("\n");
	ft5x0x_i2c_Write(packet_buf, FTS_SETTING_BUF_LEN + 6);
	//byte_write(&packet_buf[0],FTS_SETTING_BUF_LEN + 6);
	delay_qt_ms(100);

	/********* reset the new FW***********************/
	auc_i2c_write_buf[0] = 0x07;
	ft5x0x_i2c_Write(auc_i2c_write_buf, 1);

	msleep(200);

	return 0;
    
}

int fts_ctpm_auto_upg(void)
{
	unsigned char uc_host_fm_ver=FT5x0x_REG_FW_VER;
	unsigned char uc_tp_fm_ver;
	int           i_ret;
	
	ft5x0x_read_reg(FT5x0x_REG_FW_VER, &uc_tp_fm_ver);
	uc_host_fm_ver = fts_ctpm_get_i_file_ver();
	if ( uc_tp_fm_ver == FT5x0x_REG_FW_VER  ||   //the firmware in touch panel maybe corrupted
	     uc_tp_fm_ver < uc_host_fm_ver //the firmware in host flash is new, need upgrade
	    )
	{
		msleep(100);
		printk("[FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",
		    		uc_tp_fm_ver, uc_host_fm_ver);
		i_ret = fts_ctpm_fw_upgrade_with_i_file();    
		if (i_ret == 0)
		{
		    	msleep(300);
		    	uc_host_fm_ver = fts_ctpm_get_i_file_ver();
		    	printk("[FTS] upgrade to new version 0x%x\n", uc_host_fm_ver);
		}
		else
		{
		    	printk("[FTS] upgrade failed ret=%d.\n", i_ret);
		}
	}

	return 0;
}


E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
	
    	FTS_BYTE reg_val[2] = {0};
    	FTS_DWRD i = 0;

    	FTS_DWRD  packet_number;
    	FTS_DWRD  j;
   	FTS_DWRD  temp;
    	FTS_DWRD  lenght;
    	FTS_BYTE  packet_buf[FTS_PACKET_LENGTH + 6];
    	FTS_BYTE  auc_i2c_write_buf[10];
    	FTS_BYTE bt_ecc;
    	int      i_ret;

    	/*********Step 1:Reset  CTPM *****/
    	/*write 0xaa to register 0xfc*/
   	ft5x0x_write_reg(0xfc,0xaa);
    	delay_qt_ms(UPGRADE_AA_DELAY);
    	 /*write 0x55 to register 0xfc*/
    	ft5x0x_write_reg(0xfc,0x55);
    	printk("[FTS] Step 1: Reset CTPM test\n");
   
    	delay_qt_ms(UPGRADE_55_DELAY);   


    	/*********Step 2:Enter upgrade mode *****/
    	auc_i2c_write_buf[0] = 0x55;
    	auc_i2c_write_buf[1] = 0xaa;
    	do
    	{
        	i ++;
        	i_ret = ft5x0x_i2c_Write(auc_i2c_write_buf, 2);
        	delay_qt_ms(5);
    	}while(i_ret <= 0 && i < 5 );

    	/*********Step 3:check READ-ID***********************/   
	delay_qt_ms(UPGRADE_READID_DELAY);
   	auc_i2c_write_buf[0] = 0x90; 
	auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
   	//ft5x0x_i2c_Write(auc_i2c_write_buf, 4);

    	ft5x0x_i2c_Read(auc_i2c_write_buf, 4, reg_val, 2);
    	if (reg_val[0] == UPGRADE_ID_1 && reg_val[1] == UPGRADE_ID_2)
    	{
        	printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    	}
    	else
    	{
		printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
        	return ERR_READID;
    	}
	auc_i2c_write_buf[0] = 0xcd;
	ft5x0x_i2c_Read(auc_i2c_write_buf, 1, reg_val, 1);
	printk("[FTS] bootloader version = 0x%x\n", reg_val[0]);

     	/*********Step 4:erase app and panel paramenter area ********************/
	auc_i2c_write_buf[0] = 0x61;
	ft5x0x_i2c_Write(auc_i2c_write_buf, 1); //erase app area	
    	delay_qt_ms(1500); 

	auc_i2c_write_buf[0] = 0x63;
	ft5x0x_i2c_Write(auc_i2c_write_buf, 1); //erase panel parameter area
    	delay_qt_ms(100);
    	printk("[FTS] Step 4: erase. \n");

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	printk("[FTS] Step 5: start upgrade. \n");
	dw_lenth = dw_lenth - 8;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	for (j=0;j<packet_number;j++)
	{
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (FTS_BYTE)(temp>>8);
		packet_buf[3] = (FTS_BYTE)temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (FTS_BYTE)(lenght>>8);
		packet_buf[5] = (FTS_BYTE)lenght;

		for (i=0;i<FTS_PACKET_LENGTH;i++)
		{
		    packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
		    bt_ecc ^= packet_buf[6+i];
		}

		ft5x0x_i2c_Write(packet_buf, FTS_PACKET_LENGTH+6);
		delay_qt_ms(FTS_PACKET_LENGTH/6 + 1);
		if ((j * FTS_PACKET_LENGTH % 1024) == 0)
		{
		      printk("[FTS] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
		}
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
	{
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (FTS_BYTE)(temp>>8);
		packet_buf[3] = (FTS_BYTE)temp;

		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (FTS_BYTE)(temp>>8);
		packet_buf[5] = (FTS_BYTE)temp;

		for (i=0;i<temp;i++)
		{
		    packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
		    bt_ecc ^= packet_buf[6+i];
		}
  
		ft5x0x_i2c_Write(packet_buf, temp+6);
		delay_qt_ms(20);
	}

	//send the last six byte
	for (i = 0; i<6; i++)
	{
		temp = 0x6ffa + i;
		packet_buf[2] = (FTS_BYTE)(temp>>8);
		packet_buf[3] = (FTS_BYTE)temp;
		temp =1;
		packet_buf[4] = (FTS_BYTE)(temp>>8);
		packet_buf[5] = (FTS_BYTE)temp;
		packet_buf[6] = pbt_buf[ dw_lenth + i]; 
		bt_ecc ^= packet_buf[6];
  
		ft5x0x_i2c_Write(packet_buf, 7);
		delay_qt_ms(20);
	}

	/*********Step 6: read out checksum***********************/
	/*send the opration head*/
	auc_i2c_write_buf[0] = 0xcc;
	ft5x0x_i2c_Read(auc_i2c_write_buf, 1, reg_val, 1); 

	printk("[FTS] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
	if(reg_val[0] != bt_ecc)
	{
	    	return ERR_ECC;
	}

	/*********Step 7: reset the new FW***********************/
	auc_i2c_write_buf[0] = 0x07;
	ft5x0x_i2c_Write(auc_i2c_write_buf, 1);
	msleep(300);  //make sure CTP startup normally

	return ERR_OK;
}

static struct i2c_client *this_client;
/* sysfs */
static u8 ft5x0x_enter_factory(struct ft5x0x_ts_data *ft5x0x_ts)
{	
	u8 regval;
	flush_workqueue(ft5x0x_ts->ts_workqueue);
//	disable_irq_nosync(TOUCH_INT_IRQ);
    disable_irq_nosync(this_client->irq);
	ft5x0x_write_reg(0x00, 0x40);  //goto factory mode
    	delay_qt_ms(100);   //make sure already enter factory mode
	if(ft5x0x_read_reg(0x00, &regval)<0)
		pr_err("%s ERROR: could not read register\n", __FUNCTION__);
	else
	{
		if((regval & 0x70) != 0x40)
		{
			pr_err("%s() - ERROR: The Touch Panel was not put in Factory Mode. The Device Mode register contains 0x%02X\n", __FUNCTION__, regval);
			return -1;
		}
	}
	return 0;
}
static u8 ft5x0x_enter_work(struct ft5x0x_ts_data *ft5x0x_ts)
{
	u8 regval;
   	 ft5x0x_write_reg(0x00, 0x00); //return to normal mode 
   	 msleep(100);
	
	if(ft5x0x_read_reg(0x00, &regval)<0)
		pr_err("%s ERROR: could not read register\n", __FUNCTION__);
	else
	{
		if((regval & 0x70) != 0x00)
		{
			pr_err("%s() - ERROR: The Touch Panel was not put in Work Mode. The Device Mode register contains 0x%02X\n", __FUNCTION__, regval);
		//	enable_irq(TOUCH_INT_IRQ);
	        enable_irq(this_client->irq);
			return -1;
		}
	}
//	enable_irq(TOUCH_INT_IRQ);
	enable_irq(this_client->irq);
	return 0;
}

static int fts_Get_RawData(struct i2c_client * client, u16 RawData[][FT5x0x_RX_NUM])
{
	int retval  = 0;
    	int i       = 0;
//    	u16 dataval = 0x0000;
    	u8  devmode = 0x00;
    	u8  rownum  = 0x00;

    	u8 read_buffer[FT5x0x_RX_NUM * 2];
	u8 read_len = 0;
	//u8 write_buffer[2];
	struct ft5x0x_ts_data * ft5x0x_ts =  i2c_get_clientdata(client);
	if(ft5x0x_enter_factory(ft5x0x_ts)<0)
	{
		pr_err("%s ERROR: could not enter factory mode", __FUNCTION__);
		retval = -1;
		goto error_return;
	}
	//scan
	if(ft5x0x_read_reg(0x00, &devmode)<0)
	{
		pr_err("%s %d ERROR: could not read register 0x00", __FUNCTION__, __LINE__);
		retval = -1;
		goto error_return;
	}
	devmode |= 0x80;
	if(ft5x0x_write_reg(0x00, devmode)<0)
	{
		pr_err("%s %d ERROR: could not read register 0x00", __FUNCTION__, __LINE__);
		retval = -1;
		goto error_return;
	}
	msleep(20);
	if(ft5x0x_read_reg(0x00, &devmode)<0)
	{
		pr_err("%s %d ERROR: could not read register 0x00", __FUNCTION__, __LINE__);
		retval = -1;
		goto error_return;
	}
	if(0x00 != (devmode&0x80))
	{
		pr_err("%s %d ERROR: could not scan", __FUNCTION__, __LINE__);
		retval = -1;
		goto error_return;
	}
	pr_info("Read rawdata .......\n");
	for(rownum=0; rownum<FT5x0x_TX_NUM; rownum++)
	{
		memset(read_buffer, 0x00, (FT5x0x_RX_NUM * 2));

		if(ft5x0x_write_reg(0x01, rownum)<0)
		{
			pr_err("%s ERROR:could not write rownum", __FUNCTION__);
			retval = -1;
			goto error_return;
		}
		msleep(1);
		read_len = FT5x0x_RX_NUM * 2;
		if(ft5x0x_write_reg(0x10, read_len)<0)
		{
			pr_err("%s ERROR:could not write rownum", __FUNCTION__);
			retval = -1;
			goto error_return;
		}
		retval = ft5x0x_i2c_Read(NULL, 0, read_buffer, FT5x0x_RX_NUM * 2);
		if (retval < 0) 
		{
			pr_err("%s ERROR:Could not read row %u raw data", __FUNCTION__, rownum);
			retval = -1;
			goto error_return;
		}
		for(i=0; i<FT5x0x_RX_NUM; i++)
		{
			RawData[rownum][i] = (read_buffer[i<<1]<<8) + read_buffer[(i<<1)+1];
		}
	}
error_return:
	if(ft5x0x_enter_work(ft5x0x_ts)<0)
	{
		pr_err("%s ERROR:could not enter work mode ", __FUNCTION__);
		retval = -1;
	}
	return retval;
}

static int ft5x0x_GetFirmwareSize(char * firmware_name)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic; 
	off_t fsize = 0; 
	char filepath[128];memset(filepath, 0, sizeof(filepath));

	sprintf(filepath, "/sdcard/%s", firmware_name);
	pr_info("filepath=%s\n", filepath);
	if(NULL == pfile){
		pfile = filp_open(filepath, O_RDONLY, 0);
		}
	if(IS_ERR(pfile)){
		pr_err("error occured while opening file %s.\n", filepath);
		return -1;
		}
	inode=pfile->f_dentry->d_inode; 
	magic=inode->i_sb->s_magic;
	fsize=inode->i_size; 
	filp_close(pfile, NULL);
	return fsize;
}
static int ft5x0x_ReadFirmware(char * firmware_name, unsigned char * firmware_buf)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic; 
	off_t fsize; 
	char filepath[128];
	loff_t pos;

	mm_segment_t old_fs;
	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "/sdcard/%s", firmware_name);
	pr_info("filepath=%s\n", filepath);
	if(NULL == pfile){
		pfile = filp_open(filepath, O_RDONLY, 0);
		}
	if(IS_ERR(pfile)){
		pr_err("error occured while opening file %s.\n", filepath);
		return -1;
		}
	inode=pfile->f_dentry->d_inode; 
	magic=inode->i_sb->s_magic;
	fsize=inode->i_size; 
	//char * buf;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;

	vfs_read(pfile, firmware_buf, fsize, &pos);

	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}

/**************************************/

int fts_ctpm_fw_upgrade_with_app_file(char * firmware_name)
{
  	FTS_BYTE*     pbt_buf = NULL;
   	int i_ret; u8 fwver;
   	int fwsize = ft5x0x_GetFirmwareSize(firmware_name);
   	if(fwsize <= 0)
   	{
   		pr_err("%s ERROR:Get firmware size failed\n", __FUNCTION__);
		return -1;
   	}
	if(fwsize<8 || fwsize>32*1024)
	{
		pr_err("FW length error\n");
		return -1;
	}
	
    //=========FW upgrade========================*/
  	 pbt_buf = (unsigned char *) kmalloc(fwsize+1,GFP_ATOMIC);
	if(ft5x0x_ReadFirmware(firmware_name, pbt_buf))
    	{
       	pr_err("%s() - ERROR: request_firmware failed\n", __FUNCTION__);
        	kfree(pbt_buf);
		return -1;
    	}
	if((pbt_buf[fwsize-8]^pbt_buf[fwsize-6])==0xFF
		&& (pbt_buf[fwsize-7]^pbt_buf[fwsize-5])==0xFF
		&& (pbt_buf[fwsize-3]^pbt_buf[fwsize-4])==0xFF)
	{
		/*call the upgrade function*/
		i_ret =  fts_ctpm_fw_upgrade(pbt_buf, fwsize);
   		if (i_ret != 0)
   		{
       		pr_err("%s() - ERROR:[FTS] upgrade failed i_ret = %d.\n",__FUNCTION__,  i_ret);
   		}
  	 	else
   		{
       		pr_info("[FTS] upgrade successfully.\n");
			if(ft5x0x_read_reg(FT5x0x_REG_FW_VER, &fwver)>=0)
				pr_info("the new fw ver is 0x%02x\n", fwver);
      		fts_ctpm_auto_clb();  //start auto CLB
   		}
		kfree(pbt_buf);
	}
	else
	{
		pr_err("FW format error\n");
		kfree(pbt_buf);
		return -1;
	}
   	return i_ret;
}

static ssize_t ft5x0x_tpfwver_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x0x_ts_data *data = NULL;
	ssize_t num_read_chars = 0;
	u8	   fwver = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	data = (struct ft5x0x_ts_data *) i2c_get_clientdata( client );
	
	
	mutex_lock(&device_mode_mutex);
	if(ft5x0x_read_reg(FT5x0x_REG_FW_VER, &fwver) < 0)
		num_read_chars = snprintf(buf, PAGE_SIZE, "get tp fw version fail!\n");
	else
		num_read_chars = snprintf(buf, PAGE_SIZE, "%02X\n", fwver);

	mutex_unlock(&device_mode_mutex);
	return num_read_chars;
}

static ssize_t ft5x0x_tpfwver_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}

static ssize_t ft5x0x_tprwreg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

static ssize_t ft5x0x_tprwreg_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct ft5x0x_ts_data *data = NULL;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);	
	ssize_t num_read_chars = 0;
	int retval;
	u16 wmreg=0;u8 regaddr=0xff,regvalue=0xff;
	u8 valbuf[5];
	memset(valbuf, 0, sizeof(valbuf));
	data = (struct ft5x0x_ts_data *) i2c_get_clientdata( client );
	
	mutex_lock(&device_mode_mutex);
	num_read_chars = count - 1;

	if(num_read_chars!=2)
	{
		if(num_read_chars!=4)
		{
			pr_info("please input 2 or 4 character\n");
			goto error_return;
		}
	}
	
	memcpy(valbuf, buf, num_read_chars);
	retval = strict_strtoul(valbuf, 16, &wmreg);
	//pr_info("valbuf=%s wmreg=%x\n", valbuf, wmreg);
    	if (0 != retval)
    	{
        	pr_err("%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n", __FUNCTION__, buf);
        	goto error_return;
    	}

	if(2 == num_read_chars)
	{
		//read register
		regaddr = wmreg;
		if(ft5x0x_read_reg(regaddr, &regvalue) < 0)
			pr_err("Could not read the register(0x%02x)\n", regaddr);
		else
			pr_info("the register(0x%02x) is 0x%02x\n", regaddr, regvalue);
	}
	else
	{
		regaddr = wmreg>>8;
		regvalue = wmreg;
		if(ft5x0x_write_reg(regaddr, regvalue)<0)
			pr_err("Could not write the register(0x%02x)\n", regaddr);
		else
			pr_err("Write 0x%02x into register(0x%02x) successful\n", regvalue, regaddr);
	}
error_return:
	mutex_unlock(&device_mode_mutex);

	return count;
}


static ssize_t ft5x0x_fwupdate_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
    return -EPERM;
}
//upgrade from *.i
static ssize_t ft5x0x_fwupdate_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct ft5x0x_ts_data *data = NULL;
	u8 uc_host_fm_ver;int i_ret;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	data = (struct ft5x0x_ts_data *) i2c_get_clientdata( client );
//	ssize_t num_read_chars = 0;
	
	mutex_lock(&device_mode_mutex);

	//disable_irq(TOUCH_INT_IRQ);
	disable_irq(this_client->irq);
	i_ret = fts_ctpm_fw_upgrade_with_i_file();    
	if (i_ret == 0)
	{
	    msleep(300);
	    uc_host_fm_ver = fts_ctpm_get_i_file_ver();
	    pr_info("%s [FTS] upgrade to new version 0x%x\n", __FUNCTION__, uc_host_fm_ver);
	}
	else
	{
	    pr_err("%s ERROR:[FTS] upgrade failed ret=%d.\n", __FUNCTION__, i_ret);
	}
	//enable_irq(TOUCH_INT_IRQ);
	enable_irq(this_client->irq);
	
	mutex_unlock(&device_mode_mutex);

	return count;
}

static ssize_t ft5x0x_fwupgradeapp_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
    return -EPERM;
}
//upgrade from app.bin
static ssize_t ft5x0x_fwupgradeapp_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct ft5x0x_ts_data *data = NULL;
	char fwname[128];
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	data = (struct ft5x0x_ts_data *) i2c_get_clientdata( client );
//	ssize_t num_read_chars = 0;
	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count-1] = '\0';

	mutex_lock(&device_mode_mutex);
	//disable_irq(TOUCH_INT_IRQ);
	disable_irq(this_client->irq);
	
	fts_ctpm_fw_upgrade_with_app_file(fwname);
	
	//enable_irq(TOUCH_INT_IRQ);
	enable_irq(this_client->irq);

	mutex_unlock(&device_mode_mutex);

	return count;
}

static ssize_t ft5x0x_rawdata_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x0x_ts_data *data = NULL;
	ssize_t num_read_chars = 0;
	int i=0, j=0;u16	RawData[FT5x0x_TX_NUM][FT5x0x_RX_NUM];
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	data = (struct ft5x0x_ts_data *) i2c_get_clientdata( client );

	mutex_lock(&device_mode_mutex);
	
	if(fts_Get_RawData(client, RawData)<0)
		sprintf(buf, "%s", "could not get rawdata\n");
	else
	{
		for(i=0; i<FT5x0x_TX_NUM; i++)
		{
			for(j=0; j<FT5x0x_RX_NUM; j++)
			{
				num_read_chars += sprintf(&(buf[num_read_chars]), "%u ", RawData[i][j]);
			}
			buf[num_read_chars-1] = '\n';
		}
	}

	mutex_unlock(&device_mode_mutex);	
	return num_read_chars;
}
//upgrade from app.bin
static ssize_t ft5x0x_rawdata_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{

	return -EPERM;
}


/* sysfs */
static DEVICE_ATTR(ftstpfwver, S_IRUGO|S_IWUSR, ft5x0x_tpfwver_show, ft5x0x_tpfwver_store);
//upgrade from *.i
static DEVICE_ATTR(ftsfwupdate, S_IRUGO|S_IWUSR, ft5x0x_fwupdate_show, ft5x0x_fwupdate_store);
static DEVICE_ATTR(ftstprwreg, S_IRUGO|S_IWUSR, ft5x0x_tprwreg_show, ft5x0x_tprwreg_store);
//upgrade from app.bin 
static DEVICE_ATTR(ftsfwupgradeapp, S_IRUGO|S_IWUSR, ft5x0x_fwupgradeapp_show, ft5x0x_fwupgradeapp_store);
static DEVICE_ATTR(ftsrawdatashow, S_IRUGO|S_IWUSR, ft5x0x_rawdata_show, ft5x0x_rawdata_store);


static struct attribute *ft5x0x_attributes[] = {
	&dev_attr_ftstpfwver.attr,
	&dev_attr_ftsfwupdate.attr,
	&dev_attr_ftstprwreg.attr,
	&dev_attr_ftsfwupgradeapp.attr,
	&dev_attr_ftsrawdatashow.attr,
	NULL
};

static struct attribute_group ft5x0x_attribute_group = {
	.attrs = ft5x0x_attributes
};

int ft5x0x_create_sysfs(struct i2c_client * client)
{
	int err;
	err = sysfs_create_group(&client->dev.kobj, &ft5x0x_attribute_group);
   	if (0 != err)
  	{
		dev_err(&client->dev, "%s() - ERROR: sysfs_create_group() failed: %d\n", __FUNCTION__, err);
		sysfs_remove_group(&client->dev.kobj, &ft5x0x_attribute_group);
  	}
   	else
    	{		
		mutex_init(&device_mode_mutex);
        	DbgPrintk("ft5x0x:%s() - sysfs_create_group() succeeded.\n", __FUNCTION__);
    	}
	return err;
}

void ft5x0x_release_sysfs(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &ft5x0x_attribute_group);
	mutex_destroy(&device_mode_mutex);
}

