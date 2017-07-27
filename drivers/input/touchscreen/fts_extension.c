/* drivers/input/touchscreen/fts_extension.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 */

#ifdef FT5x06
#define FTS_UPGRADE_ID1			0x79
#define FTS_UPGRADE_ID2			0x03
#define FTS_UPGRADE_AA_DELAY		50
#define FTS_UPGRADE_55_DELAY		30
#define FTS_UPGRADE_READID_DELAY	1
#define FTS_UPGRADE_EARSE_DELAY	2000
#elif defined(FT5x16)
#define FTS_UPGRADE_ID1			0x79
#define FTS_UPGRADE_ID2			0x07
#define FTS_UPGRADE_AA_DELAY		50
#define FTS_UPGRADE_55_DELAY		10
#define FTS_UPGRADE_READID_DELAY	1
#define FTS_UPGRADE_EARSE_DELAY	1500
#elif defined(FT5606)
#define FTS_UPGRADE_ID1			0x79
#define FTS_UPGRADE_ID2			0x06
#define FTS_UPGRADE_AA_DELAY		50
#define FTS_UPGRADE_55_DELAY		10
#define FTS_UPGRADE_READID_DELAY	100
#define FTS_UPGRADE_EARSE_DELAY	2000
#elif defined(FT6x06)
#define FTS_UPGRADE_ID1			0x79
#define FTS_UPGRADE_ID2			0x05
#define FTS_UPGRADE_AA_DELAY		60
#define FTS_UPGRADE_55_DELAY		10
#define FTS_UPGRADE_READID_DELAY	10
#define FTS_UPGRADE_EARSE_DELAY	2000
#else
#error Sorry, no define FTS chip
#endif

#define FTS_VID_SOE 		0x71
#define FTS_VID_JTOUCH 		0x8b
#define FTS_FW_SOE 			0x0c
#define FTS_FW_JTOUCH 		0x16

static int max_tx = 0;
static int max_rx = 0;
static int raw_index = 0;
static u8 vid = 0;
static u8 ftsver = 0;
static bool isUpdate = false;

static int fts_enter_mode(struct fts_info *ts, u8 mode)
{
	u8 value;

	fts_writeb(ts, REG_DEV_MODE, mode);
	mdelay(100);

	if(fts_readb(ts, REG_DEV_MODE, &value)){
		fts_msg("%d ERROR: could not read register !\n", __LINE__);
		return -EIO;
	}
	else{
		if((value & 0x70) != mode){
			fts_msg("ERROR: The Touch Panel was not put in Factory Mode. The device Mode register contains 0x%02X\n", value);
			return -EIO;
		}
	}

	return 0;
}

static int fts_tx_voltage(struct fts_info *ts, u8 vol)
{
	int ret;
	u8 value;

	disable_irq(ts->client->irq);

	ret = fts_enter_mode(ts, FTS_MODE_TEST);
	if(ret)
		return ret;

	if(fts_writeb(ts, 0x05, vol)){
		fts_msg("%d ERROR: Can't set voltage !\n", __LINE__);
		ret = -ECOMM;
		goto exit;
	}

	mdelay(100);		  

	fts_readb(ts, 0x05, &value);

	if(value != vol)			
		fts_msg("%d ERROR: Can't set voltage !\n", __LINE__);

exit:
	fts_enter_mode(ts, FTS_MODE_NORMAL);

	enable_irq(ts->client->irq);

	return 0;
}

static int fts_calibration(struct fts_info *ts)
{
	int i, ret;
	u8 value;

	disable_irq(ts->client->irq);

	/*start auto CLB */
	msleep(200);

	ret = fts_enter_mode(ts, FTS_MODE_TEST);
	if(ret)
		return ret;

	/*write command to start calibration */
	ret = fts_writeb(ts, 0x02, 0x04);
	if(ret){
		fts_msg("%d ERROR: could not write register !\n", __LINE__);
		goto exit;
	}

	msleep(300);

	for (i = 0; i < 100; i++){
		ret = fts_readb(ts, 0, &value);
		if(ret){
			fts_msg("%d ERROR: could not read register !\n", __LINE__);
			goto exit;
		}

		/*return to normal mode, calibration finish */
		if(((value & 0x70) >> 4) == 0)
			break;

		mdelay(1);
	}

	if(i >= 100){
		fts_msg("ERROR: calibration fail !\n");
		goto exit;
	}

	/*calibration OK */
	msleep(300);

	ret = fts_enter_mode(ts, FTS_MODE_TEST);
	if(ret)
		return ret;

	ret = fts_writeb(ts, 0x02, 0x05);	/*store CLB result */

	if(ret){
		fts_msg("%d ERROR: could not write register !\n", __LINE__);
		goto exit;
	}
	msleep(300);

exit:

	fts_enter_mode(ts, FTS_MODE_NORMAL);

	enable_irq(ts->client->irq);

	return 0;
}

void fts_scan_ts(struct fts_info *ts)
{
	u8 value;

	if(fts_readb(ts, 0, &value)){
		fts_msg("%d ERROR: could not read register", __LINE__);
		return;
	}
	value |= 0x80;

	if(fts_writeb(ts, 0x00, value)){
		fts_msg("%d ERROR: could not write register", __LINE__);
		return;
	}

	msleep(20);

	if(fts_readb(ts, 0, &value)){
		fts_msg("%d ERROR: could not read register", __LINE__);
		return;
	}
	if(0x00 != (value & 0x80)){
		fts_msg("%d ERROR: could not scan", __LINE__);
	}

}

#define FTS_MODE_TEST1		0x60
#define FTS_MODE_TEST2		0x70

static int fts_get_rawdata(struct fts_info *ts, u16 *rawdata)
{
	int i, k, ret = 0;
	u8 addr = REG_RAWDATA;
	u16 *buffer;
#ifdef FT5606
	u16 *p;
	int counter;
#endif

	buffer = kzalloc((max_rx*2), GFP_KERNEL);
	if(!buffer){
		fts_msg("%d ERROR: Alloc memory Fail !\n", __LINE__);
		return -1;
	}

	ret = fts_enter_mode(ts, FTS_MODE_TEST);
	if(ret)
		goto exit;

	fts_scan_ts(ts);

	for(i = 0; i < max_tx; i++){
		fts_writeb(ts, REG_RAWDATA_ROW, i);
#ifdef FT5606
		if(max_rx > 20){
			counter = max_rx;
			p = buffer;
			fts_readsb(ts, &addr, (u8 *)p, (min(20, counter) << 1));
			p += min(20, counter);
			counter -= 20;
			fts_enter_mode(ts, FTS_MODE_TEST1);
			fts_readsb(ts, &addr, (u8 *)p, (min(20, counter) << 1));
			p += min(20, counter);
			counter -= min(20, counter);
			if(counter > 0){
				fts_enter_mode(ts, FTS_MODE_TEST2);
				fts_readsb(ts, &addr, (u8 *)p, (min(20, counter) << 1));
			}

			fts_enter_mode(ts, FTS_MODE_TEST);
		}
		else
#endif
		fts_readsb(ts, &addr, (u8 *)buffer, (max_rx << 1));

		for(k = 0; k < max_rx; k++)
			*rawdata++ = __be16_to_cpu(buffer[k]);
	}

	fts_enter_mode(ts, FTS_MODE_NORMAL);

exit:
	kfree(buffer);

	return 0;
}

#define FTS_PKT_EXTRA_LEN			6

static int fts_load_firmware(struct fts_info *ts, char *fw_name, u8 **buffer)
{
	int ret;
   	mm_segment_t orgfs;
	struct file *filp;
  	u32 fw_size;

	orgfs = get_fs();
	set_fs(KERNEL_DS);

	filp = filp_open(fw_name, O_RDONLY, 0);
	if (IS_ERR(filp))
	{
		fts_msg("Unable to open firmware file '%s'.\n",  fw_name);
		fw_size = 0;
		goto exit;
	}

	fw_size = filp->f_path.dentry->d_inode->i_size;

	*buffer = kzalloc(fw_size, GFP_KERNEL);
	if(!*buffer){
		fts_msg("%d Allocate memory fail!\n", __LINE__);
			fw_size = 0;
			goto exit;
	}

	if(filp->f_op && filp->f_op->read){
		ret = filp->f_op->read(filp, *buffer, fw_size, &filp->f_pos);
		if(ret < 0){
			fts_msg("Unable to load '%s'.\n", fw_name);
			fw_size = 0;
			goto exit;
		}
	}

	fts_msg("Load firmware %s size %d\n", fw_name, fw_size);

	filp_close(filp, NULL);

exit:
	set_fs(orgfs);

	return fw_size;
}

//static unsigned char fts_fw[] = {
	//#include "ft_app.i"
//};
static unsigned char fts_soe[] = {
      //#include "ft_app_soe_0x0c.i"
};
static unsigned char fts_jtouch[] = {
      //#include "ft_app_jt_0x16.i"
};


#define FTS_UPGRADE_PKT_LEN		128

static u8 check_id_cmd[] = {0x90, 0x00, 0x00, 0x00};

static int fts_enter_upgrade_mode(struct fts_info *ts)
{
	int ret, i = 0;
	u8 id[2];

	fts_msg("Enter upgrade mode progressing, don't turn off power\n");	

	/******write 0xaa to register 0xfc******/
	if(fts_writeb(ts, 0xFC, 0xAA))
		return -EIO;

	mdelay(FTS_UPGRADE_AA_DELAY);
	
	/******write 0x55 to register 0xfc******/
	if(fts_writeb(ts, 0xFC, 0x55))
		return -EIO;

	mdelay(FTS_UPGRADE_55_DELAY);   

	/*******Step 2:Enter upgrade mode ****/
	do{
		ret = fts_writeb(ts, 0x55, 0xAA);
		mdelay(5);
	}while(ret <= 0 && ++i < 10);

	msleep(FTS_UPGRADE_READID_DELAY);

	/********Step 3:check READ-ID********/
	if(fts_writesb(ts, check_id_cmd, sizeof(check_id_cmd)))
		return -EIO;

	if(fts_readsb(ts, NULL, id, 2))
		return -EIO;

	if (id[0] != FTS_UPGRADE_ID1 || id[1] != FTS_UPGRADE_ID2){
		fts_msg("Check ID fail! ID1 = 0x%x, ID2 = 0x%x\n", id[0], id[1]);
		return -ECOMM;
	}

	return 0;
}

static u8 fts_get_vendor_id(struct fts_info *ts)
{
	u8 id = 0;
	u8 addr = 0xa8;

	fts_readsb(ts, &addr, &id, 1);
	printk("[FTS] Vendor id = 0x%x\n", id);

	return id;
}

static u8 fts_get_bootloader_ver(struct fts_info *ts)
{
	u8 ver = 0;
	u8 addr = 0xcd;

	fts_readsb(ts, &addr, &ver, 1);
	printk("[FTS] bootloader version = 0x%x\n", ver);

	return ver;
}

static int fts_upgrade_proc(struct fts_info *ts, char *fw_name)
{
	int i, ret = 0;
	u8 *fw, *buf, *xmit_buf;
	u32 fw_size, size, pos;
	u8 ecc, fw_ecc, fw_version;

	disable_irq(ts->client->irq);

	if(fw_name){
		fw_size = fts_load_firmware(ts, fw_name, &fw);
		if(!fw_size){
			ret =-ENOENT;
			goto exit;
		}
	}
	else{
		if(isUpdate){
         if(vid == FTS_VID_SOE){
             fw = fts_soe;
             fw_size = sizeof(fts_soe);
			 isUpdate = false;
         }else if(vid == FTS_VID_JTOUCH){
	         fw = fts_jtouch;
             fw_size = sizeof(fts_jtouch);
			 isUpdate = false;
	     }
		}else
			goto exit;

//		fw = fts_fw;
//		fw_size = sizeof(fts_fw);

		if(fw_size < FTS_UPGRADE_PKT_LEN){
			fts_msg("Load firmware file fail !\n");
			ret =-ENOENT;
			goto exit;
		}
	}

	ret = fts_fw_version(ts, &fw_version);
	if(ret)
		goto free_fw_buf;

//	if(fw_version > fw[fw_size - 2]){
		fts_msg("Current firmware version %02X, upgrade firmware version %02X\n", fw_version, fw[fw_size - 2]);
//		return 0;
//	}

	xmit_buf = kzalloc(FTS_UPGRADE_PKT_LEN + FTS_PKT_EXTRA_LEN, GFP_KERNEL);
	if(!xmit_buf){
		fts_msg("%d Allocate memory fail!\n", __LINE__);
		goto free_fw_buf;
	}

	msleep(200);

	if(fts_enter_upgrade_mode(ts) < 0){
		fts_msg("Enter upgrade mode fail !\n");
		goto free_buf;
	}

	fts_get_bootloader_ver(ts);

	/*********Step 4:erase app**********/
	/* erase app and panel paramenter area */
	xmit_buf[0] = 0x61;
	fts_writesb(ts, xmit_buf, 1);
	msleep(FTS_UPGRADE_EARSE_DELAY);

	/* erase panel parameter area */
//	xmit_buf[0] = 0x63;
//	fts_writesb(ts, xmit_buf, 1);
//	msleep(100);

	fw_size -= 8;
	fw_ecc = pos = 0;
	buf = fw;

	xmit_buf[0] = 0xBF;
	xmit_buf[1] = 0x00;

	fts_msg("Upgrading ...\n");

	while(fw_size){		
		size = min(fw_size, (size_t)FTS_UPGRADE_PKT_LEN);
		xmit_buf[2] = (u8)(pos >> 8);
		xmit_buf[3] = (u8)pos;
		xmit_buf[4] = (u8)(size >> 8);
		xmit_buf[5] = (u8)size;
		memcpy(xmit_buf + 6, buf, size);

		for(i = 0; i < size; i++)
			fw_ecc ^= buf[i];

		fts_writesb(ts, xmit_buf, (size + FTS_PKT_EXTRA_LEN));
		mdelay((FTS_UPGRADE_PKT_LEN / 6) + 1);

//		printk(".");

		buf += size;
		pos += size;
		fw_size -= size;
	}

	/***********send the last six byte**********/
	for(i = 0; i < 6; i++){
		xmit_buf[2] = (u8)((0x6FFA + i) >> 8);
		xmit_buf[3] = (u8)(0x6FFA + i);;
		xmit_buf[4] = 0;
		xmit_buf[5] = 1;
		xmit_buf[6] = *buf++;
		fw_ecc ^= xmit_buf[6];
		fts_writesb(ts, xmit_buf, 7);
		mdelay(20);
//		printk(".");
	}

	/********read out checksum************/
	xmit_buf[0] = 0xCC;
	fts_writesb(ts, xmit_buf, 1);
	fts_readsb(ts, NULL, &ecc, 1);

	printk("======== ecc = 0x%x, fw_ecc = 0x%x\n", ecc, fw_ecc);
	if(ecc != fw_ecc){
		fts_msg("ECC check fail - ECC read 0x%x, firmware ECC 0x%x. \n", ecc, fw_ecc);
		goto free_buf;
	}

//	printk(KERN_INFO " Done\n");

	/*******Step 7: reset the new FW**********/
	xmit_buf[0] = 0x07;
	fts_writesb(ts, xmit_buf, 1);

	msleep(300);

	fts_msg("Upgrade finish\n");

free_buf:
	kfree(xmit_buf);

free_fw_buf:
    if(vid == FTS_VID_SOE){
	    if(fts_soe != fw){
			kfree(fw);
		}
    }else if(vid == FTS_VID_JTOUCH){
        if(fts_jtouch != fw){
             kfree(fw);
        }
	}

//	if(fts_fw != fw)
//		kfree(fw);

exit:
	enable_irq(ts->client->irq);

	return ret;
}

#ifdef FTS_DOWNLOAD_FUNCTION
#define FTS_DOWNLOAD_PKT_LEN		512
#define FT5x06_DL_CLIENT_ADDR		0x35

#define FTS_READ_MAIN_MEM			0x02
#define FTS_WRITE_MAIN_MEM		0x04
#define FTS_ERASE_MAIN_MEM		0x08

static int fts_dl_writesb(struct fts_info *ts, u8 i2c_addr, u8 *buf, u16 len)
{
	int ret;
	struct i2c_msg msg;

	FILL_I2C_MSG(msg, i2c_addr, 0, len, buf);

	ret = i2c_transfer(ts->client->adapter, &msg, 1);
	if (ret < 0){
		fts_msg("i2c_transfer failed !\n");
		return ret;
	}

	return 0;
}

static int fts_dl_readsb(struct fts_info *ts, u8 i2c_addr, u8 *buf, u16 len)
{
	int ret;
	struct i2c_msg msg[2];

	FILL_I2C_MSG(msg[0], i2c_addr, I2C_M_RD, len, buf);
	ret = i2c_transfer(ts->client->adapter, &msg[0], 1);

	if (ret < 0){
		fts_msg("i2c_transfer failed !\n");
		return ret;
	}

	return 0;
}

static int fts_download_proc(struct fts_info *ts, char *fw_name)
{
	int ret;
	u8 *fw, *buf, *rx_buf, *tx_buf;
	u32 fw_size, size, pos;

	disable_irq(ts->client->irq);

	fw_size = fts_load_firmware(ts, fw_name, &fw);

	if(!fw_size){
		ret =-ENOENT;
		goto exit;
	}

	tx_buf = kzalloc(FTS_DOWNLOAD_PKT_LEN + FTS_PKT_EXTRA_LEN, GFP_KERNEL);
	if(!tx_buf){
		fts_msg("Allocate memory fail!\n");
		goto free_fw_buf;
	}

	rx_buf = kzalloc(FTS_DOWNLOAD_PKT_LEN, GFP_KERNEL);
	if(!tx_buf){
		fts_msg("Allocate memory fail!\n");
		goto free_buf;
	}

	fts_msg("Please keep wakeup pin in low\n");
	msleep(3000);

	fts_msg("Start download firmware to FT5x06\n");

	tx_buf[0] = FTS_ERASE_MAIN_MEM;
	tx_buf[1] = ~FTS_ERASE_MAIN_MEM;

	if(fts_dl_writesb(ts, FT5x06_DL_CLIENT_ADDR, tx_buf, 2)){
		fts_msg("Erase FT5x06 memory fail!\n");
		goto free_rx_buf;
	}

	mdelay(200);

	buf = fw;
	size = pos = 0;
	buf = fw;

	while(fw_size){

		size = min(fw_size, (size_t)FTS_DOWNLOAD_PKT_LEN);

		tx_buf[0] = FTS_WRITE_MAIN_MEM;
		tx_buf[1] = ~FTS_WRITE_MAIN_MEM;
		tx_buf[2] = (u8)(pos >> 8);
		tx_buf[3] = (u8)pos;
		tx_buf[4] = (u8)((pos + size - 1) >> 8);
		tx_buf[5] = (u8)(pos + size - 1);

		memcpy(tx_buf + 6, buf, size);

		ret = fts_dl_writesb(ts, FT5x06_DL_CLIENT_ADDR, tx_buf, (size + FTS_PKT_EXTRA_LEN));
		if(ret){
			fts_msg("%d Write FT5x06 Flash fail !\n", __LINE__);
			goto free_rx_buf;
		}

		mdelay(100);

//		printk(".");

		tx_buf[0] = FTS_READ_MAIN_MEM;
		tx_buf[1] = ~FTS_READ_MAIN_MEM;
		tx_buf[2] = (u8)(pos >> 8);
		tx_buf[3] = (u8)pos;
		tx_buf[4] = (u8)((pos + size - 1) >> 8);
		tx_buf[5] = (u8)(pos + size - 1);

		ret = fts_dl_writesb(ts, FT5x06_DL_CLIENT_ADDR, tx_buf, FTS_PKT_EXTRA_LEN);
		if(ret){
			fts_msg("FT5x06 Communication error !\n");
			goto free_rx_buf;
		}

		mdelay(10);
		printk(".");

		ret = fts_dl_readsb(ts, FT5x06_DL_CLIENT_ADDR, rx_buf, size);
		if(ret){
			fts_msg("Read FT5x06 Flash fail !\n");
			goto free_rx_buf;
		}

		if(memcmp((tx_buf + 6), rx_buf, size) != 0){
			fts_msg("%d Write FT5x06 Flash fail !\n", __LINE__);
			ret = -EIO;
			break;
		}

		mdelay(10);

		buf += size;
		pos += size;
		fw_size -= size;
	}

	if(!fw_size)
		fts_msg("Download finish\n");

free_rx_buf:

	kfree(rx_buf);

free_buf:
	kfree(tx_buf);

free_fw_buf:
	kfree(fw);

exit:

	enable_irq(ts->client->irq);

	return ret;
}
#endif	/* FTS_DOWNLOAD_FUNCTION */

static ssize_t fts_vendor_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fts_info *ts = dev_get_drvdata(dev);
	u8 id;

	id = fts_get_vendor_id(ts);

	return sprintf(buf, "%02x", id);
}

static ssize_t fts_boot_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fts_info *ts = dev_get_drvdata(dev);
	u8 ver;
	u8 addr;
	
	fts_enter_upgrade_mode(ts);
	ver = fts_get_bootloader_ver(ts);

	//leave upgrade mode
	addr = 0x07;
	fts_writesb(ts, &addr, 1);

	return sprintf(buf, "FTS bootloader version is 0x%02x\n", ver);
}

static ssize_t fts_fw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fts_info *ts = dev_get_drvdata(dev);
	u8 ver = 0;
	u8 id = 0;

	fts_dbg("called\n");

	if(fts_fw_version(ts, &ver))
		sprintf(buf, "FTS read version is fail !\n");

	id = fts_get_vendor_id(ts);

	switch (id) {
		case FTS_VID_SOE:
			return sprintf(buf, "%s-%d\n", "SOE", ver);
		case FTS_VID_JTOUCH:
			return sprintf(buf, "%s-%d\n", "JTouch", ver);
		default:
			return sprintf(buf, "%d\n", ver);
	}
}

static ssize_t fts_fw_calibration(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fts_info *ts = dev_get_drvdata(dev);

	fts_dbg("called\n");

	if(fts_calibration(ts))
		return sprintf(buf, "Fail\n");

	return sprintf(buf, "Pass\n");
}

static ssize_t fts_fw_rawdata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fts_info *ts = dev_get_drvdata(dev);
	int ret = 0;
#ifdef FTS_RAWDATA_DEBUGVIEW
	static u16 *p, *tmp = NULL;
	char str[16];
	int i, k;
#else
	u16 *tmp;
#endif

	fts_dbg("called\n");

	if((max_tx == 0) || (max_rx == 0)){
		fts_msg("%d ERROR: TX number = 0, RX number = 0 !\n", __LINE__);
		return 0;
	}

#ifndef FTS_RAWDATA_DEBUGVIEW

	tmp = kzalloc((max_tx*max_rx*2), GFP_KERNEL);
	if(!tmp){
		fts_msg("%d ERROR: Alloc memory Fail !\n", __LINE__);
		return 0;
	}

	if(fts_get_rawdata(ts, tmp)){
		ret = sprintf(buf, "FTS read RawData is fail !\n");
	}
	else{
		ret = (max_tx*max_rx*2);
		memcpy(buf, tmp, ret);
	}

	kfree(tmp);

#else
	if(raw_index == 0){
		if(tmp)
			kfree(tmp);

		tmp = kzalloc((max_tx*max_rx*2), GFP_KERNEL);
		if(!tmp){
			fts_msg("%d ERROR: Alloc memory Fail !\n", __LINE__);
			return 0;
		}

		if(fts_get_rawdata(ts, tmp)){
			ret = sprintf(buf, "FTS read RawData is fail !\n");
			goto exit;
		}
	}

	if((max_tx*max_rx*6) < PAGE_SIZE){
		ret += sprintf(buf, "\n");
		for(p = tmp, i = 0; i < max_tx; i++){
			ret += sprintf(str, "\n");
			strcat(buf, str);
			for(k = 0; k < max_rx; k++){
				ret += sprintf(str, "%05d ", *p++);
				strcat(buf, str);
			}
		}
		ret += sprintf(str, "\n");
		strcat(buf, str);
	}
	else{
		ret += sprintf(buf, "\n");

		p = tmp + (raw_index * max_rx);
		raw_index++;

		for(k = 0; k < max_rx; k++){
			ret += sprintf(str, "%05d ", *p++);
			strcat(buf, str);
		}

		ret += sprintf(str, "\n");
		strcat(buf, str);

		if(raw_index < max_tx)
			return ret;
	}

exit:
	raw_index = 0;
	tmp = NULL;
	kfree(tmp);
#endif

	return ret;
}

static ssize_t fts_fw_rawdata_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct fts_info *ts = dev_get_drvdata(dev);
	char *tmp, cmd;
	int addr, value, ret;
	u8 r_value;

	fts_dbg("called - %s\n", buf);

	mutex_lock(&ts->lock);

	tmp = kzalloc(count, GFP_KERNEL);
	if(!tmp){
		fts_msg("%d ERROR: Alloc memory Fail !\n", __LINE__);
		goto exit;
	}

	memcpy(tmp, buf, count);

	tmp[count -1] = 0;

	sscanf(tmp, "%d %d", &max_tx, &max_rx);

	fts_msg("Config TX number = %d, RX number = %d \n", max_tx, max_rx);

	raw_index = 0;

	kfree(tmp);

exit:
	mutex_unlock(&ts->lock);

	return count;
}

static ssize_t fts_fw_upgrade_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fts_info *ts = dev_get_drvdata(dev);
	int i, k, ret = 0;

	fts_dbg("called\n");

	mutex_lock(&ts->lock);

	if(fts_upgrade_proc(ts, NULL))
		ret = sprintf(buf, "Upgrade fail !\n");
	else
		ret = sprintf(buf, "Do not need upgrade/upgrade finished !\n");

	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t fts_fw_upgrade_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct fts_info *ts = dev_get_drvdata(dev);
	char *fw_name;

	fts_dbg("called - %s\n", buf);

	mutex_lock(&ts->lock);

	fw_name = kzalloc(count, GFP_KERNEL);
	if(!fw_name){
		fts_msg("%d ERROR: Alloc memory Fail !\n", __LINE__);
		goto exit;
	}

	memcpy(fw_name, buf, count);

	fw_name[count -1] = 0;

	if(fts_upgrade_proc(ts, fw_name))
		fts_msg("Upgrade fail !\n");

	kfree(fw_name);

exit:
	mutex_unlock(&ts->lock);

	return count;
}

static ssize_t fts_fw_vol_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct fts_info *ts = dev_get_drvdata(dev);
	u8 vol;

	fts_dbg("called - %s\n", buf);

	mutex_lock(&ts->lock);

	vol = buf[0] - 0x30;

	fts_msg("Config tx voltage = %d !\n", vol);

	fts_tx_voltage(ts, vol);

	mutex_unlock(&ts->lock);

	return count;
}

static ssize_t fts_fw_rw_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct fts_info *ts = dev_get_drvdata(dev);
	char *tmp, cmd;
	int addr, value, ret;
	u8 r_value;

	fts_dbg("called - %s\n", buf);

	mutex_lock(&ts->lock);

	tmp = kzalloc(count, GFP_KERNEL);
	if(!tmp){
		fts_msg("%d ERROR: Alloc memory Fail !\n", __LINE__);
		goto exit;
	}

	memcpy(tmp, buf, count);

	tmp[count -1] = 0;
	
	if((tmp[0] == 'r') || (tmp[0] == 'R')){
		sscanf(tmp, "%c %x", &cmd, &addr);
		fts_dbg("Read register %02X\n", addr);
		ret = fts_readb(ts, addr, &r_value);
		if(!ret){
			fts_msg("Read register %02X = %02X\n", addr, r_value);
		}
	}
	else if((tmp[0] == 'w') || (tmp[0] == 'W')){
		sscanf(tmp, "%c %x %x", &cmd, &addr, &value);

		fts_dbg("Write register %02X = 0x%02X\n", addr, value);
		ret = fts_writeb(ts, (u8)addr, (u8)value);
		if(ret){
			fts_msg("Wreite register %02X fail \n!\n", addr);
		}
	}
	else if(tmp[0] == 'x'){
		fts_gpio_reset(ts);
	}
	else
		fts_msg("Command useage: r/R [addr], w/W [addr] [value]!\n");

	kfree(tmp);

exit:
	mutex_unlock(&ts->lock);

	return count;
}

static ssize_t fts_enable_write(struct device *dev, struct kobj_attribute *attr, const char *buf, size_t count)
{
	if (strncmp(buf, "1", count-1) == 0) {
		fts_enable = 1;
	} else {
		fts_enable = 0;
	}

	return count;
}
static ssize_t fts_enable_read(struct device *dev, struct kobj_attribute *attr, const char *buf)
{
	return sprintf(buf, "%d\n", fts_enable);
}

#define APP_FILE_PATH          "/system/etc/firmware/fts_app-"
#define APP_FILE_EXTENSION     ".bin"
#define APP_FILE_MAX_LEN       45
static ssize_t ft5x0x_fwupgradeapp_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct fts_info *ts = dev_get_drvdata(dev);
	char *fw_name;
	u8 *fw, id = 0;
	int num_read_chars = 0;
	u32 fw_size = 0;

	mutex_lock(&ts->lock);

	fw_name = kzalloc(APP_FILE_MAX_LEN, GFP_KERNEL);
	if(!fw_name){
		fts_msg("%d ERROR: Alloc memory Fail !\n", __LINE__);
		num_read_chars = sprintf(buf, "%d ERROR: Alloc memory Fail !\n", __LINE__);
		goto alloc_memory_fail;
	}

	memset(fw_name, 0, APP_FILE_MAX_LEN);

	id = fts_get_vendor_id(ts);
	switch (id) {
		case FTS_VID_SOE:
			sprintf(fw_name, "%s%s%s", APP_FILE_PATH, "SOE", APP_FILE_EXTENSION);
			break;
		case FTS_VID_JTOUCH:
			sprintf(fw_name, "%s%s%s", APP_FILE_PATH, "JTouch", APP_FILE_EXTENSION);
			break;
	}
	
	if(fw_name){
		fw_size = fts_load_firmware(ts, fw_name, &fw);
		if(!fw_size){
			num_read_chars = sprintf(buf, "%d ERROR: Read %s fail\n", __LINE__, fw_name);
			goto exit;
		}
	}

	switch (id) {
		case FTS_VID_SOE:
			num_read_chars = sprintf(buf, "%s-%d\n", "SOE", fw[fw_size - 2]);
			break;
		case FTS_VID_JTOUCH:
			num_read_chars = sprintf(buf, "%s-%d\n", "JTouch", fw[fw_size - 2]);
			break;
	}

	kfree(fw);
exit:
	kfree(fw_name);
alloc_memory_fail:
	mutex_unlock(&ts->lock);

	return num_read_chars;
}

/*upgrade from app.bin*/
static ssize_t ft5x0x_fwupgradeapp_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct fts_info *ts = dev_get_drvdata(dev);
	char *fw_name;
	unsigned long val = 0;
	int ret = -1;
	u8 id = 0;

	fts_dbg("called - %s\n", buf);

	mutex_lock(&ts->lock);

	ret = kstrtoul(buf, 10, &val);
	if (ret) {
		/* Using customer input filename */
		fw_name = kzalloc(count, GFP_KERNEL);
		if(!fw_name){
			fts_msg("%d ERROR: Alloc memory Fail !\n", __LINE__);
			goto exit;
		}

		memcpy(fw_name, buf, count);

		fw_name[count -1] = 0;
	} else {
		/* There is no mean for 0x571,
		 * just avoiding update firmware with accident.
		 */
		if (val == 0x571) {
			fw_name = kzalloc(APP_FILE_MAX_LEN, GFP_KERNEL);
			if(!fw_name){
				fts_msg("%d ERROR: Alloc memory Fail !\n", __LINE__);
				goto exit;
			}
			memset(fw_name, 0, APP_FILE_MAX_LEN);

			id = fts_get_vendor_id(ts);
			switch (id) {
				case FTS_VID_SOE:
					sprintf(fw_name, "%s%s%s", APP_FILE_PATH, "SOE", APP_FILE_EXTENSION);
					break;
				case FTS_VID_JTOUCH:
					sprintf(fw_name, "%s%s%s", APP_FILE_PATH, "JTouch", APP_FILE_EXTENSION);
					break;
			}

		}
	}

	if(fts_upgrade_proc(ts, fw_name))
		fts_msg("Upgrade fail !\n");

	kfree(fw_name);

exit:
	mutex_unlock(&ts->lock);

	return count;
}

static ssize_t ft5x0x_tpmodule_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct fts_info *ts = dev_get_drvdata(dev);
	u8 id = 0;

	id = fts_get_vendor_id(ts);
	switch (id) {
		case FTS_VID_SOE:
			return sprintf(buf, "%s\n", "SOE");
		case FTS_VID_JTOUCH:
			return sprintf(buf, "%s\n", "JTouch");
	}
}

static DEVICE_ATTR(ftsvid, S_IRUGO | S_IWUSR, fts_vendor_id, NULL);
static DEVICE_ATTR(ftsbootversion, S_IRUGO | S_IWUSR, fts_boot_show, NULL);
static DEVICE_ATTR(ftsraw, S_IRUGO | S_IWUSR, fts_fw_rawdata_show, fts_fw_rawdata_store);
static DEVICE_ATTR(ftsupgrade, S_IRUGO | S_IWUSR, fts_fw_upgrade_show, fts_fw_upgrade_store);
static DEVICE_ATTR(ftsvol, S_IRUGO | S_IWUSR, NULL, fts_fw_vol_store);
static DEVICE_ATTR(ftsregister, S_IRUGO | S_IWUSR, NULL, fts_fw_rw_reg_store);
static DEVICE_ATTR(fw_ver, S_IRUGO | S_IWUSR, fts_fw_show, NULL);
static DEVICE_ATTR(calibration, S_IRUGO | S_IWUSR, fts_fw_calibration, NULL);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, fts_enable_read, fts_enable_write);
static DEVICE_ATTR(tp_module, S_IRUGO | S_IWUSR, ft5x0x_tpmodule_show, NULL);
/* upgrade from app.bin
 * example: echo "*_app.bin" > /sys/bus/i2c/devices/1-0038/fw_upgrade_app
 */
static DEVICE_ATTR(fw_upgrade_app, S_IRUGO | S_IWUSR, ft5x0x_fwupgradeapp_show,
			ft5x0x_fwupgradeapp_store);

static struct attribute *fts_attributes[] = {
	&dev_attr_ftsvid.attr,
	&dev_attr_ftsbootversion.attr,
	&dev_attr_ftsraw.attr,
	&dev_attr_ftsupgrade.attr,
	&dev_attr_ftsvol.attr,
	&dev_attr_ftsregister.attr,
	&dev_attr_fw_ver.attr,
	&dev_attr_calibration.attr,
	&dev_attr_enable.attr,
	&dev_attr_tp_module.attr,
	&dev_attr_fw_upgrade_app.attr,
	NULL
};

static struct attribute_group fts_attribute_group = {
	.attrs = fts_attributes
};

static int fts_init_sysfs(struct fts_info *ts)
{
	int err;

	mutex_init(&ts->lock);

	err = sysfs_create_group(&ts->client->dev.kobj, &fts_attribute_group);
	if(err){
		fts_msg("Can't create sys filesystem group !\n");
		mutex_destroy(&ts->lock);
		return -EIO;
	}

//=======Detect touch firmware=====
    vid = fts_get_vendor_id(ts);

    fts_fw_version(ts, &ftsver);

	if(vid == FTS_VID_SOE && FTS_FW_SOE > ftsver){
		isUpdate = true;
	}else if(vid == FTS_VID_JTOUCH && FTS_FW_JTOUCH > ftsver){
		isUpdate = true;
	}else{
		isUpdate = false;
	}
//=================================

	return 0;
}

static void fts_release_sysfs(struct fts_info *ts)
{
	sysfs_remove_group(&ts->client->dev.kobj, &fts_attribute_group);
	mutex_destroy(&ts->lock);
}

