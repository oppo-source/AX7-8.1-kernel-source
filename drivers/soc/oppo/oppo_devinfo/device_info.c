/**
 * Copyright 2008-2013 OPPO Mobile Comm Corp., Ltd, All rights reserved.
 * VENDOR_EDIT:
 * FileName:devinfo.c
 * ModuleName:devinfo
 * Author: wangjc
 * Create Date: 2013-10-23
 * Description:add interface to get device information.
 * History:
   <version >  <time>  <author>  <desc>
   1.0		2013-10-23	wangjc	init
   2.0      2015-04-13  hantong modify as platform device  to support diffrent configure in dts
*/

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <soc/qcom/smem.h>
#include <soc/oppo/device_info.h>
#include <soc/oppo/oppo_project.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include "../../../fs/proc/internal.h"
#include <asm/uaccess.h>

#define DEVINFO_NAME "devinfo"
#define INFO_BUF_LEN 64

static int mainboard_res = 0;

static struct of_device_id devinfo_id[] = {
	{.compatible = "oppo-devinfo",},
	{},
};

struct devinfo_data {
	struct platform_device *devinfo;
	int hw_id1_gpio;
	int hw_id2_gpio;
	int hw_id3_gpio;
	int sub_hw_id1;
	int sub_hw_id2;
	int audio_hw_id1;
	int ant_select_gpio;
};

static struct proc_dir_entry *parent = NULL;

static void *device_seq_start(struct seq_file *s, loff_t *pos)
{
	static unsigned long counter = 0;
	if ( *pos == 0 ) {
		return &counter;
	}else{
		*pos = 0;
		return NULL;
	}
}

static void *device_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	return NULL;
}

static void device_seq_stop(struct seq_file *s, void *v)
{
	return;
}

static int device_seq_show(struct seq_file *s, void *v)
{
	struct proc_dir_entry *pde = s->private;
	struct manufacture_info *info = pde->data;
	if(info)
	  seq_printf(s, "Device version:\t\t%s\nDevice manufacture:\t\t%s\n",
					info->version,	info->manufacture);
	if(info->fw_path)
		seq_printf(s, "Device fw_path:\t\t%s\n",
				info->fw_path);
	return 0;
}

static struct seq_operations device_seq_ops = {
	.start = device_seq_start,
	.next = device_seq_next,
	.stop = device_seq_stop,
	.show = device_seq_show
};

static int device_proc_open(struct inode *inode,struct file *file)
{
	int ret = seq_open(file,&device_seq_ops);
	pr_err("caven %s is called\n",__func__);

	if(!ret){
		struct seq_file *sf = file->private_data;
		sf->private = PDE(inode);
	}
	return ret;
}
static const struct file_operations device_node_fops = {
	.read =  seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
	.open = device_proc_open,
	.owner = THIS_MODULE,
};

int register_device_proc(char *name, char *version, char *manufacture)
{
	struct proc_dir_entry *d_entry;
	struct manufacture_info *info;

	if(!parent) {
		parent =  proc_mkdir ("devinfo", NULL);
		if(!parent) {
			pr_err("can't create devinfo proc\n");
			return -ENOENT;
		}
	}

	info = kzalloc(sizeof *info, GFP_KERNEL);
	info->version = version;
	info->manufacture = manufacture;
	d_entry = proc_create_data (name, S_IRUGO, parent, &device_node_fops, info);
	if(!d_entry) {
		pr_err("create %s proc failed.\n", name);
		kfree(info);
		return -ENOENT;
	}
	return 0;
}

int register_devinfo(char *name, struct manufacture_info *info)
{
  	struct proc_dir_entry *d_entry;
  	if(!parent) {
  	parent =  proc_mkdir ("devinfo", NULL);
  	if(!parent) {
  			pr_err("can't create devinfo proc\n");
  			return -ENOENT;
  		}
  	}

  	d_entry = proc_create_data (name, S_IRUGO, parent, &device_node_fops, info);
  	if(!d_entry) {
  		pr_err("create %s proc failed.\n", name);
  		return -ENOENT;
  	}
  	return 0;
}

static void dram_type_add(void)
{
	struct manufacture_info dram_info;
	int *p = NULL;
	#if 0
	p  = (int *)smem_alloc2(SMEM_DRAM_TYPE,4);
	#else
	p  = (int *)smem_alloc(SMEM_DRAM_TYPE,4,0,0);
	#endif

	if(p)
	{
		switch(*p){
			case 0x01:
				dram_info.version = "K3QF4F40BM-FGCF FBGA";
				dram_info.manufacture = "SAMSUNG";
				break;
			case 0x06:
				dram_info.version = "H9CKNNNCPTMRPR FBGA";
				dram_info.manufacture = "HYNIX";
				break;
			default:
				dram_info.version = "unknown";
				dram_info.manufacture = "unknown";
		}

	}else{
		dram_info.version = "unknown";
		dram_info.manufacture = "unknown";

	}

	register_device_proc("ddr", dram_info.version, dram_info.manufacture);
}



static int get_ant_select_gpio(struct devinfo_data *devinfo_data)
{
	int ret = 0;
	struct device_node *np;

	pr_err("srd get_ant_select_gpio\n");
	if(devinfo_data != NULL && devinfo_data->devinfo !=NULL) {
		np = devinfo_data->devinfo->dev.of_node;
	}
	else {
		pr_err("devinfo_data is NULL\n");
		return 0;
	}
	devinfo_data->ant_select_gpio = of_get_named_gpio(np, "Hw,ant_select-gpio", 0);

	if( devinfo_data->ant_select_gpio >= 0){
		if( gpio_is_valid(devinfo_data->ant_select_gpio) ){
			ret = gpio_request(devinfo_data->ant_select_gpio, "ant_select-gpio");
				if(ret){
					pr_err(" unable to request gpio [%d]\n", devinfo_data->ant_select_gpio);
				}
		}
	}else{
		pr_err("devinfo_data->ant_select_gpio not specified\n");
	}

    return ret;
}

static int get_hw_opreator_version(struct devinfo_data *devinfo_data)
{
	int hw_operator_name = 0;

	hw_operator_name = get_Operator_Version();
	pr_err("hw_operator_name [%d]\n",hw_operator_name);
	return hw_operator_name;
}

static void sub_mainboard_verify(struct devinfo_data *devinfo_data)
{
	int ret;
	int id1 = -1;
	int id2 = -1;
	static char temp_manufacture_sub[INFO_BUF_LEN] = {0};
	unsigned char modem;
	unsigned char operator;
	struct device_node *np;
	struct manufacture_info mainboard_info;
	struct pinctrl *pinctrl = NULL;
	struct pinctrl_state *pinctrl_sleep = NULL;

	if(!devinfo_data){
		pr_err("devinfo_data is NULL\n");
		return;
	}

	np = devinfo_data->devinfo->dev.of_node;
	devinfo_data->sub_hw_id1 = of_get_named_gpio(np, "Hw,sub_hwid_1", 0);
	if(devinfo_data->sub_hw_id1 < 0 ) {
		pr_err("devinfo_data->sub_hw_id1 not specified\n");
	}

	devinfo_data->sub_hw_id2 = of_get_named_gpio(np, "Hw,sub_hwid_2", 0);
	if(devinfo_data->sub_hw_id2 < 0 ) {
		pr_err("devinfo_data->sub_hw_id2 not specified\n");
	}

	if(devinfo_data->sub_hw_id1 >= 0 ) {
		ret = gpio_request(devinfo_data->sub_hw_id1,"SUB_HW_ID1");
		if(ret){
			pr_err("unable to request gpio [%d]\n",devinfo_data->sub_hw_id1);
		}else{
			id1=gpio_get_value(devinfo_data->sub_hw_id1);
		}
 	}

 	if(devinfo_data->sub_hw_id2 >= 0 ) {
		ret = gpio_request(devinfo_data->sub_hw_id2,"SUB_HW_ID2");
		if(ret){
			pr_err("unable to request gpio [%d]\n",devinfo_data->sub_hw_id2);
		}else{
			id2=gpio_get_value(devinfo_data->sub_hw_id2);
		}
 	}

	pinctrl = devm_pinctrl_get(&devinfo_data->devinfo->dev);
	if (IS_ERR_OR_NULL(pinctrl)) {
		pr_err("failed to get pinctrl\n");
	} else {
		pinctrl_sleep = pinctrl_lookup_state(pinctrl, "sub_id_sleep");
		if (IS_ERR_OR_NULL(pinctrl_sleep)) {
			pr_err("failed to get state sub_id_sleep\n");
		} else {
			pinctrl_select_state(pinctrl, pinctrl_sleep);
		}
	}

	modem = get_Modem_Version();

	mainboard_info.manufacture = temp_manufacture_sub;
	mainboard_info.version ="Qcom";
	switch(get_project()) {
		case OPPO_18031:
		case OPPO_18032:
		case OPPO_18301:
		{
			if(get_PCB_Version() <= OPPO_18031_HW_DVT) { //t0 t1 evt dvt
				operator = get_Operator_Version();
				if((id2 == 0) && (id1 == 1) && ((operator == OPPO_18031_OPERATOR_FULLBAND) || (operator == OPPO_18031_OPERATOR_FOREIGN))) {
					/* 全频段/运营商+台湾小板 */
					snprintf(mainboard_info.manufacture, INFO_BUF_LEN, "%d-match", get_project());
				}
				else if((id2 == 1) && (id1 == 1) &&
					((operator == OPPO_18031_OPERATOR_ASIA) || (operator == OPPO_18031_OPERATOR_CHINA_MOBILE)
					 ||(operator == OPPO_18031_OPERATOR_ALLNET) || (operator == OPPO_18031_OPERATOR_INDIA) || (operator == OPPO_18031_OPERATOR_ASIA_VIETNAM)))
				{
					/* 亚太/CMCC/全网通/亚太版之越南专版+亚太小板 */
					snprintf(mainboard_info.manufacture, INFO_BUF_LEN, "%d-match", get_project());
				} else {
					snprintf(mainboard_info.manufacture, INFO_BUF_LEN, "%d-unmatch", get_project());
				}
			} else { //pvt...
				operator = get_Operator_Version();
				if((id2 == 0) && (id1 == 1) && ((operator == OPPO_18031_OPERATOR_FULLBAND) || (operator == OPPO_18031_OPERATOR_FOREIGN))) {
					/* 全频段/运营商+台湾小板 */
					snprintf(mainboard_info.manufacture, INFO_BUF_LEN, "%d-match", get_project());
				}
				else if((id2 == 1) && (id1 == 1) &&
					((operator == OPPO_18031_OPERATOR_ASIA) || (operator == OPPO_18031_OPERATOR_INDIA) || (operator == OPPO_18031_OPERATOR_ASIA_VIETNAM)))
				{
					/* 亚太/亚太版之越南专版+亚太小板 */
					snprintf(mainboard_info.manufacture, INFO_BUF_LEN, "%d-match", get_project());
				} else if((id2 == 1) && (id1 == 0) &&
					      ((operator == OPPO_18031_OPERATOR_CHINA_MOBILE)||(operator == OPPO_18031_OPERATOR_ALLNET)))
				{
					/* CMCC/全网通+内销小板 */
					snprintf(mainboard_info.manufacture, INFO_BUF_LEN, "%d-match", get_project());
				}
				else {
					snprintf(mainboard_info.manufacture, INFO_BUF_LEN, "%d-unmatch", get_project());
				}
			}
			pr_info("%s : pcbversion=%d id2=%d, id1=%d, opera=%d, modem=%d\n",__func__, get_PCB_Version(), id2, id2, operator, get_Modem_Version());
			break;
		}
		case OPPO_18171:
		case OPPO_18172:
		case OPPO_18571:
		{
			if(get_PCB_Version() <= HW_VERSION__11) { //t0 evt
				operator = get_Operator_Version();
				if((id2 == 1) && (id1 == 1) &&
				   ((operator == OPPO_18171_OPERATOR_ASIA) || (operator == OPPO_18171_OPERATOR_ASIA_COSTDOWN)
					|| (operator == OPPO_18171_OPERATOR_CHINA_MOBILE) || (operator == OPPO_18171_OPERATOR_ALLNET))) {
					/* 亚太版/全网通版/CMCC版 */
					snprintf(mainboard_info.manufacture, INFO_BUF_LEN, "%d-match", get_project());
				} else if((id2 == 0) && (id1 == 1) &&
				   (operator == OPPO_18171_OPERATOR_TAIWAN_MACAO)) {
					/* 营运商(台澳，日本)版本 */
					snprintf(mainboard_info.manufacture, INFO_BUF_LEN, "%d-match", get_project());
				} else if((id2 == 1) && (id1 == 0) &&
				   (operator == OPPO_18171_OPERATOR_FULLBAND)) {
					/* 全频段版本 */
					snprintf(mainboard_info.manufacture, INFO_BUF_LEN, "%d-match", get_project());
				} else {
					snprintf(mainboard_info.manufacture, INFO_BUF_LEN, "%d-unmatch", get_project());
				}
			} else { //dvt pvt...
				operator = get_Operator_Version();
				if((id2 == 1) && (id1 == 1) &&
				   ((operator == OPPO_18171_OPERATOR_ASIA) || (operator == OPPO_18171_OPERATOR_ASIA_COSTDOWN)
					|| (operator == OPPO_18171_OPERATOR_VIETNAM_1) || (operator == OPPO_18171_OPERATOR_VIETNAM_2))) {
					/* 亚太版(18571/18572/18576/18577) */
					snprintf(mainboard_info.manufacture, INFO_BUF_LEN, "%d-match", get_project());
				} else if((id2 == 0) && (id1 == 0) &&
				   ((operator == OPPO_18171_OPERATOR_ALLNET) || (operator == OPPO_18171_OPERATOR_CHINA_MOBILE))) {
					/* 内销机型(18171/17172) */
					snprintf(mainboard_info.manufacture, INFO_BUF_LEN, "%d-match", get_project());
				} else if((id2 == 1) && (id1 == 0) &&
				   (operator == OPPO_18171_OPERATOR_FULLBAND)) {
					/* 全频段版本 */
					snprintf(mainboard_info.manufacture, INFO_BUF_LEN, "%d-match", get_project());
				} else if((id2 == 0) && (id1 == 1) &&
				   (operator == OPPO_18171_OPERATOR_TAIWAN_MACAO)) {
					/* 营运商(台澳,日本:18573) */
					snprintf(mainboard_info.manufacture, INFO_BUF_LEN, "%d-match", get_project());
				} else {
					snprintf(mainboard_info.manufacture, INFO_BUF_LEN, "%d-unmatch", get_project());
				}
			}
			pr_info("%s : pcbversion=%d id2=%d, id1=%d, opera=%d, modem=%d\n",__func__, get_PCB_Version(), id2, id2, operator, get_Modem_Version());
			break;
		}
		default:
		{
			snprintf(mainboard_info.manufacture,INFO_BUF_LEN,"%d-%d",get_project(),get_Operator_Version());
			break;
		}
	}
	register_device_proc("sub_mainboard", mainboard_info.version, mainboard_info.manufacture);
}

//#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,modified 2016.9.28 for 16061
static void audio_board_verify(struct devinfo_data *devinfo_data)
{
	int ret;
	int id1 = -1;
	static char temp_manufacture_audio[INFO_BUF_LEN] = {0};
	struct device_node *np;
	struct manufacture_info mainboard_info;
	if(!devinfo_data){
		pr_err("devinfo_data is NULL\n");
		return;
	}
	np = devinfo_data->devinfo->dev.of_node;
	devinfo_data->audio_hw_id1 = of_get_named_gpio(np, "Hw,audio_hwid_1", 0);
	if(devinfo_data->audio_hw_id1 < 0 ) {
		pr_err("devinfo_data->sub_hw_id1 not specified\n");
	}

	if(devinfo_data->audio_hw_id1 >= 0 ) {
		ret = gpio_request(devinfo_data->audio_hw_id1,"audio_hw_id1");
		if(ret){
			pr_err("unable to request gpio [%d]\n",devinfo_data->audio_hw_id1);
		}else{
			id1=gpio_get_value(devinfo_data->audio_hw_id1);
		}
 	}

	mainboard_info.manufacture = temp_manufacture_audio;
	mainboard_info.version ="Qcom";
	switch(get_project()) {


		case OPPO_16017:
		case OPPO_16027:
		{

			snprintf(mainboard_info.manufacture,INFO_BUF_LEN,"%d-audio-china",get_project());
			break;
		}
		case OPPO_16061:
		{
			if(id1 == 0)
					snprintf(mainboard_info.manufacture,INFO_BUF_LEN,"%d-audio-china",get_project());
			else
					snprintf(mainboard_info.manufacture,INFO_BUF_LEN,"%d-audio-oversea",get_project());
			break;
		}
		default:
		{
			snprintf(mainboard_info.manufacture,INFO_BUF_LEN,"%d-%d",get_project(),get_Operator_Version());
			break;
		}
	}
	register_device_proc("audio_mainboard", mainboard_info.version, mainboard_info.manufacture);
}
//#endif /*VENDOR_EDIT*/

static void mainboard_verify(struct devinfo_data *devinfo_data)
{
	struct manufacture_info mainboard_info;
	int hw_opreator_version = 0;
	static char temp_manufacture[INFO_BUF_LEN] = {0};
	if(!devinfo_data){
		pr_err("devinfo_data is NULL\n");
		return;
	}
	/***Tong.han@Bsp.Group.Tp Added for Operator_Pcb detection***/
	hw_opreator_version = get_hw_opreator_version(devinfo_data);
	/*end of Add*/
	mainboard_info.manufacture = temp_manufacture;
	switch(get_PCB_Version()) {
		case HW_VERSION__10:
			mainboard_info.version ="10";
			snprintf(mainboard_info.manufacture,INFO_BUF_LEN,"%d-SA",hw_opreator_version);
	//		mainboard_info.manufacture = "SA(SB)";
			break;
		case HW_VERSION__11:
			mainboard_info.version = "11";
			snprintf(mainboard_info.manufacture,INFO_BUF_LEN,"%d-SB",hw_opreator_version);
	//		mainboard_info.manufacture = "SC";
			break;
		case HW_VERSION__12:
			mainboard_info.version = "12";
			snprintf(mainboard_info.manufacture,INFO_BUF_LEN,"%d-SC",hw_opreator_version);
	//		mainboard_info.manufacture = "SD";
			break;
		case HW_VERSION__13:
			mainboard_info.version = "13";
			snprintf(mainboard_info.manufacture,INFO_BUF_LEN,"%d-SD",hw_opreator_version);
    //        mainboard_info.manufacture = "SE";
			break;
		case HW_VERSION__14:
			mainboard_info.version = "14";
			snprintf(mainboard_info.manufacture,INFO_BUF_LEN,"%d-SE",hw_opreator_version);
	//		mainboard_info.manufacture = "SF";
			break;
		case HW_VERSION__15:
			mainboard_info.version = "15";
			snprintf(mainboard_info.manufacture,INFO_BUF_LEN,"%d-(T3-T4)",hw_opreator_version);
	//		mainboard_info.manufacture = "T3-T4";
			break;
		default:
			mainboard_info.version = "UNKOWN";
			snprintf(mainboard_info.manufacture,INFO_BUF_LEN,"%d-UNKOWN",hw_opreator_version);
	//		mainboard_info.manufacture = "UNKOWN";
	}
	register_device_proc("mainboard", mainboard_info.version, mainboard_info.manufacture);
}

//#ifdef VENDOR_EDIT
//rendong.shi@BSP.boot,2016/03/24,add for mainboard resource
static ssize_t mainboard_resource_read_proc(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;
	len = sprintf(page,"%d",mainboard_res);

	if(len > *off)
		len -= *off;
	else
		len = 0;

	if(copy_to_user(buf,page,(len < count ? len : count))){
		return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

struct file_operations mainboard_res_proc_fops = {
	.read = mainboard_resource_read_proc,
	.write = NULL,
};
//#endif


static int devinfo_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct devinfo_data *devinfo_data = NULL;
	struct proc_dir_entry *pentry;
	pr_info("%s \n",__func__);
	devinfo_data = kzalloc(sizeof(struct devinfo_data), GFP_KERNEL);
	if( devinfo_data == NULL ) {
		pr_err("devinfo_data kzalloc failed\n");
		ret = -ENOMEM;
		return ret;
	}

	/*parse_dts*/
	devinfo_data->devinfo = pdev;
	/*end of parse_dts*/
	get_ant_select_gpio(devinfo_data);
	mainboard_verify(devinfo_data);
	sub_mainboard_verify(devinfo_data);
	audio_board_verify(devinfo_data);
	dram_type_add();

    pentry = proc_create("mainboard", S_IRUGO, NULL, &mainboard_res_proc_fops);
	if(!pentry) {
		pr_err("create prjVersion proc failed.\n");
	}

	kfree(devinfo_data);
	return ret;
}

static int devinfo_remove(struct platform_device *dev)
{
	remove_proc_entry(DEVINFO_NAME, NULL);
	return 0;
}

static struct platform_driver devinfo_platform_driver = {
	.probe = devinfo_probe,
	.remove = devinfo_remove,
	.driver = {
		.name = DEVINFO_NAME,
		.of_match_table = devinfo_id,
	},
};

module_platform_driver(devinfo_platform_driver);

MODULE_DESCRIPTION("OPPO device info");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Wangjc <wjc@oppo.com>");
