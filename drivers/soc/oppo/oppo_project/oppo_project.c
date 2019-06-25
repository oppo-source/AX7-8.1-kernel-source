
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <soc/qcom/smem.h>
#include <soc/oppo/oppo_project.h>
//#include <mach/oppo_reserve3.h>
#include <linux/io.h>
#include <linux/mm.h>
#define RAM_2G 2*1024*1024
#define RAM_3G 3*1024*1024

static struct proc_dir_entry *oppoVersion = NULL;
static ProjectInfoCDTType *format = NULL;

unsigned int init_project_version(void)
{
	unsigned int len = (sizeof(ProjectInfoCDTType) + 3)&(~0x3);

	format = (ProjectInfoCDTType *)smem_alloc(SMEM_PROJECT,len,0,0);

	if(format)
		return format->nProject;

	return 0;
}


unsigned int get_project(void)
{
	if(format)
		return format->nProject;
	else
		return init_project_version();
	return 0;
}

unsigned int is_project(OPPO_PROJECT project )
{
	return (get_project() == project?1:0);
}

#ifdef VENDOR_EDIT
/* Peikun@PSW.MM.AudioDriver.HeadsetDet.1473948, 2018/10/12,
* modify for disabling 3uA pull up control for detection
*/
EXPORT_SYMBOL(is_project);
#endif /* VENDOR_EDIT */

unsigned char get_PCB_Version(void)
{
	if(format)
		return format->nPCBVersion;
	return 0;
}

unsigned char get_Modem_Version(void)
{
	if(format)
		return format->nModem;
	return 0;
}

unsigned char get_Operator_Version(void)
{
	if(format)
		return format->nOperator;
	return 0;
}

unsigned char get_bootMode(void)
{
	get_project();
	if(format)
		return format->nBootMode;
	return 0;
}

//this module just init for creat files to show which version
static ssize_t prjVersion_read_proc(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;
	len = sprintf(page,"%d",get_project());

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

struct file_operations prjVersion_proc_fops = {
	.read = prjVersion_read_proc,
	.write = NULL,
};


static ssize_t pcbVersion_read_proc(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;

	len = sprintf(page,"%d",get_PCB_Version());

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

struct file_operations pcbVersion_proc_fops = {
	.read = pcbVersion_read_proc,
};


static ssize_t operatorName_read_proc(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;

	len = sprintf(page,"%d",get_Operator_Version());

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

struct file_operations operatorName_proc_fops = {
	.read = operatorName_read_proc,
};


static ssize_t modemType_read_proc(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;

	len = sprintf(page,"%d",get_Modem_Version());

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

struct file_operations modemType_proc_fops = {
	.read = modemType_read_proc,
};

static ssize_t secureType_read_proc(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;
	void __iomem *oem_config_base;
	uint32_t secure_oem_config = 0;

	oem_config_base = ioremap(0xA0154 , 10);
	secure_oem_config = __raw_readl(oem_config_base);
	iounmap(oem_config_base);
	printk(KERN_EMERG "lycan test secure_oem_config 0x%x\n", secure_oem_config);

	len = sprintf(page,"%d", secure_oem_config);

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


struct file_operations secureType_proc_fops = {
	.read = secureType_read_proc,
};

#define QFPROM_RAW_SERIAL_NUM 0x000a0128 //different at each platform,please ref boot_images\core\systemdrivers\hwio\scripts\xxx\hwioreg.per

static unsigned int g_SerialID = 0x00; //maybe can use for debug

static ssize_t serialID_read_proc(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;

	len = sprintf(page,"0x%x", g_SerialID);

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


static ssize_t bootMode_read_proc(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;

	len = sprintf(page,"%d", get_bootMode());

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


static ssize_t ramSize_read_proc(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	struct sysinfo i;
	char page[256] = {0};
	int len = 0;

#define K(x) ((x) << (PAGE_SHIFT - 10))
	si_meminfo(&i);
	if (K(i.totalram) <= RAM_2G) {
		len = sprintf(page,"2G");
	} else if (K(i.totalram) <= RAM_3G){
		len = sprintf(page,"3G");
	} else {
		len = sprintf(page,"larger than 3G");
	}

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

struct file_operations serialID_proc_fops = {
	.read = serialID_read_proc,
};

struct file_operations ramSize_proc_fops = {
	.read = ramSize_read_proc,
};

struct file_operations bootMode_proc_fops = {
	.read = bootMode_read_proc,
};

static int __init oppo_project_init(void)
{
	int ret = 0;
	struct proc_dir_entry *pentry;
	void __iomem *serialID_addr = NULL;

	serialID_addr = ioremap(QFPROM_RAW_SERIAL_NUM , 4);
	if(serialID_addr){
		g_SerialID = __raw_readl(serialID_addr);
		iounmap(serialID_addr);
		printk(KERN_EMERG "serialID 0x%x\n", g_SerialID);
	}else
	{
		g_SerialID = 0xffffffff;
	}


	oppoVersion =  proc_mkdir("oppoVersion", NULL);
	if(!oppoVersion) {
		pr_err("can't create oppoVersion proc\n");
		goto ERROR_INIT_VERSION;
	}
	pentry = proc_create("prjVersion", S_IRUGO, oppoVersion, &prjVersion_proc_fops);
	if(!pentry) {
		pr_err("create prjVersion proc failed.\n");
		goto ERROR_INIT_VERSION;
	}
	pentry = proc_create("pcbVersion", S_IRUGO, oppoVersion, &pcbVersion_proc_fops);
	if(!pentry) {
		pr_err("create pcbVersion proc failed.\n");
		goto ERROR_INIT_VERSION;
	}
	pentry = proc_create("operatorName", S_IRUGO, oppoVersion, &operatorName_proc_fops);
	if(!pentry) {
		pr_err("create operatorName proc failed.\n");
		goto ERROR_INIT_VERSION;
	}
	pentry = proc_create("modemType", S_IRUGO, oppoVersion, &modemType_proc_fops);
	if(!pentry) {
		pr_err("create modemType proc failed.\n");
		goto ERROR_INIT_VERSION;
	}
	pentry = proc_create("secureType", S_IRUGO, oppoVersion, &secureType_proc_fops);
	if(!pentry) {
		pr_err("create secureType proc failed.\n");
		goto ERROR_INIT_VERSION;
	}
	pentry = proc_create("serialID", S_IRUGO, oppoVersion, &serialID_proc_fops);
	if(!pentry) {
		pr_err("create serialID proc failed.\n");
		goto ERROR_INIT_VERSION;
	}
	pentry = proc_create("ramSize", S_IRUGO, oppoVersion, &ramSize_proc_fops);
	if(!pentry) {
		pr_err("create ramSize proc failed.\n");
		goto ERROR_INIT_VERSION;
	}
	pentry = proc_create("bootMode", S_IRUGO, oppoVersion, &bootMode_proc_fops);
	if(!pentry) {
		pr_err("create bootMode proc failed.\n");
		goto ERROR_INIT_VERSION;
	}
	return ret;
ERROR_INIT_VERSION:
		remove_proc_entry("oppoVersion", NULL);
		return -ENOENT;
}
arch_initcall(oppo_project_init);

MODULE_DESCRIPTION("OPPO project version");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Joshua <gyx@oppo.com>");
