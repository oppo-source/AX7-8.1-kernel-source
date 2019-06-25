/************************************************************************************
** File:  \android\kernel\msm-4.9\drivers\power\reset\kernel_authenticate_fastboot.h
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd
** 
** Description: authenticate device allow fastboot mode
** 
** Version: 1.0
** Date created: 2018-08-30
** Author: Yichun.Chen  PSW.BSP.CHG
** 
** --------------------------- Revision History: ------------------------------------------------------------
** version     <author>          <data>                     <desc>
**   1.0      Yichun.Chen      2018-08-30        create for authenticate fastboot
************************************************************************************************************/

#ifndef __KERNEL_AUTHENTICATE_FASTBOOT_H__
#define __KERNEL_AUTHENTICATE_FASTBOOT_H__

struct kernel_cpuid_desc {
	char * name;
};

static struct kernel_cpuid_desc kernel_authenticate_cpuid[] = {
	{"890eeff3"},
	{"a8f1ecc6"},
	{NULL},
};

#endif

