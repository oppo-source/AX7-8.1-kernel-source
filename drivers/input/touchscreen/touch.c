/***************************************************
 * File:touch.c
 * VENDOR_EDIT
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * Description:
 *             tp dev
 * Version:1.0:
 * Date created:2016/09/02
 * Author: hao.wang@Bsp.Driver
 * TAG: BSP.TP.Init
*/

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/serio.h>
#include "oppo_touchscreen/tp_devices.h"
#include "oppo_touchscreen/touchpanel_common.h"

#include <soc/oppo/oppo_project.h>
#include "touch.h"

#define MAX_LIMIT_DATA_LENGTH         100

struct tp_dev_name tp_dev_names[] = {
     {TP_OFILM, "OFILM"},
     {TP_BIEL, "BIEL"},
     {TP_TRULY, "TRULY"},
     {TP_BOE, "BOE"},
     {TP_G2Y, "G2Y"},
     {TP_TPK, "TPK"},
     {TP_JDI, "JDI"},
     {TP_TIANMA, "TIANMA"},
     {TP_SAMSUNG, "SAMSUNG"},
     {TP_DSJM, "DSJM"},
     {TP_BOE_B8, "BOEB8"},
     {TP_INNOLUX, "INNOLUX"},
     {TP_HIMAX_DPT, "DPT"},
     {TP_AUO, "AUO"},
     {TP_DEPUTE, "DEPUTE"},
     {TP_HUAXING, "HUAXING"},   
     {TP_HLT, "HLT"},
     {TP_UNKNOWN, "UNKNOWN"},
};

#define GET_TP_DEV_NAME(tp_type) ((tp_dev_names[tp_type].type == (tp_type))?tp_dev_names[tp_type].name:"UNMATCH")

int g_tp_dev_vendor = TP_UNKNOWN;
static bool is_tp_type_got_in_match = false;    /*indicate whether the tp type is got in the process of ic match*/

/*
* this function is used to judge whether the ic driver should be loaded
* For incell module, tp is defined by lcd module, so if we judge the tp ic
* by the boot command line of containing lcd string, we can also get tp type.
*/
bool no_flash = false;
bool __init tp_judge_ic_match(char * tp_ic_name)
{
    pr_err("[TP] tp_ic_name = %s \n", tp_ic_name);
    pr_err("[TP] boot_command_line = %s \n", boot_command_line);

    switch(get_project()) {
        case 18031:
        default:
            is_tp_type_got_in_match = true;

            if (0 == strcmp(tp_ic_name, "nova-nt36525")&& strstr(boot_command_line, "tianma_nt36525")) {
                g_tp_dev_vendor = TP_TIANMA;
                return true;
            }

            if (0 == strcmp(tp_ic_name, "sec-s6d7at0")&& strstr(boot_command_line, "boe_lsi7at0")) {
                g_tp_dev_vendor = TP_BOE;
                return true;
            }

            if (0 == strcmp(tp_ic_name, "sec-s6d7at0")&& strstr(boot_command_line, "hlt_lsi7at0")) {
                g_tp_dev_vendor = TP_HLT;
                return true;
            }

            if (0 == strcmp(tp_ic_name, "sec-s6d7at0")&& strstr(boot_command_line, "tianma_lsi7at0")) {
                g_tp_dev_vendor = TP_TIANMA;
                return true;
            }


            if (0 == strcmp(tp_ic_name, "novatek,nf_nt36525") && strstr(boot_command_line, "auo_nt36525") ) {
                g_tp_dev_vendor = TP_AUO;
                no_flash = true;
                return true;
            }

            if (0 == strcmp(tp_ic_name, "novatek,nf_nt36525") && strstr(boot_command_line, "tianma_nt36525") ) {
                g_tp_dev_vendor = TP_TIANMA;
                no_flash = true;
                return true;
            }

            if (0 == strcmp(tp_ic_name, "novatek,nf_nt36525") && strstr(boot_command_line, "boe_nt36525") ) {
                g_tp_dev_vendor = TP_BOE;
                no_flash = true;
                return true;
            }

            if (0 == strcmp(tp_ic_name, "novatek,nf_nt36525") && strstr(boot_command_line, "innolux_nt36525") ) {
                g_tp_dev_vendor = TP_INNOLUX;
                no_flash = true;
                return true;
            }
          
            if (0 == strcmp(tp_ic_name, "ilitek,ili9881h") && strstr(boot_command_line, "innolux_ili9881h") ) {
                g_tp_dev_vendor = TP_INNOLUX;
                no_flash = true;
                return true;
            }

            if (0 == strcmp(tp_ic_name, "novatek,nf_nt36672") && strstr(boot_command_line, "oppo18031csot_nt36672a") ) {
                g_tp_dev_vendor = TP_HUAXING;
                no_flash = true;
                return true;
            }

            if (0 == strcmp(tp_ic_name, "himax,hx83112a_nf") && strstr(boot_command_line, "auo_hx83112a") ) {
                g_tp_dev_vendor = TP_AUO;
                no_flash = true;
                return true;
            }

        break;
    }

    pr_err("Lcd module not found\n");
    return false;
}

/*
* For separate lcd and tp, tp can be distingwished by gpio pins
* different project may have different combination, if needed,
* add your combination with project distingwished by is_project function.
*/
static void tp_get_vendor_via_pin(struct hw_resource *hw_res, struct panel_info *panel_data)
{
    panel_data->tp_type = TP_UNKNOWN;
    return;
}

/*
* If no gpio pins used to distingwish tp module, maybe have other ways(like command line)
* add your way of getting vendor info with project distingwished by is_project function.
*/
static void tp_get_vendor_separate(struct hw_resource *hw_res, struct panel_info *panel_data)
{
    pr_err("[TP]boot_command_line = %s \n", boot_command_line);
    if (strstr(boot_command_line, "tianma")) {
        panel_data->tp_type = TP_TIANMA;
        memcpy(panel_data->manufacture_info.version, "0xbc107b", 8);
        return;
    } else if (strstr(boot_command_line, "boe")) {
        panel_data->tp_type = TP_BOE;
        memcpy(panel_data->manufacture_info.version, "0xbc107c", 8);
        return;
    } else if (strstr(boot_command_line, "truly")) {
        panel_data->tp_type = TP_TRULY;
        memcpy(panel_data->manufacture_info.version, "0xbc107f", 8);
        return;
    }

    panel_data->tp_type = TP_UNKNOWN;
    return;
}

int tp_util_get_vendor(struct hw_resource *hw_res, struct panel_info *panel_data)
{
    char* vendor;
    panel_data->test_limit_name = kzalloc(MAX_LIMIT_DATA_LENGTH, GFP_KERNEL);
    if (panel_data->test_limit_name == NULL) {
        pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
    }

    /*TP is first distingwished by gpio pins, and then by other ways*/
    if (is_tp_type_got_in_match) {
        panel_data->tp_type = g_tp_dev_vendor;
        if (g_tp_dev_vendor == TP_TIANMA) {
            if(no_flash) {
                memcpy(panel_data->manufacture_info.version, "TiMa_nt525_B", (strlen("TiMa_nt525_B") > 12) ? 12 : strlen("TiMa_nt525_B"));
            } else {
                memcpy(panel_data->manufacture_info.version, "TiMa_nt525_f", (strlen("TiMa_nt525_f") > 12) ? 12 : strlen("TiMa_nt525_f_"));
            }

        } else if (g_tp_dev_vendor == TP_AUO) {
            if (strstr(boot_command_line, "auo_hx83112a")) {
                memcpy(panel_data->manufacture_info.version, "XinL_hx112_F", (strlen("XinL_hx112_F") > 12) ? 12 : strlen("XinL_hx112_F"));
            } else {
                if(no_flash) {
                    memcpy(panel_data->manufacture_info.version, "XinL_nt525_A", (strlen("XinL_nt525_A") > 12) ? 12 : strlen("XinL_nt525_A"));
                } else {
                    memcpy(panel_data->manufacture_info.version, "XinL_nt525_f", (strlen("XinL_nt525_f") > 12) ? 12 : strlen("XinL_nt525_f"));
                }
            }
        }  else if (g_tp_dev_vendor == TP_INNOLUX) {
                if (strstr(boot_command_line, "innolux_ili9881h")) {
                    memcpy(panel_data->manufacture_info.version, "QunC_ili81_D", (strlen("QunC_ili81_D") > 12) ? 12 : strlen("QunC_ili81_D"));
                } else {
                    if(no_flash) {
                         memcpy(panel_data->manufacture_info.version, "QunC_nt525_C", (strlen("QunC_nt525_C") > 12) ? 12: strlen("QunC_nt525_C"));
                    } else {
                        memcpy(panel_data->manufacture_info.version, "QunC_nt525_f_", (strlen("QunC_nt525_f_") > 8) ? 8 : strlen("QunC_nt525_f_"));
                    }
                }
        } else if (g_tp_dev_vendor == TP_HUAXING) {
            memcpy(panel_data->manufacture_info.version, "HuaX_nt672_1", (strlen("HuaX_nt672_1") > 12) ? 12 : strlen("HuaX_nt672_1"));
        }  else if (g_tp_dev_vendor == TP_BOE) {
            if (strstr(boot_command_line, "boe_nt36525")) {
                memcpy(panel_data->manufacture_info.version, "Boe_nt525_2_", (strlen("Boe_nt525_2_") > 12) ? 12 : strlen("Boe_nt525_2_"));
            }
        }
        
    } else if (gpio_is_valid(hw_res->id1_gpio) || gpio_is_valid(hw_res->id2_gpio) || gpio_is_valid(hw_res->id3_gpio)) {
        tp_get_vendor_via_pin(hw_res, panel_data);
    } else {
        tp_get_vendor_separate(hw_res, panel_data);
    }

    if (panel_data->tp_type == TP_UNKNOWN) {
        pr_err("[TP]%s type is unknown\n", __func__);
        return 0;
    }

    if (is_project(OPPO_18571) || is_project(OPPO_18171) || is_project(OPPO_18172)) {
        vendor = GET_TP_DEV_NAME(panel_data->tp_type);

        strcpy(panel_data->manufacture_info.manufacture, vendor);
        snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
            "tp/%d/FW_%s_%s.img",
            18171/*get_project()*/, panel_data->chip_name, vendor);

        if (panel_data->test_limit_name) {
            snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
                "tp/%d/LIMIT_%s_%s.img",
                18171/*get_project()*/, panel_data->chip_name, vendor);
        }

        if (strstr(boot_command_line, "auo_hx83112a")) {    //noflash
            panel_data->firmware_headfile.firmware_data = FW_18171_HX83112A_NF_AUO;
            panel_data->firmware_headfile.firmware_size = sizeof(FW_18171_HX83112A_NF_AUO);
        }

        panel_data->manufacture_info.fw_path = panel_data->fw_name;

        pr_info("fw_path: %s\n", panel_data->manufacture_info.fw_path);
        pr_info("[TP]vendor:%s fw:%s limit:%s\n",
                vendor,panel_data->fw_name,
                panel_data->test_limit_name==NULL?"NO Limit":panel_data->test_limit_name);
        return 0;
    } else {
        vendor = GET_TP_DEV_NAME(panel_data->tp_type);

        strcpy(panel_data->manufacture_info.manufacture, vendor);
        snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
            "tp/%d/FW_%s_%s.img",
            18031/*get_project()*/, panel_data->chip_name, vendor);

        if (panel_data->test_limit_name) {
            snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
                "tp/%d/LIMIT_%s_%s.img",
                18031/*get_project()*/, panel_data->chip_name, vendor);
	    }
        panel_data->manufacture_info.fw_path = panel_data->fw_name;
        pr_info("fw_path: %s\n", panel_data->manufacture_info.fw_path);
        pr_info("[TP]vendor:%s fw:%s limit:%s\n",
                vendor, panel_data->fw_name,
                panel_data->test_limit_name==NULL?"NO Limit":panel_data->test_limit_name);
        return 0;
    }
}
