/* 
 *
 * yixue.ge add for oppo project
 *
 *
 */
#ifndef _OPPO_PROJECT_H_
#define _OPPO_PROJECT_H_

enum{
	HW_VERSION__UNKNOWN           = 0,

    /*For common project definition*/
	HW_VERSION__10,
	HW_VERSION__11,
	HW_VERSION__12,
	HW_VERSION__13,
	HW_VERSION__14,
	HW_VERSION__15, 
	HW_VERSION__16,

    /*For 18031 project special definition*/
	OPPO_18031_HW_T0                = 1,  /* T0*/
	OPPO_18031_HW_T1                = 2,  /* T1*/
	OPPO_18031_HW_T2                = 3,  /* T2*/
	OPPO_18031_HW_EVT               = 4,  /* EVT*/
	OPPO_18031_HW_DVT               = 5,  /* DVT*/
	OPPO_18031_HW_PVT               = 6,  /* PVT*/
	OPPO_18031_HW_MP                = 7,  /* MP*/
};


enum{
	RF_VERSION__UNKNOWN           = 0,

    /*For common project definition*/
	RF_VERSION__11,		
	RF_VERSION__12,		
	RF_VERSION__13,
	RF_VERSION__21,		
	RF_VERSION__22,		
	RF_VERSION__23,
	RF_VERSION__31,		
	RF_VERSION__32,		
	RF_VERSION__33,

	/*For 18031 project special definition*/
	OPPO_18031_MODEM_ASIA_2                      = 7,    /* 亚太版 (SKY PA) 2.4Gwifi */
	OPPO_18031_MODEM_ASIA_2_NEW_DPDT             = 1,    /* 亚太版 (SKY PA) 2.4Gwifi--新DPDT */
	OPPO_18031_MODEM_ASIA_VIETNAM_NEW_DPDT       = 0,    /* 亚太版之越南专版--新DPDT */
	OPPO_18031_MODEM_ASIA_VIETNAM                = 5,    /* 亚太版之越南专版 */
	OPPO_18031_MODEM_INDIA                       = 14,   /* 亚太简版/印度版 */
	OPPO_18031_MODEM_INDIA_NEW_DPDT              = 2,    /* 亚太简版--新DPDT */
	OPPO_18031_MODEM_FULLBAND_2                  = 13,   /* 全频段(QCM PA)  */
	OPPO_18031_MODEM_FOREIGN                     = 12,   /* 海外运营商 */
	OPPO_18031_MODEM_FOREIGN_JAPAN               = 6,    /* 海外运营商之日本运营商专版 */
	OPPO_18031_MODEM_CHINA_MOBILE_1              = 3,    /* 移动定制(SKY PA) */
	OPPO_18031_MODEM_CHINA_MOBILE_2              = 11,   /* 移动定制(QCM PA) */
	OPPO_18031_MODEM_ALLNET_1                    = 10,   /* 全网通(QCM PA) */
	OPPO_18031_MODEM_ALLNET_2                    = 8,    /* 全网通(SKY PA) */
	OPPO_18031_MODEM_ALLNET_3                    = 15,   /* 全网通(新5G FEM+新DPDT(SKY PA-21)) */
	OPPO_18031_MODEM_ALLNET_4                    = 4,    /* 全网通(新DPDT(SKY PA-21))  */
	OPPO_18031_MODEM_TELECOM                     = 9,    /* 电信NFC版本 */


	/*For 18171 project special definition*/
	OPPO_18171_MODEM_ALLNET_1                    = 63,   /* 国内公开/全网通 */
	OPPO_18171_MODEM_ASIA_COSTDOWN_1             = 62,   /* 亚太简版 */
	OPPO_18171_MODEM_ASIA_1                      = 61,   /* 亚太版 */
	OPPO_18171_MODEM_CHINA_MOBILE_1              = 60,   /* 中国移动版 */
	OPPO_18171_MODEM_CHINA_MOBILE_2              = 56,   /* 中国移动版(QM56022)*/
	OPPO_18171_MODEM_ALLNET_1_5GWIFI_FEM         = 59,   /* 国内公开/全网通--5G WIFI_FEM */
	OPPO_18171_MODEM_VIETNAM_1                   = 58,   /* 越南4+64 */
	OPPO_18171_MODEM_VIETNAM_2                   = 57,   /* 越南4+32 */
	OPPO_18171_MODEM_FULLBAND_1                  = 33,   /* 全频段版 */
	OPPO_18171_MODEM_TAIWAN_MACAO_1              = 34,   /* 台澳版/运营商版 */
	OPPO_18171_MODEM_ASIA_COSTDOWN_1_VANCHIP     = 55,   /* 台澳版/运营商版 */

};


#define GET_PCB_VERSION() (get_PCB_Version())
#define GET_PCB_VERSION_STRING() (get_PCB_Version_String())

#define GET_MODEM_VERSION() (get_Modem_Version())
#define GET_OPERATOR_VERSION() (get_Operator_Version())



enum OPPO_PROJECT {
	OPPO_UNKOWN = 0,
	OPPO_16017 = 16017,
	OPPO_16027 = 16027,
	OPPO_16061 = 16061,
	OPPO_17001 = 17001,
	OPPO_17151 = 17151,

	OPPO_18031 = 18031,
	OPPO_18032 = 18032,
	OPPO_18301 = 18301,

	OPPO_18171 = 18171,
	OPPO_18172 = 18172,
	OPPO_18571 = 18571,
};

enum OPPO_OPERATOR {
    OPERATOR_UNKOWN                               = 0,

    /*For common project definition*/
	OPERATOR_OPEN_MARKET 		                  = 1,
	OPERATOR_CHINA_MOBILE 		                  = 2,
	OPERATOR_CHINA_UNICOM 		                  = 3,
	OPERATOR_CHINA_TELECOM 		                  = 4,
	OPERATOR_FOREIGN 			                  = 5,
	OPERATOR_FOREIGN_WCDMA 		                  = 6,
	OPERATOR_FOREIGN_RESERVED                     = 7,
	OPERATOR_ALL_CHINA_CARRIER	                  = 8,    //instead of TELECOM CARRIER because of history Tong.han@Bsp.Group.Tp add for all china carrier phone, 2015/03/23
	OPERATOR_ALL_CHINA_CARRIER_MOBILE	          = 9,    //rendong.shi@Bsp.Group.Tp add for all china carrier MOBILE phone, 2016/01/07
	OPERATOR_ALL_CHINA_CARRIER_UNICOM             = 10,   //rendong.shi@Bsp.Group.Tp add for all china carrier UNICOM  phone, 2016/01/07
	OPERATOR_FOREIGN_TAIWAN			              = 102,
	OPERATOR_FOREIGN_TAIWAN_RFMD	              = 103,  //lile@Prd6.BasicDrv.CDT, 2016/10/12, add for match new RFMD TAIWAN mainborad
	OPERATOR_FOREIGN_INDIA			              = 104,
	OPERATOR_FOREIGN_ALLNET			              = 105,
	OPERATOR_FOREIGN_ASIA			              = 106,
	OPERATOR_FOREIGN_ASIA_RFMD		              = 107,  //lile@Prd6.BasicDrv.CDT, 2016/10/12, add for match new RFMD ASIA mainborad
	OPERATOR_FOREIGN_VODAFONE_SKY	              = 108,  //libin@Prd6.BasicDrv.CDT, 2016/11/18, add for match new VODAFONE SKY mainborad
	OPERATOR_FOREIGN_VODAFONE_RFMD	              = 109,  //libin@Prd6.BasicDrv.CDT, 2016/11/18, add for match new VODAFONE RFMD mainborad
	OPERATOR_FOREIGN_ASIA_INDIA		              = 110,  //Mofei@Prd6.BasicDrv.CDT, 2016/12/30, add for match new ASIA and India mainborad
	OPERATOR_FOREIGN_ASIA_WIFI		              = 111,  //Mofei@Prd6.BasicDrv.CDT, 2016/12/30, add for match new ASIA with deffrent wifi mainborad

	/*For 18031 project special definition*/
	OPPO_18031_OPERATOR_ASIA                      = 5,     /* 亚太版 */
	OPPO_18031_OPERATOR_INDIA                     = 6,     /* 亚太简版 */
	OPPO_18031_OPERATOR_FULLBAND                  = 7,     /* 全频段 */
	OPPO_18031_OPERATOR_FOREIGN                   = 8,     /* 海外运营商/日本运营商专版 */
	OPPO_18031_OPERATOR_CHINA_MOBILE              = 9,     /* CMCC定制 */
	OPPO_18031_OPERATOR_ALLNET                    = 10,    /* 全网通 */
 	OPPO_18031_OPERATOR_TELECOM                   = 11,    /* 电信版 */
  	OPPO_18031_OPERATOR_ASIA_VIETNAM              = 12,    /* 亚太版之越南专版 */

	/*For 18171 project special definition*/
	OPPO_18171_OPERATOR_ASIA                      = 5,     /* 亚太版 18571 */
	OPPO_18171_OPERATOR_ASIA_COSTDOWN             = 6,     /* 亚太简版 18572 */
	OPPO_18171_OPERATOR_FULLBAND                  = 7,     /* 全频段 18575 */
	OPPO_18171_OPERATOR_CHINA_MOBILE              = 9,     /* CMCC定制 18172 */
	OPPO_18171_OPERATOR_ALLNET                    = 10,    /* 国内公开版/全网通 18171 */
	OPPO_18171_OPERATOR_TAIWAN_MACAO              = 13,    /* 台澳版 18573 */
	OPPO_18171_OPERATOR_VIETNAM_1                 = 14,    /* 越南版1(4+64) 18576 */
	OPPO_18171_OPERATOR_VIETNAM_2                 = 15,    /* 越南版2(4+32) 18577 */
};


typedef enum OPPO_PROJECT OPPO_PROJECT;

typedef struct
{
  unsigned int                  nProject;
  unsigned char                 nModem;
  unsigned char                 nOperator;
  unsigned char                 nPCBVersion;
  unsigned char                 nBootMode;
} ProjectInfoCDTType;

#ifdef CONFIG_OPPO_COMMON_SOFT
unsigned int init_project_version(void);
unsigned int get_project(void);
unsigned int is_project(OPPO_PROJECT project );
unsigned char get_PCB_Version(void);
unsigned char get_Modem_Version(void);
unsigned char get_Operator_Version(void);
#else
unsigned int init_project_version(void) { return 0;}
unsigned int get_project(void) { return 0;}
unsigned int is_project(OPPO_PROJECT project ) { return 0;}
unsigned char get_PCB_Version(void) { return 0;}
unsigned char get_Modem_Version(void) { return 0;}
unsigned char get_Operator_Version(void) { return 0;}
#endif
#endif
