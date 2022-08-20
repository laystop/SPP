#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>

#define CGCS2000_a 6378137
#define CGCS2000_f 1/298.257222101
#define PI 3.14159265357989

#define MAXRAWLEN 40960
#define GPSsatnum 32
#define BDSsatnum 63
#define POLYCRC32 0xEDB88320u
#define MsgID_Obs 43
#define MsgID_GPSEph 7
#define MsgID_BDSEph 1696
#define MsgID_ion 8
enum NAVSYS { GPS, BDS };

#define SpeedofLight 2.99792458e8
#define GPS_miu 3.986005e14
#define GPS_omegae_dot 7.2921151467e-5
#define BDS_miu 3.986004418e14
#define BDS_omegae_dot 7.2921150e-5

#define GPS_L1 1.57542e9
#define GPS_L2 1.2276e9
#define BDS_B1 1.561098e9
#define BDS_B3 1.26852e9
#define EarthRotation 7.292115e-5 

/*通用时间定义*/
struct COMMONTIME {
	int Year;
	int Month;
	int Day;
	int Hour;
	int Minute;
	double Second;
};

/*简化儒略日定义*/
struct MJDTIME {
	int Days;
	double FracDay;
	MJDTIME()
	{
		Days = 0;
		FracDay = 0.0;
	}
};

/*GPS时间定义*/
struct GPSTIME {
	int Week;
	double SecofWeek;

	GPSTIME()
	{
		Week = 0;
		SecofWeek = 0.0;
	}
};

/*笛卡尔坐标系定义*/
struct XYZ {
	double x;
	double y;
	double z;
};

/*大地坐标系定义*/
struct BLH {
	double b;
	double l;
	double h;
};

/*数据解码*/
struct SAT {
	unsigned short Prn;
	NAVSYS Sys;
	double Psr[2];
	double Adr[2]; // BDS, Adr1=B1, Adr2=B3; GPS, Adr1=L1, Adr2=L2
	double Dop[2];
	double snr[2], LockTime[2];
	float sigma_psr[2], sigma_adr[2];
};

struct OBS {
	SAT GPS_Sat[GPSsatnum], BDS_Sat[BDSsatnum];
	GPSTIME ObsTime;
	int GPS_SatNum, BDS_SatNum;
};

struct EPHEM {
	unsigned short PRN, iodc;
	NAVSYS Sys;
	GPSTIME EphTime;
	double tow, toe, A, deltaN, M0;
	double ecc, w, cuc, cus, crc, crs, cic, cis;
	double Iup0, Idown0, w0, wdot, toc, tgd[2], i0;
	double af0, af1, af2, URA, IDOT, omega_0, omega_dot;
	unsigned long IODE[2], Zweek, week;
};

struct IONUTC{
	double a0, a1, a2, a3;
	double b0, b1, b2, b3;
};

struct RAWDATA {
	OBS Obs;
	EPHEM GPSEph[GPSsatnum], BDSEph[BDSsatnum];
	IONUTC ion;
};

/*计算卫星位置速度*/
struct SAT_POSnVEL {
	XYZ SatPos; //卫星位置
	double v_x, v_y, v_z; //卫星速度
	double delta_tsv, delta_tsv_dot; //卫星钟差和钟速
	GPSTIME SignalTrTime; //卫星发射时间（GPS BDS都用GPST）
	double Psr_InonCorrect, TroCorrect; //电离层、对流层改正
	int iono_flag;
	double Element; //高度角
};

struct CALCULATION {
	SAT_POSnVEL GPS_POSnVEL[GPSsatnum], BDS_POSnVEL[BDSsatnum];
};

struct RECEIVE {
	XYZ RecPos;
	XYZ RecVel;
	BLH RecBlh;
	GPSTIME time;
	double delta_t_receive[2], delta_t_dot[2];
	double sigma, PDOP;
	int SatNum;
};

struct Ipinfo {
	char IP[15] = {"47.114.134.129"};
	unsigned short Port=6000;
};