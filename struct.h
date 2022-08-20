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

/*ͨ��ʱ�䶨��*/
struct COMMONTIME {
	int Year;
	int Month;
	int Day;
	int Hour;
	int Minute;
	double Second;
};

/*�������ն���*/
struct MJDTIME {
	int Days;
	double FracDay;
	MJDTIME()
	{
		Days = 0;
		FracDay = 0.0;
	}
};

/*GPSʱ�䶨��*/
struct GPSTIME {
	int Week;
	double SecofWeek;

	GPSTIME()
	{
		Week = 0;
		SecofWeek = 0.0;
	}
};

/*�ѿ�������ϵ����*/
struct XYZ {
	double x;
	double y;
	double z;
};

/*�������ϵ����*/
struct BLH {
	double b;
	double l;
	double h;
};

/*���ݽ���*/
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

/*��������λ���ٶ�*/
struct SAT_POSnVEL {
	XYZ SatPos; //����λ��
	double v_x, v_y, v_z; //�����ٶ�
	double delta_tsv, delta_tsv_dot; //�����Ӳ������
	GPSTIME SignalTrTime; //���Ƿ���ʱ�䣨GPS BDS����GPST��
	double Psr_InonCorrect, TroCorrect; //����㡢���������
	int iono_flag;
	double Element; //�߶Ƚ�
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