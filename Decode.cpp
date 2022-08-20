#include "stdafx.h"
#include "struct.h"
#include<iostream>
#include<cmath>
#include<fstream>
#include<string>
using namespace std;


/*************************************************************************
DecodeOEM7Message
目的：从OEM7数据文件中读取一条Message，并解码得到MessageID对应的数据

参数：
fp        文件指针
RawData   观测数据、星历等数据

返回值：0=文件结束，1=CRC校验不通过，MsgID_Obs=解码观测值 MsgID_GPSEph/MsgID_BDSEph=解码星历 MshID_ion=解码电离层参数
***************************************************************************/
int DecodeOEM7Message(FILE *fp, RAWDATA *RawData)
{
	unsigned char buf[MAXRAWLEN];
	unsigned short MsgID, MsgLen;

	//找AA 44 12
	while (!feof(fp))
	{
		if (fread(buf + 2, 1, 1, fp) < 1) return 0;
		if (buf[0] == 0xAA && buf[1] == 0x44 && buf[2] == 0x12) break;
		else
		{
			buf[0] = buf[1];
			buf[1] = buf[2];
		}
	}

	//找到同步字，读25个字节
	if (fread(buf + 3, 1, 25, fp) < 25) return 0;

	//解码Message长度
	MsgLen = UI2(buf + 8);

	// 读整条语句，如果读不到，返回0
	if (fread(buf + 28, 1, MsgLen + 4, fp) < (MsgLen + 4)) return 0;

	//CRC校验，不通过，返回1
	if (crc32(buf, 28 + MsgLen) != UI4(buf + 28 + MsgLen))  return 1;

	//解MsgID
	MsgID = UI2(buf + 4);

	//调用解观测值的函数
	switch (MsgID)
	{
	case MsgID_Obs:return DecodeOEM7Obs(buf, RawData);break;
	case MsgID_GPSEph:return DecodeGPSEph(buf, RawData);break;
	case MsgID_BDSEph:return DecodeBDSEph(buf, RawData);break;
	case MsgID_ion:return DecodeIon(buf, RawData);break;
	default:break;
	}
	//return 4;
}

/* CRC校验 */
unsigned int crc32(const unsigned char *buff, int len)
{
	int i, j;
	unsigned int crc = 0;

	for (i = 0; i < len; i++)
	{
		crc ^= buff[i];
		for (j = 0; j < 8; j++)
		{
			if (crc & 1) crc = (crc >> 1) ^ POLYCRC32;
			else crc >>= 1;
		}
	}
	return crc;
}

/*************************************************************************
DecodeOEM7Obs
目的：从OEM7数据文件中提取观测数据

参数：
buf       观测数据缓冲区
RawData   观测数据、星历等数据

返回值：43=观测值解码完成
***************************************************************************/
int DecodeOEM7Obs(unsigned char buf[], RAWDATA *RawData)
{
	int i, j, ObsNum, Prn, ChTS, SystemType, SigType, Freq, index_GPS, index_BDS, same_flag;
	NAVSYS Sys;
	unsigned char *p;
	p = &buf[0] + 28;
	index_GPS = 0; //已经存储的卫星个数
	index_BDS = 0;
	//解码得到观测值数量
	ObsNum = UI4(p);
	RawData->Obs.ObsTime.Week = UI2(buf + 14);
	RawData->Obs.ObsTime.SecofWeek = UI4(buf + 16) / 1000.0;

	//循环，得到下一组观测值
	for (i = 0; i < ObsNum; i++, p = p + 44)
	{
		//Channel Tracking Status，得到卫星系统（其他系统continue）
		Prn = UI2(p + 4);
		ChTS = UI4(p + 44);
		SystemType = (ChTS >> 16) & 0x07;
		if (SystemType == 0) Sys = GPS;
		else if (SystemType == 4) Sys = BDS;
		else continue;

		//信号频率，Freq=0，1
		SigType = (ChTS >> 21) & 0x1F;

		if (Sys == GPS)
		{
			if (SigType == 0) Freq = 0;       //L1
			else if (SigType == 9) Freq = 1;  //L2
			else continue;
		}
		else if (Sys == BDS)
		{
			if (SigType == 0 || SigType == 4)
				Freq = 0;  //B1
			else if (SigType == 2 || SigType == 6)
				Freq = 1;  //B3
			else continue;
		}

		same_flag = 0;
		if (Sys == GPS) 
		{
			for (j = 0; j < index_GPS; j++) //判断当前的卫星Prn是否已经被存储在数组
			{
				if (Prn == RawData->Obs.GPS_Sat[j].Prn)
				{
					same_flag = 1; //数组中存储过该卫星数据
					RawData->Obs.GPS_Sat[j].Sys = Sys;
					RawData->Obs.GPS_Sat[j].Psr[Freq] = R8(p + 8);
					RawData->Obs.GPS_Sat[j].sigma_psr[Freq] = R4(p + 16);
					RawData->Obs.GPS_Sat[j].Adr[Freq] = R8(p + 20);
					RawData->Obs.GPS_Sat[j].sigma_adr[Freq] = R4(p + 28);
					RawData->Obs.GPS_Sat[j].Dop[Freq] = R4(p + 32);
					RawData->Obs.GPS_Sat[j].snr[Freq] = R4(p + 36);
					RawData->Obs.GPS_Sat[j].LockTime[Freq] = R4(p + 40);
					break;
				}
				else same_flag = 0; //数组中未存储过该卫星数据
			}
			if (same_flag == 0)
			{
				if (Prn > GPSsatnum)
					continue;
				RawData->Obs.GPS_Sat[index_GPS].Prn = UI2(p + 4);
				RawData->Obs.GPS_Sat[index_GPS].Sys = Sys;
				RawData->Obs.GPS_Sat[index_GPS].Psr[Freq] = R8(p + 8);
				RawData->Obs.GPS_Sat[index_GPS].sigma_psr[Freq] = R4(p + 16);
				RawData->Obs.GPS_Sat[index_GPS].Adr[Freq] = R8(p + 20);
				RawData->Obs.GPS_Sat[index_GPS].sigma_adr[Freq] = R4(p + 28);
				RawData->Obs.GPS_Sat[index_GPS].Dop[Freq] = R4(p + 32);
				RawData->Obs.GPS_Sat[index_GPS].snr[Freq] = R4(p + 36);
				RawData->Obs.GPS_Sat[index_GPS].LockTime[Freq] = R4(p + 40);
				index_GPS++;
				RawData->Obs.GPS_SatNum = index_GPS;
			}
		}

		else if (Sys == BDS) 
		{
			for (j = 0; j < index_BDS; j++) //判断当前的卫星Prn是否已经被存储在数组
			{
				if (Prn == RawData->Obs.BDS_Sat[j].Prn)
				{
					same_flag = 1; //数组中存储过该卫星数据
					RawData->Obs.BDS_Sat[j].Sys = Sys;
					RawData->Obs.BDS_Sat[j].Psr[Freq] = R8(p + 8);
					RawData->Obs.BDS_Sat[j].sigma_psr[Freq] = R4(p + 16);
					RawData->Obs.BDS_Sat[j].Adr[Freq] = R8(p + 20);
					RawData->Obs.BDS_Sat[j].sigma_adr[Freq] = R4(p + 28);
					RawData->Obs.BDS_Sat[j].Dop[Freq] = R4(p + 32);
					RawData->Obs.BDS_Sat[j].snr[Freq] = R4(p + 36);
					RawData->Obs.BDS_Sat[j].LockTime[Freq] = R4(p + 40);
					break;
				}
				else same_flag = 0; //数组中未存储过该卫星数据
			}
			if (same_flag == 0)
			{
				if (Prn > BDSsatnum)
					continue;
				RawData->Obs.BDS_Sat[index_BDS].Prn = UI2(p + 4);
				RawData->Obs.BDS_Sat[index_BDS].Sys = Sys;
				RawData->Obs.BDS_Sat[index_BDS].Psr[Freq] = R8(p + 8);
				RawData->Obs.BDS_Sat[index_BDS].sigma_psr[Freq] = R4(p + 16);
				RawData->Obs.BDS_Sat[index_BDS].Adr[Freq] = R8(p + 20);
				RawData->Obs.BDS_Sat[index_BDS].sigma_adr[Freq] = R4(p + 28);
				RawData->Obs.BDS_Sat[index_BDS].Dop[Freq] = R4(p + 32);
				RawData->Obs.BDS_Sat[index_BDS].snr[Freq] = R4(p + 36);
				RawData->Obs.BDS_Sat[index_BDS].LockTime[Freq] = R4(p + 40);
				index_BDS++;
				RawData->Obs.BDS_SatNum = index_BDS;
			}
		}
	}
	return 43;
}

/*************************************************************************
DecodeGPSEph
目的：从OEM7数据文件中提取GPS星历

参数：
buf       观测数据缓冲区
RawData   观测数据、星历等数据
***************************************************************************/
int DecodeGPSEph(unsigned char buf[], RAWDATA *RawData)
{
	int prn;
	unsigned char *p;
	p = &buf[0] + 28;
	prn = UI4(p);
	if (prn > 0 && prn < GPSsatnum + 1)
	{
		RawData->GPSEph[prn - 1].Sys = GPS;
		RawData->GPSEph[prn - 1].PRN = prn;
		RawData->GPSEph[prn - 1].iodc = UI4(p + 160);
		RawData->GPSEph[prn - 1].tow = R8(p + 4);
		RawData->GPSEph[prn - 1].toe = R8(p + 32);
		RawData->GPSEph[prn - 1].A = R8(p + 40);
		RawData->GPSEph[prn - 1].deltaN = R8(p + 48);
		RawData->GPSEph[prn - 1].M0 = R8(p + 56);
		RawData->GPSEph[prn - 1].ecc = R8(p + 64);
		RawData->GPSEph[prn - 1].w = R8(p + 72);
		RawData->GPSEph[prn - 1].cuc = R8(p + 80);
		RawData->GPSEph[prn - 1].cus = R8(p + 88);
		RawData->GPSEph[prn - 1].crc = R8(p + 96);
		RawData->GPSEph[prn - 1].crs = R8(p + 104);
		RawData->GPSEph[prn - 1].cic = R8(p + 112);
		RawData->GPSEph[prn - 1].cis = R8(p + 120);
		RawData->GPSEph[prn - 1].toc = R8(p + 164);
		RawData->GPSEph[prn - 1].af0 = R8(p + 180);
		RawData->GPSEph[prn - 1].af1 = R8(p + 188);
		RawData->GPSEph[prn - 1].af2 = R8(p + 196);
		RawData->GPSEph[prn - 1].week = UI4(p + 24);
		RawData->GPSEph[prn - 1].i0 = R8(p + 128);
		RawData->GPSEph[prn - 1].IDOT = R8(p + 136);
		RawData->GPSEph[prn - 1].omega_0 = R8(p + 144);
		RawData->GPSEph[prn - 1].omega_dot = R8(p + 152);
		RawData->GPSEph[prn - 1].tgd[0] = R8(p + 172);
	}
	return 7;
}

/*************************************************************************
DecodeBDSEph
目的：从OEM7数据文件中提取BDS星历

参数：
buf       观测数据缓冲区
RawData   观测数据、星历等数据
***************************************************************************/
int DecodeBDSEph(unsigned char buf[], RAWDATA *RawData)
{
	int prn;
	unsigned char *p;
	p = &buf[0] + 28;
	prn = UI4(p);
	if (prn > 0 && prn < BDSsatnum + 1)
	{
		RawData->BDSEph[prn - 1].Sys = BDS;
		RawData->BDSEph[prn - 1].PRN = prn;
		RawData->BDSEph[prn - 1].week = UI4(p + 4);
		RawData->BDSEph[prn - 1].toc = UI4(p + 40);
		RawData->BDSEph[prn - 1].af0 = R8(p + 44);
		RawData->BDSEph[prn - 1].af1 = R8(p + 52);
		RawData->BDSEph[prn - 1].af2 = R8(p + 60);
		RawData->BDSEph[prn - 1].toe = UI4(p + 72);
		RawData->BDSEph[prn - 1].A = R8(p + 76)*R8(p + 76);
		RawData->BDSEph[prn - 1].ecc = R8(p + 84);
		RawData->BDSEph[prn - 1].w = R8(p + 92);
		RawData->BDSEph[prn - 1].deltaN = R8(p + 100);
		RawData->BDSEph[prn - 1].M0 = R8(p + 108);
		RawData->BDSEph[prn - 1].cuc = R8(p + 148);
		RawData->BDSEph[prn - 1].cus = R8(p + 156);
		RawData->BDSEph[prn - 1].crc = R8(p + 164);
		RawData->BDSEph[prn - 1].crs = R8(p + 172);
		RawData->BDSEph[prn - 1].cic = R8(p + 180);
		RawData->BDSEph[prn - 1].cis = R8(p + 188);
		RawData->BDSEph[prn - 1].i0 = R8(p + 132);
		RawData->BDSEph[prn - 1].IDOT = R8(p + 140);
		RawData->BDSEph[prn - 1].omega_0 = R8(p + 116);
		RawData->BDSEph[prn - 1].omega_dot = R8(p + 124);
		RawData->BDSEph[prn - 1].tgd[0] = R8(p + 20); //B1
		RawData->BDSEph[prn - 1].tgd[1] = R8(p + 28); //B2
	}
	return 1696;
}

/*************************************************************************
DecodeIon
目的：从OEM7数据文件中提取电离层参数

参数：
buf       观测数据缓冲区
RawData   观测数据、星历等数据
***************************************************************************/
int DecodeIon(unsigned char buf[], RAWDATA *RawData)
{
	unsigned char *p;
	p = &buf[0] + 28;
	RawData->ion.a0 = R8(p);
	RawData->ion.a1 = R8(p + 8);
	RawData->ion.a2 = R8(p + 16);
	RawData->ion.a3 = R8(p + 24);
	RawData->ion.b0 = R8(p + 32);
	RawData->ion.b1 = R8(p + 40);
	RawData->ion.b2 = R8(p + 48);
	RawData->ion.b3 = R8(p + 56);
	return 8;
}