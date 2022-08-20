#include "stdafx.h"
#include "struct.h"
#include<iostream>
#include<cmath>
#include<fstream>
#include<string>
#include<iomanip>

using namespace std;

/**************************************************************
DeleteZero
目的：删除矩阵中等于0的元素

参数：
arr   待处理的数组
n     数组维数
**************************************************************/
void DeleteZero(double *arr, int n)
{
	//直接从头到尾遍历整个数组，是0就将n减1，同时数组下表往前移
	for (int i = 0; i < n; i++)
	{
		if (abs(arr[i]) < 1e-10)
		{
			//将所有元素向前移
			int j = i;
			for (; j < n - 1; j++)
			{
				arr[j] = arr[j + 1];
			}
			//将要判断的元素的下标前移
			i = i - 1;
			//非0元素个数减1
			n--;
		}
	}
}

/**************************************************************
SPP
目的：单点定位

参数：
RawData              原始数据
Rcv                  接收机计算结果
Calculation          计算结果

返回值：0=卫星数目不足，无法计算  1=正常
***************************************************************/
int SPP(RAWDATA *RawData, RECEIVE *Rcv, CALCULATION *Calculation)
{
	int i, j, k, prn, n_GPS, n_BDS, n;
	double azimuth, element, iono_correct, tropo_correct, rho, rho_dot, position, position_new, tao;
	double v[30], B[150], B_T[150], P_value[30], P[900], BTP[150], w[30], BB_T[25], BB_T1[25], BBTP[150], BB_T2[150], BB_T3[150], x[5], v_T[30]; //最多接收30颗卫星
	double  Vv[30], VB[150], VB_T[150], VP[900], VBTP[150], Vw[30], VBB_T[25], VBB_T1[25], VBBTP[150], VBB_T2[150], Vx[5], Vv_T[30];
	double vtpv[1];
	double sigma, PDOP;
	XYZ RecTemp;
	double delta_t_receive[2], delta_t_dot[2];

	azimuth = 0.0;
	element = 0.0;
	RecTemp.x = Rcv->RecPos.x;
	RecTemp.y = Rcv->RecPos.y;
	RecTemp.z = Rcv->RecPos.z;
	position = sqrt(pow(RecTemp.x, 2) + pow(RecTemp.y, 2) + pow(RecTemp.z, 2));
	delta_t_receive[0] = Rcv->delta_t_receive[0];
	delta_t_receive[1] = Rcv->delta_t_receive[1];
	delta_t_dot[0] = Rcv->delta_t_dot[0];
	delta_t_dot[1] = Rcv->delta_t_dot[1];
	memset(P, 0, sizeof(P)); //对高度角加权
	Rcv->time = RawData->Obs.ObsTime;

	//计算卫星信号发射时间
	GPS_SignalTransmit(RawData, Calculation);
	BDS_SignalTransmit(RawData, Calculation);

	//单点定位
	for (j = 0;j < 100;j++)
	{
		IonoFree(&RecTemp, RawData, Calculation);
		n_GPS = 0; //可用于计算的卫星数量（同时有星历、观测数据且星历未过期）
		for (i = 0;i < RawData->Obs.GPS_SatNum;i++) //GPS
		{
			prn = RawData->Obs.GPS_Sat[i].Prn;
			tao = RawData->Obs.ObsTime.SecofWeek - Calculation->GPS_POSnVEL[prn - 1].SignalTrTime.SecofWeek;
			if (Calculation->GPS_POSnVEL[prn - 1].iono_flag == 1) //双频伪距之差过大
				continue;
			if (GPS_SatPositionVelocity(&Calculation->GPS_POSnVEL[prn - 1].SignalTrTime, prn, Calculation, RawData, tao) == 2) //计算卫星位置速度
				continue;
			ComputeAzimuthElement(CGCS2000_a, CGCS2000_f, &RecTemp, &Calculation->GPS_POSnVEL[prn - 1].SatPos, &azimuth, &element); //计算高度角方位角（更新）

			if (abs(Rcv->RecPos.x) < 1e-10 || element > 15 / 180.0*PI)
			{
				/////////
				if (abs(Rcv->RecPos.x) < 1e-10 || azimuth > 0)
				{
					tropo_correct = Hopfield(&RecTemp, element); //计算对流层改正（更新）
					Calculation->GPS_POSnVEL[prn - 1].TroCorrect = tropo_correct;
					Calculation->GPS_POSnVEL[prn - 1].Element = element / PI*180.0;

					rho = sqrt(pow((Calculation->GPS_POSnVEL[prn - 1].SatPos.x - RecTemp.x), 2) + pow((Calculation->GPS_POSnVEL[prn - 1].SatPos.y - RecTemp.y), 2) + pow((Calculation->GPS_POSnVEL[prn - 1].SatPos.z - RecTemp.z), 2));

					B[n_GPS * 5] = (RecTemp.x - Calculation->GPS_POSnVEL[prn - 1].SatPos.x) / rho;
					B[n_GPS * 5 + 1] = (RecTemp.y - Calculation->GPS_POSnVEL[prn - 1].SatPos.y) / rho;
					B[n_GPS * 5 + 2] = (RecTemp.z - Calculation->GPS_POSnVEL[prn - 1].SatPos.z) / rho;
					B[n_GPS * 5 + 3] = 1;
					B[n_GPS * 5 + 4] = 0;
					//P_value[n_GPS] = sin(element)*sin(element);
					//P_value[n_GPS] = 1 / (0.03*0.03 + 0.04*0.04 / sin(element) / sin(element));
					P_value[n_GPS] = 1 / (pow(0.13 + 0.56*exp(-element / PI * 180 / 10), 2));

					if (Calculation->GPS_POSnVEL[prn - 1].iono_flag == 2)
						w[n_GPS] = Calculation->GPS_POSnVEL[prn - 1].Psr_InonCorrect - rho - delta_t_receive[0] + SpeedofLight*Calculation->GPS_POSnVEL[prn - 1].delta_tsv - tropo_correct;
					else if (RawData->Obs.GPS_Sat[i].Psr[0] < 1)
					{
						iono_correct = Klobuchar(&(RawData->Obs.ObsTime), &RecTemp, &(Calculation->GPS_POSnVEL[prn - 1].SatPos), azimuth, element, RawData);
						w[n_GPS] = RawData->Obs.GPS_Sat[i].Psr[1] - rho - delta_t_receive[0] + SpeedofLight*Calculation->GPS_POSnVEL[prn - 1].delta_tsv - iono_correct - tropo_correct;
					}
					else if (RawData->Obs.GPS_Sat[i].Psr[1] < 1)
					{
						iono_correct = Klobuchar(&(RawData->Obs.ObsTime), &RecTemp, &(Calculation->GPS_POSnVEL[prn - 1].SatPos), azimuth, element, RawData);
						w[n_GPS] = RawData->Obs.GPS_Sat[i].Psr[0] - rho - delta_t_receive[0] + SpeedofLight*Calculation->GPS_POSnVEL[prn - 1].delta_tsv - iono_correct - tropo_correct;
					}
					else
					{
						iono_correct = Klobuchar(&(Calculation->GPS_POSnVEL[prn - 1].SignalTrTime), &RecTemp, &(Calculation->GPS_POSnVEL[prn - 1].SatPos), azimuth, element, RawData);
						w[n_GPS] = (RawData->Obs.GPS_Sat[i].Psr[0] + RawData->Obs.GPS_Sat[i].Psr[1]) / 2 - rho - delta_t_receive[0] + SpeedofLight*Calculation->GPS_POSnVEL[prn - 1].delta_tsv - iono_correct - tropo_correct;
					}
					n_GPS++;
				}
			}
		}

		n_BDS = 0;
		for (i = 0;i < RawData->Obs.BDS_SatNum;i++) //BDS
		{
			prn = RawData->Obs.BDS_Sat[i].Prn;
			tao = RawData->Obs.ObsTime.SecofWeek - Calculation->BDS_POSnVEL[prn - 1].SignalTrTime.SecofWeek;
			if (Calculation->BDS_POSnVEL[prn - 1].iono_flag == 1 || Calculation->BDS_POSnVEL[prn - 1].iono_flag == 3)//双频伪距之差过大
				continue;
			//if (Calculation->BDS_POSnVEL[prn - 1].iono_flag == 1)//双频伪距之差过大
			//	continue;
			if (BDS_SatPositionVelocity(&(Calculation->BDS_POSnVEL[prn - 1].SignalTrTime), prn, Calculation, RawData, tao) == 2) //计算卫星位置速度
				continue;
			ComputeAzimuthElement(CGCS2000_a, CGCS2000_f, &RecTemp, &Calculation->BDS_POSnVEL[prn - 1].SatPos, &azimuth, &element); //计算高度角方位角（更新）

			if (abs(Rcv->RecPos.x) < 1e-10 || element > 15 / 180.0*PI)
			{
				/////////
				if (abs(Rcv->RecPos.x) < 1e-10 || azimuth > 0)
				{
					tropo_correct = Hopfield(&RecTemp, element); //计算对流层改正（更新）
					Calculation->BDS_POSnVEL[prn - 1].TroCorrect = tropo_correct;
					Calculation->BDS_POSnVEL[prn - 1].Element = element / PI*180.0;

					rho = sqrt(pow((Calculation->BDS_POSnVEL[prn - 1].SatPos.x - RecTemp.x), 2) + pow((Calculation->BDS_POSnVEL[prn - 1].SatPos.y - RecTemp.y), 2) + pow((Calculation->BDS_POSnVEL[prn - 1].SatPos.z - RecTemp.z), 2));

					B[(n_GPS + n_BDS) * 5] = (RecTemp.x - Calculation->BDS_POSnVEL[prn - 1].SatPos.x) / rho;
					B[(n_GPS + n_BDS) * 5 + 1] = (RecTemp.y - Calculation->BDS_POSnVEL[prn - 1].SatPos.y) / rho;
					B[(n_GPS + n_BDS) * 5 + 2] = (RecTemp.z - Calculation->BDS_POSnVEL[prn - 1].SatPos.z) / rho;
					B[(n_GPS + n_BDS) * 5 + 3] = 0;
					B[(n_GPS + n_BDS) * 5 + 4] = 1;
					//P_value[n_GPS + n_BDS] = sin(element)*sin(element);
					//P_value[n_GPS + n_BDS] = 1 / (0.03*0.03 + 0.04*0.04 / sin(element) / sin(element));
					P_value[n_GPS + n_BDS] = 1 / (pow(0.13 + 0.56*exp(-element / PI * 180 / 10), 2));

					if (Calculation->BDS_POSnVEL[prn - 1].iono_flag == 2)
						w[n_GPS + n_BDS] = Calculation->BDS_POSnVEL[prn - 1].Psr_InonCorrect - rho - delta_t_receive[1] + SpeedofLight*Calculation->BDS_POSnVEL[prn - 1].delta_tsv - tropo_correct;
					else if (RawData->Obs.BDS_Sat[i].Psr[0] < 1)
					{
						iono_correct = Klobuchar(&(Calculation->BDS_POSnVEL[prn - 1].SignalTrTime), &RecTemp, &(Calculation->BDS_POSnVEL[prn - 1].SatPos), azimuth, element, RawData);
						w[n_GPS + n_BDS] = RawData->Obs.BDS_Sat[i].Psr[1] - rho - delta_t_receive[1] + SpeedofLight*Calculation->BDS_POSnVEL[prn - 1].delta_tsv - iono_correct - tropo_correct;
					}
					else if (RawData->Obs.BDS_Sat[i].Psr[1] < 1)
					{
						iono_correct = Klobuchar(&(Calculation->BDS_POSnVEL[prn - 1].SignalTrTime), &RecTemp, &(Calculation->BDS_POSnVEL[prn - 1].SatPos), azimuth, element, RawData);
						w[n_GPS + n_BDS] = RawData->Obs.BDS_Sat[i].Psr[0] - rho - delta_t_receive[1] + SpeedofLight*Calculation->BDS_POSnVEL[prn - 1].delta_tsv - iono_correct - tropo_correct;
					}
					else
					{
						iono_correct = Klobuchar(&(Calculation->BDS_POSnVEL[prn - 1].SignalTrTime), &RecTemp, &(Calculation->BDS_POSnVEL[prn - 1].SatPos), azimuth, element, RawData);
						w[n_GPS + n_BDS] = (RawData->Obs.BDS_Sat[i].Psr[0] + RawData->Obs.BDS_Sat[i].Psr[1]) / 2 - rho - delta_t_receive[1] + SpeedofLight*Calculation->BDS_POSnVEL[prn - 1].delta_tsv - iono_correct - tropo_correct;
					}
					n_BDS++;
				}
			}
		}
		n = n_GPS + n_BDS; //可用于计算的全部卫星数量
		if (n_GPS != 0 && n_BDS != 0 && n > 5) //GPS和北斗都有
		{
			for (k = 0;k < n;k++)
			{
				//P[k*n + k] = P_value[k];
				P[k*n + k] = 1;
			}

			MatrixTrans(n, 5, 5, n, B, B_T);
			MatrixMul(5, n, n, n, B_T, P, BTP);//
			MatrixMul(5, n, n, 5, BTP, B, BB_T);//
			//MatrixMul(5, n, n, 5, B_T, B, BB_T);
			MatrixInv(5, BB_T, BB_T1);
			MatrixMul(5, 5, 5, n, BB_T1, B_T, BB_T2);
			MatrixMul(5, n, n, n, BB_T2, P, BB_T3);//
			//MatrixMul(5, n, n, 1, BB_T2, w, x);
			MatrixMul(5, n, n, 1, BB_T3, w, x);

			RecTemp.x = RecTemp.x + x[0];
			RecTemp.y = RecTemp.y + x[1];
			RecTemp.z = RecTemp.z + x[2];
			delta_t_receive[0] = delta_t_receive[0] + x[3];
			delta_t_receive[1] = delta_t_receive[1] + x[4];
			position_new = sqrt(pow((RecTemp.x), 2) + pow((RecTemp.y), 2) + pow((RecTemp.z), 2) + pow(delta_t_receive[0],2) + pow(delta_t_receive[1], 2));

			if (fabs(position - position_new) > 1e-5)
			{
				position = position_new;
				continue;
			}
			else
			{
				MatrixMul(n, 5, 5, 1, B, x, v);
				MatrixSub(n, 1, n, 1, v, w, v);
				MatrixTrans(n, 1, 1, n, v, v_T);
				MatrixMul(1, n, n, 1, v_T, v, vtpv);
				sigma = sqrt(vtpv[0] / (n - 5));
				PDOP = sqrt(BB_T1[0] * BB_T1[0] + BB_T1[6] * BB_T1[6] + BB_T1[12] * BB_T1[12]);

				Rcv->RecPos.x = RecTemp.x;
				Rcv->RecPos.y = RecTemp.y;
				Rcv->RecPos.z = RecTemp.z;
				Rcv->delta_t_receive[0] = delta_t_receive[0];
				Rcv->delta_t_receive[1] = delta_t_receive[1];
				Rcv->delta_t_dot[0] = delta_t_dot[0];
				Rcv->delta_t_dot[1] = delta_t_dot[1];
				Rcv->sigma = sigma;
				Rcv->PDOP = PDOP;
				Rcv->SatNum = n;
				
				return 1;
			}
		}
		else if (n_BDS == 0 && n_GPS > 4) //单GPS
		{
			DeleteZero(B, 150);

			MatrixTrans(n, 4, 4, n, B, B_T);
			MatrixMul(4, n, n, 4, B_T, B, BB_T);
			MatrixInv(4, BB_T, BB_T1);
			MatrixMul(4, 4, 4, n, BB_T1, B_T, BB_T2);
			MatrixMul(4, n, n, 1, BB_T2, w, x);

			RecTemp.x = RecTemp.x + x[0];
			RecTemp.y = RecTemp.y + x[1];
			RecTemp.z = RecTemp.z + x[2];
			delta_t_receive[0] = delta_t_receive[0] + x[3];
			position_new = sqrt(pow((RecTemp.x), 2) + pow((RecTemp.y), 2) + pow((RecTemp.z), 2));

			if (fabs(position - position_new) > 1)
			{
				position = position_new;
				continue;
			}
			else
			{
				MatrixMul(n, 4, 4, 1, B, x, v);
				MatrixSub(n, 1, n, 1, v, w, v);
				MatrixTrans(n, 1, 1, n, v, v_T);
				MatrixMul(1, n, n, 1, v_T, v, vtpv);
				sigma = sqrt(vtpv[0] / (n - 4));

				Rcv->RecPos.x = RecTemp.x;
				Rcv->RecPos.y = RecTemp.y;
				Rcv->RecPos.z = RecTemp.z;
				Rcv->delta_t_receive[0] = delta_t_receive[0];
				Rcv->delta_t_dot[0] = delta_t_dot[0];

				return 1;
			}

		}
		else if (n_GPS == 0 && n_BDS > 4) //单北斗
		{
			DeleteZero(B, 150);

			MatrixTrans(n, 4, 4, n, B, B_T);
			MatrixMul(4, n, n, 4, B_T, B, BB_T);
			MatrixInv(4, BB_T, BB_T1);
			MatrixMul(4, 4, 4, n, BB_T1, B_T, BB_T2);
			MatrixMul(4, n, n, 1, BB_T2, w, x);

			RecTemp.x = RecTemp.x + x[0];
			RecTemp.y = RecTemp.y + x[1];
			RecTemp.z = RecTemp.z + x[2];
			delta_t_receive[1] = delta_t_receive[1] + x[3];
			position_new = sqrt(pow((RecTemp.x), 2) + pow((RecTemp.y), 2) + pow((RecTemp.z), 2));

			if (fabs(position - position_new) > 1)
			{
				position = position_new;
				continue;
			}
			else
			{
				MatrixMul(n, 4, 4, 1, B, x, v);
				MatrixSub(n, 1, n, 1, v, w, v);
				MatrixTrans(n, 1, 1, n, v, v_T);
				MatrixMul(1, n, n, 1, v_T, v, vtpv);
				sigma = sqrt(vtpv[0] / (n - 4));

				Rcv->RecPos.x = RecTemp.x;
				Rcv->RecPos.y = RecTemp.y;
				Rcv->RecPos.z = RecTemp.z;
				Rcv->delta_t_receive[1] = delta_t_receive[1];
				Rcv->delta_t_dot[1] = delta_t_dot[1];

				return 1;
			}
		}
		else
		{
			cout << "number of Sat too small" << endl;
			break;
		}
	}
	if (j == 100)
	{
		cout << "SPP failed!" << endl;
		return 0;
	}
}

/**************************************************************
SPP
目的：单点测速

参数：
RawData              原始数据
RecPos               接收机位置
RecVel               接收机速度
Calculation          计算结果

返回值：0=卫星数目不足，无法计算  1=正常
***************************************************************/
int SPV(CALCULATION *Calculation, RAWDATA *RawData, XYZ *RecPos, XYZ *RecVel)
{
	int i, j, prn, n_GPS, n_BDS, n;
	double Sat_X, Sat_Y, Sat_Z, rho, rho_dot, delta_t_dot[2];
	double B[150], v[30], w[30], D[30], B_T[150], BB_T[125], BB_T1[25], BB_T2[150], x[5], v_T[30];

	n_GPS = 0;
	n_BDS = 0;

	for (i = 0;i < GPSsatnum;i++)
	{
		prn = RawData->Obs.GPS_Sat[i].Prn;
		if (prn == 0)
			continue;
		Sat_X = Calculation->GPS_POSnVEL[prn - 1].SatPos.x;
		Sat_Y = Calculation->GPS_POSnVEL[prn - 1].SatPos.y;
		Sat_Z = Calculation->GPS_POSnVEL[prn - 1].SatPos.z;
		if (Sat_X*Sat_X + Sat_Y*Sat_Y + Sat_Z*Sat_Z < 1)
			continue;

		rho = sqrt(pow((Sat_X - RecPos->x), 2) + pow((Sat_Y - RecPos->y), 2) + pow((Sat_Z - RecPos->z), 2));
		B[n_GPS * 5] = (RecPos->x - Calculation->GPS_POSnVEL[prn - 1].SatPos.x) / rho;
		B[n_GPS * 5 + 1] = (RecPos->y - Calculation->GPS_POSnVEL[prn - 1].SatPos.y) / rho;
		B[n_GPS * 5 + 2] = (RecPos->z - Calculation->GPS_POSnVEL[prn - 1].SatPos.z) / rho;
		B[n_GPS * 5 + 3] = 1;
		B[n_GPS * 5 + 4] = 0;
		rho_dot = B[n_GPS * 5] * Calculation->GPS_POSnVEL[prn - 1].v_x + B[n_GPS * 5 + 1] * Calculation->GPS_POSnVEL[prn - 1].v_y + B[n_GPS * 5 + 2] * Calculation->GPS_POSnVEL[prn - 1].v_z;
		rho_dot = -rho_dot;
		D[n_GPS] = RawData->Obs.GPS_Sat[i].Dop[0] * SpeedofLight / GPS_L1;
		w[n_GPS] = -D[n_GPS] - rho_dot + SpeedofLight*Calculation->GPS_POSnVEL[prn - 1].delta_tsv_dot;

		n_GPS++;
	}
	for (i = 0;i < BDSsatnum;i++)
	{
		if (prn == 0)
			continue;
		prn = RawData->Obs.BDS_Sat[i].Prn;
		Sat_X = Calculation->BDS_POSnVEL[prn - 1].SatPos.x;
		Sat_Y = Calculation->BDS_POSnVEL[prn - 1].SatPos.y;
		Sat_Z = Calculation->BDS_POSnVEL[prn - 1].SatPos.z;
		if ((Sat_X*Sat_X + Sat_Y*Sat_Y + Sat_Z*Sat_Z) < 1)
			continue;

		rho = sqrt(pow((Sat_X - RecPos->x), 2) + pow((Sat_Y - RecPos->y), 2) + pow((Sat_Z - RecPos->z), 2));
		B[(n_GPS + n_BDS) * 5] = (RecPos->x - Calculation->BDS_POSnVEL[prn - 1].SatPos.x) / rho;
		B[(n_GPS + n_BDS) * 5 + 1] = (RecPos->y - Calculation->BDS_POSnVEL[prn - 1].SatPos.y) / rho;
		B[(n_GPS + n_BDS) * 5 + 2] = (RecPos->z - Calculation->BDS_POSnVEL[prn - 1].SatPos.z) / rho;
		B[(n_GPS + n_BDS) * 5 + 3] = 0;
		B[(n_GPS + n_BDS) * 5 + 4] = 1;
		rho_dot = B[(n_GPS + n_BDS) * 5] * Calculation->BDS_POSnVEL[prn - 1].v_x + B[(n_GPS + n_BDS) * 5 + 1] * Calculation->BDS_POSnVEL[prn - 1].v_y + B[(n_GPS + n_BDS) * 5 + 2] * Calculation->BDS_POSnVEL[prn - 1].v_z;
		rho_dot = -rho_dot;
		D[n_GPS + n_BDS] = RawData->Obs.BDS_Sat[i].Dop[0] * SpeedofLight / BDS_B1;
		w[n_GPS + n_BDS] = -D[n_GPS + n_BDS] - rho_dot + SpeedofLight*Calculation->BDS_POSnVEL[prn - 1].delta_tsv_dot;

		n_BDS++;
	}
	n = n_GPS + n_BDS;
	if (n_GPS != 0 && n_BDS != 0 && n > 5)
	{
		MatrixTrans(n, 5, 5, n, B, B_T);
		MatrixMul(5, n, n, 5, B_T, B, BB_T);
		MatrixInv(5, BB_T, BB_T1);
		MatrixMul(5, 5, 5, n, BB_T1, B_T, BB_T2);
		MatrixMul(5, n, n, 1, BB_T2, w, x);

		RecVel->x = x[0];
		RecVel->y = x[1];
		RecVel->z = x[2];
		delta_t_dot[0] = x[3];
		delta_t_dot[1] = x[4];

		return 1;
	}
	else if (n_BDS == 0 && n_GPS > 4)
	{
		DeleteZero(B, 150);

		MatrixTrans(n, 4, 4, n, B, B_T);
		MatrixMul(4, n, n, 4, B_T, B, BB_T);
		MatrixInv(4, BB_T, BB_T1);
		MatrixMul(4, 4, 4, n, BB_T1, B_T, BB_T2);
		MatrixMul(4, n, n, 1, BB_T2, w, x);

		RecVel->x = x[0];
		RecVel->y = x[1];
		RecVel->z = x[2];
		delta_t_dot[0] = x[3];

		return 1;
	}
	else if (n_GPS == 0 && n_BDS > 4)
	{
		DeleteZero(B, 150);

		MatrixTrans(n, 4, 4, n, B, B_T);
		MatrixMul(4, n, n, 4, B_T, B, BB_T);
		MatrixInv(4, BB_T, BB_T1);
		MatrixMul(4, 4, 4, n, BB_T1, B_T, BB_T2);
		MatrixMul(4, n, n, 1, BB_T2, w, x);

		RecVel->x = x[0];
		RecVel->y = x[1];
		RecVel->z = x[2];
		delta_t_dot[1] = x[3];

		return 1;
	}
	else
	{
		cout << "Compute Velocity: number of Sat too small" << endl;
		return 0;
	}
}