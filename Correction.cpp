#include "stdafx.h"
#include "struct.h"
#include<iostream>
#include<cmath>
using namespace std;

/*************************************
ComputeAzimuthElement
Ŀ�ģ������λ�Ǻ͸߶Ƚ�

������
a        ���򳤰���
f        �������
RecPos   ��վxyz����
SatPos   ����xyz����
azimuth  ��λ��
element  �߶Ƚ�
**************************************/
void ComputeAzimuthElement(double a, double f, XYZ *RecPos, XYZ *SatPos, double *azimuth, double *element)
{
	double N, E, U;
	BLH Rec;
	XYZToBLH(a, f, &Rec, RecPos);
	N = -sin(Rec.b)*cos(Rec.l)*(SatPos->x - RecPos->x) - sin(Rec.b)*sin(Rec.l)*(SatPos->y - RecPos->y) + cos(Rec.b)*(SatPos->z - RecPos->z);
	E = -sin(Rec.l)*(SatPos->x - RecPos->x) + cos(Rec.l)*(SatPos->y - RecPos->y);
	U = cos(Rec.b)*cos(Rec.l)*(SatPos->x - RecPos->x) + cos(Rec.b)*sin(Rec.l)*(SatPos->y - RecPos->y) + sin(Rec.b)*(SatPos->z - RecPos->z);
	*azimuth = atan2(E, N);
	*element = atan2(U, sqrt(N*N + E*E));
}

/**************************************************************
Klobuchar
Ŀ�ģ���������

������
t       ʱ��
RecPos  ��վλ��
SatPos  ����λ��
azimuth ��λ��
element �߶Ƚ�
RawData ԭʼ����

����ֵ����������ֵ 0=����ʧ��
***************************************************************/
double Klobuchar(GPSTIME *t, XYZ *RecPos, XYZ *SatPos, double &azimuth, double &element, RAWDATA *RawData)
{
	double T, f, psi, phi, lam, amp, per, x;
	BLH Rec;
	
	XYZToBLH(CGCS2000_a, CGCS2000_f, &Rec, RecPos);
	if (element<=0|| Rec.h<1e-3)
	{
	     return 0.0;
	}
	else
	{
		psi = 0.0137 / (element / PI + 0.11) - 0.022;
		phi = Rec.b / PI + psi*cos(azimuth);
		if (phi > 0.416)
			phi = 0.416;
		else if (phi < -0.416)
			phi = -0.416;
		lam = Rec.l / PI + psi*sin(azimuth) / cos(phi*PI);
		phi = phi + 0.064*cos((lam - 1.617)*PI);
		T = 43200.0*lam + t->SecofWeek;
		T = T - floor(T / 86400.0)*86400.0;
		f = 1.0 + 16.0*pow(0.53 - element / PI, 3);
		amp = RawData->ion.a0 + phi*(RawData->ion.a1 + phi*(RawData->ion.a2 + phi*(RawData->ion.a3)));
		if (amp < 0.0)
			amp = 0.0;
		per = RawData->ion.b0 + phi*(RawData->ion.b1 + phi*(RawData->ion.b2 + phi*(RawData->ion.b3)));
		if (per < 72000.0)
			per = 72000.0;
		x = 2.0*PI*(T - 50400.0) / per;
		if (fabs(x) < 1.57)
			return SpeedofLight*f*(5e-9 + amp*(1.0 + x*x*(-0.5 + x*x / 24.0)));
		else
			return SpeedofLight*f*5e-9;
	}
}

/**************************************************************
IonoFree
Ŀ�ģ�˫Ƶ�û�����������

������
RecPos       ���ջ�λ��
RawData      ԭʼ����
Calculation  ������

����ֵ��0=������ 1=�ɹ�
***************************************************************/
int IonoFree(XYZ *RecPos, RAWDATA *RawData, CALCULATION *Calculation)
{
	double k_GPS, k_BDS;
	int i, prn;
	BLH Rec;

	XYZToBLH(CGCS2000_a, CGCS2000_f, &Rec, RecPos);
	if (Rec.h<1e-8 || Rec.h>1e6)
	{
		return 0;
	}
	else
	{
		k_GPS = GPS_L1*GPS_L1 / GPS_L2 / GPS_L2;
		k_BDS = BDS_B1*BDS_B1 / BDS_B3 / BDS_B3;

		for (i = 0;i < RawData->Obs.GPS_SatNum;i++)
		{
			prn = RawData->Obs.GPS_Sat[i].Prn;
			if (RawData->Obs.GPS_Sat[i].Psr[0]>1 && RawData->Obs.GPS_Sat[i].Psr[1]>1) //˫Ƶ�۲�ֵ����
			{
				if (abs(RawData->Obs.GPS_Sat[i].Psr[0] - RawData->Obs.GPS_Sat[i].Psr[1])>100) //����α������
					Calculation->GPS_POSnVEL[prn - 1].iono_flag = 1;
				else
				{
					Calculation->GPS_POSnVEL[prn - 1].Psr_InonCorrect = (RawData->Obs.GPS_Sat[i].Psr[1] - k_GPS*RawData->Obs.GPS_Sat[i].Psr[0]) / (1 - k_GPS);
					Calculation->GPS_POSnVEL[prn - 1].iono_flag = 2;
				}
			}
			else
				Calculation->GPS_POSnVEL[prn - 1].iono_flag = 3;
		}
		for (i = 0;i < RawData->Obs.BDS_SatNum;i++)
		{
			prn = RawData->Obs.BDS_Sat[i].Prn;
			if (RawData->Obs.BDS_Sat[i].Psr[0]>1 && RawData->Obs.BDS_Sat[i].Psr[1] > 1) //˫Ƶ�۲�ֵ����
			{
				if (abs(RawData->Obs.BDS_Sat[i].Psr[0] - RawData->Obs.BDS_Sat[i].Psr[1])>100)
					Calculation->BDS_POSnVEL[prn - 1].iono_flag = 1;
				else
				{
					Calculation->BDS_POSnVEL[prn - 1].Psr_InonCorrect = (RawData->Obs.BDS_Sat[i].Psr[1] - k_BDS*RawData->Obs.BDS_Sat[i].Psr[0] + SpeedofLight*k_BDS*RawData->BDSEph[prn - 1].tgd[0]) / (1 - k_BDS);
					Calculation->BDS_POSnVEL[prn - 1].iono_flag = 2;
				}
			}
			else
				Calculation->BDS_POSnVEL[prn - 1].iono_flag = 3;
		}
		return 1;
	}
}

/**************************************************************
Hopfield
Ŀ�ģ����������

������
RecPos   ��վλ��
element  �߶Ƚ�

����ֵ�����������ֵ 0=����ʧ��
***************************************************************/
double Hopfield(XYZ *RecPos, double element)
{
	double H0, T0, p0, RH0, hd, hw, Kd, Kw, e, T, p, RH;
	BLH Rec;
	if ((RecPos->x*RecPos->x + RecPos->y*RecPos->y + RecPos->z*RecPos->z) < 1e-8)
	{
		Rec.b = 0.0;
		Rec.l = 0.0;
		Rec.h = -CGCS2000_a;
	}
	else
		XYZToBLH(CGCS2000_a, CGCS2000_f, &Rec, RecPos);
	H0 = 0.0;
	T0 = 15 + 273.16;
	p0 = 1013.25;
	RH0 = 0.5;
	if (Rec.h<1e-8 || Rec.h>1.7e4)
	{
		return 0;
	}
	else
	{
		T = T0 - 0.0065*(Rec.h - H0);
		p = p0*pow((1 - 0.0000226*(Rec.h - H0)), 5.225);
		RH = RH0*exp(-0.0006396*(Rec.h - H0));
		hd = 40136 + 148.72*(T0 - 273.16);
		hw = 11000.0;
		Kd = 155.2*1e-7*p / T*(hd - Rec.h);
		e = RH*exp(-37.2465 + 0.213166*T - 0.000256908*T*T);
		Kw = 155.2*1e-7 * 4810 / T / T*e*(hw - Rec.h);
		element = element*180.0 / PI;
		return Kd / sin(sqrt(element*element + 6.25) / 180.0*PI) + Kw / sin(sqrt(element*element + 2.25) / 180.0*PI);
	}
}

/**************************************************************
EarthRotate
Ŀ�ģ�������ת����

������
RecPos   ��վλ��
tao      �źŴ���ʱ��
***************************************************************/
void EarthRotate(XYZ *Pos, double &tao)
{
	Pos->x = cos(tao*EarthRotation)*Pos->x + sin(tao*EarthRotation)*Pos->y;
	Pos->y = -sin(tao*EarthRotation)*Pos->x + cos(tao*EarthRotation)*Pos->y;
	Pos->z = Pos->z;
}