#include "stdafx.h"
#include "struct.h"
#include<iostream>
#include<cmath>
using namespace std;

/**************************************************************
GPS_SatPositionVelocity
目的：计算GPS卫星位置和速度

参数：
t             时刻（如信号发射时刻）
prn           卫星prn号
Calculation   GPS卫星位置、速度的计算结果
RawData       观测数据、星历等数据
tao           信号传播时间

返回值：1=正常 2=星历过期 0=迭代失败
***************************************************************/
int GPS_SatPositionVelocity(GPSTIME *t, int &prn, CALCULATION *Calculation, RAWDATA *RawData, double &tao)
{
	double n0, t_k, n, M_k, e, E_k, delta, E_k_new, v_k, phi_k, delta_uk, delta_rk, delta_ik, uk, rk, ik, omega_k, xk, yk;
	double Ek_dot, phik_dot, uk_dot, rk_dot, ik_dot, omegak_dot, xk_dot, yk_dot, delta_t;
	int j;

	//星历是否过期
	delta_t = (t->Week - RawData->GPSEph[prn - 1].week) * 604800 + (t->SecofWeek - RawData->GPSEph[prn - 1].toe);
	if (RawData->GPSEph[prn - 1].PRN != 0 && delta_t <= 2 * 3600)
	{
		n0 = sqrt(GPS_miu / pow(RawData->GPSEph[prn - 1].A, 3));
		t_k = t->SecofWeek - RawData->GPSEph[prn - 1].toe;
		if (t_k > 302400) t_k = t_k - 604800;
		else if (t_k < -302400) t_k = t_k + 604800;
		n = n0 + RawData->GPSEph[prn - 1].deltaN;
		M_k = RawData->GPSEph[prn - 1].M0 + n*t_k;
		E_k = M_k;
		delta = 1e-14;
		e = RawData->GPSEph[prn - 1].ecc;
		for (j = 0;j < 10;j++)
		{
			E_k_new = M_k + sin(E_k)*e;
			if (abs(E_k_new - E_k) < delta) break;
			else E_k = E_k_new;
			if (j == 10) return 0;
		}
		v_k = atan2(sqrt(1 - e*e)*sin(E_k) / (1 - e*cos(E_k)), (cos(E_k) - e) / (1 - e*cos(E_k)));
		phi_k = v_k + RawData->GPSEph[prn - 1].w;
		delta_uk = RawData->GPSEph[prn - 1].cus*sin(2 * phi_k) + RawData->GPSEph[prn - 1].cuc*cos(2 * phi_k);
		delta_rk = RawData->GPSEph[prn - 1].crs*sin(2 * phi_k) + RawData->GPSEph[prn - 1].crc*cos(2 * phi_k);
		delta_ik = RawData->GPSEph[prn - 1].cis*sin(2 * phi_k) + RawData->GPSEph[prn - 1].cic*cos(2 * phi_k);
		uk = phi_k + delta_uk;
		rk = RawData->GPSEph[prn - 1].A*(1 - e*cos(E_k)) + delta_rk;
		ik = RawData->GPSEph[prn - 1].i0 + delta_ik + RawData->GPSEph[prn - 1].IDOT*t_k;
		xk = rk*cos(uk);
		yk = rk*sin(uk);
		omega_k = RawData->GPSEph[prn - 1].omega_0 + (RawData->GPSEph[prn - 1].omega_dot - GPS_omegae_dot)*t_k - GPS_omegae_dot*RawData->GPSEph[prn - 1].toe;
		Calculation->GPS_POSnVEL[prn - 1].SatPos.x = xk*cos(omega_k) - yk*cos(ik)*sin(omega_k);
		Calculation->GPS_POSnVEL[prn - 1].SatPos.y = xk*sin(omega_k) + yk*cos(ik)*cos(omega_k);
		Calculation->GPS_POSnVEL[prn - 1].SatPos.z = yk*sin(ik);

		EarthRotate(&Calculation->GPS_POSnVEL[prn - 1].SatPos, tao);

		Ek_dot = n / (1 - e*cos(E_k));
		phik_dot = sqrt((1 + e) / (1 - e))*pow(cos(v_k / 2) / cos(E_k / 2), 2)*Ek_dot;
		uk_dot = 2 * (RawData->GPSEph[prn - 1].cus*cos(2 * phi_k) - RawData->GPSEph[prn - 1].cuc*sin(2 * phi_k))*phik_dot + phik_dot;
		rk_dot = RawData->GPSEph[prn - 1].A*e*sin(E_k)*Ek_dot + 2 * (RawData->GPSEph[prn - 1].crs*cos(2 * phi_k) - RawData->GPSEph[prn - 1].crc*sin(2 * phi_k))*phik_dot;
		ik_dot = RawData->GPSEph[prn - 1].IDOT + 2 * (RawData->GPSEph[prn - 1].cis*cos(2 * phi_k) - RawData->GPSEph[prn - 1].cic*sin(2 * phi_k))*phik_dot;
		omegak_dot = RawData->GPSEph[prn - 1].omega_dot - GPS_omegae_dot;
		xk_dot = rk_dot*cos(uk) - rk*uk_dot*sin(uk);
		yk_dot = rk_dot*sin(uk) + rk*uk_dot*cos(uk);
		Calculation->GPS_POSnVEL[prn - 1].v_x = cos(omega_k)*xk_dot - sin(omega_k)*cos(ik)*yk_dot - (xk*sin(omega_k) + yk*cos(omega_k)*cos(ik))*omegak_dot + yk*sin(omega_k)*sin(ik)*ik_dot;
		Calculation->GPS_POSnVEL[prn - 1].v_y = sin(omega_k)*xk_dot + cos(omega_k)*cos(ik)*yk_dot + (xk*cos(omega_k) - yk*sin(omega_k)*cos(ik))*omegak_dot + yk*cos(omega_k)*sin(ik)*ik_dot;
		Calculation->GPS_POSnVEL[prn - 1].v_z = sin(ik)*yk_dot + yk*cos(ik)*ik_dot;
		return 1;
	}
	else
		return 2;
}

/**************************************************************
GPS_SatClock
目的：计算GPS卫星钟差与钟速

参数：
t_sv          时刻（如信号发射时刻 周内秒）
prn           卫星prn号
Calculation   卫星位置、速度的计算结果
RawData       观测数据、星历等数据

返回值：1=正常 0=迭代失败
***************************************************************/
int GPS_SatClock(GPSTIME *t_sv, int &prn, CALCULATION *Calculation, RAWDATA *RawData)
{
	double F, delta_tr, e, A, E_k, t_k, n0, n, M_k, E_k_new, t, delta_t, delta_t_new;
	double clkbias, clkdrift, clkdriftrate, Ek_dot, delta_tr_dot;
	int i, j;

	F = -4.442807633e-10;
	e = RawData->GPSEph[prn - 1].ecc;
	A = RawData->GPSEph[prn - 1].A;
	clkbias = RawData->GPSEph[prn - 1].af0;
	clkdrift = RawData->GPSEph[prn - 1].af1;
	clkdriftrate = RawData->GPSEph[prn - 1].af2;
	delta_t = 0;
	for (i = 0;i < 10;i++)
	{
		t = t_sv->SecofWeek - delta_t;
		/*计算E_k*/
		n0 = sqrt(GPS_miu / pow(A, 3));
		t_k = t_sv->SecofWeek - RawData->GPSEph[prn - 1].toe;
		if (t_k > 302400) t_k = t_k - 604800;
		else if (t_k < -302400) t_k = t_k + 604800;
		n = n0 + RawData->GPSEph[prn - 1].deltaN;
		M_k = RawData->GPSEph[prn - 1].M0 + n*t_k;
		E_k = M_k;
		e = RawData->GPSEph[prn - 1].ecc;
		for (j = 0;j < 10;j++)
		{
			E_k_new = M_k + sin(E_k)*e;
			if (abs(E_k_new - E_k) < 1e-14) break;
			else E_k = E_k_new;
			if (j == 9) return 0;
		}
		delta_tr = F*e*sqrt(A)*sin(E_k);
		delta_t_new = clkbias + clkdrift*(t - RawData->GPSEph[prn - 1].toc)
			+ clkdriftrate*pow(t - RawData->GPSEph[prn - 1].toc, 2) + delta_tr;
		//-RawData->GPSEph[prn - 1].tgd[0];
		if (abs(delta_t - delta_t_new) < 1e-10) break;
		else delta_t = delta_t_new;
		if (i == 9) return 0;
	}
	Calculation->GPS_POSnVEL[prn - 1].delta_tsv = delta_t;

	Ek_dot = n / (1 - e*cos(E_k));
	delta_tr_dot = F*e*sqrt(A)*cos(E_k)*Ek_dot;
	Calculation->GPS_POSnVEL[prn - 1].delta_tsv_dot = clkdrift + 2 * clkdriftrate*(t - delta_t - RawData->GPSEph[prn - 1].toc) + delta_tr_dot;
	return 1;
}

/**************************************************************
BDS_SatPositionVelocity
目的：计算BDS卫星位置和速度

参数：
t             时刻（如信号发射时刻）
prn           卫星prn号
Calculation   卫星位置、速度、钟差、钟速的计算结果
RawData       观测数据、星历等数据
tao           信号传播时间

返回值：1=正常 0=迭代失败
***************************************************************/
int BDS_SatPositionVelocity(GPSTIME *t, int &prn, CALCULATION *Calculation, RAWDATA *RawData,double &tao)
{
	double n0, t_k, n, M_k, e, E_k, delta, E_k_new, v_k, phi_k, delta_uk, delta_rk, delta_ik, uk, rk, ik, omega_k, xk, yk, x_GK, y_GK, z_GK;
	double Ek_dot, phik_dot, uk_dot, rk_dot, ik_dot, omegak_dot, xk_dot, yk_dot, delta_t, vx_GK, vy_GK, vz_GK, rotate_1, rotate_2;
	int j;
	//星历是否过期
	//BDST = GPST - 1356week - 14s;
	//delta_t = (t->Week - RawData->BDSEph[prn - 1].week) * 604800 + (t->SecofWeek - RawData->BDSEph[prn - 1].toe);
	delta_t = (t->Week - RawData->BDSEph[prn - 1].week - 1356) * 604800 + (t->SecofWeek - RawData->BDSEph[prn - 1].toe - 14);
	if (RawData->BDSEph[prn - 1].PRN != 0 && delta_t <= 3600)
	{
		n0 = sqrt(BDS_miu / pow(RawData->BDSEph[prn - 1].A, 3));
		//t_k = t->SecofWeek - RawData->BDSEph[prn - 1].toe;
		t_k = t->SecofWeek - 14 - RawData->BDSEph[prn - 1].toe;
		if (t_k > 302400) t_k = t_k - 604800;
		else if (t_k < -302400) t_k = t_k + 604800;
		n = n0 + RawData->BDSEph[prn - 1].deltaN;
		M_k = RawData->BDSEph[prn - 1].M0 + n*t_k;
		E_k = M_k;
		delta = 1e-14;
		e = RawData->BDSEph[prn - 1].ecc;
		for (j = 0;j < 10;j++)
		{
			E_k_new = M_k + sin(E_k)*e;
			if (abs(E_k_new - E_k) < delta) break;
			else E_k = E_k_new;
			if (j == 10) return 0;
		}
		v_k = atan2(sqrt(1 - e*e)*sin(E_k) / (1 - e*cos(E_k)), (cos(E_k) - e) / (1 - e*cos(E_k)));
		phi_k = v_k + RawData->BDSEph[prn - 1].w;
		delta_uk = RawData->BDSEph[prn - 1].cus*sin(2 * phi_k) + RawData->BDSEph[prn - 1].cuc*cos(2 * phi_k);
		delta_rk = RawData->BDSEph[prn - 1].crs*sin(2 * phi_k) + RawData->BDSEph[prn - 1].crc*cos(2 * phi_k);
		delta_ik = RawData->BDSEph[prn - 1].cis*sin(2 * phi_k) + RawData->BDSEph[prn - 1].cic*cos(2 * phi_k);
		uk = phi_k + delta_uk;
		rk = RawData->BDSEph[prn - 1].A*(1 - e*cos(E_k)) + delta_rk;
		ik = RawData->BDSEph[prn - 1].i0 + delta_ik + RawData->BDSEph[prn - 1].IDOT*t_k;
		xk = rk*cos(uk);
		yk = rk*sin(uk);
		
		Ek_dot = n / (1 - e*cos(E_k));
		phik_dot = sqrt((1 + e) / (1 - e))*pow(cos(v_k / 2) / cos(E_k / 2), 2)*Ek_dot;
		uk_dot = 2 * (RawData->BDSEph[prn - 1].cus*cos(2 * phi_k) - RawData->BDSEph[prn - 1].cuc*sin(2 * phi_k))*phik_dot + phik_dot;
		rk_dot = RawData->BDSEph[prn - 1].A*e*sin(E_k)*Ek_dot + 2 * (RawData->BDSEph[prn - 1].crs*cos(2 * phi_k) - RawData->BDSEph[prn - 1].crc*sin(2 * phi_k))*phik_dot;
		ik_dot = RawData->BDSEph[prn - 1].IDOT + 2 * (RawData->BDSEph[prn - 1].cis*cos(2 * phi_k) - RawData->BDSEph[prn - 1].cic*sin(2 * phi_k))*phik_dot;
		xk_dot = rk_dot*cos(uk) - rk*uk_dot*sin(uk);
		yk_dot = rk_dot*sin(uk) + rk*uk_dot*cos(uk);

		if (prn == 1 || prn == 2 || prn == 3 || prn == 4 || prn == 5 || prn == 59 || prn == 60 || prn == 61) //GEO
		{
			omega_k = RawData->BDSEph[prn - 1].omega_0 + RawData->BDSEph[prn - 1].omega_dot*t_k - BDS_omegae_dot*RawData->BDSEph[prn - 1].toe;
			omegak_dot = RawData->BDSEph[prn - 1].omega_dot;
			x_GK = xk*cos(omega_k) - yk*cos(ik)*sin(omega_k);
			y_GK = xk*sin(omega_k) + yk*cos(ik)*cos(omega_k);
			z_GK = yk*sin(ik);
			rotate_1 = BDS_omegae_dot*t_k;
			rotate_2 = -5.0 / 180.0 * PI;
			double Rz[9] = { cos(rotate_1), sin(rotate_1), 0, -sin(rotate_1), cos(rotate_1), 0, 0, 0, 1 };
			double Rx[9] = { 1,0,0,0,cos(rotate_2),sin(rotate_2),0,-sin(rotate_2),cos(rotate_2) };
			double Rzx[9];
			MatrixMul(3, 3, 3, 3, Rz, Rx, Rzx);
			Calculation->BDS_POSnVEL[prn - 1].SatPos.x = Rzx[0] * x_GK + Rzx[1] * y_GK + Rzx[2] * z_GK;
			Calculation->BDS_POSnVEL[prn - 1].SatPos.y = Rzx[3] * x_GK + Rzx[4] * y_GK + Rzx[5] * z_GK;
			Calculation->BDS_POSnVEL[prn - 1].SatPos.z = Rzx[6] * x_GK + Rzx[7] * y_GK + Rzx[8] * z_GK;
			
			vx_GK = cos(omega_k)*xk_dot - sin(omega_k)*cos(ik)*yk_dot - (xk*sin(omega_k) + yk*cos(omega_k)*cos(ik))*omegak_dot + yk*sin(omega_k)*sin(ik)*ik_dot;
			vy_GK = sin(omega_k)*xk_dot + cos(omega_k)*cos(ik)*yk_dot + (xk*cos(omega_k) - yk*sin(omega_k)*cos(ik))*omegak_dot + yk*cos(omega_k)*sin(ik)*ik_dot;
			vz_GK = sin(ik)*yk_dot + yk*cos(ik)*ik_dot;
			
			Calculation->BDS_POSnVEL[prn - 1].v_x = BDS_omegae_dot*(-sin(rotate_1))*x_GK + cos(rotate_1)*vx_GK
				+ BDS_omegae_dot*cos(rotate_1)*cos(rotate_2)*y_GK + sin(rotate_1)*cos(rotate_2)*vy_GK
				+ BDS_omegae_dot*cos(rotate_1)*sin(rotate_2)*z_GK + sin(rotate_1)*sin(rotate_2)*vz_GK;
			Calculation->BDS_POSnVEL[prn - 1].v_y = -BDS_omegae_dot*cos(rotate_1)*x_GK - sin(rotate_1)*vx_GK
				- BDS_omegae_dot*sin(rotate_1)*cos(rotate_2)*y_GK + cos(rotate_1)*cos(rotate_2)*vy_GK
				- BDS_omegae_dot*sin(rotate_1)*sin(rotate_2)*z_GK + cos(rotate_1)*sin(rotate_2)*vz_GK;
			Calculation->BDS_POSnVEL[prn - 1].v_z = -sin(rotate_2)*vy_GK + cos(rotate_2)*vz_GK;
		}
		else
		{
			omega_k = RawData->BDSEph[prn - 1].omega_0 + (RawData->BDSEph[prn - 1].omega_dot - BDS_omegae_dot)*t_k - BDS_omegae_dot*RawData->BDSEph[prn - 1].toe;
			omegak_dot = RawData->BDSEph[prn - 1].omega_dot - BDS_omegae_dot;
			Calculation->BDS_POSnVEL[prn - 1].SatPos.x = xk*cos(omega_k) - yk*cos(ik)*sin(omega_k);
			Calculation->BDS_POSnVEL[prn - 1].SatPos.y = xk*sin(omega_k) + yk*cos(ik)*cos(omega_k);
			Calculation->BDS_POSnVEL[prn - 1].SatPos.z = yk*sin(ik);

			Calculation->BDS_POSnVEL[prn - 1].v_x = cos(omega_k)*xk_dot - sin(omega_k)*cos(ik)*yk_dot - (xk*sin(omega_k) + yk*cos(omega_k)*cos(ik))*omegak_dot + yk*sin(omega_k)*sin(ik)*ik_dot;
			Calculation->BDS_POSnVEL[prn - 1].v_y = sin(omega_k)*xk_dot + cos(omega_k)*cos(ik)*yk_dot + (xk*cos(omega_k) - yk*sin(omega_k)*cos(ik))*omegak_dot + yk*cos(omega_k)*sin(ik)*ik_dot;
			Calculation->BDS_POSnVEL[prn - 1].v_z = sin(ik)*yk_dot + yk*cos(ik)*ik_dot;
		}

		EarthRotate(&Calculation->BDS_POSnVEL[prn - 1].SatPos, tao);
		return 1;
	}
	else 
		return 2;
}

/**************************************************************
BDS_SatClock
目的：计算BDS卫星钟差与钟速

参数：
t_sv          时刻（如信号发射时刻）
prn           卫星prn号
Calculation   卫星位置、速度的计算结果
RawData       观测数据、星历等数据

返回值：1=正常 0=迭代失败
***************************************************************/
int BDS_SatClock(GPSTIME *t_sv, int &prn, CALCULATION *Calculation, RAWDATA *RawData)
{
	double F, delta_tr, e, A, E_k, t_k, n0, n, M_k, E_k_new, t, delta_t, delta_t_new;
	double clkbias, clkdrift, clkdriftrate, Ek_dot, delta_tr_dot;
	int i, j;

	F = -4.442807633e-10;
	e = RawData->BDSEph[prn - 1].ecc;
	A = RawData->BDSEph[prn - 1].A;
	clkbias = RawData->BDSEph[prn - 1].af0;
	clkdrift = RawData->BDSEph[prn - 1].af1;
	clkdriftrate = RawData->BDSEph[prn - 1].af2;
	delta_t = 0;
	for (i = 0;i < 10;i++)
	{
		t = t_sv->SecofWeek - delta_t - 14;
		/*计算E_k*/
		n0 = sqrt(BDS_miu / pow(A, 3));
		t_k = t_sv->SecofWeek - RawData->BDSEph[prn - 1].toe - 14;
		if (t_k > 302400) t_k = t_k - 604800;
		else if (t_k < -302400) t_k = t_k + 604800;
		n = n0 + RawData->BDSEph[prn - 1].deltaN;
		M_k = RawData->BDSEph[prn - 1].M0 + n*t_k;
		E_k = M_k;
		e = RawData->BDSEph[prn - 1].ecc;
		for (j = 0;j < 10;j++)
		{
			E_k_new = M_k + sin(E_k)*e;
			if (abs(E_k_new - E_k) < 1e-10) break;
			else E_k = E_k_new;
			if (j == 9) return 0;
		}
		delta_tr = F*e*sqrt(A)*sin(E_k);
		delta_t_new = clkbias + clkdrift*(t - RawData->BDSEph[prn - 1].toc)
			+ clkdriftrate*pow(t - RawData->BDSEph[prn - 1].toc, 2) + delta_tr;
		if (abs(delta_t - delta_t_new) < 1e-10) break;
		else delta_t = delta_t_new;
		if (i == 9) return 0;
	}
	Calculation->BDS_POSnVEL[prn - 1].delta_tsv = delta_t;

	Ek_dot = n / (1 - e*cos(E_k));
	delta_tr_dot = F*e*sqrt(A)*cos(E_k)*Ek_dot;
	Calculation->BDS_POSnVEL[prn - 1].delta_tsv_dot = clkdrift + 2 * clkdriftrate*(t - delta_t - RawData->BDSEph[prn - 1].toc) + delta_tr_dot;
	return 1;
}

/**************************************************************
GPS_SignalTransmit
目的：计算GPS信号发射时间

参数：
RawData        原始数据
Calculation   计算结果

返回值：0=计算失败 1=成功
***************************************************************/
int GPS_SignalTransmit(RAWDATA *RawData, CALCULATION *Calculation)
{
	double t_tr, P, delta_t;
	int i, j, prn;

	for (i = 0;i < RawData->Obs.GPS_SatNum;i++)
	{
		prn = RawData->Obs.GPS_Sat[i].Prn;
		delta_t = 0;
		//计算伪距（单频双频）
		if (RawData->Obs.GPS_Sat[i].Psr[0]>1 && RawData->Obs.GPS_Sat[i].Psr[1] > 1)
			P = (GPS_L1*GPS_L1* RawData->Obs.GPS_Sat[i].Psr[0] - GPS_L2*GPS_L2* RawData->Obs.GPS_Sat[i].Psr[1]) / (GPS_L1*GPS_L1 - GPS_L2*GPS_L2);
		else if (RawData->Obs.GPS_Sat[i].Psr[1] < 1)
		{
			P = RawData->Obs.GPS_Sat[i].Psr[0];
		}
		else if (RawData->Obs.GPS_Sat[i].Psr[0] < 1)
		{
			P = RawData->Obs.GPS_Sat[i].Psr[1];
		}
		else
			return 0;

		Calculation->GPS_POSnVEL[prn - 1].SignalTrTime.Week = RawData->Obs.ObsTime.Week;
		for (j = 0;j < 2;j++)
		{
			t_tr = RawData->Obs.ObsTime.SecofWeek - P / SpeedofLight - delta_t;
			Calculation->GPS_POSnVEL[prn - 1].SignalTrTime.SecofWeek = t_tr;
			GPS_SatClock(&Calculation->GPS_POSnVEL[prn - 1].SignalTrTime, prn, Calculation, RawData);
			delta_t = Calculation->GPS_POSnVEL[prn - 1].delta_tsv;
		}
		Calculation->GPS_POSnVEL[prn - 1].SignalTrTime.SecofWeek = t_tr;
	}
	return 1;
}

/**************************************************************
BDS_SignalTransmit
目的：计算BDS信号发射时间

参数：
RawData        原始数据
Calculation   计算结果

返回值：0=计算失败 1=成功
***************************************************************/
int BDS_SignalTransmit(RAWDATA *RawData, CALCULATION *Calculation)
{
	double t_tr, P, delta_t;
	int i, j, prn;
	for (i = 0;i < RawData->Obs.BDS_SatNum;i++)
	{
		prn = RawData->Obs.BDS_Sat[i].Prn;
		delta_t = 0;
		//计算伪距（单频双频）
		if (RawData->Obs.BDS_Sat[i].Psr[0]>1 && RawData->Obs.BDS_Sat[i].Psr[1] > 1)
			P = (BDS_B1*BDS_B1* RawData->Obs.BDS_Sat[i].Psr[0] - BDS_B3*BDS_B3* RawData->Obs.BDS_Sat[i].Psr[1]) / (BDS_B1*BDS_B1 - BDS_B3*BDS_B3);
		else if (RawData->Obs.BDS_Sat[i].Psr[1] < 1)
		{
			P = RawData->Obs.BDS_Sat[i].Psr[0];
		}
		else if (RawData->Obs.BDS_Sat[i].Psr[0] < 1)
		{
			P = RawData->Obs.BDS_Sat[i].Psr[1];
		}
		else
			return 0;

		Calculation->BDS_POSnVEL[prn - 1].SignalTrTime.Week = RawData->Obs.ObsTime.Week;
		for (j = 0;j < 2;j++)
		{
			t_tr = RawData->Obs.ObsTime.SecofWeek - P / SpeedofLight - delta_t;
			Calculation->BDS_POSnVEL[prn - 1].SignalTrTime.SecofWeek = t_tr;
			BDS_SatClock(&Calculation->BDS_POSnVEL[prn - 1].SignalTrTime, prn, Calculation, RawData);
			delta_t = Calculation->BDS_POSnVEL[prn - 1].delta_tsv;
		}
		Calculation->BDS_POSnVEL[prn - 1].SignalTrTime.SecofWeek = t_tr;
	}
	return 1;
}
