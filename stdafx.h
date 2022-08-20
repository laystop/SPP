// stdafx.h : ��׼ϵͳ�����ļ��İ����ļ���
// ���Ǿ���ʹ�õ��������ĵ�
// �ض�����Ŀ�İ����ļ�
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>

// TODO:  �ڴ˴����ó�����Ҫ������ͷ�ļ�
#include "struct.h"
#include "Windows.h"
#include<fstream>

using namespace std;

//��������ĺ���
int VectorAdd(int n_a, int n_b, double a[], double b[], double c[]);
int VectorSub(int n_a, int n_b, double a[], double b[], double c[]);
int VectorDotMul(int n_a, int n_b, double a[], double b[], double d[]);
int VectorCrossMul(int n_a, int n_b, double a[], double b[], double c[]);

//��������ĺ���
int MatrixAdd(int line_a, int row_a, int line_b, int row_b, double a[], double b[], double c[]);
int MatrixSub(int line_a, int row_a, int line_b, int row_b, double a[], double b[], double c[]);
int MatrixMul(int line_a, int row_a, int line_b, int row_b, double a[], double b[], double c[]);
int MatrixInv(int n, double a[], double b[]);
int MatrixTrans(int line_a, int row_a, int line_b, int row_b, double a[], double b[]);

//ʱ��ת������
int CommonTimeToMJDTime(int &year, int &month, int &day, int &hour, int &minute, double &second, int &days, double &fracday);
int MJDTimeToCommonTime(int &days, double &fracday, int &year, int &month, int &day, int &hour, int &minute, double &second);
int GPSTimeToMJDTime(int &week, double &secondofweek, int &days, double &fracday);
int MJDTimeToGPSTime(int &days, double &fracday, int &week, double &secondofweek);

//����ת������
void BLHToXYZ(double a, double f, BLH *blh, XYZ *xyz);
int XYZToBLH(double a, double f, BLH *blh, XYZ *xyz);

//����
unsigned short UI2(unsigned char *buf);
unsigned int UI4(unsigned char *buf);
float R4(unsigned char* buf);
double R8(unsigned char* buf);
int ReadFile(FILE *fp, char *filename, RAWDATA *RawData);
int DecodeOEM7Message(FILE *fp, RAWDATA *RawData);
unsigned int crc32(const unsigned char *buff, int len);
int DecodeOEM7Obs(unsigned char buf[], RAWDATA *RawData);
int DecodeGPSEph(unsigned char buf[], RAWDATA *RawData);
int DecodeBDSEph(unsigned char buf[], RAWDATA *RawData);
int DecodeIon(unsigned char buf[], RAWDATA *RawData);

//������
int GPS_SatPositionVelocity(GPSTIME *t, int &prn, CALCULATION *Calculation, RAWDATA *RawData, double &tao);
int BDS_SatPositionVelocity(GPSTIME *t, int &prn, CALCULATION *Calculation, RAWDATA *RawData, double &tao);
int GPS_SatClock(GPSTIME *t_sv, int &prn, CALCULATION *Calculation, RAWDATA *RawData);
int BDS_SatClock(GPSTIME *t_sv, int &prn, CALCULATION *Calculation, RAWDATA *RawData);
void ComputeAzimuthElement(double a, double f, XYZ *RecPos, XYZ *SatPos, double *azimuth, double *element);
//
double Klobuchar(GPSTIME *t, XYZ *RecPos, XYZ *SatPos, double &azimuth, double &element, RAWDATA *RawData);
int IonoFree(XYZ *RecPos, RAWDATA *RawData, CALCULATION *Calculation);
double Hopfield(XYZ *RecPos, double element);
void EarthRotate(XYZ *RecPos, double &tao);


int GPS_SignalTransmit(RAWDATA *RawData, CALCULATION *Calculation);
int BDS_SignalTransmit(RAWDATA *RawData, CALCULATION *Calculation);
void DeleteZero(double *arr, int n);
int SPP(RAWDATA *RawData, RECEIVE *Rcv, CALCULATION *Calculation);
int SPV(CALCULATION *Calculation, RAWDATA *RawData, XYZ *RecPos, XYZ *RecVel);

//����
bool OpenSocket(SOCKET& sock, const char IP[], const unsigned short Port);
void CloseSocket(SOCKET& sock);
int DecodeSocketsMessage(SOCKET& sock, RAWDATA *RawData, unsigned char *buf_all, int *len, int *i);
//int DecodeSocketsMessage_1(SOCKET& sock, RAWDATA *RawData, unsigned char *buf_all, int &len, int &i);