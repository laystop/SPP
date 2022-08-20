#pragma once

#include<stdio.h>
#include<windows.h>
#include "stdafx.h"
#include<iostream>
using namespace std;

#pragma comment(lib,"WS2_32.lib")
#pragma warning(disable:4996)

bool OpenSocket(SOCKET& sock, const char IP[], const unsigned short Port)
{
	WSADATA wsaData;
	SOCKADDR_IN addrSrv;

	if (!WSAStartup(MAKEWORD(1, 1), &wsaData))
	{
		if ((sock = socket(AF_INET, SOCK_STREAM, 0)) != INVALID_SOCKET)
		{
			addrSrv.sin_addr.S_un.S_addr = inet_addr(IP);
			addrSrv.sin_family = AF_INET;
			addrSrv.sin_port = htons(Port);
			connect(sock, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR));
			return true;
		}
	}
	return false;
}

void CloseSocket(SOCKET& sock)
{
	closesocket(sock);
	WSACleanup();
}

unsigned short UI2(unsigned char *buf)
{
	unsigned short r;
	memcpy(&r, buf, 2);
	return r;
}

unsigned int UI4(unsigned char *buf)
{
	unsigned int r;
	memcpy(&r, buf, 4);
	return r;
}

float R4(unsigned char* buf)
{
	float r;
	memcpy(&r, buf, 4);
	return r;
}

double R8(unsigned char* buf)
{
	double r;
	memcpy(&r, buf, 8);
	return r;
}

/*************************************************************************
DecodeSocketsMessage
Ŀ�ģ������������ļ��ж�ȡ��Message��������

������
sock      ����
RawData   �۲����ݡ�����������
buf_all   ��ȡ��ȫ������
len       ���ݳ���
i         ָ����ȫ�������е�λ��

����ֵ��0=crcУ��ʧ�� MsgID_Obs=����۲�ֵ MsgID_GPSEph/MsgID_BDSEph=�������� MshID_ion=�����������
***************************************************************************/
int DecodeSocketsMessage(SOCKET& sock, RAWDATA *RawData, unsigned char *buf_all, int *len, int *i)
{
	unsigned short MsgID, MsgLen;
	unsigned char buff[MAXRAWLEN];
	unsigned char *buf;
	unsigned char find_head[3];
	find_head[0] = 0;
	find_head[1] = 0;
	find_head[2] = 0;
	int obs_flag = 0;
	int long_buf;

	//��AA 44 12
	for ((*i); (*i) < (*len); (*i)++)
	{
		if (find_head[0] == 0xAA && find_head[1] == 0x44 && find_head[2] == 0x12)
		{
			find_head[0] = 0;
			find_head[1] = 0;
			find_head[2] = 0;
			break;
		}
		else
		{
			find_head[0] = find_head[1];
			find_head[1] = find_head[2];
			find_head[2] = buf_all[(*i)];
		}
	}
	buf = buf_all + ((*i) - 3);
	MsgLen = UI2(buf + 8);
	MsgID = UI2(buf + 4);

	//CRCУ��
	if (crc32(buf, 28 + MsgLen) != UI4(buf + 28 + MsgLen))
	{
		if (MsgLen + (*i) > (*len))
		{
			(*i) = 0;
			memset(&buff, 0, MAXRAWLEN * sizeof(unsigned char));
			Sleep(100);
			long_buf = recv(sock, (char*)buff, MAXRAWLEN, 0);
			if (long_buf == -1)
				return 0;
			memcpy(buf_all + (*len), buff, long_buf);
			(*len) = long_buf + (*len);
			return 0;
		}
		else
		{
			return 0;
		}
	}

	//���ý�۲�ֵ�ĺ���
	switch (MsgID)
	{
	case MsgID_Obs:DecodeOEM7Obs(buf, RawData); return MsgID_Obs;break;
	case MsgID_GPSEph:DecodeGPSEph(buf, RawData); return MsgID_GPSEph; break;
	case MsgID_BDSEph:DecodeBDSEph(buf, RawData); return MsgID_BDSEph; break;
	case MsgID_ion:DecodeIon(buf, RawData); return MsgID_ion; break;
	default:break;
	}
	return 0;
}

/*
int DecodeSocketsMessage_1(SOCKET& sock, RAWDATA *RawData, unsigned char *buf_all, int &len, int &i)
{
	unsigned short MsgID, MsgLen;
	unsigned char buff[MAXRAWLEN];
	unsigned char *buf;
	unsigned char find_head[3];
	find_head[0] = 0;
	find_head[1] = 0;
	find_head[2] = 0;
	int obs_flag = 0;
	int long_buf;

	//��AA 44 12
	for (i;i < len;i++)
	{
		if (find_head[0] == 0xAA && find_head[1] == 0x44 && find_head[2] == 0x12)
		{
			find_head[0] = 0;
			find_head[1] = 0;
			find_head[2] = 0;
			break;
		}
		else
		{
			find_head[0] = find_head[1];
			find_head[1] = find_head[2];
			find_head[2] = buf_all[i];
		}

		buf = buf_all + (i - 3);
		MsgLen = UI2(buf + 8);
		MsgID = UI2(buf + 4);
		//CRCУ��
		if (crc32(buf, 28 + MsgLen) != UI4(buf + 28 + MsgLen))
		{
			if (MsgLen + i > len)
			{
				i = 0;
				memset(&buff, 0, MAXRAWLEN * sizeof(unsigned char));
				Sleep(100);
				long_buf = recv(sock, (char*)buff, MAXRAWLEN, 0);
				if (long_buf == -1)
					return 0;
				memcpy(buf_all + len, buff, long_buf);
				len = long_buf + len;
				return 0;
			}
			else
			{
				return 0;
			}

		}

		//���ý�۲�ֵ�ĺ���
		switch (MsgID)
		{
		case MsgID_Obs:DecodeOEM7Obs(buf, RawData); return MsgID_Obs;break;
		case MsgID_GPSEph:DecodeGPSEph(buf, RawData); return MsgID_GPSEph; break;
		case MsgID_BDSEph:DecodeBDSEph(buf, RawData); return MsgID_BDSEph; break;
		case MsgID_ion:DecodeIon(buf, RawData); return MsgID_ion; break;
		default:break;
		}
		return 0;
	}
}
*/