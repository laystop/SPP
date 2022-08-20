// SPP.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "struct.h"
#include<iostream>
#include<iomanip>
#include<cmath>
#include<fstream>
#include<string>
#include<Windows.h>

using namespace std;

int main()
{
	//char *filename = "D:\\SPP\\OEM719\\202010260900.oem719";
	char *filename = "D:\\SPP\\OEM719\\202011021550.oem719";
	//char *filename = "D:\\SPP\\LongSPPData";
	ofstream outfile("D:\\SPP\\RecPos_11_P4.txt", ios::out);
	FILE *fp;
	RAWDATA RawData;
	CALCULATION Calculation;
	int value;
	RECEIVE Rcv;
	SOCKET sock;
	Ipinfo CfgInfo_oem;
	unsigned char buf[MAXRAWLEN];
	int long_buf, mode, index, Sock_flag;
	double sow;
	sow = 0;
	//cout << "选择定位模式  1=文件  2=网口" << endl;
	//cin >> mode;
	mode = 1;

	/*读文件*/
	if (mode == 1)
	{
		value = 0;
		memset(&Rcv, 0, sizeof(RECEIVE));

		if ((fp = fopen(filename, "rb")) == NULL)
		{
			cout << "File open error！" << endl;
			return 0;
		}
		while (!feof(fp))
		{
			memset(&(RawData.Obs), 0, sizeof(OBS));
			memset(&Calculation, 0, sizeof(CALCULATION));
			value = DecodeOEM7Message(fp, &RawData); //解码
			if (value == MsgID_Obs)
			{
				if (SPP(&RawData, &Rcv, &Calculation) == 1);
				{
					SPV(&Calculation, &RawData, &Rcv.RecPos, &Rcv.RecVel);
					XYZToBLH(CGCS2000_a, CGCS2000_f, &Rcv.RecBlh, &Rcv.RecPos);
					/*outfile << fixed << setprecision(8) << Rcv.time.Week << "  " << Rcv.time.SecofWeek << "  X:" << Rcv.RecPos.x << "  Y:"
						<< Rcv.RecPos.y << "  Z:" << Rcv.RecPos.z << "  B:" << Rcv.RecBlh.b*180.0 / PI << "  L:" << Rcv.RecBlh.l*180.0 / PI << "  H:" << Rcv.RecBlh.h
						<< "  GPS Clk:" << Rcv.delta_t_receive[0] << "  BDS Clk:" << Rcv.delta_t_receive[1] << endl; */
					outfile << fixed << setprecision(8) << Rcv.time.Week << "  " << Rcv.time.SecofWeek << "  " << Rcv.RecPos.x << "  "
						<< Rcv.RecPos.y << "  " << Rcv.RecPos.z << "  " << Rcv.RecBlh.b*180.0 / PI << "  " << Rcv.RecBlh.l*180.0 / PI
						<< "  " << Rcv.RecBlh.h << "   " << Rcv.delta_t_receive[0] << "   " << Rcv.delta_t_receive[1] << "  "
						<< Rcv.sigma << "  " << Rcv.PDOP << "  " << Rcv.SatNum << endl;
				}
			}
		}
		fclose(fp);
	}

	/*读网口*/
	else if (mode == 2)
	{
		Sock_flag = OpenSocket(sock, CfgInfo_oem.IP, CfgInfo_oem.Port);
		memset(&Rcv, 0, sizeof(RECEIVE));
		while (Sock_flag)
		{
			memset(&(RawData.Obs), 0, sizeof(OBS));
			memset(&Calculation, 0, sizeof(CALCULATION));
			memset(&buf, 0, MAXRAWLEN * sizeof(unsigned char));
			Sleep(999);
			long_buf = recv(sock, (char*)buf, MAXRAWLEN, 0);
			index = 0;
			while (index < long_buf)
			{
				value = DecodeSocketsMessage(sock, &RawData, buf, &long_buf, &index); //解码
				//value = DecodeSocketsMessage_1(sock, &RawData, buf, long_buf, index);
				if (value == MsgID_Obs)
				{
					if (SPP(&RawData, &Rcv, &Calculation) == 1)
					{
						SPV(&Calculation, &RawData, &Rcv.RecPos, &Rcv.RecVel);
						XYZToBLH(CGCS2000_a, CGCS2000_f, &Rcv.RecBlh, &Rcv.RecPos);
						if (sow == 0 || sow != RawData.Obs.ObsTime.SecofWeek)
						{
							cout << fixed << setprecision(4) << Rcv.time.Week << "  " << Rcv.time.SecofWeek << "  X:" << Rcv.RecPos.x << "  Y:"
								<< Rcv.RecPos.y << "  Z:" << Rcv.RecPos.z << "  B:" << Rcv.RecBlh.b*180.0 / PI << "  L:" << Rcv.RecBlh.l*180.0 / PI << "  H:" << Rcv.RecBlh.h
								<< "  GPS Clk:" << Rcv.delta_t_receive[0] << "  BDS Clk:" << Rcv.delta_t_receive[1] << endl;
						}
						sow = RawData.Obs.ObsTime.SecofWeek;
					}
				}
			}
		}
	}

	return 0;
}