#include "stdafx.h"
#include <iostream>
using namespace std;

/*************************************
VectorAdd
Ŀ�ģ������ӷ�

������
a,b  ��������
n_a  ����aԪ�ظ���
n_b  ����bԪ�ظ���
c    ������
����ֵ��1=������0=��������
**************************************/
int VectorAdd(int n_a, int n_b, double a[], double b[], double c[])
{
	int i, n;
	if (n_a != n_b)
	{
		cout << "Error dimension in VectorAdd!" << endl;
		return 0;
	}
	else
	{
		n = n_a;
		for (i = 0; i < n; i++)
		{
			c[i] = a[i] + b[i];
		}
		return 1;
	}
}

/*************************************
VectorSub
Ŀ�ģ���������

������
a,b  ��������
n_a  ����aԪ�ظ���
n_b  ����bԪ�ظ���
c    ������
����ֵ��1=������0=��������
**************************************/
int VectorSub(int n_a, int n_b, double a[], double b[], double c[])
{
	int i, n;
	if (n_a != n_b)
	{
		cout << "Error dimension in VectorSub!" << endl;
		return 0;
	}
	else
	{
		n = n_a;
		for (i = 0; i < n; i++)
		{
			c[i] = a[i] - b[i];
		}
		return 1;
	}
}

/*************************************
VectorDotMul
Ŀ�ģ��������

������
a,b  ��������
n_a  ����aԪ�ظ���
n_b  ����bԪ�ظ���
d    ������
����ֵ��1=������0=��������
**************************************/
int VectorDotMul(int n_a, int n_b, double a[], double b[], double d[])
{
	int i, n;
	if (n_a != n_b)
	{
		cout << "Error dimension in VectorDotMul!" << endl;
		return 0;
	}
	else
	{
		n = n_a;
		d[0] = 0;
		for (i = 0; i < n; i++)
		{
			d[0] = d[0] + a[i] * b[i];
		}
		return 1;
	}
}

/*************************************
VectorCrossMul
Ŀ�ģ��������

������
a,b  ��������
n_a  ����aԪ�ظ���
n_b  ����bԪ�ظ���
c    ������
����ֵ��1=������0=��������
**************************************/
int VectorCrossMul(int n_a, int n_b, double a[], double b[], double c[])
{
	int n = n_a;
	if (n_a == n_b&&n_a == 2)
	{
		c[0] = a[1] * b[0] - a[0] * b[1];
		return 1;
	}
	else if (n_a == n_b&&n_a == 3)
	{
		c[0] = a[1] * b[2] - a[2] * b[1];
		c[1] = a[2] * b[0] - a[0] * b[2];
		c[2] = a[0] * b[1] - a[1] * b[0];
		return 1;
	}
	else
	{
		cout << "Error dimension in VectorCrossMul!" << endl;
		return 0;
	}
}