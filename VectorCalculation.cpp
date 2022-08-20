#include "stdafx.h"
#include <iostream>
using namespace std;

/*************************************
VectorAdd
目的：向量加法

参数：
a,b  输入向量
n_a  向量a元素个数
n_b  向量b元素个数
c    输出结果
返回值：1=正常，0=致命错误
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
目的：向量减法

参数：
a,b  输入向量
n_a  向量a元素个数
n_b  向量b元素个数
c    输出结果
返回值：1=正常，0=致命错误
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
目的：向量点积

参数：
a,b  输入向量
n_a  向量a元素个数
n_b  向量b元素个数
d    输出结果
返回值：1=正常，0=致命错误
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
目的：向量叉积

参数：
a,b  输入向量
n_a  向量a元素个数
n_b  向量b元素个数
c    输出结果
返回值：1=正常，0=致命错误
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