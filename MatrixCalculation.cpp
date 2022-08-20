#include "stdafx.h"
#include <iostream>
using namespace std;

/*************************************
MatrixAdd
Ŀ�ģ�����ӷ�

������
line_a  ����a����
row_a   ����a����
lin_b   ����b����
row_b   ����b����
a,b     �������
c       �������
����ֵ��1=������0=��������
**************************************/
int MatrixAdd(int line_a, int row_a, int line_b, int row_b, double a[], double b[], double c[])
{
	int i, j, line, row;
	if (line_a != line_b || row_a != row_b)
	{
		cout << "Error dimension in MatrixAdd!" << endl;
		return 0;
	}
	else
	{
		line = line_a;
		row = row_a;
		for (i = 0; i < line; i++)
		{
			for (j = 0; j < row; j++)
			{
				c[i*row + j] = a[i*row + j] + b[i*row + j];
			}
		}
		return 1;
	}
}

/*************************************
MatrixSub
Ŀ�ģ��������

������
line_a  ����a����
row_a   ����a����
lin_b   ����b����
row_b   ����b����
a,b     �������
c       �������
����ֵ��1=������0=��������
**************************************/
int MatrixSub(int line_a, int row_a, int line_b, int row_b, double a[], double b[], double c[])
{
	int i, j, line, row;
	if (line_a != line_b || row_a != row_b)
	{
		cout << "Error dimension in MatrixSub!" << endl;
		return 0;
	}
	else
	{
		line = line_a;
		row = row_a;
		for (i = 0; i < line; i++)
		{
			for (j = 0; j < row; j++)
			{
				c[i*row + j] = a[i*row + j] - b[i*row + j];
			}
		}
		return 1;
	}
}

/*************************************
MatrixMul
Ŀ�ģ�����˷�

������
a,b     �������
c       �������
line_a  ����a����
row_a   ����a����
lin_b   ����b����
row_b   ����b����
����ֵ��1=������0=��������
**************************************/
int MatrixMul(int line_a, int row_a, int line_b, int row_b, double a[], double b[], double c[])
{
	int i, j, k, m, n, q;
	if (row_a != line_b)
	{
		cout << "Error dimension in MatrixMul!" << endl;
		return 0;
	}
	else
	{
		m = line_a;
		n = row_a;
		q = row_b;
		for (i = 0; i < m; i++)
		{
			for (j = 0; j < q; j++)
			{
				c[i*q + j] = 0;
				for (k = 0; k < n; k++)
				{
					c[i*q + j] = c[i*q + j] + a[i*n + k] * b[k*q + j];
				}
			}
		}
		return 1;
	}
}

/*************************************
MatrixInv
Ŀ�ģ���������

������
n  ��������������
a  �������
b  �������b=inv(a)
����ֵ��1=������0=��������
**************************************/
int MatrixInv(int n, double a[], double b[])
{
	int i, j, k, l, u, v, is[10], js[10];   /* matrix dimension <= 10 */
	double d, p;

	if (n <= 0)
	{
		printf("Error dimension in MatrixInv!\n");
		exit(EXIT_FAILURE);
	}

	/* ���������ֵ���������b�������b�������棬a���󲻱� */
	for (i = 0; i<n; i++)
	{
		for (j = 0; j<n; j++)
		{
			b[i*n + j] = a[i*n + j];
		}
	}

	for (k = 0; k<n; k++)
	{
		d = 0.0;
		for (i = k; i<n; i++)   /* �������½Ƿ�������Ԫ�ص�λ�� */
		{
			for (j = k; j<n; j++)
			{
				l = n*i + j;
				p = fabs(b[l]);
				if (p>d)
				{
					d = p;
					is[k] = i;
					js[k] = j;
				}
			}
		}

		if (d<DBL_EPSILON)   /* ��Ԫ�ؽӽ���0�����󲻿��� */
		{
			printf("Divided by 0 in MatrixInv!\n");
			return 0;
		}

		if (is[k] != k)  /* ����Ԫ�����ڵ��������½Ƿ�������н��е��� */
		{
			for (j = 0; j<n; j++)
			{
				u = k*n + j;
				v = is[k] * n + j;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}

		if (js[k] != k)  /* ����Ԫ�����ڵ��������½Ƿ�������н��е��� */
		{
			for (i = 0; i<n; i++)
			{
				u = i*n + k;
				v = i*n + js[k];
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}

		l = k*n + k;
		b[l] = 1.0 / b[l];  /* �����б任 */
		for (j = 0; j<n; j++)
		{
			if (j != k)
			{
				u = k*n + j;
				b[u] = b[u] * b[l];
			}
		}
		for (i = 0; i<n; i++)
		{
			if (i != k)
			{
				for (j = 0; j<n; j++)
				{
					if (j != k)
					{
						u = i*n + j;
						b[u] = b[u] - b[i*n + k] * b[k*n + j];
					}
				}
			}
		}
		for (i = 0; i<n; i++)
		{
			if (i != k)
			{
				u = i*n + k;
				b[u] = -b[u] * b[l];
			}
		}
	}

	for (k = n - 1; k >= 0; k--)  /* ����������е������»ָ� */
	{
		if (js[k] != k)
		{
			for (j = 0; j<n; j++)
			{
				u = k*n + j;
				v = js[k] * n + j;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}
		if (is[k] != k)
		{
			for (i = 0; i<n; i++)
			{
				u = i*n + k;
				v = is[k] + i*n;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}
	}

	return 1;
}

/*************************************
MatrixTrans
Ŀ�ģ�����ת��

������
a  �������
line_a  ����a����
row_a   ����a����
b       �������b=trans(a)
line_b  ����b����
row_b   ����b����
����ֵ��1=������0=��������
**************************************/
int MatrixTrans(int line_a, int row_a, int line_b, int row_b, double a[], double b[])
{
	int i, j;
	if (line_a != row_b || row_a != line_b)
	{
		cout << "Error dimension in MatrixTrans!" << endl;
		return 0;
	}
	else
	{
		for (i = 0; i < line_a; i++)
		{
			for (j = 0; j < row_a; j++)
			{
				b[j*line_a + i] = a[i*row_a + j];
			}
		}
		return 1;
	}
}