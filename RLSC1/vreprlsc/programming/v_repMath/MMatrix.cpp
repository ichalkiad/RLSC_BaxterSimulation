// This file is part of V-REP, the Virtual Robot Experimentation Platform.
// 
// Copyright 2006-2013 Dr. Marc Andreas Freese. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// V-REP is dual-licensed, under the terms of EITHER (at your option):
//   1. V-REP commercial license (contact us for details)
//   2. GNU GPL (see below)
// 
// GNU GPL license:
// -------------------------------------------------------------------
// V-REP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// V-REP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with V-REP.  If not, see <http://www.gnu.org/licenses/>.
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.0.4 on July 8th 2013

#include "MMatrix.h"
#include "mathDefines.h"



CMatrix::CMatrix()
{
}

CMatrix::CMatrix(int nRows,int nCols) : data  (new float[nRows*nCols]),rows(nRows),cols(nCols)
{
}

CMatrix::CMatrix(const C3X3Matrix& m) : data (new float[9])
{
	rows=3;
	cols=3;
	(*this)=m;
}

CMatrix::CMatrix(const C4X4Matrix& m) : data (new float[16])
{
	rows=4;
	cols=4;
	(*this)=m;
}

CMatrix::CMatrix(const C6X6Matrix& m) : data (new float[36])
{
	rows=6;
	cols=6;
	(*this)=m;
}

CMatrix::CMatrix(const CMatrix& m) : data (new float[m.rows*m.cols])
{
	rows=m.rows;
	cols=m.cols;
	(*this)=m;
}
 
CMatrix::~CMatrix()
{
   delete[] data;
} 

void CMatrix::clear()
{
	for (int i=0;i<(cols*rows);i++)
		data[i]=0.0f;
}

void CMatrix::setIdentity()
{
	for (int i=0;i<rows;i++)
	{
		for (int j=0;j<cols;j++)
		{
			if (i!=j)
				(*this)(i,j)=0.0f;
			else
				(*this)(i,j)=1.0f;
		}
	}
}

CMatrix CMatrix::operator* (const C3X3Matrix& m) const
{
	CMatrix retM(rows,3);
	for (int i=0;i<rows;i++)
	{
		for (int j=0;j<3;j++)
		{
			retM(i,j)=0.0f;
			for (int k=0;k<3;k++)
				retM(i,j)+=( (*this)(i,k)*m.axis[j](k) );
		}
	}
	return(retM);
}

CMatrix CMatrix::operator* (const C4X4Matrix& m) const
{
	CMatrix retM(rows,4);
	for (int i=0;i<rows;i++)
	{
		for (int j=0;j<3;j++)
		{
			retM(i,j)=0.0f;
			for (int k=0;k<3;k++)
				retM(i,j)+=( (*this)(i,k)*m.M.axis[j](k) );
		}
		retM(i,3)=(*this)(i,3);
		for (int k=0;k<3;k++)
			retM(i,3)+=( (*this)(i,k)*m.X(k) );
	}
	return(retM);
}

CMatrix CMatrix::operator* (const C6X6Matrix& m) const
{
	CMatrix retM(rows,6);
	for (int i=0;i<rows;i++)
	{
		for (int j=0;j<6;j++)
		{
			retM(i,j)=0.0f;
			for (int k=0;k<6;k++)
				retM(i,j)+=( (*this)(i,k)*m(k,j) );
		}
	}
	return(retM);
}

CMatrix CMatrix::operator* (const CMatrix& m) const
{
	CMatrix retM(rows,m.cols);
	for (int i=0;i<rows;i++)
	{
		for (int j=0;j<m.cols;j++)
		{
			retM(i,j)=0.0f;
			for (int k=0;k<cols;k++)
				retM(i,j)+=( (*this)(i,k)*m(k,j) );
		}
	}
	return(retM);
}

CMatrix CMatrix::operator+ (const CMatrix& m) const
{
	CMatrix retM(rows,cols);
	for (int i=0;i<(rows*cols);i++)
		retM.data[i]=data[i]+m.data[i];
	return(retM);
}

CMatrix CMatrix::operator- (const CMatrix& m) const
{
	CMatrix retM(rows,cols);
	for (int i=0;i<(rows*cols);i++)
		retM.data[i]=data[i]-m.data[i];
	return(retM);
}

CMatrix CMatrix::operator* (float d) const
{
	CMatrix retM(rows,cols);
	for (int i=0;i<(rows*cols);i++)
		retM.data[i]=data[i]*d;
	return(retM);
}

CMatrix CMatrix::operator/ (float d) const
{
	CMatrix retM(rows,cols);
	for (int i=0;i<(rows*cols);i++)
		retM.data[i]=data[i]/d;
	return(retM);
}

void CMatrix::operator*= (const CMatrix& m)
{
	(*this)=(*this)*m;
}

void CMatrix::operator+= (const CMatrix& m)
{
	for (int i=0;i<(rows*cols);i++)
		data[i]+=m.data[i];
}

void CMatrix::operator-= (const CMatrix& m)
{
	for (int i=0;i<(rows*cols);i++)
		data[i]-=m.data[i];
}

void CMatrix::operator*= (float d)
{
	for (int i=0;i<(rows*cols);i++)
		data[i]*=d;
}

void CMatrix::operator/= (float d)
{
	for (int i=0;i<(rows*cols);i++)
		data[i]/=d;
}


CVector CMatrix::operator* (const CVector& v) const
{
	CVector retV(v.elements);
	for (int i=0;i<rows;i++)
	{
		retV(i)=0.0f;
		for (int k=0;k<cols;k++)
			retV(i)+=( (*this)(i,k)*v(k) );
	}
	return(retV);
}

CMatrix& CMatrix::operator= (const C3X3Matrix& m)
{
	for (int i=0;i<3;i++)
	{
		for (int j=0;j<3;j++)
			(*this)(i,j)=m.axis[j](i);
	}
	return(*this);
}

CMatrix& CMatrix::operator= (const C4X4Matrix& m)
{
	for (int i=0;i<3;i++)
	{
		for (int j=0;j<3;j++)
			(*this)(i,j)=m.M.axis[j](i);
		(*this)(i,3)=m.X(i);
	}
	(*this)(3,0)=0.0f;
	(*this)(3,1)=0.0f;
	(*this)(3,2)=0.0f;
	(*this)(3,3)=1.0f;
	return(*this);
}

CMatrix& CMatrix::operator= (const C6X6Matrix& m)
{
	for (int i=0;i<6;i++)
	{
		for (int j=0;j<6;j++)
			(*this)(i,j)=m(i,j);
	}
	return(*this);
}

CMatrix& CMatrix::operator= (const CMatrix& m)
{
	int t=rows*cols;
	for (int i=0;i<t;i++)
		data[i]=m.data[i];
	return(*this);
}

void CMatrix::transpose()
{ // Write a faster routine later!	
	CMatrix n(cols,rows);
	for (int i=0;i<rows;i++)
	{
		for (int j=0;j<cols;j++)
			n(j,i)=(*this)(i,j);
	}
	rows=n.rows;
	cols=n.cols;
	(*this)=n;
}

bool CMatrix::inverse()
{
// I found the following routine on internet (numerical recipes) and modified it.
// Linear equation solution by Gauss-Jordan elimination:
	int n=rows;
	int i,irow,j,k,l,ll;
	float big,dum,pivinv;
	int* indxc=new int[n+1];
	int* indxr=new int[n+1];
	int* ipiv=new int[n+1];
	for (j=1;j<=n;j++)
		ipiv[j]=0;
	for (i=1;i<=n;i++)
	{
		int icol=-1;
		big=0.0f;
		for (j=1;j<=n;j++)
		{
			if (ipiv[j] != 1)
			{
				for (k=1;k<=n;k++)
				{
					if (ipiv[k] == 0)
					{
						if (fabs((*this)(j-1,k-1)) >= big)
						{
							big=(float)fabs((*this)(j-1,k-1));
							irow=j;
							icol=k;
						}
					}
					else if (ipiv[k] > 1)
					{	// The system cannot be solved
						delete[] ipiv;
						delete[] indxr;
						delete[] indxc;
						return(false);
					}
				}
			}
		}
		if (icol==-1)
		{ // There are probably nan numbers in the matrix!
			delete[] ipiv;
			delete[] indxr;
			delete[] indxc;
			return(false);
		}
		++(ipiv[icol]);

		if (irow != icol)
		{
			for (l=1;l<=n;l++)
			{
				float tmp=(*this)(irow-1,l-1);
				(*this)(irow-1,l-1)=(*this)(icol-1,l-1);
				(*this)(icol-1,l-1)=tmp;
			}
		}
		indxr[i]=irow;
		indxc[i]=icol;
		if ((*this)(icol-1,icol-1) == 0.0f)
		{	// The system cannot be solved
			delete[] ipiv;
			delete[] indxr;
			delete[] indxc;
			return(false);
		}
		pivinv=1.0f/(*this)(icol-1,icol-1);
		(*this)(icol-1,icol-1)=1.0f;
		for (l=1;l<=n;l++)
			(*this)(icol-1,l-1) *= pivinv;
		for (ll=1;ll<=n;ll++)
		{
			if (ll != icol)
			{
				dum=(*this)(ll-1,icol-1);
				(*this)(ll-1,icol-1)=0.0f;
				for (l=1;l<=n;l++)
					(*this)(ll-1,l-1) -= (*this)(icol-1,l-1)*dum;
			}
		}
	}

	for (l=n;l>=1;l--)
	{
		if (indxr[l] != indxc[l])
		{
			for (k=1;k<=n;k++)
			{
				float tmp=(*this)(k-1,indxr[l]-1);
				(*this)(k-1,indxr[l]-1)=(*this)(k-1,indxc[l]-1);
				(*this)(k-1,indxc[l]-1)=tmp;
			}
		}
	}
	delete[] ipiv;
	delete[] indxr;
	delete[] indxc;
	return(true);
}
