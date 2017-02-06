
/*
    Copyright 2010 University of Twente and Delft University of Technology
 
       This file is part of the Mapping libraries and tools, developed
  for research, education and projects in photogrammetry and laser scanning.

  The Mapping libraries and tools are free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation, either version 3 of the License,
                   or (at your option) any later version.

 The Mapping libraries and tools are distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
                GNU General Public License for more details.

      You should have received a copy of the GNU General Public License
          along with the Mapping libraries and tools.  If not, see
                      <http://www.gnu.org/licenses/>.

----------------------------------------------------------------------------*/



#ifndef __BUFFER_T___H_____
#define __BUFFER_T___H_____
#include <algorithm>
#include "BoxFilters.h"

//----------------------------------------------------------------------
//				2D buffer for managing hough space or other 2D matrices.
//----------------------------------------------------------------------
template<class T>
class Buffer2D
{
public:
	Buffer2D(int r=0,int c=0)
	:pData(NULL)
	{
		Allocate(r,c);
	}
	
	Buffer2D(int r,int c,const T& init)
	:pData(NULL)
	{
		Allocate(r,c);
		Fill(init);
	}
	
	///Allocate the space.
	void Allocate(int r,int c)
	{
		rows = r;
		cols = c;
		
		if(rows>=1 && cols>=1)
		{
			pData = new T[rows*cols];
			pRows = new T*[rows];
			
			for(int i=0;i<rows;i++)
				pRows[i] = pData + i*cols;
		}
	}
	
	///Save as pgm.
	void SavePGM(string filename)const
	{
		ofstream os(filename.c_str());
		
		if(os.bad())
			return;
			
		os<<"P5\n";
		os<<cols<<"  "<<rows<<"  "<<Max()<<endl;
		
		for(int j=0;j<rows;j++)
		{
			for(int k=0;k<cols;k++)
			{
				os<<(*this)(j,k)<<" ";
			}
			os<<endl;
		}
		os<<endl;
	}
	
	double SumSq()const
	{
	    double sum = 0;
		for(int i=0;i<rows*cols;i++)
			sum += pData[i]*pData[i];
		
		return sum;
	}
		
	
	///View as vrml.
	void ViewVRML(string str)const
	{
		LaserPoints pts;
		
		for(int j=0;j<rows;j++)
		{
			for(int k=0;k<cols;k++)
			{
				pts.push_back(LaserPoint(k,j,0,(*this)(j,k)));
			}
		}
		pts.ShowVrml(str);
	}
	
	///Deallocate the space
	void Deallocate()
	{
		delete[] pRows; pRows = NULL;
		delete[] pData; pData = NULL;
	}
	
	///Destructor.
	~Buffer2D()
	{
		Deallocate();
	}
	
	///Resize the array.
	void Resize(int r,int c)
	{
		Deallocate();
		Allocate(r,c);	
	}
	
	///Get number of rows.
	int Nrows()const
	{
		return rows;
	}
	
	///Get number of cols.
	int Ncols()const
	{
		return cols;
	}
	
	///Return data read only.
	const T* GetData()const
	{
		return pData;
	}
	
	///Copy constructor.
	Buffer2D(const Buffer2D&b)
	{
		if(this==&b)
			return;
		
		Deallocate();
		Allocate(b.Nrows(),b.Ncols());
		
		copy(b.GetData(),b.GetData() + b.Nrows()*b.Ncols(),pData);
	}
	
	///Return the element at i,j.
	const T& operator()(int i,int j)const
	{
		return pRows[i][j];
	}
	
	///Return the element at i,j read/write
	T& operator()(int i,int j)
	{
		return pRows[i][j];
	}
	
	///Find max value.
	T Max()const
	{
		int i,j;
		return Max(i,j);	
	}
	
	///Find max value with location.
	T Max(int &r, int & y)const
	{
		T mx = pData[0];
		r = 0; y = 0;
		
		for(int i=0;i<rows;i++)
		{
			for(int j=0;j<cols;j++)
			{
				if (pRows[i][j]>mx)
				{
					mx = pRows[i][j];
					r = i; y = j;
				}
			}
		}
		return mx;
	}
	
	///Find min value with location.
	T Min(int &r, int & y)const
	{
		T mn = pData[0];
		r = 0; y = 0;
		
		for(int i=0;i<rows;i++)
		{
			for(int j=0;j<cols;k++)
			{
				if (pRows[i][j]<mx)
				{
					mx = pRows[i][j];
					r = i; y = j;
				}
			}
		}
		return mn;
	}
	
	///Fill with a given value.
	void Fill(const T& t)
	{
		std::fill(pData,pData+rows*cols,t);
	}
	
	///Check if the given sub is valid.
	bool IsValid(int i,int j)const
	{
		if(pData && i>=0 && i<rows && j>=0 && j<cols)
			return true;
		return false;
	}
	
	///Box filter the data.
	void BoxFilter(int x,int y)
	{
		int s[]= {rows,cols};
		int f[] = {x,y};
		BoxFilter2(pRows,s,f);
	}
	
protected:
	T* pData;	
	T** pRows;
	int rows;
	int cols;

};


//----------------------------------------------------------------------
//				3D buffer for managing hough space or other 2D matrices.
//----------------------------------------------------------------------
template<class T>
class Buffer3D
{
public:
	Buffer3D(int r=0,int c=0,int t=0)
	:pData(NULL)
	{
		Allocate(r,c,t);
	}
	
	Buffer3D(int r,int c, int t,const T& init)
	:pData(NULL)
	{
		Allocate(r,c,t);
		Fill(init);
	}
	
	///Allocate the space.
	void Allocate(int r,int c,int t)
	{
		rows = r;
		cols = c;
		dim3 = t;
		
		if(rows>=1 && cols>=1 && dim3>=1)
		{
			pData = new T[rows*cols*dim3];
		}
	}
	
	///convert from index to subscript.
	void ind2sub(int ind, int& d1,int& d2,int& d3)const
	{
		d1 = ind/(cols*dim3);
		ind = d1* (cols*dim3);
		d2 = ind/dim3;
		d3 = ind - d2*dim3;                                                	
	}
	
	///convert from subscript to index.
	int sub2ind(int d1,int d2,int d3)const
	{
		return (d1*(cols*dim3)+d2*(dim3)+d3);                                                	
	}
	
	///Deallocate the space
	void Deallocate()
	{
		delete[] pData; pData = NULL;
	}
	
	///Destructor.
	~Buffer3D()
	{
		Deallocate();
	}
	
	///Resize the array.
	void Resize(int r,int c,int t)
	{
		Deallocate();
		Allocate(r,c,t);	
	}
	
	///Get number of rows.
	int Nrows()const
	{
		return rows;
	}
	
	///Get number of cols.
	int Ncols()const
	{
		return cols;
	}
	
	///Get number of third dimension.
	int Ndim3()const
	{
		return dim3;
	}
	
	///Return data read only.
	const T* GetData()const
	{
		return pData;
	}
	
	///Copy constructor.
	Buffer3D(const Buffer3D&b)
	{
		if(this==&b)
			return;
		
		Deallocate();
		Allocate(b.Nrows(),b.Ncols(),b.Ndim3());
		
		copy(b.GetData(),b.GetData() + b.Nrows()*b.Ncols()*b.Ndim3(),pData);
	}
	
	///Return the element at i,j.
	const T& operator()(int i,int j,int k)const
	{
		return pData[sub2ind(i,j,k)];
	}
	
	///Return the element at i,j read/write
	T& operator()(int i,int j,int k)
	{
		return pData[sub2ind(i,j,k)];
	}
	
	///Check if the given sub is valid.
	bool IsValid(int i,int j,int k)const
	{
		if(pData && i>=0 && i<rows && j>=0 && j<cols && k>=0 && k<dim3)
			return true;
		return false;
	}
			
	
	///Find max value.
	T Max()const
	{
		int i,j,k;
		return Max(i,j,k);	
	}
	
	///Find max value with location.
	T Max(int &r, int & y, int & z)const
	{
		T mx = pData[0];
		r = 0; y = 0;
		
		for(int i=0;i<rows;i++)
		{
			for(int j=0;j<cols;j++)
			{
				for(int k=0;k<dim3;k++)
				if ((*this)(i,j,k)>mx)
				{
					mx = (*this)(i,j,k);
					r = i; y = j; z = k;
				}
			}
		}
		return mx;
	}
	
	///Fill with a given value.
	void Fill(const T& t)
	{
		std::fill(pData,pData+rows*cols*dim3,t);
	}
	
protected:
	T* pData;	
	int rows;
	int cols;
	int dim3;

};


#endif // __BUFFER_T___H_____
