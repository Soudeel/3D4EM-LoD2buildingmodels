
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



#ifndef __Box__Filters___h____
#define __Box__Filters___h____
/** Apply box filter to two dimensional data 

	@param ppData 2D-input data (Modified on output)
	@param sizes[] array containing x and y sizes of ppData
	@param filterSizes[] array containing x and y size of filter.
	
	@ return void
*/	
template <class T>		
void BoxFilter2(T** ppData, int sizes[],int filterSizes[])
{
	int i,j,k,t;
	double dSum;
	for(i=0;i<2;i++)
	{
		if(filterSizes[i]>sizes[i])
		{
			return;
		}
	}
	
	int tempSize = max(sizes[0],sizes[1])+max(filterSizes[0],filterSizes[1])+10;
	T* pBuffer = new T[tempSize];
	T* pTemp;
	int leftSizes[] = {filterSizes[0]/2,filterSizes[1]/2};
	int rightSizes[] = {filterSizes[0]-leftSizes[0],filterSizes[1]-leftSizes[1]};
	
	//Filter along first direction.
	for(i=0;i<sizes[1];i++)
	{
		pTemp = pBuffer + leftSizes[0];		
		
		//fill temporary array.
		for(j=-leftSizes[0];j<0;j++)
			pTemp[j] = 0;
		for(j=sizes[0];j<(sizes[0]+rightSizes[0]+1);j++)
			pTemp[j] = 0;
		for(j=0;j<sizes[0];j++)
			pTemp[j] = ppData[j][i];
			
		//calculate first sum.
		dSum = 0;
		for(j=0;j<leftSizes[0];j++)
			dSum += pTemp[j];
		ppData[0][i] = dSum;
		
		for(j=1;j<sizes[0];j++)
		{
			dSum += pTemp[j+rightSizes[0]-1];
			dSum -= pTemp[j-leftSizes[0]-1];
			ppData[j][i]=dSum;
		}
	}
	
	//Filter along second direction.
	for(i=0;i<sizes[0];i++)
	{
		pTemp = pBuffer + leftSizes[1];		
		
		//fill temporary array.
		for(j=-leftSizes[1];j<0;j++)
			pTemp[j] = 0;
		for(j=sizes[1];j<(sizes[1]+rightSizes[1]+1);j++)
			pTemp[j] = 0;
		for(j=0;j<sizes[1];j++)
			pTemp[j] = ppData[i][j];
			
		//calculate first sum.
		dSum = 0;
		for(j=0;j<leftSizes[1];j++)
			dSum += pTemp[j];
		ppData[i][0] = dSum;
		
		for(j=1;j<sizes[1];j++)
		{
			dSum += pTemp[j+rightSizes[1]-1];
			dSum -= pTemp[j-leftSizes[1]];
			ppData[i][j]=dSum;
		}
	}
	delete[] pBuffer;
}					

/** Apply box filter to three dimensional data 

	@param pppData 3D-input data (Modified on output)
	@param sizes[] array containing x,y,z sizes of pppData
	@param filterSizes[] array containing x,y and z size of filter.
	
	@ return void
*/	

template <class T>		
void BoxFilter3(T*** ppData, int sizes[],int filterSizes[])
{
	int i,j,k,t;
	double dSum;
	for(i=0;i<3;i++)
	{
		if(filterSizes[i]>sizes[i])
		{
			return;
		}
	}
	
	int tempSize = max(sizes[0],max(sizes[1],sizes[2]))+max(filterSizes[0],max(filterSizes[1],sizes[2]))+10;
	T* pBuffer = new T[tempSize];
	T* pTemp;
	int leftSizes[] = {filterSizes[0]/2,filterSizes[1]/2,filterSizes[2]/2};
	int rightSizes[] = {filterSizes[0]-leftSizes[0],filterSizes[1]-leftSizes[1],filterSizes[2]-leftSizes[2]};
	
	//Filter along first direction.
	for(k=0;k<sizes[2];k++)
	for(i=0;i<sizes[1];i++)
	{
		pTemp = pBuffer + leftSizes[0];		
		
		//fill temporary array.
		for(j=-leftSizes[0];j<0;j++)
			pTemp[j] = 0;
		for(j=sizes[0];j<(sizes[0]+rightSizes[0]+1);j++)
			pTemp[j] = 0;
		for(j=0;j<sizes[0];j++)
			pTemp[j] = ppData[j][i][k];
			
		//calculate first sum.
		dSum = 0;
		for(j=0;j<leftSizes[0];j++)
			dSum += pTemp[j];
		ppData[0][i][k] = dSum;
		
		for(j=1;j<sizes[0];j++)
		{
			dSum += pTemp[j+rightSizes[0]-1];
			dSum -= pTemp[j-leftSizes[0]-1];
			ppData[j][i][k]=dSum;
		}
	}
	
	//Filter along second direction.
	for(k=0;k<sizes[2];k++)
	for(i=0;i<sizes[0];i++)
	{
		pTemp = pBuffer + leftSizes[1];		
		
		//fill temporary array.
		for(j=-leftSizes[1];j<0;j++)
			pTemp[j] = 0;
		for(j=sizes[1];j<(sizes[1]+rightSizes[1]+1);j++)
			pTemp[j] = 0;
		for(j=0;j<sizes[1];j++)
			pTemp[j] = ppData[i][j][k];
			
		//calculate first sum.
		dSum = 0;
		for(j=0;j<leftSizes[1];j++)
			dSum += pTemp[j];
		ppData[i][0][k] = dSum;
		
		for(j=1;j<sizes[1];j++)
		{
			dSum += pTemp[j+rightSizes[1]-1];
			dSum -= pTemp[j-leftSizes[1]];
			ppData[i][j][k]=dSum;
		}
	}
	
	//Filter along third direction.
	for(k=0;k<sizes[1];k++)
	for(i=0;i<sizes[0];i++)
	{
		pTemp = pBuffer + leftSizes[2];		
		
		//fill temporary array.
		for(j=-leftSizes[2];j<0;j++)
			pTemp[j] = 0;
		for(j=sizes[1];j<(sizes[2]+rightSizes[2]+1);j++)
			pTemp[j] = 0;
		for(j=0;j<sizes[2];j++)
			pTemp[j] = ppData[i][k][j];
			
		//calculate first sum.
		dSum = 0;
		for(j=0;j<leftSizes[2];j++)
			dSum += pTemp[j];
		ppData[i][k][0] = dSum;
		
		for(j=1;j<sizes[2];j++)
		{
			dSum += pTemp[j+rightSizes[2]-1];
			dSum -= pTemp[j-leftSizes[2]];
			ppData[i][k][j]=dSum;
		}
	}

	delete[] pBuffer;
}
/** Apply box filter to one dimensional data 

	@param pData 1D-input data (Modified on output)
	@param sizes contains size of Data
	@param filterSize gives size of filter.
	
	@ return void
*/	
template <class T>		
void BoxFilter1(T* pData, int size,int filterSize)
{
	int i,j,k,t;
	double dSum;
	if(filterSize>size)
	{
		return;
	}
	
	int tempSize =size+filterSize+10;
	T* pBuffer = new T[tempSize];
	T* pTemp;
	int leftSize = filterSize/2;
	int rightSize = filterSize-leftSize;
	
	//Filter along first direction.
	pTemp = pBuffer + leftSize;		
		
	//fill temporary array.
	for(j=-leftSize;j<0;j++)
		pTemp[j] = 0;
		
	for(j=size;j<(size+rightSize+1);j++)
		pTemp[j] = 0;
	for(j=0;j<size;j++)
		pTemp[j] = pData[j];
			
	//calculate first sum.
	dSum = 0;
	for(j=0;j<leftSize;j++)
		dSum += pTemp[j];
	pData[0] = dSum;
		
	for(j=1;j<size;j++)
	{
		dSum += pTemp[j+rightSize-1];
		dSum -= pTemp[j-leftSize-1];
		pData[j]=dSum;
	}
}

#endif
