
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



/*--------------------------------------------------------------------
*   Project   : Automated reconstruction of industrial installations
*
*   File made : September 2004 
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Class to encapsulate the K nearest neighbour search operation.
*				Build the tree in the constructor and then do the search.
*				Destroy the ANN structures in the destructor.
*
*--------------------------------------------------------------------*/

#ifndef __KNN_FINDER_H__
#define __KNN_FINDER_H__

#include <ANN.h>
#include <vector>
using std::vector;
#define BUCKET_SIZE 10
#define EPS_DEFAULT 1e-4
///Template for encapsulating the functionality of knn search from ANN library.
///
///The class T must support operator [] with 0,1,2.
///The constructors take vector<T> as we don't copy data internally, but use
///the memory of the vector, assuming that the implementation is using an array, and is 
///not moving the data around.

template <class T>
class  KNNFinder 
{
public:

	//Initialize all variables.
	void Init(int dimension=3)
	{
		queryPt = NULL;
		dataPts = NULL;
		nnIndices = NULL;
		pANNTree = NULL;
		dists = NULL;	
		kNNAllocated = 0;
		pointsInTree = 0; 
		reservedSize = 0;
		kNNDimension = dimension;
	}
		
	///Default Constructor.
	KNNFinder(int dimension=3)
	{
		Init(dimension);			
	}
	
	//Constructor With arguments.
	KNNFinder(const vector<T>& dataIn, int dimension=3)
	{
		Init(dimension);
		MakeTree(dataIn);	
	}
	
	///A utility function to allocate using laserpoints.
	ANNpointArray AllocPts(int n)	
	{
		ANNpointArray pa = new ANNpoint[n];		// allocate points

		return pa;
	}
	
	///A utility function to allocate using laserpoints.
	///Uses selected points only.
	void MakeTree(const vector<T>& ptsIn,const vector<int>& indices)	
	{
		Reserve(indices.size());
		//We need a non-const local version. Ugly but no way out.
		vector<T>& pts = *((vector<T>*)(&ptsIn));

		int n = indices.size();
		ANNpointArray pa = this->dataPts;

		typename vector<T>::iterator iter = pts.begin();
		for (int i = 0; i < n; i++) 
		{
			pa[i] = (ANNpoint)(&((*(iter+indices[i]))[0]));
		}
		
		delete pANNTree;
		//Get the ANN tree.
		pANNTree = new ANNkd_tree(dataPts,ptsIn.size(),kNNDimension,BUCKET_SIZE);	
		pointsInTree = ptsIn.size();
	}
	
	
	///A utility function to allocate using laserpoints.
	void MakeTree(const vector<T>& ptsIn)	
	{
		Reserve(ptsIn.size());
		//We need a non-const local version. Ugly but no way out.
		vector<T>& pts = *((vector<T>*)(&ptsIn));

		int n = ptsIn.size();
		ANNpointArray pa = this->dataPts;

		typename vector<T>::iterator iter = pts.begin();
		for (int i = 0; i < n; i++) 
		{
			pa[i] = (ANNpoint)(&((*(iter+i))[0]));
		}
		
		delete pANNTree;
		//Get the ANN tree.
		pANNTree = new ANNkd_tree(dataPts,ptsIn.size(),kNNDimension,BUCKET_SIZE);	
		pointsInTree = ptsIn.size();
	}
	
	///Expand points if necessary.
	void Reserve(int nSize, int kNNSize=256)
	{
		if(nSize>reservedSize)
		{
			FreeResources();
			AllocateResources(nSize, kNNSize);
			reservedSize = nSize;
		}
	}
	
	
	///A utility function to deallocate using laserpoints.
	void DeallocPts(ANNpointArray &pa)	
	{
		delete[] pa;
		pa = NULL;
	}
	
	///Allocate resources.
	void AllocateResources(int dataSize,int kNNSize=256)
	{
		kNNAllocated = kNNSize;
		
		//Allocate memory
		queryPt = annAllocPt(kNNDimension);			// allocate query point
		dataPts = AllocPts(dataSize);	        	// allocate data points
		nnIndices = new ANNidx[kNNAllocated];			// allocate near neigh indices
		dists = new ANNdist[kNNAllocated];	
		
	}
	///Free resources.
	void FreeResources()
	{
		if(!kNNAllocated)
			return;
			
		//Free resources.
		delete[] nnIndices;
		delete[] dists;
		delete(pANNTree);
		DeallocPts(dataPts);
		// Due to some bug, we don't deallocate the query point in case of two dimensions
		if (kNNDimension != 2) {
		  annDeallocPt(queryPt);
		
		//Set pointers to NULL.
          queryPt = NULL;
        }
		dataPts = NULL;
		nnIndices = NULL;
		pANNTree = NULL;
		dists = NULL;	
		kNNAllocated = 0;
		pointsInTree = 0;
	}
	
	///Validate kNN, if the arrays are not of enough size increase them.
	bool Validate(int kNN)
	{
		if(!queryPt || !dataPts || !nnIndices || !dists || !kNNAllocated)
		{
			cerr<<"Invalid KNNFinder object. Memory pointers are NULL"<<flush<<endl;
			cerr<<"queryPt "<<queryPt<<"  "
				<<"dataPts "<<dataPts<<"  "
				<<"nnIndices "<< nnIndices<<"  "
				<<"dists "<<dists<<"  "
				<<"kNNAllocated "<<kNNAllocated<<"  "<<endl;
			return false;
		}
		
		if(kNN<=kNNAllocated)
			return true;
			
		delete[] nnIndices;
		delete[] dists;
		
		kNNAllocated = max(kNN,max(2*kNNAllocated,256));
		nnIndices = new ANNidx[kNNAllocated];
		dists = new ANNdist[kNNAllocated];
		
		return true;
	}
	
	///Set using a vector<T> and all its points.
	void SetData(const vector<T>& newData)
	{
		MakeTree(newData);
	}
	
	///Set vector<T> but using only selected points.
	void SetData(const vector<T>& newData,const vector<int>& indices)
	{
		MakeTree(newData,indices);
	}
		
	
	///Search KNN.
	vector<double> FindDistances(const T& v,int kNN=1,double eps=EPS_DEFAULT)
	{
		Find(v,kNN,eps);
		
		vector<double> result(kNN);
		copy(dists,dists+kNN,result.begin());
		
		return result;
	
	}
	
	///Only return maximum distance.
	double FindDistance(const T& v,int kNN=1,double eps=EPS_DEFAULT)
	{
		Find(v,kNN,eps);
		return dists[kNN-1];
	}
	
	///Return maximum distances but for a container.
	vector<double> FindDistance(const vector<T>& v,int kNN=1,double eps=EPS_DEFAULT)
	{
		vector<double> result(v.size());
		
		for(int i=0;i<v.size();i++)
			result[i] = FindDistance(v[i],kNN,eps);
		return result;
	}
	
	///Return indices of neighbours for one query item.
	vector<int> FindIndices(const T& v,int kNN=1,double eps=EPS_DEFAULT)
	{
		Find(v,kNN,eps);
		vector<int> result(kNN);
		copy(nnIndices,nnIndices+kNN,result.begin());
		
		return result;
	}
	
	///Return index of nth point.
	int FindIndex(const T& v,int kNN=1,double eps = EPS_DEFAULT)
	{
		Find(v,kNN,eps);
		return nnIndices[kNN-1];
		
	}
	
	/// Return index and distance of the n'th point
	void FindIndexAndDistance(const T& v, int &index, double &distance,
	                          int kNN=1, double eps=EPS_DEFAULT)
	{
		Find(v, kNN, eps);
		index = nnIndices[kNN-1];
		distance = dists[kNN-1];
	}
	
	///Return both distances and indices.
	bool FindKnn(const T& v,int kNN,vector<double>& distances,vector<int>& indices,double eps=EPS_DEFAULT)
	{
		Find(v,kNN,eps);
		
		//copy the results.
		distances.resize(kNN);
		indices.resize(kNN);
		
		copy(dists,dists+kNN,distances.begin());
		copy(nnIndices,nnIndices+kNN,indices.begin());
		
		return true;
	}
	
	///Return both distances and indices. but for a container.
	bool FindKnn(const vector<T>& v,int kNN,vector<double>& distances,vector<int>& indices,double eps=EPS_DEFAULT)
	{
		//copy the results.
		distances.resize(v.size());
		indices.resize(v.size());
		
		for(int i=0;i<v.size();i++)
		{
			Find(v[i],kNN,eps);
			distances[i] = dists[kNN-1];
			indices[i] = nnIndices[kNN-1];
		}
		return true;
	}
	
	///Return indices for a container.
	vector<int> FindIndices(const vector<T>& v,int kNN=1,vector<double>* pDists = NULL,double eps=EPS_DEFAULT)
	{
		vector<int> indices(v.size());
				
		for(int i=0;i<v.size();i++)
		{
			Find(v[i],kNN,eps);
			indices[i] = nnIndices[kNN-1];
			
			if(pDists)
			{
				(*pDists)[i] = dists[kNN-1];			
			}
		}
		return indices;
	}

	
	///Destructor. Automatically frees the resources.
	//With arguments.
	~KNNFinder()
	{
		FreeResources();
	}
	
private:

	///internal search funcion.
	bool Find(const T& v,int kNN,double eps=EPS_DEFAULT)
	{
		if(!Validate(kNN))
			return false;
			
		for(int i=0;i<kNNDimension;i++)
			queryPt[i]= v[i];
		
		if(kNN>pointsInTree)
		{
			cerr<<"KNNFinder::Find requesting "<<kNN<<" neighbours from tree of "
				<<pointsInTree<<" returning maximum available points"<<endl;
			pANNTree->annkSearch(queryPt,pointsInTree,nnIndices,dists,eps);
			
			
			//Copy the value of maximum distance and index to all inidices higher than
			//the pointsIntree.
			for(int i=pointsInTree;i<kNN;i++)
			{
				nnIndices[i] = nnIndices[pointsInTree-1];
				dists[i] = dists[pointsInTree-1];
			}
				
			return false;
		}
		
		pANNTree->annkSearch(queryPt,kNN,nnIndices,dists,eps);
		return true;
	}
	
	///ANN related variables.
	ANNpointArray	dataPts;
    ANNpoint		queryPt;
    ANNidxArray		nnIndices;
    ANNdistArray	dists;
    ANNkd_tree		*pANNTree;
	int kNNAllocated;
	int pointsInTree;
	int reservedSize;
	int kNNDimension;
};
#endif // __KNN_FINDER_H__
