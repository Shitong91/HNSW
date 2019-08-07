/*
 * ann_kd implementation
 *
 * Copyright (C) Shitong Du 
 *
 * Released under the GPL version 3.
 *
 */



#ifdef _MSC_VER
#define  _USE_MATH_DEFINES
#endif

#include <fstream>
#include "slam6d/hnsw.h"
#include "slam6d/globals.icc"          
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <algorithm>
using std::swap;
#include <cmath>
#include <cstring>



/**
 * Constructor
 *
 * Create ANN KD tree from the points pointed to by the array pts
 *
 * @param pts 3D array of points
 * @param n number of points
 */
HNSW::HNSW(PointerArray<double>&_pts,  size_t n)
{
    char path_index[4096];
    int efConstruction =150;
    int M =16 ;
    size_t vecdim = 3;
    sprintf(path_index, "close_%dm_ef_%d_M_%d.bin", n, efConstruction, M);
    L2Space l2space(vecdim);
    hnswgraph = new HierarchicalNSW<float>(&l2space, n, M, efConstruction);
    size_t j1 = 0; 
  pts = new double*[n];
  tpts = new double[3*n];
  float sq[3];
  //double* tpts = new double[3*n];
  for( size_t i = 0, j = 0; i < n; i++) {
     pts[i] = &tpts[j];
     tpts[j++] = _pts.get()[i][0];
     tpts[j++] = _pts.get()[i][1];
     tpts[j++] = _pts.get()[i][2];
    sq[0] =_pts.get()[i][0];
    sq[1] = _pts.get()[i][1];
    sq[2] = _pts.get()[i][2];
    hnswgraph->addPoint((void *)sq, (size_t) j1++);  
  }
    hnswgraph->saveIndex(path_index);
   hnswgraph->ef_=50;
}

/**
 * Destructor
 *
 * Cleans up the instance of HNSW Graph
 *
 *
 */
HNSW::~HNSW()
{
  delete hnswgraph; //links to the destructor of ANNkd_tree
 // delete [] nn;
  //delete [] nn_idx;
 // delete [] pts[0]; 
  delete [] pts;
  delete [] tpts;
}


/**
 * Finds the closest point within the tree,
 * wrt. the point given as first parameter.
 * @param _p point
 * @param maxdist2 maximal search distance.
 * @param threadNum Thread number, for parallelization
 * @return Pointer to the closest point
 */  
double *HNSW::FindClosest(double *_p, double maxdist2, int threadNum) const
{

//#pragma omp critical
  //hnswkd->annkSearch(_p, 1, nn_idx, nn, 0.0);
  // cout<<*_p<<endl;
  float s[3];
  s[0] = _p[0];
  s[1] = _p[1];
  s[2] = _p[2];
  std::priority_queue<std::pair<float, labeltype >> result =hnswgraph->searchKnn(s, 1);
  size_t index=result.top().second;
  //cout<<index<<endl;
  if (Dist2(_p, pts[index]) > maxdist2) return 0;
  return pts[index];
  //delete [] s;
}  

