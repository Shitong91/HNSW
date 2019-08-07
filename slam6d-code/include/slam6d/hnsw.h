 /*  @author Shitong Du.
     date 25/07/2019 
*/

#ifndef __HNSW_H__
#define __HNSW_H__

//#include "slam6d/kdparams.h"
#include "slam6d/searchTree.h"
//#include "ANN/ANN.h"
#include "slam6d/hnswlib.h"
#include "slam6d/hnswalg.h"
using namespace hnswlib;
class HNSW : public SearchTree {

public:
  
  /**
   * default constructor
   */
  HNSW();
  
  /**
   * Constructor using the point set pts and the num_of_pts n
   */
  HNSW(PointerArray<double>&_pts,  size_t n);
  
  /**
   * destructor
   */
  virtual ~HNSW(); 
  
  
 /**
  * Finds the closest point within the tree,
  * wrt. the point given as first parameter.
  * @param _p point
  * @param maxdist2 maximal search distance.
  * @param threadNum Thread number, for parallelization
  * @return Pointer to the closest point
  */  
  double *FindClosest(double *_p, double maxdist2, int threadNum = 0) const;

private:

  /**
   * a pointer to ANNkd_tree instance
   */
 // ANNkd_tree* annkd;
 // ANNdistArray nn; //temporary ANNdistArray instance to use for storing the nearest neighbor
 // ANNidxArray nn_idx; //temporary ANNdistArray instance to use for storing the nearest neighbor
  HierarchicalNSW<float> *hnswgraph;
  double **pts;
  double* tpts ;
  //double* tpts;
};

#endif

