/**
 * @function CheckProcess.h
 */
#ifndef CHECK_PROCESS_H_
#define CHECK_PROCESS_H_

#include <Eigen/Core>
#include <time.h>
#include <stdio.h>
#include <planning/Robot.h>
#include <vector>
#include "CheckObject.h"
#include <LJM2/LJM2.h>

/**
 * @class CheckProcess
 */
class CheckProcess
{
  public:

  RAPID_model *mSlideBox;
  std::vector< CheckObject > mObjs;
  std::vector< CheckObject > mLinks;
  int mNumObjs;
  int mNumLinks;

  Eigen::VectorXi mLinksID;

  double mSizeX;
  double mSizeY;
  double mSizeZ; 
  double mOriginX;
  double mOriginY;
  double mOriginZ;
  double mResolution;

  CheckProcess( double _sizeX, double _sizeY, double _sizeZ,
 	    	    double _originX, double _originY, double _originZ, 
	            double _resolution );
  void build_slideBox();
  void getObjectsData( std::vector<planning::Object*> _objects );
  void getLinksData( planning::Robot* _robot, Eigen::VectorXi _linksID );
  void reportObjects();

  //-- Voxel construction
  void build_voxel( std::vector<planning::Object*> _objects, LJM2 &_ljm2 );

  //-- Distance transform 
  std::vector< Eigen::Vector3i >mObjsVoxels;
};

#endif /** CHECK_PROCESS_H */
