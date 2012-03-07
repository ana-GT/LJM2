/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#ifndef _LJM2_TAB_
#define _LJM2_TAB_

#include <Tabs/GRIPTab.h>
#include <Tabs/GRIPThread.h>

#include <planning/Robot.h>
#include <planning/Object.h>
#include <kinematics/BodyNode.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/Transformation.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>

#include "myFunctions/CheckProcess.h"

#include <iostream>
#include <list>

using namespace std;

/**
 * @class JacobianTests
 * @brief Tab with Tester Planners
 */
class LJM2Tab : public GRIPTab
{
public:

    /// Variables to save info for the planners
    Eigen::VectorXd mStartConfig;
    Eigen::VectorXd mTargetConfig;
    Eigen::VectorXd mTargetPose; /**< Always dimension 6, otherwise you are toasted */
    std::string mTargetName;

	Eigen::VectorXd mStartXYZ;
	Eigen::VectorXd mTargetXYZ;
    int mStartNodeX; int mStartNodeY; int mStartNodeZ;
	int mTargetNodeX; int mTargetNodeY; int mTargetNodeZ;


    //-- Robot specific info
    int mRobotId;
    int mEEId; /**< End Effector ID */
    kinematics::BodyNode *mEENode;
    std::string mEEName;
    Eigen::VectorXi mLinks;

	std::vector< std::vector<Eigen::Vector3i> > mWorkspacePaths;
	int mNumPaths;
	double mAlpha;


    /// Miscellaneous stuff ( no idea why this is here )
    wxTextCtrl *timeText;

    /// Public vars to capture external selection stuff 
    planning::Object* selectedObject;
    planning::Robot* selectedRobot;
    kinematics::BodyNode* selectedNode;

    /// Functions about Robina's left arm specifically
    Eigen::VectorXi GetLeftArmIds();

    /// Functions related to Tab
    LJM2Tab(){};
    LJM2Tab( wxWindow * parent, wxWindowID id = -1,
             const wxPoint & pos = wxDefaultPosition,
             const wxSize & size = wxDefaultSize,
             long style = wxTAB_TRAVERSAL);
    virtual ~LJM2Tab();

    void OnSlider(wxCommandEvent &evt);
    void OnRadio(wxCommandEvent &evt);
    void OnButton(wxCommandEvent &evt);
    void OnCheckBox(wxCommandEvent &evt);

	//-- Workspace functions
	void WorkspacePlan(); 
	Eigen::VectorXd GetEE_XYZ( const Eigen::VectorXd &_q );


    void SetTimeline( std::list<Eigen::VectorXd> _path );
    void GRIPStateChange();

	void Get3DInfo();

	CheckProcess *mCp;
	LJM2 *mLjm2;

	double mSizeX;
	double mSizeY;
    double mSizeZ;
    double mResolution;
    double mOriginX;
    double mOriginY; 
    double mOriginZ;

    // Thread specific
    // GRIPThread* thread;

    // Your Thread routine
    // call GRIPThread::CheckPoint() regularly
    // void Thread();
    // void onCompleteThread();

    DECLARE_DYNAMIC_CLASS( LJM2Tab )
    DECLARE_EVENT_TABLE()
};

#endif /** _LJM2_TAB_ */

