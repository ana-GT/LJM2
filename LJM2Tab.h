/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved	
 * Author(s): Ana C. Huaman Quispe <ahuaman3@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
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

	std::vector< std::vector<Eigen::VectorXd> > mWorkspacePaths;
	std::vector< std::vector<Eigen::Vector3i> > mNodePaths;
	std::vector< std::vector<Eigen::VectorXd> > mConfigPaths;
	int mNumPaths;
	double mAlpha;

	static int mPathCounter;


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
	void FollowWorkspacePlan(); 
	Eigen::VectorXd GetEE_XYZ( const Eigen::VectorXd &_q );


    void SetTimeline( std::vector<Eigen::VectorXd> _path );
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

	//-- Objects to enter Planning information
	GRIPSlider* mAlphaSlider;
    GRIPSlider* mNumPathsSlider; 

    DECLARE_DYNAMIC_CLASS( LJM2Tab )
    DECLARE_EVENT_TABLE()
};

#endif /** _LJM2_TAB_ */

