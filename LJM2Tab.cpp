/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <Tabs/GRIPTab.h>
#include <string>
#include <iostream>

#include <Tabs/AllTabs.h>
#include <GRIPApp.h>

#include "LJM2Tab.h"

using namespace std;

/* Quick intro to adding tabs:
 * 1- Copy template cpp and header files and replace with new class name
 * 2- include classname.h in AllTabs.h, and use the ADD_TAB macro to create it
 */

// Control IDs (used for event handling - be sure to start with a non-conflicted id)
enum LJM2TabEvents {
	button_SetStartConf = 50,
	button_ShowStartConf,

	button_SetTargetPose,
	button_ShowTargetPose,

    button_Get3DInfo,
	button_Plot3DConfiguration,
	button_Plot3DPaths,
	button_PlotDebug,

	button_Plan,
	button_Stop,
	button_UpdateTime,
	button_ExportSequence,
	button_ShowPath,
	slider_Time,

};


// sizer for whole tab
wxBoxSizer* sizerFull;

//Add a handler for any events that can be generated by the widgets you add here (sliders, radio, checkbox, etc)
BEGIN_EVENT_TABLE( LJM2Tab, wxPanel )
EVT_COMMAND ( wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, LJM2Tab::OnSlider )
EVT_COMMAND ( wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, LJM2Tab::OnButton )
EVT_COMMAND( wxID_ANY, wxEVT_COMMAND_RADIOBOX_SELECTED, LJM2Tab::OnRadio )
END_EVENT_TABLE()

// Class constructor for the tab: Each tab will be a subclass of RSTTab
IMPLEMENT_DYNAMIC_CLASS( LJM2Tab, GRIPTab )

/**
 * @function LJM2Tab
 * @brief Constructor
 */
LJM2Tab::LJM2Tab( wxWindow *parent, const wxWindowID id,
		                      const wxPoint& pos, const wxSize& size, long style) :
	                          GRIPTab(parent, id, pos, size, style ) {

    mStartConfig.resize(0);
    mTargetConfig.resize(0);
    mTargetPose.resize(0);

    mRobotId = 0;
    mLinks.resize(0);

	mSizeX = 0.80;
	mSizeY = 1.00;
    mSizeZ = 1.0;
    mResolution = 0.02;
    mOriginX = 0.0;
    mOriginY = -0.10; 
    mOriginZ = 0.0;

    mEEName = "LJ6";

    sizerFull = new wxBoxSizer( wxHORIZONTAL );
 
    // ** Create left static box for configuring the planner **

    // Create StaticBox container for all items
    wxStaticBox* configureBox = new wxStaticBox(this, -1, wxT("Configure"));

    // Create sizer for this box with horizontal layout
    wxStaticBoxSizer* configureBoxSizer = new wxStaticBoxSizer(configureBox, wxHORIZONTAL);

    // Create sizer for start buttons in 1st column
    wxBoxSizer *col1Sizer = new wxBoxSizer(wxVERTICAL);
    col1Sizer->Add( new wxButton(this, button_SetStartConf, wxT("Set &Start Conf")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together
    col1Sizer->Add( new wxButton(this, button_ShowStartConf, wxT("Show S&tart Conf")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together

    // Add col1Sizer to the configuration box
    configureBoxSizer->Add( col1Sizer,
			    1, // takes half the space of the configure box
			    wxALIGN_NOT ); // no border and center horizontally


    // Create sizer for target buttons in 2st column
    wxBoxSizer *col2Sizer = new wxBoxSizer(wxVERTICAL);
    col2Sizer->Add( new wxButton(this, button_SetTargetPose, wxT("Set &Target Pose")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together
    col2Sizer->Add( new wxButton(this, button_ShowTargetPose, wxT("Show Target Pose")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together

    // Add col1Sizer to the configuration box
    configureBoxSizer->Add( col2Sizer,
			    1, // takes half the space of the configure box
			    wxALIGN_NOT ); // no border and center horizontally



    // Create sizer for planner buttons in 4th column
    wxBoxSizer *col4Sizer = new wxBoxSizer(wxVERTICAL);
    col4Sizer->Add( new wxButton(this, button_Get3DInfo, wxT("Get 3D Info")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together

    col4Sizer->Add( new wxButton(this, button_Plot3DConfiguration, wxT("Plot 3D Info")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together
    col4Sizer->Add( new wxButton(this, button_Plot3DPaths, wxT("Plot 3D Paths")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together
    col4Sizer->Add( new wxButton(this, button_PlotDebug, wxT("Plot Debug")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together

    // Add col2Sizer to the configuration box
    configureBoxSizer->Add( col4Sizer,
			    1, // size evenly with radio box and checkboxes
			    wxALIGN_NOT ); // no border and center horizontally

    // Add this box to parent sizer
    sizerFull->Add( configureBoxSizer,
		    4, // 4-to-1 ratio with execute sizer, since it has 4 buttons
		    wxEXPAND | wxALL,
		    6 );

    // ** Create right static box for running the planner **
    wxStaticBox* executeBox = new wxStaticBox(this, -1, wxT("Execute Planner"));

    // Create sizer for this box
    wxStaticBoxSizer* executeBoxSizer = new wxStaticBoxSizer(executeBox, wxVERTICAL);

    // Add buttons for "plan", "save movie", and "show path"
    executeBoxSizer->Add( new wxButton(this, button_Plan, wxT("&Run")),
	 		  1, // stretch to fit horizontally
			  wxGROW ); // let it hog all the space in it's column

    executeBoxSizer->Add( new wxButton(this, button_Stop, wxT("&Stop")),
			  1, // stretch to fit horizontally
			  wxGROW );


    wxBoxSizer *timeSizer = new wxBoxSizer(wxHORIZONTAL);
    timeText = new wxTextCtrl(this,1008,wxT("5.0"),wxDefaultPosition,wxSize(40,20),wxTE_RIGHT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);
    timeSizer->Add(timeText,2,wxALL,1);
    timeSizer->Add(new wxButton(this, button_UpdateTime, wxT("Set T(s)")),2,wxALL,1);
    executeBoxSizer->Add(timeSizer,1,wxALL,2);

    executeBoxSizer->Add( new wxButton(this, button_ShowPath, wxT("&Print")),
			  1, // stretch to fit horizontally
			  wxGROW );

    sizerFull->Add(executeBoxSizer, 1, wxEXPAND | wxALL, 6);

    SetSizer(sizerFull);

}

/**
 * @function ~LJM2Tab
 * @brief Destructor
 */ 
LJM2Tab::~LJM2Tab(){

	if( mCp != NULL ) {
		delete mCp;
	}
	if( mLjm2 != NULL ) {
		delete mLjm2;
	}

}

/**
 * @function getLeftArmIds
 * @brief Get DOF's IDs for ROBINA's left arm
 */
Eigen::VectorXi LJM2Tab::GetLeftArmIds() {

  string LINK_NAMES[7] = {"LJ0", "LJ1", "LJ2", "LJ3", "LJ4", "LJ5", "LJ6" };
   
  Eigen::VectorXi linksAll = mWorld->mRobots[mRobotId]->getQuickDofsIndices(); 

  Eigen::VectorXi linksLeftArm(7);
  for( unsigned int i = 0; i < 7; i++ ) {
      for( unsigned int j = 0; j < linksAll.size(); j++ ) {      
          if( mWorld->mRobots[mRobotId]->getDof( linksAll[j] )->getJoint()->getChildNode()->getName() == LINK_NAMES[i] ) {
              linksLeftArm[i] = linksAll[j]; 
              break;   
          }
      }
  }
  
  return linksLeftArm;
}

/**
 * @function setTimeLine
 * @brief 
 */
void LJM2Tab::SetTimeline( std::list<Eigen::VectorXd> _path ) {
    
    if( mWorld == NULL || _path.size() == 0 ) {
        std::cout << "--(!) Must create a valid plan before updating its duration (!)--" << std::endl;
	      return;
    }
    
    double T;
    timeText->GetValue().ToDouble(&T);
    
    int numsteps = _path.size();
    double increment = T/(double)numsteps;

    cout << "-->(+) Updating Timeline - Increment: " << increment << " Total T: " << T << " Steps: " << numsteps << endl;

    frame->InitTimer( string("RRT_Plan"),increment );


    Eigen::VectorXd vals( mLinks.size() );

    for( std::list<Eigen::VectorXd>::iterator it = _path.begin(); it != _path.end(); it++ ) {

        mWorld->mRobots[mRobotId]->setDofs( *it, mLinks );
	      mWorld->mRobots[mRobotId]->update();

        frame->AddWorld( mWorld );
    }
   
}


/**
 * @function OnButton
 * @brief Handle Button Events
 */
void LJM2Tab::OnButton(wxCommandEvent &evt) {

    int button_num = evt.GetId();
    mLinks = GetLeftArmIds();

    switch (button_num) {

        /** Set Start Configuration */
        case button_SetStartConf: {

	          if ( mWorld != NULL ) {
	              if( mWorld->mRobots.size() < 1) {
            	      cout << "--(!) Must have a world with a robot to set a Start state (!)--" << endl;
		                break;
		            }
		            std::cout << "--(i) Setting Start state for " << mWorld->mRobots[mRobotId]->getName() << ":" << std::endl;
                
                    mStartConfig = mWorld->mRobots[mRobotId]->getDofs( mLinks );

		            for( unsigned int i = 0; i < mStartConfig.size(); i++ )
                	{  std::cout << mStartConfig(i) << " ";  } 
		            std::cout << endl;
	          } else {
	              std::cout << "--(!) Must have a world loaded to set a Start state.(!)--" << std::endl;
	          }
		}
	    break;


        /** Show Start */
	    case button_ShowStartConf: {

	          if( mStartConfig.size() < 1 ) {
	              std::cout << "--(x) First, set a start configuration (x)--" << std::endl;
		            break;
	          } 

            mWorld->mRobots[mRobotId]->setDofs( mStartConfig, mLinks );

	        for( unsigned int i = 0; i< mStartConfig.size(); i++ )
            {  cout << mStartConfig(i) << " "; }
	        std::cout << std::endl;

	        mWorld->mRobots[mRobotId]->update();
	        viewer->UpdateCamera();
        }
      	break;

		/** Set Target Pose */
		case button_SetTargetPose: {

          if( mWorld != NULL )
          {  if( mWorld->mRobots.size() < 1 )
             {  printf("---------(xx) No robot in the loaded world, you idiot, I need one! (xx)---------- \n"); break; }
 
             if( selectedObject != NULL )
             { 
                mTargetName = selectedObject->getName();
				double x; double y; double z;
                selectedObject->getPositionXYZ( x, y, z );
 				mTargetXYZ.resize(3);
				mTargetXYZ << x,y,z;
			    std::cout<<"** Target object: "<< mTargetName << "-- ("<< x <<","<< y <<","<< z <<" )" << std::endl;
             }
             else
             { cout<<"------xx (!) Please, select an object in the Viewer Tree and try again xx------"<<endl; }                    
          }
          else
          { cout<<"------xx (!) No world loaded, I cannot set a goal xx------"<<endl; }
		}
		break;


        /** Get 3D Info */
	      case button_Get3DInfo:
          {
			Get3DInfo();
          }		
	          break;

        /** Plot 3D Configuration */
	      case button_Plot3DConfiguration:
          {
			printf( "--------- Plotting 3D Configuration ---------- \n" );
	        pcl::visualization::PCLVisualizer *viewer;
			viewer = new pcl::visualization::PCLVisualizer( "Discretized Workspace" );

			mLjm2->ViewObstacles( viewer, 0, 0, 255 );

   	        while( !viewer->wasStopped() ) {
		    	viewer->spin();
	        }
           delete viewer;
          }		
	          break;

        /** Plot Paths */
	      case button_Plot3DPaths:
          {
			printf( "--------- Plotting 3D Paths ---------- \n" );
	        pcl::visualization::PCLVisualizer *viewer;
			viewer = new pcl::visualization::PCLVisualizer( "Workspace Paths" );
			mLjm2->ViewObstacles( viewer, 0, 0, 255 );
			mLjm2->ViewPaths( mNodePaths, viewer );
            mLjm2->ViewBall( viewer, mStartNodeX, mStartNodeY, mStartNodeZ, "Start" );
            mLjm2->ViewBall( viewer, mTargetNodeX, mTargetNodeY, mTargetNodeZ, "Target" );

   	        while( !viewer->wasStopped() ) {
		    	viewer->spin();
	        }
           delete viewer;
          }		
	          break;

		/** Plot Debug */
		case button_PlotDebug:
		{
			printf("------- Plot Debugger ------- \n");
	        pcl::visualization::PCLVisualizer *viewer;
			viewer = new pcl::visualization::PCLVisualizer( "Debugger" );
			mLjm2->ViewObstacles( viewer, 0, 0, 255 );
			mLjm2->ViewPaths( mNodePaths, viewer );
            mLjm2->ViewBall( viewer, mStartNodeX, mStartNodeY, mStartNodeZ, "Start" );
            mLjm2->ViewBall( viewer, mTargetNodeX, mTargetNodeY, mTargetNodeZ, "Target" );
            mLjm2->ViewBlameDTPoints( viewer, 0, 255, 0 );

   	        while( !viewer->wasStopped() ) {
		    	viewer->spin();
	        }
           delete viewer;
		}
        	break;

        /** UpdateTime */
	      case button_UpdateTime:
          {
	          /// Update the time span of the movie timeline
	          //SetTimeline();
          }		
	          break;

        /** Show Path */
	      case button_ShowPath: {           
	          if( mWorld == NULL ) {
	              cout << "--(!) Must create a valid plan before printing. (!)--" << endl;
		            return;
	          } else {
                printf("--(i) Printing (i)-- \n");
              }        
		  }
	        break;

        /** Execute Plan */
		  case button_Plan: {
		  	WorkspacePlan();
		  }
		  break;

    } // end of switch
}


/**
 * @function WorkspacePlan
 */
void LJM2Tab::WorkspacePlan() {

    mEENode = mWorld->mRobots[mRobotId]->getNode( mEEName.c_str() );
    mEEId = mEENode->getSkelIndex();
 
    /// Check start position is not in collision
    mWorld->mRobots[mRobotId]->setDofs( mStartConfig, mLinks );

    if( mCollision->CheckCollisions() ) {   
      printf(" --(!) Initial status is in collision. I am NOT proceeding. Exiting \n");
      return; 
    }

   //-- Setting start cell
   mStartXYZ = GetEE_XYZ( mStartConfig );

  
   if( mLjm2->WorldToGrid( mStartXYZ(0), mStartXYZ(1), mStartXYZ(2), mStartNodeX, mStartNodeY, mStartNodeZ ) == false )
   {  printf("-------(x) Error: Start Position no valid (off limits) (x)-------\n"); return; } 
   if( mLjm2->WorldToGrid( mTargetXYZ(0), mTargetXYZ(1), mTargetXYZ(2), mTargetNodeX, mTargetNodeY, mTargetNodeZ ) == false )
   {  printf("-------(x) Error: Target Position no valid (off limits) (x)-------\n"); return; }    


	//-- Plan now workspace guys
	mNumPaths = 1;
	mAlpha = 0.01;
	printf("-------(o) Planning from (%d %d %d) to (%d %d %d) (o)-------\n", mStartNodeX, mStartNodeY, mStartNodeZ, mTargetNodeX, mTargetNodeY, mTargetNodeZ );
	printf("-------(o) Start State: %d  Target state: %d (FREE: 1 OBSTACLE: 2) (o)-------\n", mLjm2->GetState(mStartNodeX, mStartNodeY, mStartNodeZ), mLjm2->GetState(mTargetNodeX, mTargetNodeY, mTargetNodeZ));
	mNodePaths = mLjm2->FindVarietyPaths2( mStartNodeX, mStartNodeY, mStartNodeZ, mTargetNodeX, mTargetNodeY, mTargetNodeZ, mNumPaths, mAlpha );
	mWorkspacePaths = mLjm2->NodePathToWorkspacePath( mNodePaths );
	printf("-------(i) Finished Workpace Planning (i)------- \n");
}

/*
 * @function GetEE_XYZ
 */
Eigen::VectorXd LJM2Tab::GetEE_XYZ( const Eigen::VectorXd &_q ) {

    mWorld->mRobots[mRobotId]->setDofs( _q, mLinks );
    mWorld->mRobots[mRobotId]->update();
    Eigen::MatrixXd pose = mEENode->getWorldTransform(); 
    Eigen::VectorXd xyz(3); xyz << pose(0,3), pose(1,3), pose(2,3);

    return xyz;
}


/**
 * @function Get3DInfo
 */
void LJM2Tab::Get3DInfo() {

  //-- Build voxel - LJM2 object
  printf("------- Getting Info 3D -------\n" );
  printf("--(!) You'd better have set the start config and the target object, otherwise I will output rubbish \n");
  mLjm2 = new LJM2( mSizeX, mSizeY, mSizeZ, mOriginX, mOriginY, mOriginZ, mResolution );
  mCp = new CheckProcess( mSizeX, mSizeY, mSizeZ, mOriginX, mOriginY, mOriginZ, mResolution );
  mCp->getObjectsData( mWorld->mObjects, mTargetName );  
  mCp->build_voxel( mWorld->mObjects, *mLjm2 ); // Here your LJM2 is built
  mCp->reportObjects(); 
  printf("----- (...) Process Geometry (...) ----- \n");
  mLjm2->ProcessGeometry();
  printf("----- (i) End process geometry (i) ----- \n");
}


/**
 * @function OnSlider
 * @brief Handle slider changes
 */
void LJM2Tab::OnSlider(wxCommandEvent &evt) {

    /*
    if ( selectedTreeNode == NULL ) {
        return;
    } */ /// Do I need this now? - AHQ: Dec 6th, 2012

    int slnum = evt.GetId();
    double pos = *(double*) evt.GetClientData();
    char numBuf[1000];

    switch (slnum) {

        case slider_Time:
	          sprintf(numBuf, "X Change: %7.4f", pos);
	          std::cout << "-->(i) Timeline slider output: " << numBuf << std::endl;
	          //handleTimeSlider(); // uses slider position to query plan state
	          break;

      	default:
	          return;
    }

    //world->updateCollision(o);
    //viewer->UpdateCamera();

    if (frame != NULL)
        frame->SetStatusText(wxString(numBuf, wxConvUTF8));
}

/**
 * @function OnRadio
 */
void LJM2Tab::OnRadio( wxCommandEvent &evt ) {
   
}

/**
 * @function GRIPStateChange -- Keep using this name as it is a virtual function
 * @brief This function is called when an object is selected in the Tree View or other
 *        global changes to the RST world. Use this to capture events from outside the tab.
 */
void LJM2Tab::GRIPStateChange() {
    if ( selectedTreeNode == NULL ) {

        return;
    }

    string statusBuf;
    string buf, buf2;

    switch (selectedTreeNode->dType) {

        case Return_Type_Object:
	          selectedObject = (planning::Object*) ( selectedTreeNode->data );
	          statusBuf = " Selected Object: " + selectedObject->getName();
	          buf = "You clicked on object: " + selectedObject->getName();
	          // Enter action for object select events here:
	          break;

	      case Return_Type_Robot:
	          selectedRobot = (planning::Robot*) ( selectedTreeNode->data );
	          statusBuf = " Selected Robot: " + selectedRobot->getName();
	          buf = " You clicked on robot: " + selectedRobot->getName();
      	    // Enter action for Robot select events here:
	          break;
	      case Return_Type_Node:
	          selectedNode = (kinematics::BodyNode*) ( selectedTreeNode->data );
	          statusBuf = " Selected Body Node: " + string(selectedNode->getName()) + " of Robot: "
			      + ( (planning::Robot*) selectedNode->getSkel() )->getName();
	          buf = " Node: " + string(selectedNode->getName()) + " of Robot: " + ( (planning::Robot*) selectedNode->getSkel() )->getName();
	          // Enter action for link select events here:
      	    break;
        default:
            fprintf(stderr, "--( :D ) Someone else's problem!\n");
            assert(0);
            exit(1);
    }

    //cout << buf << endl;
    frame->SetStatusText(wxString(statusBuf.c_str(), wxConvUTF8));
    sizerFull->Layout();
}
