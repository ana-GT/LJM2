 /*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

/**
 * @author A. Huaman
 * @date 2012-03-07
 */
#include "GRIPApp.h"
#include "LJM2Tab.h"

extern wxNotebook* tabView;

class LJM2TabApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new LJM2Tab(tabView), wxT("LJM2 Tab"));
	}
};

IMPLEMENT_APP(LJM2TabApp)
