
// simple_dialog_test.h : main header file for the PROJECT_NAME application
//

#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"		// main symbols


// Csimple_dialog_testApp:
// See simple_dialog_test.cpp for the implementation of this class
//

class Csimple_dialog_testApp : public CWinApp
{
public:
	Csimple_dialog_testApp();

// Overrides
	public:
	virtual BOOL InitInstance();

// Implementation

	DECLARE_MESSAGE_MAP()
};

extern Csimple_dialog_testApp theApp;