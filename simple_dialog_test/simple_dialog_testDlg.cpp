
// simple_dialog_testDlg.cpp : implementation file
//

#include "stdafx.h"
#include "simple_dialog_test.h"
#include "simple_dialog_testDlg.h"

#include "rMath/rMath.h"
using namespace rMath;

#include "rCommand/rCmdDefine.h"
#include "rxSDK/rxSDK.h"

#include "rTerm/rTerm.h"
using namespace rTerm;

#include "process.h"

#include "../simple_dialog_test_ctrl/simple_dialog_test_ctrlCmd.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif
#pragma comment(linker , "/entry:WinMainCRTStartup /subsystem:console")

#define MY_TIMER 100


////////////////////////////////////////////////////
// RoboticsLab world updater control
////////////////////////////////////////////////////
static uintptr_t s_updater = 0;
volatile static bool s_run_updater = false;
const static rTime s_delT = 0.005;
////////////////////////////////////////////////////


unsigned __stdcall updateWorld(void *udata)
{
	
	while (s_run_updater)
	{
		PHYSICS_WORLD->update();
		
		Sleep(1);
	}
	return 0;
}

// CAboutDlg dialog used for App About

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// Dialog Data
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
END_MESSAGE_MAP()


// Csimple_dialog_testDlg dialog




Csimple_dialog_testDlg::Csimple_dialog_testDlg(CWnd* pParent /*=NULL*/)
	: CDialog(Csimple_dialog_testDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void Csimple_dialog_testDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_BTN_START, m_btnStart);
}

BEGIN_MESSAGE_MAP(Csimple_dialog_testDlg, CDialog)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	//}}AFX_MSG_MAP
	ON_WM_DESTROY()
	ON_BN_CLICKED(IDC_BTN_START, &Csimple_dialog_testDlg::OnBnClickedBtnStart)
	ON_BN_CLICKED(IDC_BUTTON_HOME, &Csimple_dialog_testDlg::OnBnClickedButtonHome)
	ON_BN_CLICKED(IDC_RADIO1, &Csimple_dialog_testDlg::OnBnClickedRadio1)
	ON_BN_CLICKED(IDC_BUTTON2, &Csimple_dialog_testDlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_RADIO2, &Csimple_dialog_testDlg::OnBnClickedRadio2)
	ON_WM_LBUTTONDOWN()
	ON_BN_CLICKED(IDC_RADIO3, &Csimple_dialog_testDlg::OnBnClickedRadio3)
//	ON_EN_UPDATE(IDC_STATUS_JOINT1, &Csimple_dialog_testDlg::OnEnUpdateStatusJoint1)
//ON_WM_LBUTTONDBLCLK()
//ON_WM_UPDATEUISTATE()
//ON_WM_ACTIVATE()
ON_WM_TIMER()
END_MESSAGE_MAP()


// Csimple_dialog_testDlg message handlers

BOOL Csimple_dialog_testDlg::OnInitDialog()
{
	CDialog::OnInitDialog();
	value_init = 0;
	SetTimer(MY_TIMER,100,0);
	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	bool bContact = false;

	PHYSICS_WORLD->createWorld(bContact, s_delT,rTimer_MODE_REAL_WIN32);
	if (!PHYSICS_WORLD->isWorldCreated())
	{
		MessageBox(_T("Cannot create a world."), _T("RoboticsLab Error"), MB_OK);
		return FALSE;
	}

	PHYSICS_WORLD->setGravity(0, 0, -GRAV_ACC);
	if (bContact)
		PHYSICS_WORLD->createPlane(0, 0, 1, 0);
	
#ifdef USING_NAMESPACE_RTERM
	rTerm::startPlayer("-ip 127.0.0.1 -port 5150");
#endif
	PHYSICS_WORLD->makeNetwork(1000);

	string_type aml = _T("models/Manipulator/SAP1/simlab_arm.aml");
													// Set AML file path to load including file extension, 'aml'.
													// It can be a relative path based on the program working directory or absolute file path.
													// Usually the working directory is set from the system environment variable, $(RLAB_BIN_PATH).
	string_type name = _T("P3DOF");					// Set the name of your model. Each name should be unique.

	HTransform T0;									// Initial position and orientation of your robot. 
	dVector q0;										// Initial joint position, q values of robot.

	_sys = PHYSICS_WORLD->createSystem(aml, name, T0, q0); // Load a system and put it into the world.

	if (!_sys)
	{
		MessageBox(_T("Cannot create a system."), _T("RoboticsLab Error"), MB_OK);
		return FALSE;
	}

	PHYSICS_WORLD->initialize();

	if (_sys)
	{
		int step = 1;	// Controller is updated once per 'step' simulation times
						// when step = 0, the controller should be updated by user manually. 
		_control = PHYSICS_WORLD->createController(_T("simple_dialog_test_ctrl"), _sys, step);
		string_type control_plugin_path = _T("controls/simple_dialog_test_ctrl.dll");// set your control plugin DLL file 
					// relative to working directory (whose default value is $(RLAB_BIN_PATH))

		_control->setAlgorithmDll(control_plugin_path);	
		_control->setPeriod(step*s_delT);
		_control->setNominalSystem(aml, name, T0, q0);
		_control->initAlgorithm();
	}

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void Csimple_dialog_testDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void Csimple_dialog_testDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
	
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR Csimple_dialog_testDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void Csimple_dialog_testDlg::OnDestroy()
{
	CDialog::OnDestroy();

	if (_control) 
		_control->command(RESERVED_CMD_SERVO_OFF);

	if (s_run_updater)
	{
		PHYSICS_WORLD->deactivateWorld();
		s_run_updater = false;
		WaitForSingleObject((HANDLE)s_updater, 3000);
		CloseHandle((HANDLE)s_updater);
		s_updater = 0;
	}

	if(PHYSICS_WORLD->isWorldCreated())
		DESTROY_PHYSICS_WORLD();

#ifdef USING_NAMESPACE_RTERM
    stopPlayer();
#endif
}
void Csimple_dialog_testDlg::OnBnClickedBtnStart()
{

	static bool bServoOn = false;
	if (!bServoOn)
	{
		if (_control) 
			_control->command(RESERVED_CMD_SERVO_ON);

		bServoOn = true;
	}

	if (s_run_updater)
	{
		PHYSICS_WORLD->deactivateWorld();
		s_run_updater = false;
		WaitForSingleObject((HANDLE)s_updater, 3000);
		CloseHandle((HANDLE)s_updater);
		s_updater = 0;
	}
	else
	{
		PHYSICS_WORLD->activateWorld();
		s_run_updater = true;
		s_updater = _beginthreadex(NULL, 0, updateWorld, NULL, 0, NULL);
		if (!s_updater)
			s_run_updater = false;
	}

	if (s_run_updater)
		m_btnStart.SetWindowText(_T("PAUSE"));
	else
		m_btnStart.SetWindowText(_T("START"));
}
void Csimple_dialog_testDlg::OnBnClickedButtonHome()
{
	// TODO: Add your control notification handler code here
	if(_control)
		_control->command(RESERVED_CMD_GO_HOME,10);
	
}
void Csimple_dialog_testDlg::OnBnClickedRadio1()
{
	float Angle[6];
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	
	rxDevice* angle1 = _sys ->findDevice(_T("enc1"));
	rxDevice* angle2 = _sys ->findDevice(_T("enc2"));
	rxDevice* angle3 = _sys ->findDevice(_T("enc3"));
	rxDevice* angle4 = _sys ->findDevice(_T("enc4"));
	rxDevice* angle5 = _sys ->findDevice(_T("enc5"));
	rxDevice* angle6 = _sys ->findDevice(_T("enc6"));
	if(angle1 && angle2 && angle3 && angle4 && angle5 && angle6){
		angle1->readDeviceValue(&Angle[0],1*sizeof(float));
		angle2->readDeviceValue(&Angle[1],1*sizeof(float));
		angle3->readDeviceValue(&Angle[2],1*sizeof(float));
		angle4->readDeviceValue(&Angle[3],1*sizeof(float));
		angle5->readDeviceValue(&Angle[4],1*sizeof(float));
		angle6->readDeviceValue(&Angle[5],1*sizeof(float));
	}
	SetDlgItemInt(IDC_JOINT1,Angle[0]/DEGREE);
	SetDlgItemInt(IDC_JOINT2,Angle[1]/DEGREE);
	SetDlgItemInt(IDC_JOINT3,Angle[2]/DEGREE);
	SetDlgItemInt(IDC_JOINT4,Angle[3]/DEGREE);
	SetDlgItemInt(IDC_JOINT5,Angle[4]/DEGREE);
	SetDlgItemInt(IDC_JOINT6,Angle[5]/DEGREE);

	int strText;
	strText = GetDlgItemInt(IDC_JOINT1);
	if(_control)
		_control->command(JSC,strText);
	strText = GetDlgItemInt(IDC_JOINT2);
	if(_control)
		_control->command(JSC,strText);
	strText = GetDlgItemInt(IDC_JOINT3);
	if(_control)
		_control->command(JSC,strText);
	strText = GetDlgItemInt(IDC_JOINT4);
	if(_control)
		_control->command(JSC,strText);
	strText = GetDlgItemInt(IDC_JOINT5);
	if(_control)
		_control->command(JSC,strText);
	strText = GetDlgItemInt(IDC_JOINT6);
	if(_control)
		_control->command(JSC,strText);
	if(_control)
		_control->command(RUN_MODE,1);
	
}
void Csimple_dialog_testDlg::OnBnClickedRadio2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	int strText;
	
	rxBody* body = _sys->findBody(_T("Link06-4"));
	HTransform body_T = body->T();
	dVector Now_x;
	Now_x.resize(3);
	Now_x = body_T.r;
	SetDlgItemInt(IDC_DES_X,Now_x[0]*100);
	SetDlgItemInt(IDC_DES_Y,Now_x[1]*100);
	SetDlgItemInt(IDC_DES_Z,Now_x[2]*100);
	strText = GetDlgItemInt(IDC_DES_X);
	if(_control)
		_control->command(OSC,strText);
	strText = GetDlgItemInt(IDC_DES_Y);
	if(_control)
		_control->command(OSC,strText);
	strText = GetDlgItemInt(IDC_DES_Z);
	if(_control)
		_control->command(OSC,strText);
	if(_control)
		_control->command(RUN_MODE,2);
	
}
void Csimple_dialog_testDlg::OnBnClickedRadio3()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float Angle[6];
	int strText;
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	
	rxDevice* angle1 = _sys ->findDevice(_T("enc1"));
	rxDevice* angle2 = _sys ->findDevice(_T("enc2"));
	rxDevice* angle3 = _sys ->findDevice(_T("enc3"));
	rxDevice* angle4 = _sys ->findDevice(_T("enc4"));
	rxDevice* angle5 = _sys ->findDevice(_T("enc5"));
	rxDevice* angle6 = _sys ->findDevice(_T("enc6"));
	if(angle1 && angle2 && angle3 && angle4 && angle5 && angle6){
		angle1->readDeviceValue(&Angle[0],1*sizeof(float));
		angle2->readDeviceValue(&Angle[1],1*sizeof(float));
		angle3->readDeviceValue(&Angle[2],1*sizeof(float));
		angle4->readDeviceValue(&Angle[3],1*sizeof(float));
		angle5->readDeviceValue(&Angle[4],1*sizeof(float));
		angle6->readDeviceValue(&Angle[5],1*sizeof(float));
	}
	SetDlgItemInt(IDC_SIMPLE_JOINT1,Angle[0]/DEGREE);
	SetDlgItemInt(IDC_SIMPLE_JOINT2,Angle[1]/DEGREE);
	SetDlgItemInt(IDC_SIMPLE_JOINT3,Angle[2]/DEGREE);
	SetDlgItemInt(IDC_SIMPLE_JOINT4,Angle[3]/DEGREE);
	SetDlgItemInt(IDC_SIMPLE_JOINT5,Angle[4]/DEGREE);
	SetDlgItemInt(IDC_SIMPLE_JOINT6,Angle[5]/DEGREE);

	
	strText = GetDlgItemInt(IDC_SIMPLE_JOINT1);
	if(_control)
		_control->command(SIMPLE_JSC,strText);
	strText = GetDlgItemInt(IDC_SIMPLE_JOINT2);
	if(_control)
		_control->command(SIMPLE_JSC,strText);
	strText = GetDlgItemInt(IDC_SIMPLE_JOINT3);
	if(_control)
		_control->command(SIMPLE_JSC,strText);
	strText = GetDlgItemInt(IDC_SIMPLE_JOINT4);
	if(_control)
		_control->command(SIMPLE_JSC,strText);
	strText = GetDlgItemInt(IDC_SIMPLE_JOINT5);
	if(_control)
		_control->command(SIMPLE_JSC,strText);
	strText = GetDlgItemInt(IDC_SIMPLE_JOINT6);
	if(_control)
		_control->command(SIMPLE_JSC,strText);
	strText = GetDlgItemInt(IDC_SIMPLE_TIME);
	if(_control)
		_control->command(SIMPLE_JSC,strText);
	if(_control)
		_control->command(RUN_MODE,0);
}
void Csimple_dialog_testDlg::OnBnClickedButton2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	UpdateData(TRUE);
	int strText;
	//For JSC_SIMPLE
	strText = GetDlgItemInt(IDC_SIMPLE_JOINT1);
	if(_control)
		_control->command(SIMPLE_JSC,strText);
	strText = GetDlgItemInt(IDC_SIMPLE_JOINT2);
	if(_control)
		_control->command(SIMPLE_JSC,strText);
	strText = GetDlgItemInt(IDC_SIMPLE_JOINT3);
	if(_control)
		_control->command(SIMPLE_JSC,strText);
	strText = GetDlgItemInt(IDC_SIMPLE_JOINT4);
	if(_control)
		_control->command(SIMPLE_JSC,strText);
	strText = GetDlgItemInt(IDC_SIMPLE_JOINT5);
	if(_control)
		_control->command(SIMPLE_JSC,strText);
	strText = GetDlgItemInt(IDC_SIMPLE_JOINT6);
	if(_control)
		_control->command(SIMPLE_JSC,strText);
	strText = GetDlgItemInt(IDC_SIMPLE_TIME);
	if(_control)
		_control->command(SIMPLE_JSC,strText);
	

	//For JSC
	strText = GetDlgItemInt(IDC_JOINT1);
	if(_control)
		_control->command(JSC,strText);
	strText = GetDlgItemInt(IDC_JOINT2);
	if(_control)
		_control->command(JSC,strText);
	strText = GetDlgItemInt(IDC_JOINT3);
	if(_control)
		_control->command(JSC,strText);
	strText = GetDlgItemInt(IDC_JOINT4);
	if(_control)
		_control->command(JSC,strText);
	strText = GetDlgItemInt(IDC_JOINT5);
	if(_control)
		_control->command(JSC,strText);
	strText = GetDlgItemInt(IDC_JOINT6);
	if(_control)
		_control->command(JSC,strText);

	//For JSC P Gain
	strText = GetDlgItemInt(IDC_JSC_P1);
	if(_control)
		_control->command(JSC_P_gain,strText);
	strText = GetDlgItemInt(IDC_JSC_P2);
	if(_control)
		_control->command(JSC_P_gain,strText);
	strText = GetDlgItemInt(IDC_JSC_P3);
	if(_control)
		_control->command(JSC_P_gain,strText);
	strText = GetDlgItemInt(IDC_JSC_P4);
	if(_control)
		_control->command(JSC_P_gain,strText);
	strText = GetDlgItemInt(IDC_JSC_P5);
	if(_control)
		_control->command(JSC_P_gain,strText);
	strText = GetDlgItemInt(IDC_JSC_P6);
	if(_control)
		_control->command(JSC_P_gain,strText);

	//For JSC P Gain
	strText = GetDlgItemInt(IDC_JSC_D1);
	if(_control)
		_control->command(JSC_D_gain,strText);
	strText = GetDlgItemInt(IDC_JSC_D2);
	if(_control)
		_control->command(JSC_D_gain,strText);
	strText = GetDlgItemInt(IDC_JSC_D3);
	if(_control)
		_control->command(JSC_D_gain,strText);
	strText = GetDlgItemInt(IDC_JSC_D4);
	if(_control)
		_control->command(JSC_D_gain,strText);
	strText = GetDlgItemInt(IDC_JSC_D5);
	if(_control)
		_control->command(JSC_D_gain,strText);
	strText = GetDlgItemInt(IDC_JSC_D6);
	if(_control)
		_control->command(JSC_D_gain,strText);

	//For OSC
	strText = GetDlgItemInt(IDC_DES_X);
	if(_control)
		_control->command(OSC,strText);
	strText = GetDlgItemInt(IDC_DES_Y);
	if(_control)
		_control->command(OSC,strText);
	strText = GetDlgItemInt(IDC_DES_Z);
	if(_control)
		_control->command(OSC,strText);
//	strText = GetDlgItemInt(IDC_DES_A);
//	if(_control)
//		_control->command(USER_CMD_3,strText);
//	strText = GetDlgItemInt(IDC_DES_B);
//	if(_control)
//		_control->command(USER_CMD_3,strText);
//	strText = GetDlgItemInt(IDC_DES_R);
//	if(_control)
//		_control->command(USER_CMD_3,strText);

	//For OSC P Gain
	strText = GetDlgItemInt(IDC_OSC_PX);
	if(_control)
		_control->command(OSC_P_gain,strText);
	strText = GetDlgItemInt(IDC_OSC_PY);
	if(_control)
		_control->command(OSC_P_gain,strText);
	strText = GetDlgItemInt(IDC_OSC_PZ);
	if(_control)
		_control->command(OSC_P_gain,strText);
//	strText = GetDlgItemInt(IDC_DES_A);
//	if(_control)
//		_control->command(OSC_P_gain,strText);
//	strText = GetDlgItemInt(IDC_DES_B);
//	if(_control)
//		_control->command(OSC_P_gain,strText);
//	strText = GetDlgItemInt(IDC_DES_R);
//	if(_control)
//		_control->command(OSC_P_gain,strText);

	//For OSC D Gain
	strText = GetDlgItemInt(IDC_OSC_DX);
	if(_control)
		_control->command(OSC_D_gain,strText);
	strText = GetDlgItemInt(IDC_OSC_DY);
	if(_control)
		_control->command(OSC_D_gain,strText);
	strText = GetDlgItemInt(IDC_OSC_DZ);
	if(_control)
		_control->command(OSC_D_gain,strText);
//	strText = GetDlgItemInt(IDC_DES_A);
//	if(_control)
//		_control->command(OSC_D_gain,strText);
//	strText = GetDlgItemInt(IDC_DES_B);
//	if(_control)
//		_control->command(OSC_D_gain,strText);
//	strText = GetDlgItemInt(IDC_DES_R);
//	if(_control)
//		_control->command(OSC_D_gain,strText);
}
void Csimple_dialog_testDlg::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	if(value_init < 1){
		rxBody* body = _sys->findBody(_T("Link06-4"));
		HTransform body_T = body->T();
		dVector Now_x;
		Now_x.resize(3);
		Now_x = body_T.r;
		
		float Angle[6];
		rxDevice* angle1 = _sys ->findDevice(_T("enc1"));
		rxDevice* angle2 = _sys ->findDevice(_T("enc2"));
		rxDevice* angle3 = _sys ->findDevice(_T("enc3"));
		rxDevice* angle4 = _sys ->findDevice(_T("enc4"));
		rxDevice* angle5 = _sys ->findDevice(_T("enc5"));
		rxDevice* angle6 = _sys ->findDevice(_T("enc6"));
		if(angle1 && angle2 && angle3 && angle4 && angle5 && angle6){
			angle1->readDeviceValue(&Angle[0],1*sizeof(float));
			angle2->readDeviceValue(&Angle[1],1*sizeof(float));
			angle3->readDeviceValue(&Angle[2],1*sizeof(float));
			angle4->readDeviceValue(&Angle[3],1*sizeof(float));
			angle5->readDeviceValue(&Angle[4],1*sizeof(float));
			angle6->readDeviceValue(&Angle[5],1*sizeof(float));
		}
		SetDlgItemInt(IDC_SIMPLE_JOINT1,Angle[0]/DEGREE);	
		SetDlgItemInt(IDC_SIMPLE_JOINT2,Angle[1]/DEGREE);
		SetDlgItemInt(IDC_SIMPLE_JOINT3,Angle[2]/DEGREE);
		SetDlgItemInt(IDC_SIMPLE_JOINT4,Angle[3]/DEGREE);
		SetDlgItemInt(IDC_SIMPLE_JOINT5,Angle[4]/DEGREE);
		SetDlgItemInt(IDC_SIMPLE_JOINT6,Angle[5]/DEGREE);
		SetDlgItemInt(IDC_SIMPLE_TIME,1);

		SetDlgItemInt(IDC_JOINT1,Angle[0]/DEGREE);	
		SetDlgItemInt(IDC_JOINT2,Angle[1]/DEGREE);
		SetDlgItemInt(IDC_JOINT3,Angle[2]/DEGREE);
		SetDlgItemInt(IDC_JOINT4,Angle[3]/DEGREE);
		SetDlgItemInt(IDC_JOINT5,Angle[4]/DEGREE);
		SetDlgItemInt(IDC_JOINT6,Angle[5]/DEGREE);

		SetDlgItemInt(IDC_JSC_P1,1);
		SetDlgItemInt(IDC_JSC_P2,1);
		SetDlgItemInt(IDC_JSC_P3,1);
		SetDlgItemInt(IDC_JSC_P4,1);
		SetDlgItemInt(IDC_JSC_P5,1);
		SetDlgItemInt(IDC_JSC_P6,1);

		SetDlgItemInt(IDC_JSC_D1,0);
		SetDlgItemInt(IDC_JSC_D2,0);
		SetDlgItemInt(IDC_JSC_D3,0);
		SetDlgItemInt(IDC_JSC_D4,0);
		SetDlgItemInt(IDC_JSC_D5,0);
		SetDlgItemInt(IDC_JSC_D6,0);

		SetDlgItemInt(IDC_DES_X,Now_x[0]*100);
		SetDlgItemInt(IDC_DES_Y,Now_x[1]*100);
		SetDlgItemInt(IDC_DES_Z,Now_x[2]*100);
		SetDlgItemInt(IDC_DES_A,0);
		SetDlgItemInt(IDC_DES_B,0);
		SetDlgItemInt(IDC_DES_R,0);

		SetDlgItemInt(IDC_OSC_PX,1);
		SetDlgItemInt(IDC_OSC_PY,1);
		SetDlgItemInt(IDC_OSC_PZ,1);
		SetDlgItemInt(IDC_OSC_PA,1);
		SetDlgItemInt(IDC_OSC_PB,1);
		SetDlgItemInt(IDC_OSC_PR,1);

		SetDlgItemInt(IDC_OSC_DX,0);
		SetDlgItemInt(IDC_OSC_DY,0);
		SetDlgItemInt(IDC_OSC_DZ,0);
		SetDlgItemInt(IDC_OSC_DA,0);
		SetDlgItemInt(IDC_OSC_DB,0);
		SetDlgItemInt(IDC_OSC_DR,0);
		CheckRadioButton(IDC_RADIO1,IDC_RADIO3,IDC_RADIO3);
		if(_control)
		_control->command(RUN_MODE,0);
		value_init++;
	}
	CDialog::OnLButtonDown(nFlags, point);
}

//void Csimple_dialog_testDlg::OnActivate(UINT nState, CWnd* pWndOther, BOOL bMinimized)
//{
//	CDialog::OnActivate(nState, pWndOther, bMinimized);
//
//	// TODO: 여기에 메시지 처리기 코드를 추가합니다.
//
//	rxBody* body = _sys->findBody(_T("Link06-4"));
//	HTransform body_T = body->T();
//	dVector Now_x;
//	Now_x.resize(3);
//	Now_x = body_T.r;
//		
//	float Angle[6];
//	rxDevice* angle1 = _sys ->findDevice(_T("enc1"));
//	rxDevice* angle2 = _sys ->findDevice(_T("enc2"));
//	rxDevice* angle3 = _sys ->findDevice(_T("enc3"));
//	rxDevice* angle4 = _sys ->findDevice(_T("enc4"));
//	rxDevice* angle5 = _sys ->findDevice(_T("enc5"));
//	rxDevice* angle6 = _sys ->findDevice(_T("enc6"));
//	if(angle1 && angle2 && angle3 && angle4 && angle5 && angle6){
//		angle1->readDeviceValue(&Angle[0],1*sizeof(float));
//		angle2->readDeviceValue(&Angle[1],1*sizeof(float));
//		angle3->readDeviceValue(&Angle[2],1*sizeof(float));
//		angle4->readDeviceValue(&Angle[3],1*sizeof(float));
//		angle5->readDeviceValue(&Angle[4],1*sizeof(float));
//		angle6->readDeviceValue(&Angle[5],1*sizeof(float));
//	}
//	SetDlgItemInt(IDC_STATUS_JOINT1,Angle[0]/DEGREE);	
//	SetDlgItemInt(IDC_STATUS_JOINT2,Angle[1]/DEGREE);
//	SetDlgItemInt(IDC_STATUS_JOINT3,Angle[2]/DEGREE);
//	SetDlgItemInt(IDC_STATUS_JOINT4,Angle[3]/DEGREE);
//	SetDlgItemInt(IDC_STATUS_JOINT5,Angle[4]/DEGREE);
//	SetDlgItemInt(IDC_STATUS_JOINT6,Angle[5]/DEGREE);
//	
//	SetDlgItemInt(IDC_STATUS_X,Now_x[0]*100);
//	SetDlgItemInt(IDC_STATUS_Y,Now_x[1]*100);
//	SetDlgItemInt(IDC_STATUS_Z,Now_x[2]*100);
//}


void Csimple_dialog_testDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	switch(nIDEvent){
	case MY_TIMER:
		rxBody* body = _sys->findBody(_T("Link06-4"));
		HTransform body_T = body->T();
		dVector Now_x;
		Now_x.resize(3);
		Now_x = body_T.r;
			
		float Angle[6];
		rxDevice* angle1 = _sys ->findDevice(_T("enc1"));
		rxDevice* angle2 = _sys ->findDevice(_T("enc2"));
		rxDevice* angle3 = _sys ->findDevice(_T("enc3"));
		rxDevice* angle4 = _sys ->findDevice(_T("enc4"));
		rxDevice* angle5 = _sys ->findDevice(_T("enc5"));
		rxDevice* angle6 = _sys ->findDevice(_T("enc6"));
		if(angle1 && angle2 && angle3 && angle4 && angle5 && angle6){
			angle1->readDeviceValue(&Angle[0],1*sizeof(float));
			angle2->readDeviceValue(&Angle[1],1*sizeof(float));
			angle3->readDeviceValue(&Angle[2],1*sizeof(float));
			angle4->readDeviceValue(&Angle[3],1*sizeof(float));
			angle5->readDeviceValue(&Angle[4],1*sizeof(float));
			angle6->readDeviceValue(&Angle[5],1*sizeof(float));
		}
		SetDlgItemInt(IDC_STATUS_JOINT1,Angle[0]/DEGREE);	
		SetDlgItemInt(IDC_STATUS_JOINT2,Angle[1]/DEGREE);
		SetDlgItemInt(IDC_STATUS_JOINT3,Angle[2]/DEGREE);
		SetDlgItemInt(IDC_STATUS_JOINT4,Angle[3]/DEGREE);
		SetDlgItemInt(IDC_STATUS_JOINT5,Angle[4]/DEGREE);
		SetDlgItemInt(IDC_STATUS_JOINT6,Angle[5]/DEGREE);
	
		SetDlgItemInt(IDC_STATUS_X,Now_x[0]*100);
		SetDlgItemInt(IDC_STATUS_Y,Now_x[1]*100);
		SetDlgItemInt(IDC_STATUS_Z,Now_x[2]*100);
		break;
	}
	CDialog::OnTimer(nIDEvent);
}
