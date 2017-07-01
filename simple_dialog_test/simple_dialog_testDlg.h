
// simple_dialog_testDlg.h : header file
//

#pragma once
#include "afxwin.h"

class rxSystem;
class rxControlInterface;

// Csimple_dialog_testDlg dialog
class Csimple_dialog_testDlg : public CDialog
{
// Construction
public:
	Csimple_dialog_testDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	enum { IDD = IDD_TESTDLG_ROBOTICSLAB_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();

	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnDestroy();
	afx_msg void OnBnClickedBtnStart();
	afx_msg void OnBnClickedButtonHome();
	CButton m_btnStart;
private:
	// variables for simulation
	rxSystem*			_sys;
	rxControlInterface*	_control;
	int					value_init;
	

public:
	afx_msg void OnBnClickedRadio1();
	afx_msg void OnBnClickedButton2();
	afx_msg void OnBnClickedRadio2();
	afx_msg UINT OnGetDlgCode();
	afx_msg void OnEnChangeEdit6();
	afx_msg void OnInitMenuPopup(CMenu* pPopupMenu, UINT nIndex, BOOL bSysMenu);
	afx_msg int  OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnBnClickedRadio3();

//	afx_msg void OnLButtonDblClk(UINT nFlags, CPoint point);
//	afx_msg void OnUpdateUIState(UINT /*nAction*/, UINT /*nUIElement*/);
//	afx_msg void OnActivate(UINT nState, CWnd* pWndOther, BOOL bMinimized);
	afx_msg void OnTimer(UINT_PTR nIDEvent);
};
