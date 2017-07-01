/* RoboticsLab, Copyright 2008-2010 SimLab Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
 */
#ifndef __SIMPLE_DIALOG_TEST_CTRL_H__
#define __SIMPLE_DIALOG_TEST_CTRL_H__


#include "rControlAlgorithm/rControlAlgorithm.h"
#include "rxControlSDK/rxControlSDK.h"

//#define _USE_RCONTROLALGORITHM_EX_



#ifdef _USE_RCONTROLALGORITHM_EX_
class REXPORT simple_dialog_test_ctrl : public rControlAlgorithmEx
#else
class REXPORT simple_dialog_test_ctrl : public rControlAlgorithm
#endif
{
public:
	simple_dialog_test_ctrl(rDC rdc);
	~simple_dialog_test_ctrl();

	virtual void init(int mode = 0);
	virtual void update(const rTime& t);
	virtual void setNominalSystem(const TCHAR* path, const TCHAR* aml, const HTransform& T0, const dVector& q0);
	virtual void setPeriod(const rTime& dT);
	virtual int command(const short& cmd, const int& arg = 0);
	virtual void datanames(vector<string_type>& names, int channel = -1);
	virtual void collect(vector<double>& data, int channel = -1);
	virtual void onSetInterestFrame(const TCHAR* name, const HTransform& T);

private:
	virtual void _estimate();
	virtual void _readDevices();
	virtual void _writeDevices();

	virtual void _reflect();
	virtual void _compute(const rTime& t);

	void _arrangeJointDevices();

	void _servoOn();
	void _servoOff();

	void _setControllers(int mode = 0); 

private:

	dVector				_q;
	dVector				_qdot;
	dVector				_torque;
	dVector				Arm_desired_x;
	dVector				Arm_x;
	dVector				init_x;
	rxSystem*			_sys;
	rxControlSetBase*	_controlSet;
	rxJointController*	_home;

	string_type			_path;
	string_type			_aml;
	HTransform			_T0;
	dVector				_q0;

	double				_dT;

	rxAMBSInverseDynamics*	_idyn;

	
	float				R_xd[3];
	int					_iter[7];
	int					_run_mode;
	dVector				_desired_q;
	dMatrix				_JSC_Kp;
	dMatrix				_JSC_Kd;
	dMatrix				_OSC_Kp;
	dMatrix				_OSC_Kd;
};
#endif