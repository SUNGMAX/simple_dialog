/* RoboticsLab, Copyright 2008-2010 SimLab Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
 */
#include "simple_dialog_test_ctrl.h"
#include "simple_dialog_test_ctrlCmd.h"

simple_dialog_test_ctrl::simple_dialog_test_ctrl(rDC rdc) 
#ifdef _USE_RCONTROLALGORITHM_EX_
: rControlAlgorithmEx(rdc)
#else
: rControlAlgorithm(rdc)
#endif
, _sys(NULL), _controlSet(NULL), _home(NULL)
, _dT(0)
{
}

simple_dialog_test_ctrl::~simple_dialog_test_ctrl()
{
	delete _controlSet;

	FREE_SYSTEMS();	
}

void simple_dialog_test_ctrl::_servoOn()
{
}

void simple_dialog_test_ctrl::_servoOff()
{
}

void simple_dialog_test_ctrl::_arrangeJointDevices()
{
	for (int i = 0; i < _sys->jointDOF() + _sys->earthDOF() + _sys->constraintDOF(); i++)
	{
		TCHAR devname[32];
		_stprintf(devname, _T("motor%d"), i + 1);
		rHANDLE motor = findDevice(devname); 
		if (motor != INVALID_RHANDLE)
			addJTorDevice(motor);

		_stprintf(devname, _T("enc%d"), i + 1);
		rHANDLE enc = findDevice(devname); 
		if (enc != INVALID_RHANDLE)
			addJPosDevice(enc);

		_stprintf(devname, _T("tacho%d"), i + 1);
		rHANDLE tacho = findDevice(devname); 
		if (tacho != INVALID_RHANDLE)
			addJVelDevice(tacho);
	}
}

void simple_dialog_test_ctrl::init(int mode)
{
	// if _USE_RCONTROLALGORITHM_EX_ is defined 
	// You can import the user-defined properties from its XDL file
	//const TCHAR* kp = getProperty(_T("kp"));
	//const TCHAR* kv = getProperty(_T("kv"));

	// construct a nominal system
	_sys = LOAD_SYSTEM(_path, _aml, _T0, _q0);
	assert(_sys);
	_idyn=new rxAMBSInverseDynamics(_sys);
	_arrangeJointDevices();
	
	_q.resize(_sys->jointDOF() + _sys->earthDOF() + _sys->constraintDOF());
	_qdot.resize(_sys->jointDOF() + _sys->earthDOF() + _sys->constraintDOF());
	_torque.resize(_sys->jointDOF() + _sys->earthDOF() + _sys->constraintDOF());
	_JSC_Kp.resize(6,6);
	_JSC_Kd.resize(6,6);
	_OSC_Kp.resize(6,6);
	_OSC_Kd.resize(6,6);

	_q.zero();
	_qdot.zero();
	_torque.zero();
	_JSC_Kp.zero();
	_JSC_Kd.zero();
	_OSC_Kp.zero();
	_OSC_Kd.zero();
	for(int i = 0; i<7; i++){
		_iter[i] = 0;
	}
	_run_mode = 0;
	R_xd[0]=0;
	R_xd[1]=0;
	R_xd[2]=0;
	Arm_x.resize(3);
	Arm_x.zero();
	Arm_desired_x.resize(3);
	Arm_desired_x.zero();
	_desired_q.resize(6);
	_desired_q.zero();

	rxBody* Arm_ee = _sys->findBody(_T("Link06-4"));
	HTransform Arm_ee_T = Arm_ee->T();
	Arm_desired_x = Arm_ee_T.r;

	for(int i = 0; i<6; i++){
		_JSC_Kp(i,i) = 1;
		_JSC_Kd(i,i) = 0;
		_OSC_Kp(i,i) = 1;
		_OSC_Kd(i,i) = 0;
	}
	//  Example code for adding an interest frame..
	//	addInterestFrame(_T("desired frame"));
}

void simple_dialog_test_ctrl::_setControllers(int mode)
{
	// Before setting controllers, systems reflect current states.
	_readDevices();
	_estimate();
	_reflect();
	
	// construct the control set
	_controlSet = new rxControlSet(_sys, _dT);
	_controlSet->setGravity(0, 0, -GRAV_ACC);

	// To Do ...
	// add subtask controller 
	_home = new rxInterpolatedJointController(_sys, _dT);
	_home->setGain(20.0, 100.0);
	_home->deactivate();
	_controlSet->addController(_home, _T("home"), 100);

	// For example, 
	// 	rxBody* base = _sys->findBody(_T("base"));
	// 	assert(base != NULL);
	// 	if (base == NULL)
	// 		ucout << _T("A body named \"base\" does not exists..") << std::endl;
	// 
	// 	rxBody* ee = _sys->findBody(_T("ee"));
	// 	assert(ee != NULL);
	// 	if (ee == NULL)
	// 		ucout << _T("A body named \"ee\" does not exists..") << std::endl;
	// 
	// 	HTransform T_ee_target;
	// 	T_ee_target.r[2] = 0.0315;
	// 
	// 	HTransform T_base_ref;
	// 	T_base_ref.R.ZRotate(180*DEGREE);
	// 	T_base_ref.r.Set(0.1, 0, 0.1);
	// 
	// 	rxInterpolatedHTransformController* ctrl = new rxInterpolatedHTransformController(_sys, ee, T_ee_target, base, T_base_ref, _dT);
	// 	ctrl->setGain(20.0, 100.0, 0.0);
	// 	ctrl->deactivate();
	// 
	// 	_controlSet->addController(ctrl, _T("motion"));

	// one can replace the default internal algorithm of the control set
	// 	_controlSet->setInverseDynamicsAlgorithm(new rxAMBSGravCompensation(_sys));
	// 	_controlSet->setNullMotionController(NULL);

	// This will set the null motion gain.
	_controlSet->nullMotionController()->setNullMotionGain(1.0, 0.0);
};

void simple_dialog_test_ctrl::update(const rTime& t)
{
	rControlAlgorithm::update(t);
}

void simple_dialog_test_ctrl::setNominalSystem(const TCHAR* path, const TCHAR* aml, const HTransform& T0, const dVector& q0)
{
	_path = path;
	_aml = aml;
	_T0 = T0;
	_q0 = q0;
}

void simple_dialog_test_ctrl::setPeriod(const rTime& dT)
{
	_dT = dT;
}

void simple_dialog_test_ctrl::_readDevices()
{
}

void simple_dialog_test_ctrl::_writeDevices()
{
}

void simple_dialog_test_ctrl::_reflect()
{
	_q = *q();
	_qdot = *qdot();

	_sys->q(_q);
	_sys->qdot(_qdot);

	_idyn->update();
}

void simple_dialog_test_ctrl::_compute(const double& t)
{
	if (_controlSet)
	{
		rInverseDynamicsContext idyn_context;
		_idyn->getContext(idyn_context);
		int full_jointnumbers = 6;
		int Jointnumbers = 6;

		dMatrix A(full_jointnumbers,full_jointnumbers);
		A=idyn_context.Me;
		dVector g(full_jointnumbers);
		g=-idyn_context.Ge;

		rxBody* Arm_ee = _sys->findBody(_T("Link06-4"));
		HTransform Arm_ee_T = Arm_ee->T();
		dMatrix Arm_ee_J_local = Arm_ee->J();
		//Arm_ee_J_local = Arm_ee_J_local(0,0,6,6);
		dMatrix Arm_ee_dJ_local = Arm_ee->dJ();
		////Arm_ee_dJ_local = Arm_ee_dJ_local(0,0,6,6);
		dMatrix Arm_ee_J_global(6,Jointnumbers);
		dMatrix Arm_ee_dJ_global(6,Jointnumbers);
		dMatrix Arm_Rotation(6,6);
		dVector Arm_dx(6); // ee의 velocity
		Arm_dx.zero();
		Arm_Rotation.zero();
		Arm_Rotation.set(0,0,3,3,Arm_ee_T.R); // T_ee_global.R은 end effector 프레임의 회전 행렬
		Arm_Rotation.set(3,3,3,3,Arm_ee_T.R);
		dMatrix Arm_ee_J_global_trans;
		Arm_ee_J_global = Arm_Rotation*Arm_ee_J_local; // 글로벌 자코비안 계산
		Arm_ee_dJ_global = Arm_Rotation*Arm_ee_dJ_local;
		Arm_dx = Arm_ee_J_global*_qdot;
		Arm_dx.reduce(3,3);
		Arm_ee_J_global.rreduce(3,3); // 자코비안 4-6행은 불필요하므로 없애는 과정, rreduce(없애기 시작할 행, 없애는 행 개수), 열을 없앨 때는 creduce
		Arm_ee_J_global.transpose(Arm_ee_J_global_trans);
		Arm_x = Arm_ee_T.r; // T_ee_global.r은 end effector 프레임의 위치 벡터
		if(_run_mode==0)
			_controlSet->compute(t, _torque);
		if(_run_mode == 1)
			_torque = _JSC_Kp*(_desired_q - _q) - _JSC_Kd * _qdot+ g;
		if(_run_mode == 2)
			_torque = Arm_ee_J_global_trans * (_OSC_Kp*(Arm_desired_x - Arm_x) - _OSC_Kd * Arm_dx)+g;
		//_torque.print(_T("torque"));
		
		setTorque(_torque);
	}
}

void simple_dialog_test_ctrl::_estimate()
{
}

int simple_dialog_test_ctrl::command(const short& cmd, const int& arg)
{
	switch (cmd)
	{
	case DEFAULT_CMD:
		break;
	case SIMPLE_JSC:
		{
		if (!_home)
			return -1;
		if (!_home->activated())
			_home->activate();
		dVector qhome(_home->dim());
		if(_iter[6]<6){
			qhome.set(_iter[6],1,arg*DEGREE);
			printf("%d : %d\n",_iter[6],arg);
			
		}
		
		if(_iter[6]==6){
			printf("%d : %d\n",_iter[6],arg);
			_home->addPoint(qhome, arg, false, eInterpolatorType_Quintic);
			_iter[6] = -1;
		}
		_iter[6]++;
		}
		break;
	case JSC:
		{
		_desired_q(_iter[0]) = arg*DEGREE;
		_iter[0]++;
		if(_iter[0]>5){
			_iter[0] = 0;
		}
		}
		break;
	case JSC_P_gain:
		{
			_JSC_Kp(_iter[4],_iter[4]) = arg;
			_iter[4]++;
			if(_iter[4]>5){
				_iter[4] = 0;
			}
				
		}
		break;
	case JSC_D_gain:
		{
			_JSC_Kd(_iter[5],_iter[5]) = arg*0.01;
			_iter[5]++;
			if(_iter[5]>5){
				_iter[5] = 0;
			}
				
		}
		break;
	case RUN_MODE:
		{
			_run_mode = arg;
		}
		break;
	case OSC:
		{
			Arm_desired_x(_iter[1]) = arg*0.01;
			_iter[1]++;
			if(_iter[1]>2){
				_iter[1] = 0;
			}
				
		}
		break;
	case OSC_P_gain:
		{
			_OSC_Kp(_iter[2],_iter[2]) = arg;
			_iter[2]++;
			if(_iter[2]>2){
				_iter[2] = 0;
			}
				
		}
		break;
	case OSC_D_gain:
		{
			_OSC_Kd(_iter[3],_iter[3]) = arg;
			_iter[3]++;
			if(_iter[3]>2){
				_iter[3] = 0;
			}
				
		}
		break;
	
		// 		{
		//			// one can find a specific controller and sets a new target point..
		// 			rxInterpolatedHTransformController* motion = dynamic_cast<rxInterpolatedHTransformController*>(_controlSet->findController(_T("motion")));
		// 			assert(motion);
		// 			motion->activate();
		// 			HTransform Tf(Rotation(0, 0, -1, 0, -1, 0, -1, 0, 0), Displacement(0.11, 0.3, 0.1));
		// 			motion->addPoint(Tf, 2);
		// 		}
		// 
		// 		break;

	case RESERVED_CMD_GO_HOME:
		{
			if (!_home)
				return -1;

			if (!_home->activated())
				_home->activate();

			dVector qhome(_home->dim());			

			switch (arg)
			{
			case VK_Z:
				qhome.zero();
				break;

			case VK_H:
				qhome.all(30*DEGREE);
				break;
			}			

			_home->addPoint(qhome, 1, false, eInterpolatorType_Quintic);
		}

		break;

	case RESERVED_CMD_SERVO_ON:
		_setControllers(arg);

		_servoOn();

		break;

	case RESERVED_CMD_SERVO_OFF:
		if (!_home)
			return -1;

		if (_home->activated())
			_home->deactivate();
		_servoOff();

		break;

	default:
		break;
	}

	return 0;
}

void simple_dialog_test_ctrl::datanames(vector<string_type>& names, int channel)
{
}

void simple_dialog_test_ctrl::collect(vector<double>& data, int channel)
{
}

void simple_dialog_test_ctrl::onSetInterestFrame(const TCHAR* name, const HTransform& T)
{
}

rControlAlgorithm* CreateControlAlgorithm(rDC& rdc)
{
	return new simple_dialog_test_ctrl(rdc);
}

