/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/
#include "Robot.h"
using namespace std;
using namespace Eigen;
using namespace RobotJoint;

Robot::Robot()
/**     @brief default constructor
*/
{
        //some default values
        mass = 32.0 /*kg*/;
        preffered_com_height = 0.46 /*m*/;
        
        RJMatrixX A,B,C,D,Q,R;

        RJReal dt = 0.001;
        A.resize(3,3);
        A<<
        1,      dt,      dt*dt/2,
        0,      1,      dt,
        0,      0,      1;
        B.resize(3,1);
        B<<
        dt*dt*dt/6.0,
        dt*dt/2.0,
        dt;
        C.resize(1,3);
        C<<     
        1,      0,      -preffered_com_height/9.81;
        D.resize(3,1);
        D <<
        0,      0,      0;
        Q.resize(3,3);
        Q<<
        1,      0,      0,
        0,      0,      0,
        0,      0,      0;
        R.resize(1,1);
        R(0,0) = 1E-6;
        
        COMPreviewController = Controller(A,B,C,D,Q,R);
}

Robot::~Robot()
/** @brief Destructor: Deletes Manipulator objects pointed to by pointers in Manipulators Map.
*/
{
        ManipMapType::iterator it;
        for(it=Manipulators.begin(); it!=Manipulators.end(); it++)
        {
                delete it->second;
        }
}

int Robot::update(const RJVectorXMap &Angles)
/**     @brief Computes most robot and manipulator updates in a efficent manner
*       @param[in] Angles joint angles
*/
{

        ManipMapType::iterator it;
        RJVectorXMap::const_iterator it2;
	RJMatrixX temp;
	RJVectorX qtemp;
	int index_start = 0;
	int index_end = 0;
        it=Manipulators.begin();
//         while(it!=Manipulators.end())
        for(it=Manipulators.begin(); it!=Manipulators.end(); it++)
        {
                it2 = Angles.find(it->first);
		RJASSERT(it2 != Angles.end(), "Robot::update, can't find manipulator with RJID "<<it->first<<endl);
                //This code tries to split one continous angles vector into the sub vectors needed for each manipulator
// 		index_end = index_end + (*it).second->getDOF() - 1; //use getDOF because immobile links dont take in angles
// 		assert(index_end < q.size());
		//cout<<"index_start "<<index_start<<"index_end "<<index_end<<"\n";
		//### TODO Handle jacobian inverse updates for manipulators with less than 6 joints
//                 assert(it2->second.size() == it->second->getDOF());
		RJASSERT(it2->second.size() == it->second->getDOF(),"Robot::update, size mismatch! internal vector size for manipulator ID "<<it->first<<" is "<<it->second->getDOF()<<" but provided vector is size "<<it2->second.size()<<endl);
// 			qtemp = q.segment(index_start,(*it).second->getDOF());
// 			(*it).second->update(qtemp);
                it->second->update(it2->second);
			//Now calculate the scaled COM Jacobians and their inverse's
			temp = (*it).second->getCOMJacobian();
			COMJac[(*it).first] = _ScaleCOMJacobian(temp,(*it).first);
			temp = (*it).second->getCOMJacobianWRTFootFrame();
			COMJacWRT2Tool[(*it).first] = _ScaleCOMJacobian(
			temp,(*it).first);
		if(index_end - index_start >= 5 )
		{
//@			iCOMJac[(*it).first] = SVDinv(COMJac[(*it).first],0.00001); 
//@			iCOMJacWRT2Tool[(*it).first] = SVDinv(COMJacWRT2Tool[(*it).first],0.00001); 
		}
		//calculate the COM
		COM = getCOM(true);
// 		index_start = index_end+1;
//                 it++;
        }

return 0;
}


RJVector4 Robot::getCOM(const RJVectorXMap &Angles)
/** @brief Gets the COM of all manipulators attached to this robot
*/
{
//for (MapTypeManip::iterator inner = this->Manipulator.begin(); inner != this->Manipulator.end(); ++inner)
        ManipMapType::iterator it;
        RJVectorXMap::const_iterator it2;
        RJVector4 Pm; //com position(3 rows), mass (1 row)
        RJVector3 top = RJVector3::Zero();
	RJVectorX qtemp;
        RJReal bottom=0;
        it=Manipulators.begin();  
        int q_index_from = 1;
        int q_index_to   = 0;

        //it++;
        //it++;
        //(*it).second->getCOM(q.rows(13,13));
        while(it!=Manipulators.end())
        for(it=Manipulators.begin(); it!=Manipulators.end(); it++)
        {
                it2 = Angles.find(it->first);
                assert(it2 != Angles.end());
                //q_index_from    = q_index_to + 1;
                //q_index_to     += (*it).second->getDOF();
               // cout<<"from "<<q_index_from<<" to "<<q_index_to<<" \n";
		//qtemp = q.segment(q_index_from,q_index_to);
                //Pm=(*it).second->getCOM(qtemp);
                Pm=it->second->getCOM(it2->second);
                top+=Pm.head(3)*Pm(3);
                //cerr<<(*it).first<<" "<<Pm.rows(1,3)<<endl;
                bottom+=Pm(3);
//                 it++;
        }
        Pm.head(3)= top*(1.0/bottom); 
        Pm(3)=bottom;
        return Pm;
}

RJVector4 Robot::getCOM(bool calculate)
/** @brief Gets the COM of all manipulators attached to this robot - cached
* TODO split this into two overloaded functions, the one taking in void should return the cached value
*/
{
	if(calculate)
	{
		RJVector4 Pm; //com position(3 rows), mass (1 row)
		RJVector3 top = RJVector3::Zero();
		RJReal bottom=0;

		ManipMapType::iterator it;
		it=Manipulators.begin();
		while(it!=Manipulators.end())
		{
			Pm=(*it).second->getCOM();
			top+=Pm.head(3)*Pm(3);
			bottom+=Pm(3);
			it++;
        	}
		Pm.head(3)= top*(1.0/bottom);
		Pm(3)=bottom;
		return Pm;
	}
	else return COM;
}

RJVector3 Robot::getCOMvel()
/** @brief Gets the COM  velocity - cached
*/
{
		RJVector3 COMvel = RJVector3::Zero();	
		for(ManipMapType::iterator it = Manipulators.begin(); it != Manipulators.end(); it++)
		{
			COMvel += it->second->getCOMJacobian() * it->second->getdq();
		}
		return COMvel;
}

RJVector3 Robot::getCOMvelWRTFootFrame()
/** @brief Gets the COM  velocity - cached
*/
{
		RJVector3 COMvel = RJVector3::Zero();	
		for(ManipMapType::iterator it = Manipulators.begin(); it != Manipulators.end(); it++)
		{
			COMvel += (it->second->getCOMJacobianWRTFootFrame() * it->second->getdq()).head<3>();
		}
		return COMvel;
}

RJVector3 Robot::getCOMvelWRTFootFrame2()
/** @brief Gets the COM  velocity - cached
*/
{
		static RJVector3 COMPosold = RJVector3::Zero();
		RJVector3 COMPos = RJVector3::Zero();
		RJVector3 COMvel = RJVector3::Zero();

		COMPos = BaseVector2ToolFrame(getCOM().head<3>(), RJ_RightLeg);
		COMvel = 1E2*(COMPos - COMPosold);
		COMPosold = COMPos;
		return COMvel;
}

RJVector3 Robot::getCOMvel(RJVectorXMap q, RJVectorXMap dq)
/** @brief Gets the COM velocity
*/
{
		RJVector3 COMvel = RJVector3::Zero();	
		for(ManipMapType::iterator it = Manipulators.begin(); it != Manipulators.end(); it++)
		{
			COMvel = it->second->getCOMJacobian(q[it->first]) * dq[it->first];
		}
		return COMvel;
}

RJMatrixX Robot::getJacobian(const RoboJointVars &var)
/** @brief Gets the manipulators Jacobian - cached
*/
{
	return Manipulators[var]->getJacobian();
}

RJMatrixX Robot::getInvJacobian(const RoboJointVars &var)
/** @brief Gets the manipulators inverse Jacobian - cached
*/
{
	return Manipulators[var]->getInvJacobian();
}

RJMatrixX Robot::getCOMJacobian(const RoboJointVars &var, const RJVectorX &q)
/**     @brief Gets the mass scaled COM Jacobian of a attached manipulator
*/
{
	RJMatrixX temp;
	temp = Manipulators[var]->getCOMJacobian(q);
        return _ScaleCOMJacobian(temp,var);
}

RJMatrixX Robot::getCOMJacobian(const RoboJointVars &var)
/**     @brief Gets the manipulators COM Jacobian - cached
*       This function also scales the COM Jacobian to account for mass of the other manipulators
*/
{
	return COMJac[var];
}

RJMatrixX Robot::getInvCOMJacobian(const RoboJointVars &var)
/**     @brief Gets the manipulators inverse COM Jacobian - cached
*       This function also scales the COM Jacobian to account for mass of the other manipulators
*/
{
//	return _ScaleCOMJacobian(Manipulators[var]->getInvCOMJacobian(),var);
	return iCOMJac[var];
}

RJMatrixX Robot::getCOMJacobianWRTFootFrame(const RoboJointVars &var)
/**     @brief Gets the manipulators COM Jacobian in foot frame - cached
*       This function also scales the COM Jacobian to account for mass of the other manipulators
*/
{
//	return _ScaleCOMJacobian(Manipulators[var]->getCOMJacobianWRTFootFrame(),var);
	return COMJacWRT2Tool[var];
}

RJMatrixX Robot::getCOMJacobianWRTFootFrame(const RoboJointVars &var, const RJVectorX &q)
/**     @brief Gets the manipulators COM Jacobian in foot frame - cached
*       This function also scales the COM Jacobian to account for mass of the other manipulators
*/
{
        RJMatrixX temp = Manipulators[var]->getCOMJacobianWRTFootFrame(q);
//         return temp;
        return _ScaleCOMJacobian(temp,var);
}

RJMatrixX Robot::getInvCOMJacobianWRTFootFrame(const RoboJointVars &var)
/**     @brief Gets the manipulators COM Jacobian in foot frame - cached
*       This function also scales the COM Jacobian to account for mass of the other manipulators
*/
{
	return iCOMJacWRT2Tool[var];
}

RJMatrixX Robot::_ScaleCOMJacobian(RJMatrixX &jac, const RoboJointVars &manip_name)
/**     @brief Scales the COM Jacobian so that the mass of other manipulators is accounted for
*       Since the jacobian on the manipulators side is made with the scaling "Mass_link/Mass_Manipulator" and we want to 
*       rescale it to "Mass_link/Mass_all_Manipulators", we multiply by "Mass_Manipulator/Mass_all_Manipulators" 
*/
{
	ManipMapType::iterator it;
	RJReal mass_total=0;
	RJReal manip_mass=0;
	//find total mass
        for(it = Manipulators.begin(); it != Manipulators.end(); it++)	mass_total+=it->second->get_total_mass();

	manip_mass=Manipulators[manip_name]->get_total_mass();
	//scale the first 3 rows of the jac by total mass
	for(int i=0;i < jac.cols();i++)
	{
		for(int j=0;j < 3;j++)
		{
			/* Since the jacobian on the manipulators side is made with the scaling
			"Mass_link/Mass_Manipulator" and we want to rescale it 
			to "Mass_link/Mass_all_Manipulators", we multiply 
			by "Mass_Manipulator/Mass_all_Manipulators" */
			jac(j,i)=(manip_mass/mass_total)*jac(j,i); 
		}
	}
	return jac;
}

RobotManipulator* Robot::getManipulator(RoboJointVars var)
/** @brief Returns a pointer to an attached manipulator
*/
{
        return Manipulators[var];
}

int Robot::addManipulator(RoboJointVars name, RobotManipulator* manip)
/** @brief Adds a manipulator to the "Manipulators" map
*/
{
	RJMatrixX Dummy(6,6);
        Manipulators.insert(make_pair(name,manip));

	COMJac.insert(make_pair(name,Dummy));
	iCOMJac.insert(make_pair(name,Dummy));
	COMJacWRT2Tool.insert(make_pair(name,Dummy));
	iCOMJacWRT2Tool.insert(make_pair(name,Dummy));
        _Angles.insert(make_pair(name,manip->getAngles()));
}

RJVector3 Robot::BaseVector2ToolFrame(const RJVector3 &v, const RJVectorX &angles, const RoboJointVars &manip_name)
/** @brief transforms a given vector from the base to the tool tip frame
*/
{
        return Manipulators[manip_name]->BaseVector2ToolFrame(v,angles);
}

RJVector3 Robot::BaseVector2ToolFrame(const RJVector3 &v, const RoboJointVars &manip_name)
/** @brief transforms a given vector from the base to the tool tip frame - cached
*/
{
        return Manipulators[manip_name]->BaseVector2ToolFrame(v);
}

RJMatrixX Robot::MakeZMPTrajectory(int from, int to)
/**     @brief Converts the SupportPhases vector to a ZMP trajectory
*       @param from SupportPhases index to start at.
*       @param to SupportPhases index to end at.
*       @returns nx1 ZMP trajectory
*        
*       So far the following support phase transitions are supported
*               Pure Left Support
*               Pure Right Support
*               Left to Right transition via RJReal
*               Right to Left transition via RJReal
*               Starting with RJReal support and transitioning to left or right
*               
*               **Todo**
*               left/right -> RJReal ->RJReal
*               RJReal -> RJReal
*/
{
        
        if (from == -1 && to == -1)
        {
                from = 0;
                to = SupportPhases.size()-1;
        }
        
        if ( (from < 0) || (from > SupportPhases.size() - 1) ) throw "Robot::getZMPTrajectory() : From is invalid!";
        if ( (to < 0) || (to < SupportPhases.size() - 1) ) throw "Robot::getZMPTrajectory() : to is invalid!";
        
        int nsamples = Sec2Samples(SupportPhases[to].EndTime - SupportPhases[from].StartTime);
        RJMatrixX ret(nsamples,2);
        
        for(int j=from; j<=to; j++)
        {
                if (SupportPhases[j].Phase == LeftSup)
                {
                        for(int i=Sec2Samples(SupportPhases[j].StartTime); i<Sec2Samples(SupportPhases[j].EndTime); i++)
                        {
                                ret(i, 0) = SupportPhases[j].LeftFoot->FootPosX;
                                ret(i, 1) = SupportPhases[j].LeftFoot->FootPosY;
                        }
                }
                else if (SupportPhases[j].Phase == RightSup)
                {
                        for(int i=Sec2Samples(SupportPhases[j].StartTime); i<Sec2Samples(SupportPhases[j].EndTime); i++)
                        {
                                ret(i, 0) = SupportPhases[j].RightFoot->FootPosX;
                                ret(i, 1) = SupportPhases[j].RightFoot->FootPosY;
                        }
                }
                else if (SupportPhases[j].Phase == DoubleSup)
                {
                        if((j+1<SupportPhases.size()) && (j>0))
                        {
                                if((SupportPhases[j+1].Phase == RightSup) && (SupportPhases[j-1].Phase == LeftSup))
                                {
                                        poly5thMakeCoefficents("x",SupportPhases[j].StartTime, SupportPhases[j].EndTime, 
                                        SupportPhases[j].LeftFoot->FootPosX, SupportPhases[j].RightFoot->FootPosX);

                                        poly5thMakeCoefficents("y",SupportPhases[j].StartTime, SupportPhases[j].EndTime, 
                                        SupportPhases[j].LeftFoot->FootPosY, SupportPhases[j].RightFoot->FootPosY);
                                        
                                        for(int i=Sec2Samples(SupportPhases[j].StartTime); i<Sec2Samples(SupportPhases[j].EndTime); i++)
                                        {
                                                ret(i, 0) = poly5th("x",Samples2Sec(i));
                                                ret(i, 1) = poly5th("y",Samples2Sec(i));
                                        }
                                }
                                //Left to right Transition
                                if((SupportPhases[j+1].Phase == LeftSup) && (SupportPhases[j-1].Phase == RightSup))
                                {
                                        poly5thMakeCoefficents("x",SupportPhases[j].StartTime, SupportPhases[j].EndTime, 
                                        SupportPhases[j].RightFoot->FootPosX, SupportPhases[j].LeftFoot->FootPosX);
                                        
                                        poly5thMakeCoefficents("y",SupportPhases[j].StartTime, SupportPhases[j].EndTime, 
                                        SupportPhases[j].RightFoot->FootPosY, SupportPhases[j].LeftFoot->FootPosY);
                                        
                                        for(int i=Sec2Samples(SupportPhases[j].StartTime); i<Sec2Samples(SupportPhases[j].EndTime); i++)
                                        {
                                                ret(i, 0) = poly5th("x",Samples2Sec(i));
                                                ret(i, 1) = poly5th("y",Samples2Sec(i));
                                        }
                                }
                        }
                        else if((j+1<SupportPhases.size()) && (j==0) && ((SupportPhases[j+1].Phase == RightSup) || 
                                (SupportPhases[j+1].Phase == LeftSup)))
                        {
                                //Left to Right Transition
                                if((SupportPhases[j+1].Phase == RightSup))
                                {
                                        poly5thMakeCoefficents("x",SupportPhases[j].StartTime, SupportPhases[j].EndTime, 
                                        (SupportPhases[j].LeftFoot->FootPosX + SupportPhases[j].RightFoot->FootPosX)*0.5, SupportPhases[j].RightFoot->FootPosX);

                                        poly5thMakeCoefficents("y",SupportPhases[j].StartTime, SupportPhases[j].EndTime, 
                                        (SupportPhases[j].LeftFoot->FootPosY + SupportPhases[j].RightFoot->FootPosY)*0.5, SupportPhases[j].RightFoot->FootPosY);
                                        
                                        for(int i=Sec2Samples(SupportPhases[j].StartTime); i<Sec2Samples(SupportPhases[j].EndTime); i++)
                                        {
                                                ret(i, 0) = poly5th("x",Samples2Sec(i));
                                                ret(i, 1) = poly5th("y",Samples2Sec(i));
                                        }
                                }
                                //Left to right Transition
                                else if((SupportPhases[j+1].Phase == LeftSup))
                                {
                                        poly5thMakeCoefficents("x",SupportPhases[j].StartTime, SupportPhases[j].EndTime, 
                                        (SupportPhases[j].LeftFoot->FootPosX + SupportPhases[j].RightFoot->FootPosX)*0.5, SupportPhases[j].LeftFoot->FootPosX);

                                        poly5thMakeCoefficents("y",SupportPhases[j].StartTime, SupportPhases[j].EndTime, 
                                        (SupportPhases[j].LeftFoot->FootPosY + SupportPhases[j].RightFoot->FootPosY)*0.5, SupportPhases[j].LeftFoot->FootPosY);
                                        
                                        for(int i=Sec2Samples(SupportPhases[j].StartTime); i<Sec2Samples(SupportPhases[j].EndTime); i++)
                                        {
                                                ret(i, 0) = poly5th("x",Samples2Sec(i));
                                                ret(i, 1) = poly5th("y",Samples2Sec(i));
                                        }
                                }
                        }
                        else throw "MakeZMPTrajectory::Phase tansition not supported!";
                }
        }
        return ret;
}

int Robot::addMovementPhase(LegPhase phase,RJReal start, RJReal end, RJReal footAposX, RJReal footAposY, RJReal footBposX, RJReal footBposY)
/**     @brief adds a movement phase with absolute timing
*       @param phase LeftSupport, RightSupport, DoubleSupport, etc
*       @param start global start time in seconds
*       @param end global end time in seconds
*       @param footAposX left foots x position
*       @param footAposY left foots y position
*       @param footBposX right foots x position
*       @param footBposY right foots y position
*/
{
        SupportPhases.push_back(SupportPhase(phase, start,end,new Step(start, end, footAposX, footAposY), new Step(start, end, footBposX, footBposY)));
        
        return 0;
}

int Robot::addMovementPhase(LegPhase phase,RJReal dt, RJReal footAposX, RJReal footAposY, RJReal footBposX, RJReal footBposY)
/**     @brief adds a movement phase with relative timing
*       @param phase LeftSupport, RightSupport, DoubleSupport, etc
*       @param dt time duration of phase
*       @param footAposX left foots x position
*       @param footAposY left foots y position
*       @param footBposX right foots x position
*       @param footBposY right foots y position
*/
{
        RJReal start;
        if (SupportPhases.size() > 0) start = SupportPhases[SupportPhases.size()-1].EndTime;
        else start = 0;
        RJReal end = start + dt;
        SupportPhases.push_back(SupportPhase(phase, start,end,new Step(start, end, footAposX, footAposY), new Step(start, end, footBposX, footBposY)));
        return 0;
}

RJMatrixX Robot::MakeCOMTrajectory(const RJMatrixX &ZMPTrj)
/**     @brief Uses ZMP preview control to generate a COM trajectory
*       @param ZMPTrj nx2 matrix(where n is the #of timesamples). 1st column is ZMP-X trajectory, second is ZMP-Y.
*       @returns 6x1 COM trajectory with this structure: [x dx ddx y dy ddy]
*/
{
        RJMatrixX COMTrj= RJMatrixX::Zero(ZMPTrj.rows(),6);
        COMPreviewController.CalcPreviewGain(2000);        
        
        
#ifdef RJ_THREADED          
                //TODO BUG this is actually slower, than non threaded, may be that on a dual core machine this is a waste
    auto PreviewControlX = [&] () {
        COMTrj.block(0,0,ZMPTrj.rows(),3) = COMPreviewController.PreviewControl(ZMPTrj.col(0)); //X
};
                std::thread t[2];                
                t[0] =  std::thread(PreviewControlX);
                COMTrj.block(0,3,ZMPTrj.rows(),3) = COMPreviewController.PreviewControl(ZMPTrj.col(1)); //Y
                t[0].join();
#else
        COMTrj.block(0,0,ZMPTrj.rows(),3) = COMPreviewController.PreviewControl(ZMPTrj.col(0)); //X
        COMTrj.block(0,3,ZMPTrj.rows(),3) = COMPreviewController.PreviewControl(ZMPTrj.col(1)); //Y
#endif        
        
        return COMTrj;
}

std::vector<RJVectorXMap > Robot::MakeJointTrajectory(const RJMatrixX &COMTrj)
/**     @brief constructs joint trajectory (must use addMovementPhase first!)
        @param[in] COMTrj Center of mass trajectory, 9x1 matrix , row 1 = [x dx ddx y dy ddy]
        This function uses the Robot::SupportPhases vector and CL_IK inerse kinematics; Assumes arms keep their current positions
*/
{
        RJMatrixX RightLeg2World(COMTrj.rows(),3);
        RJMatrixX LeftLeg2World(COMTrj.rows(),3);
        int SupportPhaseIndex=0;
        std::vector<RJVectorXMap > ret;
        RJVector3 COM2World = RJVector3::Zero();
        
        RJMatrixX prnt = RJMatrixX::Zero(COMTrj.rows(),6);
        
        for(int j = 0; j < SupportPhases.size(); j++)
        {
                if(isSingleSupport(SupportPhases[j]) 
                        && (j<SupportPhases.size()-1) && isDoubleSupport(SupportPhases[j+1])
                        && (j>0)                      && isDoubleSupport(SupportPhases[j-1]) )
                {
                        if(isLeftSupport(SupportPhases[j]))
                        {
                                poly5thMakeCoefficents("xSwing",SupportPhases[j].StartTime, SupportPhases[j].EndTime, 
                                SupportPhases[j-1].RightFoot->FootPosX, SupportPhases[j+1].RightFoot->FootPosX);

                                poly5thMakeCoefficents("ySwing",SupportPhases[j].StartTime, SupportPhases[j].EndTime, 
                                SupportPhases[j-1].RightFoot->FootPosY, SupportPhases[j+1].RightFoot->FootPosY);
                                
                                for(int k=Sec2Samples(SupportPhases[j].StartTime); k<Sec2Samples(SupportPhases[j].EndTime); k++)
                                {
                                        RightLeg2World(k,0) = poly5th("xSwing",Samples2Sec(k));
                                        RightLeg2World(k,1) = poly5th("ySwing",Samples2Sec(k));
                                        RightLeg2World(k,2) = 0;
                                        
                                        LeftLeg2World(k,0) = SupportPhases[j].LeftFoot->FootPosX;
                                        LeftLeg2World(k,1) = SupportPhases[j].LeftFoot->FootPosY;
                                        LeftLeg2World(k,2) = 0;
                                }
                        }
                        else
                        {
                                poly5thMakeCoefficents("xSwing",SupportPhases[j].StartTime, SupportPhases[j].EndTime, 
                                SupportPhases[j-1].LeftFoot->FootPosX, SupportPhases[j+1].LeftFoot->FootPosX);

                                poly5thMakeCoefficents("ySwing",SupportPhases[j].StartTime, SupportPhases[j].EndTime, 
                                SupportPhases[j-1].LeftFoot->FootPosY, SupportPhases[j+1].LeftFoot->FootPosY);
                                
                                for(int k=Sec2Samples(SupportPhases[j].StartTime); k<Sec2Samples(SupportPhases[j].EndTime); k++)
                                {
                                        RightLeg2World(k,0) = SupportPhases[j].RightFoot->FootPosX;
                                        RightLeg2World(k,1) = SupportPhases[j].RightFoot->FootPosY;
                                        RightLeg2World(k,2) = 0;
                                        
                                        LeftLeg2World(k,0) = poly5th("xSwing",Samples2Sec(k));
                                        LeftLeg2World(k,1) = poly5th("ySwing",Samples2Sec(k));
                                        LeftLeg2World(k,2) = 0;
                                }
                        }
                }
                else
                {
                                for(int k=Sec2Samples(SupportPhases[j].StartTime); k<Sec2Samples(SupportPhases[j].EndTime); k++)
                                {
                                        RightLeg2World(k,0) = SupportPhases[j].RightFoot->FootPosX;
                                        RightLeg2World(k,1) = SupportPhases[j].RightFoot->FootPosY;
                                        RightLeg2World(k,2) = 0;
                                        
                                        LeftLeg2World(k,0) = SupportPhases[j].LeftFoot->FootPosX;
                                        LeftLeg2World(k,1) = SupportPhases[j].LeftFoot->FootPosY;
                                        LeftLeg2World(k,2) = 0;
                                }                        
                }
                
        }
        
        for(int i = 0; i < COMTrj.rows(); i++)
        {
                COM2World << COMTrj(i,0), COMTrj(i,3), 0.3;//preffered_com_height;
                
                RJVector3 ll2w = LeftLeg2World.row(i);
                ret.push_back(CL_IK(COM2World-ll2w, RJQuaternion(RJMatrix3::Identity()), 
                      RightLeg2World.row(i)-LeftLeg2World.row(i), RJQuaternion(RJMatrix3::Identity()),500));
                
//                 prnt(i,0) = COM2World(0)-LeftLeg2World(i,0);
//                 prnt(i,1) = COM2World(1)-LeftLeg2World(i,1);
//                 prnt(i,2) = COM2World(2)-LeftLeg2World(i,2);
                prnt(i,3) = RightLeg2World(i,0)-LeftLeg2World(i,0);
                prnt(i,4) = RightLeg2World(i,1)-LeftLeg2World(i,1);
                prnt(i,5) = RightLeg2World(i,2)-LeftLeg2World(i,2);
   
                
//                 RJVector6 qqr = ret[i][RJ_RightLeg];
//                 RJVector6 qql = ret[i][RJ_LeftLeg];
//                 
//                 RJVector3 xx = TRANS(getManipulator(RJ_RightLeg)->fk(qqr));
//                 RJVector3 vv= getManipulator(RJ_LeftLeg)->BaseVector2ToolFrame(xx,qql);
//                 
//                 prnt(i,0) = vv(0);
//                 prnt(i,1) = vv(1);
//                 prnt(i,2) = vv(2);
                
                prnt(i,0) = ret[i][RJ_LeftLeg](2);
                prnt(i,1) = ret[i][RJ_LeftLeg](3);
                prnt(i,2) = ret[i][RJ_LeftLeg](4);
        }
        Log("L2C.dat",prnt);
        return ret;
}

RJVectorXMap Robot::CL_IK(const RJVector3Map &XRef, const RJQuaternionMap &OriRef,  const std::map<RoboJointVars, bool> &use_com)
/**     @brief Inverse Kinematics for robot: EXPERIMENTAL FUNCTION!!!
*/
{
        RJMatrixXMap iJacob;
        RJMatrixX iiJacob;
        RJVector6Map Error;
        RJVector6 eError;
        RJVectorXMap angles;
        RJQuaternion OrientationCurr;
        RJQuaternion OrientationError;
        ManipMapType::iterator it;
        RJVector3Map::const_iterator itXR;
        RJQuaternionMap::const_iterator itOR;
        RJVector3 compos;
        
        angles = _Angles;
        for(it=Manipulators.begin(); it!=Manipulators.end(); it++) angles[it->first] = it->second->getAngles();
        
        
        for(int i=0; i<1000; i++){
        for(itXR=XRef.begin(); itXR!=XRef.end(); itXR++)
        {
                it = Manipulators.find(itXR->first);
                assert(it!=Manipulators.end());
                itOR = OriRef.find(it->first);
                assert(itOR!=OriRef.end());
                
                RJMatrixX temp;
                temp = it->second->getCOMJacobianWRTFootFrame(angles[it->first]);
//                 temp = getCOMJacobianWRTFootFrame(it->first,angles[it->first]); //BUG scaling the jacobian makes convergence bad, strange..
                pseudoInverse(temp, iiJacob );
                
                compos = getCOM(angles).segment(0,3);
                eError.segment(0,3) = (itXR->second - BaseVector2ToolFrame(compos, angles[it->first], itXR->first));

                OrientationCurr = RJQuaternion(ROT(it->second->fk(angles[it->first])));
                OrientationCurr.normalize();
                
                OrientationCurr.vec() = -OrientationCurr.vec();//frame e, frame 1
                OrientationError = itOR->second*OrientationCurr;

                eError.segment(3,3) = ROT((*it).second->fk(angles[(*it).first]))*OrientationError.vec();                               
                angles[it->first] += iiJacob*eError;
                
                //TODO make a proper completion check
                if(eError.norm() < 0.0001) break;
        }
        }
        
//         it = Manipulators.find(RJ_RightLeg);
//         compos = getCOM(angles).segment(0,3);
//         cout << "COM 2 right leg"
//         <<
//         BaseVector2ToolFrame(compos, angles[it->first], RJ_RightLeg)
//         <<endl;
//         
//         it = Manipulators.find(RJ_LeftLeg);
//         
//         cout << "COM 2 left leg"
//         <<
//         BaseVector2ToolFrame(compos, angles[it->first], RJ_LeftLeg)
//         <<endl;
//         
//         cout << "COM"
//         <<getCOM(angles)
//         <<endl;
//         
//         cout << "foot-COM"<<
//         (TRANS((*it).second->fk(angles[(*it).first]))-getCOM(angles).segment(0,3))
//         <<endl;
        
        return angles;

}


// RJVectorXMap Robot::CL_IK(const RJVector3Map &XRef, const RJQuaternionMap &OriRef,  const RJVector3 &LeftLeg2COM, const RJQuaternion &LeftLeg2COMOrientation, const RJVector3 &LeftLeg2RightLeg, const RJQuaternion &LeftLeg2RightLegOrientation)
RJVectorXMap Robot::CL_IK(const RJVector3 &LeftLeg2COM, const RJQuaternion &LeftLeg2COMOrientation, const RJVector3 &LeftLeg2RightLeg, const RJQuaternion &LeftLeg2RightLegOrientation, int maxIter, double epsError)
/**     @brief Inverse Kinematics for robot: EXPERIMENTAL FUNCTION!!!
*/
{
        /*
         * TODO OPTIMIZE:
         * first store angles, then ...
         */
        RJQuaternion OrientationCurr;
        RJQuaternion OrientationError;        
        RJMatrixXMap iJacob;
        RJMatrixX iiJacob;
        RJVector12 eError = RJVector12::Zero();
        RJVectorXMap angles;
        ManipMapType::iterator it;
        RJVector3Map::const_iterator itXR;
        RJQuaternionMap::const_iterator itOR;
        RJVector3 compos;
        
        angles = _Angles;
        for(it=Manipulators.begin(); it!=Manipulators.end(); it++) angles[it->first] = it->second->getAngles();
        
        
        for(int i=0; i<maxIter; i++){
//first contruct the combined Jacobian
                
                //this is the total jacobian; [dXCOM dThetaCOM dXLeft2Right dThetaLeft2Right] = temp * dQ
                // temp = [J11 J12; J21 J22]
//                 RJMatrixX temp = RJMatrixX::Zero(12,12);//TODO use getDOF()
                RJMatrixX Rot2 = RJMatrixX::Zero(6,6);
//                 RJMatrixX temp2 = RJMatrixX::Zero(6,6);
                RJMatrix3 Rot = RJMatrix3::Zero();


                //get COM portion of total Jacobian
//                 temp.block<6,6>(0,0) = Manipulators[RJ_LeftLeg]->getCOMJacobianWRTFootFrame(getCOM(angles), angles[RJ_LeftLeg]);
// //                 Rot = ROT(Manipulators[RJ_RightLeg]->fk(angles[RJ_RightLeg]))*ROT(Manipulators[RJ_LeftLeg]->fk(angles[RJ_LeftLeg])).transpose();
//                 Rot = ROT(Manipulators[RJ_LeftLeg]->fk(angles[RJ_LeftLeg])).transpose();
// //                 BaseVector2ToolFrame()
//                 Rot2.block<3,3>(0,0) = Rot;
//                 Rot2.block<3,3>(3,3) = Rot;
//                 temp2 = Manipulators[RJ_RightLeg]->getCOMJacobian(angles[RJ_RightLeg]);
//                 temp2 = Rot2*temp2;
//                 temp.block<6,6>(0,6) = temp2;
//                 //make orientation of J12 = J11
//                 temp.block<3,6>(3,6) = temp.block<3,6>(3,0);

                //get kinematic portion of total Jacobian
//                 temp.block<6,6>(6,0) = Manipulators[RJ_LeftLeg]->getCOMJacobianWRTFootFrame(angles[RJ_LeftLeg], 
// TRANS(Manipulators[RJ_LeftLeg]->fk(angles[RJ_LeftLeg])) );/*Think if its ok to parse this in base frame coordinates*/
                
//                 temp.block<6,6>(6,0) = Manipulators[RJ_LeftLeg]->getJacobianWRTFootFrame(
//                                                                 angles[RJ_LeftLeg], 
//                                                                 Rot*TRANS( Manipulators[RJ_RightLeg]->fk(angles[RJ_RightLeg]) ) 
//                                                                 );
//                 Rot2 = Rot;
                
//                 temp.block<6,6>(6,6) = Rot2*Manipulators[RJ_RightLeg]->getJacobian(angles[RJ_RightLeg]);
// //                 temp.block<6,6>(6,6) = Manipulators[RJ_RightLeg]->getJacobian(angles[RJ_RightLeg]);
// // 
// //                 pseudoInverse(temp, iiJacob );
                
//                 compos = getCOM(angles).segment(0,3);
//                 eError.segment(0,3) = (LeftLeg2COM - BaseVector2ToolFrame(compos, angles[RJ_LeftLeg], RJ_LeftLeg));
// 
//                 OrientationCurr = RJQuaternion(Rot);
//                 OrientationCurr.normalize();
//                 
//                 OrientationCurr.vec() = -OrientationCurr.vec();//frame e, frame 1
//                 OrientationError = LeftLeg2COMOrientation*OrientationCurr;
// 
// //                 eError.segment(3,3) = ROT(Manipulators[RJ_LeftLeg]->fk(angles[RJ_LeftLeg]))*OrientationError.vec();                               
// 
//                 eError.segment(6,3) = LeftLeg2RightLeg - Rot*
//                                         (-TRANS(Manipulators[RJ_LeftLeg]->fk(angles[RJ_LeftLeg])) +TRANS(Manipulators[RJ_RightLeg]->fk(angles[RJ_RightLeg])));
// //                 eError.segment(6,3) = LeftLeg2RightLeg - TRANS(Manipulators[RJ_RightLeg]->fk(angles[RJ_RightLeg]));
// 
//                 OrientationCurr = RJQuaternion(Rot*ROT(Manipulators[RJ_RightLeg]->fk(angles[RJ_RightLeg])));         
//                 
//                 OrientationCurr.normalize();
//                 
//                 OrientationCurr.vec() = -OrientationCurr.vec();//frame e, frame 1
//                 OrientationError = LeftLeg2RightLegOrientation*OrientationCurr;
// 
// //                 eError.segment(9,3) = Rot*OrientationError.vec();

//                 RJVector12 anglesdeltas = iiJacob*eError;
// //                 angles[RJ_LeftLeg] += anglesdeltas.segment<6>(0);
// //                 angles[RJ_RightLeg] += anglesdeltas.segment<6>(6);
//                 angles[RJ_RightLeg] += Manipulators[RJ_RightLeg]->getInvJacobian(angles[RJ_RightLeg])*eError.segment(6,6);
                
                //retryin
//                 RJMatrixX J22 = Rot2*Manipulators[RJ_RightLeg]->getJacobian(angles[RJ_RightLeg]);
//                 RJMatrixX iJ22;
//                 pseudoInverse(J22, iJ22 );
//                 RJMatrixX J21 = Manipulators[RJ_LeftLeg]->getJacobianWRTFootFrame(
//                                                                 angles[RJ_LeftLeg], 
//                                                                 Rot*TRANS( Manipulators[RJ_RightLeg]->fk(angles[RJ_RightLeg]) ) 
//                                                                 );
//                 RJMatrixX J12 = Manipulators[RJ_RightLeg]->getCOMJacobian(angles[RJ_RightLeg]);
//                 RJMatrixX J11 = Manipulators[RJ_LeftLeg]->getCOMJacobianWRTFootFrame(getCOM(angles), angles[RJ_LeftLeg]);
//                 RJMatrixX iJ11;
//                 RJMatrixX iJ12;
//                 pseudoInverse(J11, iJ11 );
//                 pseudoInverse(J12, iJ12 );
//                 
//                 
//                 RJMatrixX iJ21;
//                 pseudoInverse(J21, iJ21 );
// //                 angles[RJ_RightLeg] += iJ22*eError.segment(6,6) + iJ12*eError.segment(0,6);
// //                 angles[RJ_LeftLeg] += iJ21*eError.segment(6,6)  + iJ11*eError.segment(0,6);
// //                 angles[RJ_RightLeg] += iJ12*eError.segment(0,6);
// //                 angles[RJ_LeftLeg] += iJ11*eError.segment(0,6);
//                 
//                 angles[RJ_RightLeg] += iJ22*eError.segment(6,6);
//                 angles[RJ_LeftLeg] += iJ11*eError.segment(0,6);
                
                //############ Refactored Version ######################
//                 struct update_thread_data leftData;
//                 struct update_thread_data rightData;
//                 
//                 leftData.thread_id=0;
//                 rightData.thread_id=1;
//                 leftData.angles = angles[RJ_LeftLeg];
//                 rightData.angles = angles[RJ_RightLeg];
//                 
//                 pthread_t threads[2];                
//                 if(pthread_create(&threads[i], NULL, Manipulators[RJ_LeftLeg]->updateThreaded,(void *)&leftData))
//                 {
//                         throw "Critical failure: Robot::CL_IK failed to create thread";
//                 }
// #ifdef RJ_THREADED          
//                 //TODO BUG this is actually slower, than non threaded, may be that on a dual core machine this is a waste
//     auto updateLeft = [&] () {
// 	Manipulators[RJ_LeftLeg]->update(angles[RJ_LeftLeg]);
// };
//                 std::thread t[2];                
// //                 t[0] =  std::thread(&RobotManipulator::update2,std::ref(Manipulators[RJ_LeftLeg]),angles[RJ_LeftLeg]);
//                 t[0] =  std::thread(updateLeft);
//                 Manipulators[RJ_RightLeg]->update(angles[RJ_RightLeg]);
//                 t[0].join();
// #else
                Manipulators[RJ_LeftLeg]->update(angles[RJ_LeftLeg]);
                Manipulators[RJ_RightLeg]->update(angles[RJ_RightLeg]);
// #endif
                

                
                
//                 RJMatrix4 fk_left = Manipulators[RJ_LeftLeg]->fk(angles[RJ_LeftLeg]);
//                 RJMatrix4 fk_right = Manipulators[RJ_RightLeg]->fk(angles[RJ_RightLeg]);
                RJMatrix4 fk_left = Manipulators[RJ_LeftLeg]->fk();
                RJMatrix4 fk_right = Manipulators[RJ_RightLeg]->fk();
                
//                 RJVector4 comvec = getCOM(angles);
                //the idea here is that since arm angles arent changing and legs have been updated, its more efficent to
                //use getCOM(bool) since it uses the precalculated COM's of the manipulators.
                RJVector4 comvec = getCOM(true);
                
                Rot = ROT(fk_left).transpose();
                Rot2.block<3,3>(0,0) = Rot;
                Rot2.block<3,3>(3,3) = Rot;
                        //calculate error vector
                compos = comvec.segment(0,3);
//                 eError.segment(0,3) = (LeftLeg2COM - BaseVector2ToolFrame(compos, angles[RJ_LeftLeg], RJ_LeftLeg));
                eError.segment(0,3) = (LeftLeg2COM - getManipulator(RJ_LeftLeg)->BaseVector2ToolFrame(compos));
                OrientationCurr = RJQuaternion(Rot.transpose());
                OrientationCurr.normalize();                
                OrientationCurr.vec() = -OrientationCurr.vec();//frame e, frame 1
                OrientationError = LeftLeg2COMOrientation*OrientationCurr;
                eError.segment(3,3) = ROT(fk_left)*OrientationError.vec();
                eError.segment(6,3) = LeftLeg2RightLeg - Rot*
                                        (-TRANS(fk_left) +TRANS(fk_right));

                OrientationCurr = RJQuaternion(Rot*ROT(fk_right));                         
                OrientationCurr.normalize();                
                OrientationCurr.vec() = -OrientationCurr.vec();//frame e, frame 1
                OrientationError = LeftLeg2RightLegOrientation*OrientationCurr;
                eError.segment(9,3) = Rot*OrientationError.vec();
                
                
                //Calculate Relevant Jacobians. To avoid interference, left leg tracks COM and Right leg tracks left leg.
                RJMatrixX J11 = Manipulators[RJ_LeftLeg]->getCOMJacobianWRTFootFrame(comvec);
//                 RJMatrixX J22 = Rot2*Manipulators[RJ_RightLeg]->getJacobian(angles[RJ_RightLeg]);
                RJMatrixX J22 = Rot2*Manipulators[RJ_RightLeg]->getJacobian();
                RJMatrixX iJ22, iJ11;
                
                pseudoInverse(J11, iJ11 );
                pseudoInverse(J22, iJ22 );
                
                angles[RJ_RightLeg] += iJ22*eError.segment(6,6);
                angles[RJ_LeftLeg] += iJ11*eError.segment(0,6);
                
                //######################################################
                
                //TODO make a proper completion check
                if(eError.norm() < epsError) break;
        }

//                         compos = getCOM(angles).segment(0,3);
//                 eError.segment(0,3) = (LeftLeg2COM - BaseVector2ToolFrame(compos, angles[RJ_LeftLeg], RJ_LeftLeg));
//         cout << "COM " << BaseVector2ToolFrame(getCOM(angles).segment(0,3), angles[RJ_LeftLeg], RJ_LeftLeg) << endl;
//         cout << "L2R " << 
//         ROT(Manipulators[RJ_LeftLeg]->fk(angles[RJ_LeftLeg])).transpose()*(-TRANS(Manipulators[RJ_LeftLeg]->fk(angles[RJ_LeftLeg])) +TRANS(Manipulators[RJ_RightLeg]->fk(angles[RJ_RightLeg])))
//         << endl;
//         cout << "L2R " << 
//         TRANS(Manipulators[RJ_RightLeg]->fk(angles[RJ_RightLeg]))
//         << endl;        
//         PI2minusPI(angles[RJ_RightLeg]);
//         PI2minusPI(angles[RJ_LeftLeg]);        
        for(it=Manipulators.begin(); it!=Manipulators.end(); it++)
        {
                PI2minusPI(angles[it->first]);
                it->second->setAngles(angles[it->first]);
        }
        
        return angles;
}

int Robot::getDOF()
{
                ManipMapType::iterator it;
                int ret = 0;
                for(it=Manipulators.begin(); it!=Manipulators.end(); it++)
                {
                        ret += it->second->getDOF();
                }
                return ret;
}

RJReal Robot::getMass(bool calculate)
/**     @brief calculates or returns total robot mass
*/
{
        if(calculate)
        {
                ManipMapType::iterator it;
                mass = 0;
                for(it=Manipulators.begin(); it!=Manipulators.end(); it++) mass += it->second->get_total_mass();
        }
        else return mass;
}
