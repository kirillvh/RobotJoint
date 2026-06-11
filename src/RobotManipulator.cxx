/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/
#include "RobotManipulator.h"
using namespace std;
using namespace Eigen;
using namespace RobotJoint;

RobotManipulator::RobotManipulator() : RobotManipulator_start(true)
/**     @brief Default constructor
*/
{
        _ManipName = RJ_unNamed;
        _DOF       = 0;
        _angles.resize(0);

	FK.resize(4,4);
	COM.resize(4);
	Jac.resize(6,_DOF);
	iJac.resize(_DOF,6);
	COMJac.resize(6,_DOF);
	iCOMJac.resize(_DOF,6);
        
        currFrictionIdJoint = -1;
	_gravity = GRAVITY;

}

RobotManipulator::~RobotManipulator()
/**     @brief Destructor: deletes JointLink objects referenced to by pointers in JointLinks vector
*/
{
        JointLinksType::iterator it;
        for(it=JointLinks.begin(); it!=JointLinks.end(); it++)
        {
                delete *it;
        }
}

RobotManipulator::RobotManipulator(RoboJointVars name) : RobotManipulator_start(true)
/**     @brief Constructor
*       @param[in] name Identity tag like 'RJ_LeftLeg' defined in RJointTypes.h
*/
{
        _ManipName = name;
        _DOF       = 0;
        _angles.resize(0);
	_gravity = GRAVITY;
}

RobotManipulator::RobotManipulator(RoboJointVars name, RJVectorX angles) : RobotManipulator_start(true)
/**     @brief Constructor
*       @param[in] name Identity tag like 'RJ_LeftLeg' defined in RJointTypes.h
*/
{
        _ManipName = name;
        _DOF       = 0;
        _angles    = angles;
	_gravity = GRAVITY;
}

// void* RobotManipulator::updateThreaded(void *threadarg)
// {
//       struct update_thread_data *my_data;
//       
//       my_data = (struct update_thread_data *) threadarg;
//       
//       update(my_data->angles);
//       
//       pthread_exit(NULL);
// }

void RobotManipulator::update2(const RJVectorX &q)
{
        update(q);
}

int RobotManipulator::update(const RJVectorX &q)
/**     @brief Computes forward kinematics, COM, Jacobian(kinematic&COM based) in efficent order with caching
        @param[in] q joint angles
*/
{
	_angles=q;
	//order is important because functions like getCOM and getJacobian can compute 
	//faster if forward kine results are stored in memory first
  	fk(_angles, Remember);
  	Jac = getJacobian(_angles, Remember);

	getCOM(_angles,0,true,true);
	COMJac = getCOMJacobian(_angles,true);
	COMJac_wrt2foot = getCOMJacobianWRTFootFrame(_angles,true);
        
        //update joint states (dx, ddx)
        for (int i=0; i<_DOF; i++)
        {
                update_state_x_poly(i, q(i), _sampling_period);
                qi(i)    = getState_x  (i);
                dqi(i)   = getState_dx (i);
                ddqi(i)  = getState_ddx(i);
        }
        if(RobotManipulator_start)
        { //update the state a total of 3 times so that the differentiators work correctly
                for (int i=0; i<_DOF; i++)
                {
                        update_state_x_poly(i, q(i), _sampling_period); //TODO actually this is no longer needed if the polynomial method is used
                        update_state_x_poly(i, q(i), _sampling_period);
                        qi(i)    = getState_x  (i);
                        dqi(i)   = getState_dx (i);
                        ddqi(i)  = getState_ddx(i);
                }
                RobotManipulator_start = false;
        }

// 	if(_DOF>=6)
// 	{
//                 pseudoInverse(Jac,iJac);
//                 pseudoInverse(COMJac,iCOMJac);
//                 pseudoInverse(COMJac_wrt2foot,iCOMJac_wrt2foot);
// 	}
	
  	return 0;
}

RJVector4 RobotManipulator::getCOM(const RJVectorX &angles, int from, bool use_cached, bool memorize)
/**     @brief returns [COM_position COM_weight]'  from a certain link to the last link. Results are in the base JointLinks coordinates
        @param[in] angles the joint angles to work with
        @param[in] from the link from which to start calculating, useful for getting partial COM
        @param[in] use_cached dont calculate COM, just recall memorized value
        @param[in] memorize memorize the result for future caching
*/
{
	RJVector4 ret = RJVector4::Zero();
	RJVectorX cangles(JointLinks.size());
	RJMatrix4 temp;
	RJMatrix4 T;
	RJVector3 top = RJVector3::Zero();
	RJVector3 top_precalc(3);
	RJReal bottom=0;
	int i=0;

	if(use_cached)
	{
		for(int i = 0, j = 0; i<JointLinks.size();i++ , j++)
		{
			if(JointLinks[i]->get_immobile())	
			{
				j--;
				cangles(i) = 0;
			}
			else	cangles(i) = angles(j);
		}
	}

	/*iterate over the JointLinks and perform
	sum(Xi * Mi)/sum(Mi)
	where Xi is the position of the COM of link i w.r.t the base frame and Mi is the mass of link i */
	for(int i = JointLinks.size()-1; i >= from; i--)
	{
		if(use_cached)
		{
			//First make a transformation matrix from the links frame to the links COM
			temp = RJMatrix4::Identity();
			T = RJMatrix4::Identity();
			// fill the homogenous matrix with the COM position
			temp.block<3,1>(0,3) = JointLinks[i]->get_pc(cangles(i)).head(3); 
			// and the joints rotation
			temp.block<3,3>(0,0) = JointLinks[i]->get_r(cangles(i)); 
			// accumulate the transformation as usual
			if(i>0)
			{//if i==0 then the value from set_eye(T) as above will be used
				T = FK_accumulated[i-1]*temp; 
			}
			else
			{
				T = temp;
			}

//		top_precalc(1) = T(1,4); // ### todo, get this to work
		top += T.block<3,1>(0,3)*JointLinks[i]->getMass();
		bottom += fabs(JointLinks[i]->getMass());
		}
		else
		{
		T = fk(angles,0,i,true);
		top += T.block<3,1>(0,3)*JointLinks[i]->getMass();
		//top += fk(angles,0,i,true).submatrix(1,3,4,4)*JointLinks[i]->getMass();
		bottom += fabs(JointLinks[i]->getMass());
		}

		if(memorize)
		{ //commit top and bottom to memory
			COM_accumulated[i]<<top(0)/bottom,top(1)/bottom,top(2)/bottom,bottom;
		}
	}
        if(bottom==0) ret.head(3) = RJVector3::Zero();
	else ret.head(3) = top/bottom; // COM position stored in first 3 values of the result
	ret(3)	      = bottom; //COM weight stored in the 4th value of the result

	return ret;
}

RJVector4 RobotManipulator::getCOM(int k)
/**     @brief overload version of getCom. Returns cached results
        @param[in] k link from which to start calculating
*/
{
	if(k>=0 && k<JointLinks.size()) return COM_accumulated[k];
	else cerr<<"Error: RobotManipulator::getCOM - index out of bounds \n";
}

RJMatrixX RobotManipulator::getCOMJacobianWRTFootFrame(const RJVectorX &angles, bool use_cached)
/**     @brief transforms the COM Jacobian into the foot frame
*/
{
	RJMatrixX jacCOM(6,_DOF),jacKine(6,_DOF),jacCOMwrtFoot(6,_DOF);
//         RJMatrix6X jacCOM(6,_DOF), jacKine(6,_DOF), jacCOMwrtFoot(6,_DOF);
	RJMatrixX temp, rot;
	RJVector3 P_foot2com, pos;
		

	if(use_cached)
	{//cached version
		jacCOM	= getCOMJacobian();
		jacKine	= getJacobian();	
		temp=fk();
		rot=ROT(temp).transpose(); //foot orientation wrt base
		pos=TRANS(temp);// foot position wrt base
		P_foot2com=rot*(getCOM().head(3)-pos); //Foot to COM vector in foot coordinates
	}
	else
	{//non cached version
		jacCOM	= getCOMJacobian(angles);
		jacKine	= getJacobian(angles);	
		temp=fk(angles);
		rot=temp.block<3,3>(0,0).transpose(); //foot orientation wrt base
		pos=temp.block<3,1>(0,3);// foot position wrt base
		P_foot2com=rot*(getCOM(angles).head(3)-pos); //Foot to COM vector in foot coordinates
	}

	for(int i = 0; i < _DOF; i++)
	{
		jacCOMwrtFoot.block<3,1>(0,i) = rot*(-jacKine.block<3,1>(0,i) + jacCOM.block<3,1>(0,i)) -
		(rot*jacCOM.block<3,1>(3,i)).head<3>().cross(P_foot2com);
		jacCOMwrtFoot.block<3,1>(3,i)	= rot*jacCOM.block<3,1>(3,i);
	}

	return jacCOMwrtFoot;
}


RJMatrixX RobotManipulator::getCOMJacobian(const RJVector4 &COM, const RJVectorX &angles, bool use_cached)
/**     @brief calculated the COM Jacobian - Uses a externally supplied COM position
        @param[in] COM Top 3 entries represent COM position WRT base. Last represents mass in Kg
        @param[in] angles joint angles to use
        @param[in] use_cached use cached forward kinematics and COM values or recompute them
*/
{        
        RJMatrixX jac(6,_DOF);
//         RJMatrix6X jac(6,_DOF); //this is actually slower...
        RJVector3 z = RJVector3::Zero();// rotation matrix
        RJVector3 p = RJVector3::Zero();//distance to the end effector
        RJVector3 pL = RJVector3::Zero();//distance to tip of the last link
        RJMatrix3 rotn = RJMatrix3::Identity();// rotation from link 0 to n
        RJVector4 temp;// temporary buffer for results from getCOM()
        RJVector4 COM_others;// combined COM of manipulators other than this one
        z(2) = 1;

        RJReal m_total; //total mass
        RJReal m_partial; //mass of partial COM    
        m_total = COM(3);//get_total_mass(); 
        
        if(use_cached)  temp = getCOM();
        else            temp = getCOM(angles);
//         
        COM_others(3) = COM(3) - temp(3);//find the mass of the other parts of the robot
        COM_others.head<3>()=(COM(3)*COM.head<3>()-temp(3)*temp.head<3>())/COM_others(3);//find the COM of the other parts of the robot

        for(int i=0,j=0; j < JointLinks.size(); i++,j++) 
        { // i in Jacobian space, j in JointLinks space
                if(JointLinks[j]->get_immobile())
                {
                        i--;
                }
                else
                {
                        if(!use_cached)
                        { //calculate forward kine as well.
                                if(j>0)
                                {
                                        rotn = rotn*ROT(fk(angles,j-1,j-1));
                                        pL = TRANS(fk(angles,0,j-1));
                                }
                                // rotation axis of the joint
                                z = rotn.col(2);
                                // COM position from link j to last link in the base frames coordinates
                                temp = getCOM(angles,j);
                                temp.head<3>() = (temp.head<3>()*temp(3) + COM_others.head<3>()*COM_others(3))/m_total;
                                // partial com position from the point of rotation of current link to COM
                                p = -pL + temp.head<3>();
                                m_partial = temp(3) ;//+ COM_others(3);
                        }
                        else
                        { // dont calculate forward kine nor COM positions, use the precalculated values
                                if(j > 0)
                                { // use default z and p values
                                        z = FK_accumulated[j-1].block<3,1>(0,2);
                                        //first find the partial COM while considering the mass of other manipulators
                                        temp.head<3>() = (COM_accumulated[j].head<3>()*COM_accumulated[j](3) + COM_others.head<3>()*COM_others(3))/m_total;
                                        p = -FK_accumulated[j-1].block<3,1>(0,3) +
                                        temp.head<3>();
                                }
                                m_partial = COM_accumulated[j](3) ;//+ COM_others(3);
                        }
                        
                        jac.block<3,1>(0,i) = (m_partial/m_total)*z.cross(p);
                        jac.block<3,1>(3,i) = z;
                }
        }

        return jac;
}


RJMatrixX RobotManipulator::getCOMJacobianWRTFootFrame(const RJVector4 &COM, const RJVectorX &angles)
/**     @brief transforms the COM Jacobian into the foot frame - this version uses a externally supplied COM position
*/
{
        RJMatrixX jacCOM(6,_DOF),jacKine(6,_DOF),jacCOMwrtFoot(6,_DOF);
//         RJMatrix6X jacCOM(6,_DOF), jacKine(6,_DOF), jacCOMwrtFoot(6,_DOF);
        RJMatrixX temp, rot;
        RJVector3 P_foot2com, pos;
                

                jacCOM  = getCOMJacobian(COM,angles);
                jacKine = getJacobian(angles);  
                temp=fk(angles);
                rot=temp.block<3,3>(0,0).transpose(); //foot orientation wrt base
                pos=temp.block<3,1>(0,3);// foot position wrt base
                P_foot2com=rot*(COM.head<3>()-pos); //Foot to COM vector in foot coordinates

        for(int i = 0; i < _DOF; i++)
        {
                jacCOMwrtFoot.block<3,1>(0,i) = rot*(-jacKine.block<3,1>(0,i) + jacCOM.block<3,1>(0,i)) -
                (rot*jacCOM.block<3,1>(3,i)).head<3>().cross(P_foot2com);
                jacCOMwrtFoot.block<3,1>(3,i)   = rot*jacCOM.block<3,1>(3,i);
        }

        return jacCOMwrtFoot;
}

RJMatrixX RobotManipulator::getCOMJacobianWRTFootFrame(const RJVector4 &COM)
/**     @brief transforms the COM Jacobian into the foot frame - this version uses a externally supplied COM position and cached data
*/
{
        RJMatrixX jacCOM(6,_DOF),jacKine(6,_DOF),jacCOMwrtFoot(6,_DOF);
//         RJMatrix6X jacCOM(6,_DOF), jacKine(6,_DOF), jacCOMwrtFoot(6,_DOF);
        RJMatrixX temp, rot;
        RJVector3 P_foot2com, pos;
                

                jacCOM  = getCOMJacobian(COM,RJVector6::Zero(),true);
                jacKine = getJacobian();  
                temp=fk();
                rot=temp.block<3,3>(0,0).transpose(); //foot orientation wrt base
                pos=temp.block<3,1>(0,3);// foot position wrt base
                P_foot2com=rot*(COM.head<3>()-pos); //Foot to COM vector in foot coordinates

        for(int i = 0; i < _DOF; i++)
        {
                jacCOMwrtFoot.block<3,1>(0,i) = rot*(-jacKine.block<3,1>(0,i) + jacCOM.block<3,1>(0,i)) -
                (rot*jacCOM.block<3,1>(3,i)).head<3>().cross(P_foot2com);
                jacCOMwrtFoot.block<3,1>(3,i)   = rot*jacCOM.block<3,1>(3,i);
        }

        return jacCOMwrtFoot;
}


RJMatrixX RobotManipulator::getJacobianWRTFootFrame()
{        
//         RJMatrixX  jacFoot = RJMatrixX::Zero(6,getDOF());
//         RJMatrixX jacKine = getJacobian();  
//         RJMatrixX temp=fk();
//         RJMatrix3 rot=ROT(temp).transpose(); //foot orientation wrt base
//         RJVector3 pos=TRANS(temp);// foot position wrt base
// 
//         for(int i = 0; i < _DOF; i++)
//         {
//                 jacFoot.block<3,1>(0,i) = -rot*(jacKine.block<3,1>(0,i)) -
//                 (rot*jacKine.block<3,1>(3,i)).head<3>().cross(-pos);
//                 jacFoot.block<3,1>(3,i)   = rot*jacKine.block<3,1>(3,i);
//         }
//         return jacFoot;
	return getJacobianWRTFootFrame(_angles);
}

RJMatrixX RobotManipulator::getJacobianWRTFootFrame(RJVectorX angles, RJVector3 offset)
{
        RJMatrixX jac(6,_DOF);
// 	RJMatrix6X jac(6,_DOF); //this is actually slower...
//         Eigen::Matrix<RJReal, 6, Eigen::Dynamic> jac = RJMatrixX(6,_DOF);
	RJVector3 z ;//= RJVector3::Identity();// rotation matrix
	z << 0, 0, 1;
	RJVector3 p = RJVector3::Zero();//distance to the end effector
	RJMatrix3 rotn = RJMatrix3::Identity();// rotation from link 0 to n
	RJMatrix3 rotT = ROT(fk(angles)).transpose(); // tool to base rotation
// 	z(2) = 1.0;

	for(int i = 0,j = 0; j < JointLinks.size(); i++,j++)// i in Jacobian space, j in JointLinks space
	{
		if(JointLinks[j]->get_immobile())
		{
			i--;
		}
		else
		{
		/*
		According to the DH convention the axis of rotation is always the Z axis
		therefore each joint transformation will be considered as a rotation 
		around the previous Z axis and some translation after that.
		As a result the 'j' index is usually lagging (i.e j-1) so as to retrive the
		joints rotation axis.
		*/
				if(j>0)
				{
                                        //TODO reverse order so that accumulation can happen for computational time reduction
					rotn = ROT(fk(angles,0,j-1));
	 			}	 

				/* This represents the rotation axis of the current link. since rotation
			 	matrixes are comprised of 3 unit vectors(one for each axis) in
				their columns and since the z axis always represents the 
				axis of rotation in the DH convention. The column associated with 
				the z axis shall be selected from the rotation matrix to 
				represent the rotation axis of the current link. */
				z = (rotT*rotn).col(2); 
				/* This represents the distance from the current link to the tool tip.
				Since the fk(angles,j,..) function returns results wrt link j's 
				cordinate frame, they are multiplied by rotn to bring them
				into the base's coordinates.*/
				p = rotT*(-TRANS(fk(angles,0,j-1))+offset);
			/*This represents the linear velocity portion of the Jacobian. 
			Linear_velocity= z X P , where X is the cross produce */
			jac.block<3,1>(0,i) = -z.cross(p); 
			//This represents the rotational velocity portion of the Jacobian 
			jac.block<3,1>(3,i) = z;
		}
	}
	return jac; 
}

RJMatrixX RobotManipulator::getInvJacobian(const RJVectorX &qi)
/**     @brief gets the inverse Jacobian
        @param[in] qi joint angles to use
*/
{
        RJMatrixX jac(6,_DOF), ijac;
        jac = getJacobian(qi);
        //JacobiSVD<RJMatrixX> svd(Jac, ComputeThinU | ComputeThinV);
        pseudoInverse(jac,ijac);
        return ijac;
}

RJMatrixX RobotManipulator::getInvJacobian()
/**     @brief gets the inverse Jacobian - cached
*/
{
        RJMatrixX jac(6,_DOF), ijac;
        jac = getJacobian();
        pseudoInverse(jac,ijac);
	return ijac;
}

RJMatrixX RobotManipulator::getInvJacobianWRTFootFrame()
/**     @brief gets the inverse Jacobian w.r.t foot frame - cached
*/
{
        RJMatrixX jac(6,_DOF), ijac;
        jac = getJacobianWRTFootFrame();
        pseudoInverse(jac,ijac);
	return ijac;
}

RJMatrixX RobotManipulator::getCOMJacobianWRTFootFrame()
/**     @brief gets the COM Jacobian wrt to the foot frame - cached
*/
{
	return COMJac_wrt2foot;
}


RJMatrixX RobotManipulator::getInvCOMJacobianWRTFootFrame(const RJVectorX &qi)
/**     @brief gets the Inverse COM Jacobian wrt to the foot frame - cached
        @param[in] qi joint angles to use
*/
{
        RJMatrixX jac(6,_DOF), ijac;
        jac = getCOMJacobianWRTFootFrame(qi);
        pseudoInverse(jac,ijac);
        return ijac;
}

RJMatrixX RobotManipulator::getInvCOMJacobianWRTFootFrame()
/**     @brief gets the Inverse COM Jacobian wrt to the foot frame - cached
*/
{
        RJMatrixX jac(6,_DOF), ijac;
        jac = getCOMJacobianWRTFootFrame();
        pseudoInverse(jac,ijac);
        return ijac;
}

RJMatrixX RobotManipulator::getInvCOMJacobian(const RJVectorX &qi)
/**     @brief gets the Inverse Jacobian
        @param[in] qi joint angles to use
*/
{
        RJMatrixX jac(6,_DOF), ijac;
        jac = getCOMJacobian(qi);
        pseudoInverse(jac,ijac);
        return ijac;
}

RJMatrixX RobotManipulator::getInvCOMJacobian()
/**     @brief gets the Inverse Jacobian - cached
*/
{
        RJMatrixX jac(6,_DOF), ijac;
        jac = getCOMJacobian();
        pseudoInverse(jac,ijac);
        return ijac;
}

RJMatrix4 RobotManipulator::fk(const RJVectorX &angles,int from, int to, bool use_com, bool memorize)
/**     @brief performs forward kinematics, COM kinematics can also be handled. Results are in the "from" JointLinks coordinates
        @param[in] angles the joint angles of the mobile JointLinks
        @param[in] from the link from which to start calculating, this link link is included in the calculations
        @param[in] to the link at which to stop calculating, this link is included in the calculations
        @param[in] use_com if this flag is set to true then the COM position of the "to" link in the "from" JointLinks coordinates is returned, otherwise the tip of the "to" link in the "from" JointLinks coordinates is returned
        @param[in] memorize memorize result for future caching

        The returned result is a 4x4 homogeneous transformation matrix containing the orientation and the position of the transformation between the "to" and "from" JointLinks.
        Immobile JointLinks are also handled by this function.
*/
{
        RJVectorX cangles(JointLinks.size());
	//check the input paramaters to make sure the "to" and "from" values are within bounds
	if(from<0) //### TODO : Handle this sort of stuff via exceptions
	{
		cerr<<"RobotManipulator::FK error: from < 0 , corrected to from=0"<<endl;
		from = 0;
	}
	if(to>JointLinks.size())
	{
		cerr<<"RobotManipulator::FK error: to > # of JointLinks , corrected to to=#JointLinks-1"<<endl;
		cerr<<"RobotManipulator::FK error: to > # of JointLinks , corrected to to=#JointLinks-1"<<endl;
		to=JointLinks.size()-1;
	}

	//initilize some working variables
	RJMatrix4 T = RJMatrix4::Identity();
// 	int i=0;

	//make JointLinks and angles the same size by filling the angles vecor with zeros 
	//this is needed because some of the JointLinks may be marked as immobile and therefore dont have
	//associated joint angles
	for(int i = 0, j = 0; i<JointLinks.size();i++, j++)
	{
		if(JointLinks[i]->get_immobile())	
		{
			j--;
			cangles(i) = 0;
		}
		else	cangles(i) = angles(j);
	}

	//iterate through the JointLinks
	for(int i = from; i <= to; i++)
	{ 
		if(i == to)
		{ //if the last link is encountered then there is a choice between getting the
		  //tip of the last link and getting its COM
	      		if(use_com)
			{ //the COM option
			RJMatrix4 temp = RJMatrix4::Identity();
			// fill the homogenous matrix with the COM position
			TRANS(temp) = JointLinks[to]->get_pc(cangles(to)).head(3); 
			// and the joints rotation
			ROT(temp) = JointLinks[to]->get_r(cangles(to)); 
			// accumulate the transformation as usual
			T = T*temp; 
      			}
			else
			{ // the kinematic option

				// accumulate the transformation as usual
				T = T*JointLinks[i]->get_t(cangles(i)); 
			}
    		}
		else
		{
			// accumulate the transformation as usual
    			T = T*JointLinks[i]->get_t(cangles(i)); 
    		}
		if(memorize)
		{ // append transformation matrix of base link to current link into memory
			FK_accumulated[i] = T;
		}
	}
	return T;
}

RJMatrix4 RobotManipulator::fk(const RJVectorX &angles, bool memorize)
/**     @brief overloaded version of the fk(Forward Kinematics) function. Gives tooltip transformation wrt to base.
        @param[in] angles joint angles to use
        @param[in] memorize memorize result for future caching
*/
{
	assert(JointLinks.size()>0);
	FK = fk(angles,0,JointLinks.size()-1,false,memorize);
	return FK;
}

RJMatrix4 RobotManipulator::fk(int from, int to)
/**     @brief overloaded version of the fk(Forward Kinematics) function. Uses the angles stored with the update() function.
        @param[in] from Compute forward kinematics starting at this joint index
        @param[in] to Compute forward kinematics ending at this joint index
*/
{
	return fk(_angles,from,to);
}

RJMatrix4 RobotManipulator::fk()
/**     @brief overloaded version of the fk(Forward Kinematics) function. Returns precalculated fk values that were stored with update() function.

*/  
{
	return FK;
}

RJReal RobotManipulator::get_total_mass(int k)
/**     @brief gets the total mass of the JointLinks from link k
        @param[in] k compute mass starting at joint index k until the last joint in the chain
*/
{
	RJReal bottom=0;
	for(int i=k; i<JointLinks.size(); i++)
	{
                bottom+=fabs(JointLinks[i]->getMass());
	}
	return bottom;
}

RJMatrixX RobotManipulator::getJacobian(const RJVectorX &angles, bool use_cached)
/**     @brief Calculates the kinematic Jacobian of the manipulator
        @param[in] angles joint angles to use
        @param[in] use_cached use cached forward kinematics or recompute forward kinematics
*/
{
        RJMatrixX jac(6,_DOF);
// 	RJMatrix6X jac(6,_DOF); //this is actually slower...
//         Eigen::Matrix<RJReal, 6, Eigen::Dynamic> jac = RJMatrixX(6,_DOF);
	RJVector3 z ;//= RJVector3::Identity();// rotation matrix
	z << 0, 0, 1;
	RJVector3 p = RJVector3::Zero();//distance to the end effector
	RJMatrix3 rotn = RJMatrix3::Identity();// rotation from link 0 to n
// 	z(2) = 1.0;

	for(int i = 0,j = 0; j < JointLinks.size(); i++,j++)// i in Jacobian space, j in JointLinks space
	{
		if(JointLinks[j]->get_immobile())
		{
			i--;
		}
		else
		{
		/*
		According to the DH convention the axis of rotation is always the Z axis
		therefore each joint transformation will be considered as a rotation 
		around the previous Z axis and some translation after that.
		As a result the 'j' index is usually lagging (i.e j-1) so as to retrive the
		joints rotation axis.
		*/
			if(!use_cached)
			{
				if(j>0)
				{
					// accumulates the rotation link by link for computational efficency
					rotn = rotn*ROT(fk(angles,j-1,j-1));
	 			}	 

				/* This represents the rotation axis of the current link. since rotation
			 	matrixes are comprised of 3 unit vectors(one for each axis) in
				their columns and since the z axis always represents the 
				axis of rotation in the DH convention. The column associated with 
				the z axis shall be selected from the rotation matrix to 
				represent the rotation axis of the current link. */
				z = rotn.col(2); 
				/* This represents the distance from the current link to the tool tip.
				Since the fk(angles,j,..) function returns results wrt link j's 
				cordinate frame, they are multiplied by rotn to bring them
				into the base's coordinates.*/
				p = rotn*TRANS(fk(angles,j,JointLinks.size()-1));
			}
			else
			{
				if(j>0)
				{ /* Usually immobile links are used to move the first links orientation and
				  position away from the base link, otherwise the joint will rotate around the 
				  base links Z axis */
					z = FK_accumulated[j-1].block<3,1>(0,2); //TODO possible BUG should be .block<3,1>(0,3)?
					p = TRANS(FK) - TRANS(FK_accumulated[j-1]);
				}
			}
			/*This represents the linear velocity portion of the Jacobian. 
			Linear_velocity= z X P , where X is the cross produce */
			jac.block<3,1>(0,i) = z.cross(p); 
			//This represents the rotational velocity portion of the Jacobian 
			jac.block<3,1>(3,i) = z;
		}
	}
	return jac; 
}
  

RJMatrixX RobotManipulator::getJacobian()
/**     @brief returns precalculated kinematic Jacobian as calculated by the update() function
*/
{ 
        return Jac;
}

RJMatrixX RobotManipulator::getCOMJacobian(const RJVectorX &angles, bool use_cached)
/**     @brief calculated the COM Jacobian
        @param[in] angles joint angles to use
        @param[in] use_cached use cached forward kinematics and COM values or recompute them
*/
{        
	RJMatrixX jac(6,_DOF);
//         RJMatrix6X jac(6,_DOF); //this is actually slower...
	RJVector3 z = RJVector3::Zero();// rotation matrix
	RJVector3 p = RJVector3::Zero();//distance to the end effector
	RJVector3 pL = RJVector3::Zero();//distance to tip of the last link
	RJMatrix3 rotn = RJMatrix3::Identity();// rotation from link 0 to n
	RJVector4 temp;// temporary buffer for results from getCOM()
	z(2) = 1;

	RJReal m_total; //total mass
	RJReal m_partial; //mass of partial COM    
	m_total = get_total_mass(); 

	for(int i=0,j=0; j < JointLinks.size(); i++,j++) 
	{ // i in Jacobian space, j in JointLinks space
		if(JointLinks[j]->get_immobile())
		{
			i--;
		}
		else
		{
			if(!use_cached)
			{ //calculate forward kine as well.
				if(j>0)
				{
					rotn = rotn*ROT(fk(angles,j-1,j-1));
					pL = TRANS(fk(angles,0,j-1));
				}
				// rotation axis of the joint
				z = rotn.col(2);
				// COM position from link j to last link in the base frames coordinates
				temp = getCOM(angles,j);
				// partial com position from the point of rotation of current link to COM
				p = -pL+temp.head(3);
				m_partial = temp(3);
			}
			else
			{ // dont calculate forward kine nor COM positions, use the precalculated values
				if(j > 0)
				{ // use defaultz and p values
					z = FK_accumulated[j-1].block<3,1>(0,2);
					p = -FK_accumulated[j-1].block<3,1>(0,3) +
					COM_accumulated[j].head(3);
				}
				m_partial = COM_accumulated[j](3);
			}
			jac.block<3,1>(0,i) = (m_partial/m_total)*z.cross(p);
			jac.block<3,1>(3,i) = z;
		}
	}

	return jac;
}

RJMatrixX RobotManipulator::getCOMJacobian()
/**     @brief returns the cached version of the COM Jacobian
*/
{
	return COMJac;
}

RoboJointVars RobotManipulator::getName(){
  return _ManipName;
}

int RobotManipulator::setAngles(const RJVectorX &qi)
/**     @brief sets _angles
        @param[in] qi joint angles to use
*/
{
	_angles=qi;
}

int RobotManipulator::addJoint(JointLink* newlink)
/**     @brief adds a joint to the chain
        @param[in] newlink Pointer to a JointLink
*/
{
	RJMatrix4 T = RJMatrix4::Zero();
	RJVector4 C = RJVector4::Zero();
        JointLinks.push_back(newlink);
	FK_accumulated.push_back(T);
	COM_accumulated.push_back(C);

        if(!newlink->get_immobile())
        {
                _DOF++;
                _angles.resize(_DOF);

                Jac.resize(6,_DOF);
                iJac.resize(_DOF,6);
                COMJac.resize(6,_DOF);
                iCOMJac.resize(_DOF,6);
		LimitedMotorForce.resize(_DOF);
		LimitedMotorForce = RJVectorX::Zero(_DOF);
                //TODO: handle compunding of immobile links in a special way
                JointControllers.push_back(Controller(newlink->getInertia()(2,2),_sampling_period));
                addState(); // add a state to track joint position, velocity & acceleration
                qi.resize  (  qi.size() + 1);
                dqi.resize ( dqi.size() + 1);
                ddqi.resize(ddqi.size() + 1);
                prevFrictionIDTorque.resize(prevFrictionIDTorque.size() + 1);
        }	
}

int RobotManipulator::getDOF()
/**     @brief returns the number of degrees of freedom
*/
{
        return _DOF;
}

RJVector3 RobotManipulator::BaseVector2ToolFrame(const RJVector3 &v)
/**     @brief transforms a given vector from the base to the tool tip frame - cached version
        @param v vector to transform
*/
{
	RJVector3 ret;
	RJMatrix4 tool_frame;
	ret = v;
	tool_frame = fk();
	ret.head(3) = ROT(tool_frame).transpose()*(v.head(3) - TRANS(tool_frame));
	return ret;
}

RJVector3 RobotManipulator::BaseVector2ToolFrame(const RJVector3 &v, const RJVectorX &angles)
/**     @brief transforms a given vector from the base to the tool tip frame - non cached version
        @param v vector to transform
        @param angles joint angles to use
*/
{
	RJVector3 ret;
	RJMatrix4 tool_frame;
	ret = v;
	tool_frame = fk(angles);
	ret.head(3) = ROT(tool_frame).transpose()*(v.head(3) - TRANS(tool_frame));
	return ret;
}

RJVectorX RobotManipulator::CLIK_IK(const RJVector3 &XRef, const RJQuaternion &OrientationRef, IKMode mode, int max_runs, RJReal error_eps)
/**     @brief Inverse kinematics using the Closed Loop Inverse Kinematics method.
        @param XRef Reference cartesian position
        @param OrientationRef Reference Orientation(unit quaternion)
        @param COMIK Converge on COM or tooltip position?
        @param max_runs Maximum number iterations before algorithim gives up
        @param error_eps Sum of error on all axes at which algorithim concludes that current result is good enough
        Reference: "THE ROLE OF EULER PARAMETERS IN ROBOT CONTROL" by Fabrizio Caccavale, Bruno Siciliano, and Luigi Villani
        Asian Journal of Control, Vol. 1, No. 1, pp. 25-34, March 1999
*/
{
        RJVectorX angles = _angles; //TODO think about making this a param so that the old result can be imported to improved in a future iteration
        RJVector3 x;
        RJVectorX error = RJVectorX::Zero(6);
        RJMatrixX ijac(6,_DOF);
        RJQuaternion OrientationCurr;
        RJQuaternion OrientationError;
        int runs = 0;
        double kTheta = 1.0;
        double kX = 1.0;
        
//         OrientationRef.normalize();
        
        do
        {
                if(mode == IK_Kinematic)
                {
                        ijac = getInvJacobian(angles);
//TODO : read "The Role of Euler Paramaters in Robot Control"
                       x = TRANS(fk(angles));
                }
                else if(mode == IK_COM)
                {
                        ijac = getInvCOMJacobian(angles);
                        x = getCOM(angles).head<3>();
                }
                else if(mode == IK_COMwrtFoot)
                {
                        ijac = getInvCOMJacobianWRTFootFrame(angles);
                        RJVector3 c = getCOM(angles).head<3>();
                        x = BaseVector2ToolFrame(c, angles);
                }
//                 else if(mode == IK_COMwrtFootExt)
//                 {
//                         RJVector3 c = getCOM(angles).head(3);
//                         x = BaseVector2ToolFrame(c, angles);
//                         max_runs = 1;
//                 }
                
                

//                 dx = XRef - x;
//                 x = TRANS(fk(angles));
//                error = RJVectorX::Zero(6);
                error.head<3>() = kX*(XRef - x);
                OrientationCurr = RJQuaternion(ROT(fk(angles)));
                OrientationCurr.normalize();
                OrientationCurr.vec() = -OrientationCurr.vec();//frame e, frame 1
//                 OrientationCurr.conjugate();
//                 TODO confirm that quaternion multiplication is equivelent to composition
              OrientationError = OrientationRef*OrientationCurr;
//             error.segment(3,3) = kTheta*ROT(fk(angles))*OrientationError.vec();
                 error.segment(3,3) = RJVector3::Zero(); //HACK: testing WRT2FootFrame stuff
             //TODO apply a sort of binnary search on the kTheta, kX to ensure convergence. i.e if result leads to divergence then half the error signal
                angles += ijac*error;

                runs++;
        //} while( (runs<max_runs) && (dx(1) < error_tol)); //TODO HACK dx(1)
        } while( (runs<max_runs) ); //TODO try and check if target is in the reachable region
        
        return angles;
}


RJVectorX RobotManipulator::RNE(const RJVectorX &cq, const RJVectorX &cdq, const RJVectorX & cddq, const RJVector6 &Fextern)
/**
        @brief Recusive Newton-Euler method.
        EXPERIMENTAL FUNCTION
        Ref: "Introduction to Robotics" by John Craig
        
        TODO make this "const RJVectorX &q", need to change some Link code for it though
*/
{
        int i;
        /*
        w_i     -       angular velocity 
        dw_i    -       angular acceleration of frame i
        ac_i    -       linear acceleration of link i's COM
        ae_i    -       linear acceleration of links i's end point
        */
        RJVectorX ret = RJVectorX::Zero(_DOF);
        vector<RJVector3, Eigen::aligned_allocator<RJVector3> > w, dw, alpha, ac, ae, f, tau; 
        for(int i =0; i < JointLinks.size() + 1; i++)
        {//initilizing arrays
                alpha.push_back(RJVector3::Zero());
                w.push_back(RJVector3::Zero());
                dw.push_back(RJVector3::Zero());
                ae.push_back(RJVector3::Zero());
                ac.push_back(RJVector3::Zero());
                f.push_back(RJVector3::Zero());
                tau.push_back(RJVector3::Zero());
        }

        //take imobile links into account by constructing a internal representation of the joint state
        RJVectorX q(JointLinks.size());
        RJVectorX dq(JointLinks.size());
        RJVectorX ddq(JointLinks.size());
        
        for(int i = 0, j = 0; i<JointLinks.size();i++ , j++)
        {// i is the joint index, j is the inxed for q, dq, ddq
                if(JointLinks[i]->get_immobile())       
                {
                        j--;
                        q(i) = 0;
                        dq(i) = 0;
                        ddq(i) = 0;
                }
                else
                {
                        q(i) = cq(j);
                        dq(i) = cdq(j);
                        ddq(i) = cddq(j);
                }
        }
        
        RJVector3 z0, F, N;
        z0 << 0,0,1; // axis of rotation in the DH convention is the z axis
        RJMatrixX Rt;
         ae[0] << 0,0,-_gravity;
//          ae[0] << 0,-10,0;
//        ae[0] << 0,0,0;
        for(int i = 0; i < JointLinks.size(); i++)
        {
                Rt = ROT(JointLinks[i]->get_t(q(i))).transpose();
                if(JointLinks[i]->get_joint_type() == Revolute)
                {
                        w[i+1]  = Rt*w[i]  + z0*dq(i);
                        dw[i+1] = Rt*dw[i] + z0*ddq(i) + ((Rt*w[i]).head<3>()).cross(z0*dq(i));
                        if(i==0)
                        {
                                ae[i+1] = Rt*ae[i];
                        }
                        else
                        {
                                ae[i+1] = Rt*(dw[i].cross(JointLinks[i-1]->get_p().head<3>())
                                + w[i].cross(w[i].cross(JointLinks[i-1]->get_p().head<3>())) 
                                + ae[i]);
                        }

                }
                else if(JointLinks[i]->get_joint_type() == Prismatic)
                {
                        w[i+1] = Rt*w[i-1];
                        dw[i+1] = Rt*dw[i-1];                        
                        ae[i+1] = Rt*(dw[i].cross(JointLinks[i-1]->get_p().head<3>())
                        + w[i].cross(w[i].cross(JointLinks[i-1]->get_p().head<3>())) 
                        + ae[i])+2.0*w[i+1].cross(dq(i)*z0.head<3>())
                        + ddq(i)*z0;
                }
                ac[i+1] = dw[i+1].cross(JointLinks[i]->get_pc().head<3>()) 
                + w[i+1].cross(w[i+1].cross(JointLinks[i]->get_pc().head<3>())) + ae[i+1];
                
// cout<<"i="<<i<<"\n";
// cout<<"w[i+1]="<<w[i+1]<<"\n";
// cout<<"dw[i+1]="<<dw[i+1]<<"\n";
// cout<<"ae[i+1]="<<ae[i+1]<<"\n";
// cout<<"ac[i+1]="<<ac[i+1]<<"\n";
        }

        for(int i = JointLinks.size(), j = 0; i >= 1; i--)
        {
                RJMatrix3 R = ROT(JointLinks[i-1]->get_t(q(i-1)));
//                 F = JointLinks[i-1]->getMass()*ac[i];
//                 N = JointLinks[i-1]->getInertia()*dw[i] + w[i].cross(JointLinks[i-1]->getInertia()*w[i]);
                //the line below differs from Craig because RobotJoint uses transformation matrices that have the rotational element
                //at the begining of the link instead of at the end
                F = R*(JointLinks[i-1]->getMass()*ac[i]);
                N = R*(JointLinks[i-1]->getInertia()*dw[i] + w[i].cross(JointLinks[i-1]->getInertia()*w[i]));
//                 F = JointLinks[i-1]->getMass()*R*ac[i];
//                 N = JointLinks[i-1]->getInertia()*R*dw[i] + (R*w[i]).head<3>().cross(JointLinks[i-1]->getInertia()*(R*w[i]).head<3>());
                if(i == JointLinks.size())
                {
                        f[i] = F + Fextern.head<3>();
                        
                        tau[i] = N
                        + JointLinks[i-1]->get_pc(q(i-1)).head<3>().cross(F)
                        + Fextern.tail<3>();
                } 
                else 
                {
                        f[i] = R*f[i+1] + F;
                        tau[i] = N + R*tau[i+1]
                        + JointLinks[i-1]->get_pc(q(i-1)).head<3>().cross(F)
                        + JointLinks[i-1]->get_p(q(i-1)).head<3>().cross((R*f[i+1]).head<3>());
//                         tau[i] = N + R*tau[i+1]
//                         + JointLinks[i-1]->get_pc().head<3>().cross(F)
//                         + JointLinks[i-1]->get_p().head<3>().cross((f[i]).head<3>());
                }
                if(!JointLinks[i-1]->get_immobile())
                {
                        if(JointLinks[i-1]->get_joint_type() == Revolute)
                        {
//                                 RJVectorX temp = (R.transpose()*z0).transpose()*tau[i];
//                                 ret(_DOF- j - 1) = temp(0);
                                ret(_DOF- j - 1) = tau[i](2);
//                                 ret(j) = tau[i](2);
                                j++;
                        }
                        else if(JointLinks[i-1]->get_joint_type() == Prismatic)
                        {
                                ret(j++) = f[i](2);
                        }
                }
// cout<<"i = "<<i<<endl;
// cout<<"f[i]="<<f[i]<<endl;
// cout<<"tau[i]="<<tau[i]<<endl;
        }

return ret;
}

RJVectorX RobotManipulator::GravityTorque(const RJVectorX &q)
/**     @brief returns torque due to gravity via RNE method
        @param[in] q joint angles
        @returns Gravity compensated joint torques
*/
{
        RJVectorX dq = RJVectorX::Zero(_DOF);
        RJVectorX ddq = RJVectorX::Zero(_DOF);
        return -RNE(q, dq, ddq);
}

RJMatrixX RobotManipulator::InertiaMatrix(const RJVectorX &q)
/**     @brief returns the joint space inertia matrix
        @param[in] q joint angles
        @returns DOF x DOF Matrix representing the "I" in tau = I * ddq
*/
{
        RJVectorX dq = RJVectorX::Zero(_DOF);
	RJVectorX ddq = RJVectorX::Zero(_DOF);
	RJMatrixX inertia = RJMatrixX::Zero(_DOF,_DOF);
	RJReal temp = _gravity;
	_gravity = 0;

	for(int i = 0; i < _DOF; i++)
	{
		ddq = RJVectorX::Zero(_DOF);
		ddq(i) = 1.0;
		inertia.row(i) = RNE(q,dq,ddq);
	}
	_gravity = temp;
        return inertia;
}

RJVectorX RobotManipulator::getFrictionTorque()
/**     @brief wrapper for getFrictionTorque(const RJVectorX &dq) using internal velocity value
*/
{
        return getFrictionTorque(dqi);
}

RJVectorX RobotManipulator::getFrictionTorque(const RJVectorX &dq)
/**     @brief wrapper for JointLinks staticFriction function
*/
{
        RJVectorX frictionTorque(_DOF);
        for(int i = 0; i < JointLinks.size(); i++) frictionTorque(i) = JointLinks[i]->staticFriction(dq(i));
        return frictionTorque;
}

RJVectorX RobotManipulator::frictionIdentification(int jointIndex, RJVector2 &velVsfrictionTau, bool reset)
/**
        under construction
        returns [torquesForCurrentTimeStep; prevTimeStepsFrictionTorque, prevTimeStepsVelocity]
*/
{
        RJVectorX ret(_DOF);
        RJVector6 nil = RJVector6::Zero();
        RJVector2 nil2 = RJVector2::Zero();
        RJVectorX dynamic_torques = RJVectorX::Zero(2);
        dynamic_torques = RNE(qi,dqi,ddqi,nil);
//         RJVectorX dynamic_torques = RNE(qi,dqi,nil2,nil);
        RJVectorX position_control_torques(_DOF);
        #define FID_P_GAIN 50.0
        #define FID_D_GAIN 5.0
        #define FID_t_initialization_end PI
        #define FID_t_id_end PI*4.0
        #define FID_t_reset_end 15.0
        
        if (reset || (currFrictionIdJoint != jointIndex)) //FrictionIDstart = false;
        
//         if(!FrictionIDstart)
        {
//                 FrictionIDstart = true;
                frictionIDTime = 0;
                for(int i=0; i<_DOF; i++)
                {// prepare the position reference polynomials to make all joints go to the zero position
                        poly5thMakeCoefficents("0"+i, 0, FID_t_initialization_end, qi(i), 0, 0, 0, 0, 0);
                }
        }
        currFrictionIdJoint = jointIndex;
        
        
        //First generate the desired torques
//         cout<<"FIDTime "<<frictionIDTime<<endl;
        //make the joints go to the zero position
        if(Samples2Sec(frictionIDTime) < FID_t_initialization_end)
        {
                for(int i=0; i<_DOF; i++) position_control_torques(i) = JointControllers[i].PD_feedback2(FID_P_GAIN, FID_D_GAIN, qi(i), poly5th("0"+i, Samples2Sec(frictionIDTime)), dqi(i));
// for(int i=0; i<_DOF; i++) position_control_torques(i) = JointControllers[i].PD_feedback2(FID_P_GAIN, FID_D_GAIN, qi(i), 0, dqi(i));
        }
        
        //if time>5.0 then the target joint is made to follow a sine pattern
        else if((Samples2Sec(frictionIDTime) > FID_t_initialization_end) && (Samples2Sec(frictionIDTime) < FID_t_id_end))
        {
                //by default all joints are made to go to the zero position        
                for(int i=0; i<_DOF; i++) position_control_torques(i) = JointControllers[i].PD_feedback2(FID_P_GAIN, FID_D_GAIN, qi(i), 0, dqi(i));
                
                position_control_torques(jointIndex) = JointControllers[jointIndex].PD_feedback2(FID_P_GAIN, FID_D_GAIN, qi(jointIndex), 0.1*sin(2.0*Samples2Sec(frictionIDTime)), dqi(jointIndex));
                
                if(Samples2Sec(frictionIDTime+1) >= FID_t_id_end)
                {
                        for(int i=0; i<_DOF; i++)
                        {
                                poly5thMakeCoefficents("0"+i, FID_t_id_end, FID_t_reset_end, qi(i), 0, 0, 0, 0, 0);
                        }
                }
        }
        // finally go back to the zero position
        else if(Samples2Sec(frictionIDTime) > FID_t_id_end)
        {
                for(int i=0; i<_DOF; i++) position_control_torques(i) = JointControllers[i].PD_feedback2(FID_P_GAIN, FID_D_GAIN, qi(i), poly5th("0"+i, Samples2Sec(frictionIDTime)), dqi(i));
        }
        
//         position_control_torques = RJVectorX::Zero(2);
        ret = position_control_torques - dynamic_torques;
        
        //next the friction torque is calculated
        velVsfrictionTau(1) = JointControllers[jointIndex].Disturbance_Observer3(prevFrictionIDTorque(jointIndex), ddqi(jointIndex), 0.4*_sampling_frequency);
        //and the velocity is recorded
        velVsfrictionTau(0) = dqi(jointIndex);
        
                
//         prevFrictionIDTorque = ret;
        prevFrictionIDTorque = position_control_torques;
        
// if(Samples2Sec(frictionIDTime)<2.0*PI)       
        frictionIDTime++;
        
        return ret;
        
}

void RobotManipulator::frictionIdentification_holdData(const RJVector2 velVsfrictionTau , RJReal reject_limit)
/**     @brief classifies and stores friction data to be identified later
        @param[in] velVsfrictionTau joint velocity at first index, measured friction torque at second
        @param[in] reject_limit threshold at which too small a velocity values are ignored
*/
{
        if(velVsfrictionTau(0) > reject_limit)
        {
                positive_friction_data.push_back(velVsfrictionTau);
        }
        else if(velVsfrictionTau(0) < -reject_limit)
        {
                negative_friction_data.push_back(velVsfrictionTau);
        }
}

RJVector4 RobotManipulator::frictionIdentification_calculate()
/**     @brief computes static friction paramaters  **Computationally Intensive!!**        
*/
{
        RJVector4 ret;
        RJVector2 coeffs_positive, coeffs_negative;
        
        RJMatrixX A_p(positive_friction_data.size(),2);
        RJMatrixX A_n(negative_friction_data.size(),2);
        
        RJVectorX b_p(positive_friction_data.size());
        RJVectorX b_n(negative_friction_data.size());
        
        for(int i = 0 ;i<positive_friction_data.size();i++)
        {
                A_p(i,0) = 1.0;
                A_p(i,1) = positive_friction_data[i](0);
                b_p(i) = positive_friction_data[i](1);
        }
        
        for(int i = 0 ;i<negative_friction_data.size();i++)
        {
                A_n(i,0) = 1.0;
                A_n(i,1) = negative_friction_data[i](0);
                b_n(i) = negative_friction_data[i](1);
        }
        
        RJMatrixX iA_p, iA_n;
        pseudoInverseNonSquare(A_p, iA_p);
        pseudoInverseNonSquare(A_n, iA_n);
        
        coeffs_positive = iA_p*b_p;
        coeffs_negative = iA_n*b_n;
        
        ret(0) = coeffs_positive(0);
        ret(1) = coeffs_positive(1);
        ret(2) = coeffs_negative(0);
        ret(3) = coeffs_negative(1);
        
        return ret;
}

RJVectorX RobotManipulator::GRFC_TF(const RJVectorX &Torques, const RJVectorX &TipForceSens, const RJMatrixX &MaxWrench)
/**     @brief Ground reaction force constrained torque filter
        @param Torques Desired joint torques
        @param TipForceSens Force sensor data from the tool tip
        @param MaxWrench Maximum allowable force wrench at the tool tip
        @returns The limited joint torques which prevent the resultant force from becoming larger than MaxWrench
    
                        EXPERIMENTAL!!!
    
        Implements the algorithim described in 
        "Maintaining floor-foot contact of a biped robot by force constraint position control" by Kirill Van Heerden
*/
{
        
}

RJVectorX RobotManipulator::GRFC_TF(const RJVectorX &Torques, const RJVectorX &TipForceSens, const RJMatrixX &MaxWrench, const Vector6b &axes_to_use,  const VectorXb &joints_to_use, const double &fil_freq)
/** @brief Experimental: Ground reaction force constrained torque filter
    @param Torques Desired joint torques
    @param TipForceSens Force sensor data from the tool tip
    @param MaxWrench Maximum allowable force wrench at the tool tip
    @param axes_to_use Decides which axes to enable. Cant be smaller than joints_to_use
    @param joints_to_use Decides which joints to enable. Must be greater or equal to axes_to_use.
    @param fil_freq cutoff frequency of the workspace observer.
    @returns The limited joint torques which prevent the resultant force from becoming larger than MaxWrench
    
                        EXPERIMENTAL!!!
    
    Implements the algorithim described in 
    "Maintaining floor-foot contact of a biped robot by force constraint position control" by Kirill Van Heerden
*/
{
//         RJVectorX LimitedMotorForce = RJVectorX::Zero(6);
        
/*        RJVectorX MotorForce(6);
        RJVectorX FutureGRF(6);
        RJVectorX LimitedGRF = RJVectorX::Zero(6);
        RJVectorX LimitedTorque(6);
        RJVectorX DisturbanceForce(6);     */   
        
        char j;
        int  i, k, m, n;
        
        int nrows = 0;
        int ncols = 0;
        //find the required dimensions for internal Jacobian
        for(i = 0; i<axes_to_use.size()  ; i++) if(axes_to_use(i))   nrows++;
        for(i = 0; i<joints_to_use.size(); i++) if(joints_to_use(i)) ncols++;
        
        RJMatrixX Jac_internal = RJMatrixX::Zero(nrows,ncols);
        RJMatrixX iJac_internal = RJMatrixX::Zero(nrows,ncols);
        RJVectorX LimitedMotorForce_internal(nrows);
//         RJVectorX LimitedTorque_internal(nrows);
        RJVectorX Torques_internal(nrows);
        
        RJVectorX MotorForce(nrows);
        RJVectorX FutureGRF(nrows);
        RJVectorX LimitedGRF = RJVectorX::Zero(nrows);
        RJVectorX LimitedTorque(Torques.size());
        RJVectorX LimitedTorque_internal(nrows);
        RJVectorX DisturbanceForce = RJVectorX::Zero(nrows);
        RJVectorX TipForceSens_internal(nrows);
        RJMatrixX MaxWrench_internal(nrows,2);
        
//         RJVectorX MotorForce_internal(6);
//         RJVectorX FutureGRF_internal(6);
//         RJVectorX LimitedGRF_internal = RJVectorX::Zero(6);
//         RJVectorX LimitedTorque_internal(6);
//         RJVectorX DisturbanceForce_internal(6);   
        
        
        //First Construct inernal quantaties
        
//         cout << "Jac_internal" << endl << Jac_internal << endl;
        
        //Construct the internal Jacobian
        m = 0;// Jac_interal row index
        n = 0;// Jac_interal col index
        for(i = 0; i < joints_to_use.size(); i++)
        {
                if(joints_to_use(i))
                {
                        m = 0;
                        for(j = 0; j < axes_to_use.size(); j++)
                        {
                                if(axes_to_use(j))
                                {
                                        Jac_internal(m,n) = Jac(j, i); // can transpose this later for better efficency
                                        m++;
                                }
                        }
                        n++;
                }
        }
        
        pseudoInverse(Jac_internal, iJac_internal);
        //Internal torques and force sensor data
        m = 0;
        for(i = 0; i<joints_to_use.size()  ; i++) 
        {
                if(joints_to_use(i))
                {
                        Torques_internal(m)      = Torques(i);
                        m++;
                }
        }
        
        //internal force limits
        m = 0;
        for(i = 0; i<axes_to_use.size()  ; i++) 
        {
                if(axes_to_use(i))
                {
                        //should think about applying a rotation to the sensor
                        TipForceSens_internal(m) = TipForceSens(i);
                        MaxWrench_internal(m,0) = MaxWrench(i,0);
                        MaxWrench_internal(m,1) = MaxWrench(i,1);
                        m++;
                }
        }
        
        //Next calculate the workspace disturbance
        //the grf forces may need to be treated differently from the torques
// return Torques;
        if(TipForceSens(2)>20.0)
        {
                for(i = 0, j ='0'; i < nrows; i++, j++) DisturbanceForce(i) = Filter("GRFC_TF"+j,LimitedMotorForce(i)+TipForceSens_internal(i), fil_freq);
        }
        else
        {
                for(i = 0, j ='0'; i < nrows; i++, j++) DisturbanceForce(i) = Filter("GRFC_TF"+j,0, fil_freq);
        }

// for(i = 0, j ='0'; i < nrows; i++, j++) DisturbanceForce(i) = 0;
// j = '4';
// i=4;
// DisturbanceForce(i) = Filter("GRFC_TF"+j,LimitedMotorForce(i)+TipForceSens_internal(i), fil_freq);

        //Calculate the motor force
        MotorForce = iJac_internal.transpose()*Torques_internal;
//         cout << "TipForceSens_internal "<<TipForceSens_internal.transpose()<<endl;
//         cout << "DisturbanceForce "<<DisturbanceForce.transpose()<<endl;
//         cout << "MotorForce "<<MotorForce.transpose()<<endl;
        //Calculate the future GRF
//         DisturbanceForce(0) = 0;
//         DisturbanceForce(1) = 0;
//      DisturbanceForce(2) = 0;        
//         if(DisturbanceForce(2)>55.0) DisturbanceForce(2) = 55.0;
//         else if(DisturbanceForce(2)<-55.0) DisturbanceForce(2) = -55.0;
        FutureGRF = MotorForce - DisturbanceForce;
// FutureGRF(4) = MotorForce(4) - DisturbanceForce(4);

//         cout << "FutureGRF "<<FutureGRF.transpose()<<endl;
//         cout << "MaxWrench "<<MaxWrench_internal<<endl;
//         cout << "Axestouse "<<axes_to_use<<endl;
        
        //Limit the future GRF
        if(TipForceSens(2)>20.0) // only apply force limits if foot is in contact
        {
        for(i = 0; i < nrows; i++)
        {
                        if      (FutureGRF(i) > MaxWrench_internal(i,0)) LimitedGRF(i) = MaxWrench_internal(i,0);
                        else if (FutureGRF(i) < MaxWrench_internal(i,1)) LimitedGRF(i) = MaxWrench_internal(i,1);
                        else                                    LimitedGRF(i) = FutureGRF(i);
        }
        }
        else  LimitedGRF = FutureGRF;
        
//         cout << "LimitedGRF "<<LimitedGRF.transpose()<<endl;
        //Apply force control to realize the limted GRF
        LimitedMotorForce = LimitedGRF + DisturbanceForce;
//         cout << "LimitedMotorForce "<<LimitedMotorForce.transpose()<<endl;
        
//         m = 0;
//         for(i = 0; i < 6; i++)
//         {
//                 if(joints_to_use(i))
//                 {
//                         LimitedMotorForce_internal(m) = LimitedMotorForce(i);
//                         m++;
//                 }
//         }
        
        LimitedTorque_internal = Jac_internal.transpose()*LimitedMotorForce;
//         cout << "Limitedtorqueinternal "<<LimitedTorque_internal.transpose()<<endl;
        
        //transform from the inernal limited torques to the final limited torques
        m = 0;
        for(i = 0; i < 6; i++)
        {
                if(joints_to_use(i))
                {
                        LimitedTorque(i) = LimitedTorque_internal(m);
                        m++;
                }
                else    LimitedTorque(i) = Torques(i);
        }
        
//         cout << "Limitedtorque "<<LimitedTorque.transpose()<<endl;
        
//         cout << " jac " << endl << Jac<<endl;
//         cout << " jac_internal " << endl << Jac_internal<<endl;
        
        return LimitedTorque;
}


RJVectorX RobotManipulator::GRFC_TF_ankle(const RJVectorX &Torques, const RJVectorX &TipForceSens, const RJMatrixX &MaxWrench, const Vector6b &axes_to_use,  const VectorXb &joints_to_use, const double &fil_freq)
/** @brief Experimental: Ground reaction force constrained torque filter
    @param Torques Desired joint torques
    @param TipForceSens Force sensor data from the tool tip
    @param MaxWrench Maximum allowable force wrench at the tool tip
    @param axes_to_use Decides which axes to enable. Cant be smaller than joints_to_use
    @param joints_to_use Decides which joints to enable. Must be greater or equal to axes_to_use.
    @param fil_freq cutoff frequency of the workspace observer.
    @returns The limited joint torques which prevent the resultant force from becoming larger than MaxWrench
    
                        EXPERIMENTAL!!!
    
    Implements the algorithim described in 
    "Maintaining floor-foot contact of a biped robot by force constraint position control" by Kirill Van Heerden
*/
{
        char j;
        int  i, k, m, n;

        int nrows = 6;
        int ncols = 6;
        //find the required dimensions for internal Jacobian
//         for(i = 0; i<axes_to_use.size()  ; i++) if(axes_to_use(i))   nrows++;
//         for(i = 0; i<joints_to_use.size(); i++) if(joints_to_use(i)) ncols++;

        RJMatrixX Jac_internal = RJMatrixX::Zero(nrows,ncols);
        RJMatrixX iJac = RJMatrixX::Zero(nrows,ncols);
        RJVectorX LimitedMotorForce_internal(nrows);
//         RJVectorX LimitedTorque_internal(nrows);
        RJVectorX Torques_internal(nrows);

        RJVectorX MotorForce = RJVectorX::Zero(nrows);
        RJVectorX FutureGRF = RJVectorX::Zero(nrows);
        RJVectorX LimitedGRF = RJVectorX::Zero(nrows);
        RJVectorX LimitedTorque = RJVectorX::Zero(Torques.size());
        RJVectorX LimitedTorque_internal(nrows);
        RJVectorX DisturbanceForce = RJVectorX::Zero(nrows);
        RJVectorX TipForceSens_internal(nrows);
        RJMatrixX MaxWrench_internal(nrows,2);

	RJVectorX GRFerror = RJVectorX::Zero(nrows);

        //Construct the internal Jacobian
        m = 0;// Jac_interal row index
        n = 0;// Jac_interal col index

        //Next calculate the workspace disturbance
        //the grf forces may need to be treated differently from the torques
// return Torques;

        pseudoInverse(Jac, iJac);

        if(TipForceSens(2)>20.0)
        {
		//do for pitch and roll torques only
                for(i = 0, j ='0'; i < 5; i++, j++) DisturbanceForce(i) = Filter("GRFC_TF"+j,LimitedMotorForce(i)+TipForceSens(i), fil_freq);
        }
        else
        {
                for(i = 0, j ='0'; i < 5; i++, j++) DisturbanceForce(i) = Filter("GRFC_TF"+j,0, fil_freq);
        }

// 	DisturbanceForce(0) = 0;
// 	DisturbanceForce(1) = 0;
// 	DisturbanceForce(2) = 0;
// 	DisturbanceForce(5) = 0;

        //Calculate the motor force
        MotorForce = iJac.transpose()*Torques;
	#define ROLL  3
	#define PITCH 4

// 	cout << "dist(PITCH) "<<DisturbanceForce(PITCH)<<endl;
// 	MotorForce(ROLL) = Torques(5);
// 	MotorForce(PITCH) = Torques(4);

        FutureGRF = MotorForce - DisturbanceForce;

        //Limit the future GRF
        if(TipForceSens(2)>20.0) // only apply force limits if foot is in contact
        {
        for(i = 3; i < 5; i++)
        {
                        if      (FutureGRF(i) > MaxWrench(i,0)) LimitedGRF(i) = MaxWrench(i,0);
                        else if (FutureGRF(i) < MaxWrench(i,1)) LimitedGRF(i) = MaxWrench(i,1);
                        else                                    LimitedGRF(i) = FutureGRF(i);
        }
        }
        else  LimitedGRF = FutureGRF;

	GRFerror = FutureGRF - LimitedGRF;

        LimitedMotorForce = LimitedGRF + DisturbanceForce;

// 	MotorForce = Jac.transpose()*LimitedMotorForce;

	LimitedTorque(0) = Torques(0);
	LimitedTorque(1) = Torques(1);
	LimitedTorque(2) = Torques(2);
	LimitedTorque(3) = Torques(3);

// 	LimitedTorque(4) = MotorForce(4)
	if(fabs((iJac.transpose())(PITCH, 4))>0.005)	LimitedTorque(4) = Torques(4) - (1.0/(iJac.transpose()(PITCH, 4)))*GRFerror(PITCH);//LimitedMotorForce(PITCH);
	else LimitedTorque(4) = Torques(4);

	LimitedTorque(5) = Torques(5);//LimitedMotorForce(ROLL);

// 	cout << "LimitedTorque(4) "<<LimitedTorque(4)<<endl;

//         LimitedTorque_internal = Jac_internal.transpose()*LimitedMotorForce;
//         m = 0;
//         for(i = 0; i < 6; i++)
//         {
//                 if(joints_to_use(i))
//                 {
//                         LimitedTorque(i) = LimitedTorque_internal(m);
//                         m++;
//                 }
//                 else    LimitedTorque(i) = Torques(i);
//         }

        return LimitedTorque;
}
