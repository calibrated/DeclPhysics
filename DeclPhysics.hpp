#include <variable.hpp>
#include <VP/LieGroup.h>
//#include <VP/vpJoint.h>
#include <vector>
#include <VP/vpWorld.h>
class yFrame
{
    public:
        // vpJoint* joint;
        // vpBody* body;
        Symbol<SE3>             dc_bT_icom;
        Symbol<SE3>             dc_bT_0i;
        Symbol<SE3>             dc_jT_pi;  //M * JointT
        Symbol<SE3>		        dc_jT;	//Joint only transform
        SE3		                jbT_i;
        SE3		                bjT_i;
        Symbol<se3>				 dc_jV;
        Symbol<se3>				 dc_jDV;
        Symbol<dse3>            dc_jF;   //F_i  구하고자 하는 값.
        Symbol<dse3>            dc_bF_applied;
        Inertia                 b_I;
        Inertia                  j_I;
        SE3                     jM;
        //Symbol<dse3>            dc_jF_pi;
        std::vector<yFrame*> childs;
        Symbol<dse3> dc_pjF;
        std::vector<Symbol<dse3>*> dc_jF_ik_arr;
        Symbol<dse3>        dc_bF_ext;
        Symbol<dse3>        dc_bF_Gravity;
        yFrame * Parent;
        Symbol<Vec3> dc_CenterOfMass;
    yFrame()
    {
        dc_bT_icom = lambda<
        [](SE3 bT_0i, Vec3 CenterOfMass){
            SE3 T;
            T.Set(bT_0i, CenterOfMass);
            return T;
        }
        >(dc_bT_0i,dc_CenterOfMass);

        // dc_bF_Gravity = lambda<
        //    [](SE3 T_sb){
        //     return b_I * InvAd(T_sb, World->GetGravity());
        //    }
        // >(dc_bT_0i);
        j_I = body->m_sI.Transform(joint->m_sRightBodyFrame);
        
        dc_jT_pi = lambda<[](SE3 JointTransform){
            return  cj_M  * JointTransform;
        }>	(dc_jT);
        
        dc_jF_pi = lambda<
            [](SE3 jT_pi, dse3 jFi){
                return InvdAd(jT_pi, jFi);
            }
        >(dc_jT_pi,dc_jF); 
        dc_jF = lambda<
		[](se3 DVi,se3 Vi, auto jF_ik_arr,SE3 jT_ik,dse3 bFext_k){
            auto f = j_I * DVi - dad(Vi, j_I * Vi);
            for (auto jF_ik : jF_ik_arr)  //F i -> k. 반작용 힘. 
			    f += jF_ik;
            //f -= dAd(jbT_i, bFext_k);   // 
            f -= InvdAd(bjT_i, bFext_k);
            return f;
		}>(dc_jDV, dc_jV , dc_jF_ik_arr, dc_bFext);
        dc_bF_ext = lambda<
            [](dse3 bF_applied , SE3 T_icom){
                return = bF_applied + b_I * InvAd(bT_icom, m_pWorld->GetGravity());
            }
        >(dc_bF_applied,dc_bT_icom);
    }
    virtual void InitSymbols()
    {
        dc_jF_ik_arr.clear();
        for (auto child : childs) {
            dc_jF_ik_arr.push_back(child->dc_jF_pi);
        }
        
        
    }
};
vpWorld* World;
// class yRFrame : public yFrame
// {
//     Symbol<Axis>		vj_A;	
// 	Symbol<scalar>		vj_Q;
//     Symbol<scalar>      vj_dQ;
//     Symbol<scalar>      vj_ddQ;
   
// public :
//     yRFrame()
//     {
//         v_JointTransform = lambda<[](Axis s,scalar q){
//             return Exp(s,q);
//         }>(v_A,v_Q);
//     }
//     void InitSymbols()
//     {
//         vj_V = lambda<
// 		[](SE3 pCurrent_m_sRelativeFrame, se3 pParent_m_sV, scalar m_rDq, Axis m_sS){
// 			return InvAd(pCurrent_m_sRelativeFrame, pParent_m_sV) +  m_rDq * m_sS;
// 		}>(vj_T_lambdai, Parent->vj_V, vj_dQ, vj_A);

//         vj_DV = lambda<
// 		[](SE3 RelativeFrame, se3 pParent_m_sDV, scalar m_rDq,scalar m_rDdq, Axis m_sS ,se3 m_sV){
// 			return InvAd(RelativeFrame, pParent_m_sDV) 
//             +  m_rDdq * m_sS 
//             + ad(m_sV, m_rDdq * m_sS);
// 		}>(vj_T_lambdai, Parent->vj_DV,vj_dQ, vj_ddQ, vj_A, vj_V);
 
        

//     }
// };

