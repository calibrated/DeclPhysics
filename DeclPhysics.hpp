#include <variable.hpp>
#include <VP/LieGroup.h>

class dcJoint
{
    public:
    Symbol<se3> dc_V;
    Symbol<se3> dc_dV;
    Symbol<SE3> dc_JointTransform;
    Symbol<SE3> dc_T_mi_i;
    Symbol<SE3>  dc_M;

        dcJoint()
        {
            dc_T_mi_i =  dc_M * dc_JointTransform;
        }
};

class dcRJoint : public dcJoint
{
    Symbol<Axis> dc_A;
    Symbol<scalar> dc_Q;
    Symbol<scalar> dc_dQ;
    Symbol<scalar> dc_ddQ;

    public :
    dcRJoint(){}
    void Parenting(dcJoint* parent)
    {
         dc_V = lambda<
             [](SE3 pCurrent_m_sRelativeFrame, se3 pParent_m_sV, se3 dqA)-> se3{
                 return InvAd(pCurrent_m_sRelativeFrame , pParent_m_sV) + dqA; 
             }
         >(dc_T_mi_i, parent->dc_V ,dc_dQ *dc_A);
    }
};