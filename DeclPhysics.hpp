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
    Symbol<SE3>  dc_T_ki;
    Symbol<Inertia> dc_I;
    Symbol<dse3> dc_F;
        dcJoint()
        {
            dc_T_mi_i =  dc_M * dc_JointTransform;
            dc_I = lambda<
                [](Inertia& m_sI, SE3& m_sRightBodyFrame) {
                    return m_sI.Transform(m_sRightBodyFrame);
                }
            >(child->dc_I, dc_T_ki);
            dc_F = lambda <
                [](Inertia I, se3 dV, se3 V) {
                    auto F = I * dV - dad(V, I * V);
                    for (int j = 0; j < ChildJoints.size(); j++)
                    {
                        auto pChild = ChildJoints[j];
                        F += InvdAd(pChild->m_sRelativeFrame, pChild->m_sF);
                    }
                }
            > (dc_I, dc_dV, dc_V, );  
        }
    dcBody* parent; 
    dcBody* child;
    std::vector<dcJoint*> ChildJoints;
};
class dcBody
{
public:
    Symbol<Inertia> dc_I;
    Symbol<SE3> dc_frame;
    dcJoint* parent;
    std::vector<dcJoint*> childs;
    dcBody()
    {
    }
};

class dcRJoint : public dcJoint
{
    Symbol<Axis> dc_A;
    Symbol<scalar> dc_Q;
    Symbol<scalar> dc_dQ;
    Symbol<scalar> dc_ddQ;

    public :
    dcRJoint(){
        dc_JointTransform = lambda <
            [](Axis m_sS, scalar m_rQ) {
                return Exp(m_sS, m_rQ);
            }
        > (dc_A, dc_Q);
    }
    void Parenting(const dcJoint& parent)
    {
         dc_V = lambda<
             [](SE3 pCurrent_m_sRelativeFrame, se3 pParent_m_sV, se3 dqA)-> se3{
                 return InvAd(pCurrent_m_sRelativeFrame , pParent_m_sV) + dqA; 
             }
         >(dc_T_mi_i, parent.dc_V ,dc_dQ *dc_A);

             
    }
};

void IDIteration1(void)
{
	std::vector< dcJoint*> Jointarr;
	dcJoint* pCurrent, * pParent;
	SE3 _T;
	//cout << m_pJoint.size() << "joint count" << endl;
	for (int i = 0; i < Jointarr.size(); i++)
	{
		pCurrent = Jointarr[i];
		pParent = Jointarr[i]->parent;
	}
}
