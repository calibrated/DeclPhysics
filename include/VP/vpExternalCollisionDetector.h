#ifndef VP_EXTERNAL_COLLISION_DETECTOR
#define VP_EXTERNAL_COLLISION_DETECTOR

#include <VP/vpCollisionDetector.h>

/*!
\class vpPrimitiveCollisionDetector
\brief detecting collision between rigid bodies built as a compound of pritimive geometries
*/
class vpExternalCollisionDetector : public vpCollisionDetector
{
public:

	/*!
	m_sCollidablePair, which is an array of pairs of collidable bodies, is built.
	*/
	virtual void			 Initialize(void);

	/*!
	For each elements in m_sCollidablePair, examines a collision by checking a pairwise inter-penetration between primitive geomtries composing the bodies.
	If a penetraion is found, then it adds the pair to vpCollisionDetector::m_sCollisionList.
	*/
	virtual void			 DetectCollision(void);

	int						 m_iNumConllision;

protected:

	static int				 m_iMaxNumContact4Box;
};

#endif
