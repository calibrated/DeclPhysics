#include <constant.h>

// 3 dimensional vector but used for angular part of se(3) or dse(3)
class Axis
{
public:
	Axis();

	/*!
		constructor : (c0, c1, c2)
	*/
	explicit		 Axis(scalar c0, scalar c1, scalar c2);

	/*!
		constructor : (c, c, c)
	*/
	explicit		 Axis(int c);

	/*!
		constructor : (c, c, c)
	*/
	explicit		 Axis(scalar c);

	/*!
		constructor : (v[0], v[1], v[2])
	*/
	explicit		 Axis(const scalar v[]);

	/*!
		constructor : (v[0], v[1], v[2])
	*/
	explicit		 Axis(const Vec3& v);

	/** @name Operators
	*/
	//@{
	/*!
		unary plus operator
	*/
	const Axis& operator + (void) const;

	/*!
		unary minus operator
	*/
	Axis			 operator - (void) const;

	/*!
		access to the idx th element.
	*/
	scalar& operator [] (int idx);
	const scalar& operator [] (int) const;

	/*!
		substitution operator
	*/
	const Axis& operator = (const Axis&);

	/*!
		fast version of = Axis(s[0], s[1], s[2])
	*/
	const Axis& operator = (const se3&);

	/*!
		substitute operator
		set all the elements to be c.
	*/
	const Axis& operator = (scalar c);

	/*!
		*= operator
	*/
	const Axis& operator *= (scalar);

	/*!
		multiplication operator
	*/
	Axis			 operator * (scalar) const;

	/*!
		addition operator
	*/
	Axis			 operator + (const Axis&) const;

	/*!
		addition operator
	*/
	se3				 operator + (const Vec3&) const;

	/*!
		subtraction operator
	*/
	Axis			 operator - (const Axis&) const;

	/*!
		addition and substitution operator
	*/
	const Axis& operator += (const Axis&);

	/*!
		-= operator
	*/
	const Axis& operator -= (const Axis&);
	//@}

	/*!
		normalize the vector.
		\return length of the vector.
	*/
	scalar			 Normalize(void);

	/*!
		reparameterize such that ||s'|| <= M_PI and Exp(s) == Epx(s')
	*/
	void			 Reparameterize(void);

	/*!
		standard output operator
	*/
	friend ostream& operator << (ostream&, const Axis&);

	/*!
		scalar multiplication
	*/
	friend Axis		 operator * (scalar c, const Axis& p);

	/*!
		get a magnitude of p.
	*/
	friend scalar	 Norm(const Axis& p);

	/*!
		get a normalized vector from p.
	*/
	friend Axis		 Normalize(const Axis& p);

	/*!
		reparameterize such as ||s'|| < M_PI and Exp(s) == Epx(s')
	*/
	friend Axis		 Reparameterize(const Axis& s);

	/*!
		get a cross product of p and q.
	*/
	friend Axis		 Cross(const Axis& p, const Axis& a);

	/*!
		get an inner product of p and q.
	*/
	friend scalar	 Inner(const Axis& p, const Axis& a);
	friend scalar	 Inner(const Vec3& p, const Axis& a);
	friend scalar	 Inner(const Axis& p, const Vec3& a);

	/*!
		get a squared sum of all the elements in p.
	*/
	friend scalar	 SquareSum(const Axis&);

	/*!
		rotate p by T.
		\return \f$R q\f$, where \f$T=(R,p)\f$.
	*/
	friend Axis		 Rotate(const SE3& T, const Axis& q);

	/*!
		rotate q by Inv(T).
	*/
	friend Axis		 InvRotate(const SE3& T, const Axis& q);

	/*!
		fast version of ad(se3(w, 0), se3(v, 0))	-> check
	*/
	friend Axis		 ad(const Axis& w, const Axis& v);

	/*!
		fast version of Ad(T, se3(w, Vec3(0))
	*/
	friend se3		 Ad(const SE3& T, const Axis& w);

	/*!
		fast version of Ad(T, se3(Axis(0), v)
	*/
	friend se3		 Ad(const SE3& T, const Vec3& v);

	/*!
		fast version of Ad(Inv(T), se3(w, Vec3(0)))
	*/
	friend Axis		 InvAd(const SE3& T, const Axis& w);


private:
	scalar			_v[3];
};
inline Axis::Axis()
{
}

inline Axis::Axis(int d)
{
	_v[0] = _v[1] = _v[2] = (scalar)d;
}

inline Axis::Axis(scalar d)
{
	_v[0] = _v[1] = _v[2] = d;
}

inline Axis::Axis(const scalar v[])
{
	_v[0] = v[0];
	_v[1] = v[1];
	_v[2] = v[2];
}

inline Axis::Axis(scalar v0, scalar v1, scalar v2)
{
	_v[0] = v0;
	_v[1] = v1;
	_v[2] = v2;
}

inline Axis::Axis(const Vec3& v)
{
	_v[0] = v[0];
	_v[1] = v[1];
	_v[2] = v[2];
}

inline const Axis& Axis::operator + (void) const
{
	return *this;
}

inline Axis Axis::operator - (void) const
{
	return Axis(-_v[0], -_v[1], -_v[2]);
}

inline scalar& Axis::operator [] (int i)
{
	return _v[i];
}

inline const scalar& Axis::operator [] (int i) const
{
	return _v[i];
}

inline const Axis& Axis::operator = (const Axis& v)
{
	_v[0] = v[0];
	_v[1] = v[1];
	_v[2] = v[2];
	return *this;
}

inline const Axis& Axis::operator = (const se3& v)
{
	_v[0] = v[0];
	_v[1] = v[1];
	_v[2] = v[2];
	return *this;
}

inline const Axis& Axis::operator = (scalar d)
{
	_v[0] = _v[1] = _v[2] = d;
	return *this;
}

inline const Axis& Axis::operator *= (scalar d)
{
	_v[0] *= d;
	_v[1] *= d;
	_v[2] *= d;
	return *this;
}

inline Axis Axis::operator * (scalar d) const
{
	return Axis(d * _v[0], d * _v[1], d * _v[2]);
}

inline scalar Axis::Normalize(void)
{
	scalar mag = sqrt(_v[0] * _v[0] + _v[1] * _v[1] + _v[2] * _v[2]);
	if (mag < LIE_EPS)	// make a unit vector in z-direction
	{
		_v[0] = _v[1] = SCALAR_0;
		_v[2] = SCALAR_1;
	}
	else
	{
		_v[0] /= mag;
		_v[1] /= mag;
		_v[2] /= mag;
	}
	return mag;
}

inline void Axis::Reparameterize(void)
{
	scalar theta = sqrt(_v[0] * _v[0] + _v[1] * _v[1] + _v[2] * _v[2]);
	if (theta > LIE_EPS)
	{
		scalar eta = 1.0 - (scalar)((int)(theta / M_PI + 1.0) / 2) * M_2PI / theta;
		_v[0] *= eta;
		_v[1] *= eta;
		_v[2] *= eta;
	}
}

inline Axis Reparameterize(const Axis& s)
{
	scalar theta = sqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]);
	scalar eta = theta < LIE_EPS ? 1.0 : 1.0 - (scalar)((int)(theta / M_PI + 1.0) / 2) * M_2PI / theta;
	return eta * s;
}

inline Axis Rotate(const SE3& T, const Axis& v)
{
	return Axis(T[0] * v[0] + T[3] * v[1] + T[6] * v[2],
		T[1] * v[0] + T[4] * v[1] + T[7] * v[2],
		T[2] * v[0] + T[5] * v[1] + T[8] * v[2]);
}

inline Axis InvRotate(const SE3& T, const Axis& v)
{
	return Axis(T[0] * v[0] + T[1] * v[1] + T[2] * v[2],
		T[3] * v[0] + T[4] * v[1] + T[5] * v[2],
		T[6] * v[0] + T[7] * v[1] + T[8] * v[2]);
}

inline Axis operator * (scalar d, const Axis& v)
{
	return Axis(d * v[0], d * v[1], d * v[2]);
}

inline scalar Norm(const Axis& v)
{
	return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

inline Axis Normalize(const Axis& v)
{
	scalar mag = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	if (mag < LIE_EPS)	// make a unit vector in z-direction
		return Axis(SCALAR_0, SCALAR_0, SCALAR_1);

	mag = SCALAR_1 / mag;
	return Axis(mag * v[0], mag * v[1], mag * v[2]);
}

inline Axis Cross(const Axis& p, const Axis& q)
{
	return Axis(p[1] * q[2] - p[2] * q[1],
		p[2] * q[0] - p[0] * q[2],
		p[0] * q[1] - p[1] * q[0]);
}

inline scalar Inner(const Axis& p, const Axis& q)
{
	return (p[0] * q[0] + p[1] * q[1] + p[2] * q[2]);
}

inline scalar Inner(const Vec3& p, const Axis& q)
{
	return (p[0] * q[0] + p[1] * q[1] + p[2] * q[2]);
}

inline scalar Inner(const Axis& p, const Vec3& q)
{
	return (p[0] * q[0] + p[1] * q[1] + p[2] * q[2]);
}

inline scalar SquareSum(const Axis& p)
{
	return (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
}

inline Axis Square(const Axis& p)
{
	return Axis(p[0] * p[0], p[1] * p[1], p[2] * p[2]);
}

inline Axis InvAd(const SE3& T, const Axis& v)
{
	return Axis(T[0] * v[0] + T[1] * v[1] + T[2] * v[2],
		T[3] * v[0] + T[4] * v[1] + T[5] * v[2],
		T[6] * v[0] + T[7] * v[1] + T[8] * v[2]);
}

inline Axis ad(const Axis& s1, const se3& s2)
{
	return Axis(s2[2] * s1[1] - s2[1] * s1[2],
		s2[0] * s1[2] - s2[2] * s1[0],
		s2[1] * s1[0] - s2[0] * s1[1]);
}

inline Axis ad(const Axis& s1, const Axis& s2)
{
	return Axis(s2[2] * s1[1] - s2[1] * s1[2],
		s2[0] * s1[2] - s2[2] * s1[0],
		s2[1] * s1[0] - s2[0] * s1[1]);
}

inline Axis Axis::operator + (const Axis& v) const
{
	return Axis(_v[0] + v[0], _v[1] + v[1], _v[2] + v[2]);
}

inline se3 Axis::operator + (const Vec3& v) const
{
	return se3(_v[0], _v[1], _v[2], v[0], v[1], v[2]);
}

inline Axis Axis::operator - (const Axis& v) const
{
	return Axis(_v[0] - v[0], _v[1] - v[1], _v[2] - v[2]);
}

inline const Axis& Axis::operator += (const Axis& v)
{
	_v[0] += v[0];
	_v[1] += v[1];
	_v[2] += v[2];
	return *this;
}

inline const Axis& Axis::operator -= (const Axis& v)
{
	_v[0] -= v[0];
	_v[1] -= v[1];
	_v[2] -= v[2];
	return *this;
}

// I + sin(t) / t * [S] + (1 - cos(t)) / t^2 * [S]^2, where t = |S|
inline Axis::Exp(const Axis& S)
{
	scalar s2[] = { S[0] * S[0], S[1] * S[1], S[2] * S[2] };
	scalar s3[] = { S[0] * S[1], S[1] * S[2], S[2] * S[0] };
	scalar theta = sqrt(s2[0] + s2[1] + s2[2]), cos_t = cos(theta), alpha, beta;

	if (theta > LIE_EPS)
	{
		alpha = sin(theta) / theta;
		beta = (SCALAR_1 - cos_t) / theta / theta;
	}
	else
	{
		alpha = SCALAR_1 - SCALAR_1_6 * theta * theta;
		beta = SCALAR_1_2 - SCALAR_1_24 * theta * theta;
	}

	return SE3(beta * s2[0] + cos_t, beta * s3[0] + alpha * S[2], beta * s3[2] - alpha * S[1],
		beta * s3[0] - alpha * S[2], beta * s2[1] + cos_t, beta * s3[1] + alpha * S[0],
		beta * s3[2] + alpha * S[1], beta * s3[1] - alpha * S[0], beta * s2[2] + cos_t);
}

// I + sin(t) * [S] + (1 - cos(t)) * [S]^2,, where |S| = 1
inline SE3 Exp(const Axis& S, scalar theta)
{
	scalar s2[] = { S[0] * S[0], S[1] * S[1], S[2] * S[2] };

	if (abs(s2[0] + s2[1] + s2[2] - SCALAR_1) > LIE_EPS) return Exp(theta * S);

	scalar s3[] = { S[0] * S[1], S[1] * S[2], S[2] * S[0] };
	scalar alpha = sin(theta), cos_t = cos(theta), beta = SCALAR_1 - cos_t;

	return SE3(beta * s2[0] + cos_t, beta * s3[0] + alpha * S[2], beta * s3[2] - alpha * S[1],
		beta * s3[0] - alpha * S[2], beta * s2[1] + cos_t, beta * s3[1] + alpha * S[0],
		beta * s3[2] + alpha * S[1], beta * s3[1] - alpha * S[0], beta * s2[2] + cos_t);
}
