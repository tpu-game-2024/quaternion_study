#include "pch.h"
#include <cmath>
#include "../include/libquat.h"

// コンストラクタ
quaternion::quaternion()
{
	x_ = 0.0f;
	y_ = 0.0f;
	z_ = 0.0f;
	w_ = 1.0f;
}

quaternion::quaternion(float x, float y, float z, float w)
{
	x_ = x;
	y_ = y; 
	z_ = z; 
	w_ = w;
}

quaternion::quaternion(const vector3 &v, float arg)
{
	float co = cosf(0.5f * arg);
	float si = sinf(0.5f * arg);

	// 0割りを防ぐ
	if (abs(co) < 0.0001f) co = 0.0f;
	if (abs(si) < 0.0001f) si = 0.0f;

	x_ = si * v.x();
	y_ = si * v.y();
	z_ = si * v.z();
	w_ = co;
}

// デストラクタ
quaternion::~quaternion()
{
}

quaternion quaternion::operator*(float f) const
{
	return quaternion(x_ * f, y_ * f, z_ * f, w_ * f);
}

quaternion quaternion::operator/(float f) const
{
	return quaternion(x_ / f, y_ / f, z_ / f, w_ / f);
}

vector3 quaternion::operator*(const vector3& v) const
{
	const quaternion qv(v.x(), v.y(), v.z(), 1.0f);
	const quaternion res = *this * qv * this->conjugate();
	return vector3(res.x(), res.y(), res.z());
}

quaternion quaternion::operator*(const quaternion& rhs) const
{
	const float x = w_ * rhs.x() + x_ * rhs.w() + y_ * rhs.z() - z_ * rhs.y();
	const float y = w_ * rhs.y() - x_ * rhs.z() + y_ * rhs.w() + z_ * rhs.x();
	const float z = w_ * rhs.z() + x_ * rhs.y() - y_ * rhs.x() + z_ * rhs.w();
	const float w = w_ * rhs.w() - x_ * rhs.x() - y_ * rhs.y() - z_ * rhs.z();
	return quaternion(x, y, z, w);
}

quaternion quaternion::operator+(const quaternion& rhs) const
{
	return quaternion(x_ + rhs.x(), y_ + rhs.y(), z_ + rhs.z(), w_ + rhs.w());
}


// 単位元にする
quaternion &quaternion::identity()
{
	x_ = 0.0f;
	y_ = 0.0f;
	z_ = 0.0f;
	w_ = 1.0f;
	return *this;
}

// 正規化する
quaternion &quaternion::normalize()
{
	const float norm = sqrt(length_sq());
	x_ /= norm;
	y_ /= norm;
	z_ /= norm;
	w_ /= norm;
	return *this;
}

// 大きさの2乗
float quaternion::length_sq() const
{
	return x_ * x_ + y_ * y_ + z_ * z_ + w_ * w_;
}

// 共役を返す
quaternion quaternion::conjugate() const
{
	return quaternion(
		x_ != 0 ? -x_ : 0,
		y_ != 0 ? -y_ : 0,
		z_ != 0 ? -z_ : 0,
		w_
	);
}

// 逆元を返す
quaternion quaternion::inverse() const
{
	const float norm = sqrt(length_sq());
	return conjugate() / (norm * norm);
}

// 球面線形補間
quaternion quaternion::slerp(const quaternion& q0, const quaternion& q1, float t) {
	const float dot = q0.x() * q1.x() + q0.y() * q1.y() + q0.z() * q1.z() + q0.w() * q1.w();

	if (fabs(dot) > 0.9995) {
		return (q0 * (1 - t) + q1 * t).normalize();
	}

	// theta および sin(theta) の計算
	const float theta = acos(dot);
	const float sin_theta = sin(theta);

	const float s0 = sin((1 - t) * theta) / sin_theta;
	const float s1 = sin(t * theta) / sin_theta;
	
	return q0 * s0 + q1 * s1;
}