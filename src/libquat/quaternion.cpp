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
	const quaternion qv(0, v.x(), v.y(), v.z());
	const quaternion res = qv * *this * this->conjugate();
	return vector3(res.x_, res.y_, res.z_);
}

quaternion quaternion::operator*(const quaternion& rhs) const
{
	const float x = w_ * rhs.x_ + x_ * rhs.w_ + y_ * rhs.z_ - z_ * rhs.y_;
	const float y = w_ * rhs.y_ - x_ * rhs.z_ + y_ * rhs.w_ + z_ * rhs.x_;
	const float z = w_ * rhs.z_ + x_ * rhs.y_ - y_ * rhs.x_ + z_ * rhs.w_;
	const float w = w_ * rhs.w_ - x_ * rhs.x_ - y_ * rhs.y_ - z_ * rhs.z_;
	return quaternion(x, y, z, w);
}

quaternion quaternion::operator+(const quaternion& rhs) const
{
	return quaternion(x_ + rhs.x_, y_ + rhs.y_, z_ + rhs.z_, w_ + rhs.w_);
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
	float dot = q0.x_ * q1.x_ + q0.y_ * q1.y_ + q0.z_ * q1.z_ + q0.w_ * q1.w_;

	if (fabs(dot) > 0.9995) {
		return (q0 * (1 - t) + q1 * t).normalize();
	}

	// theta および sin(theta) の計算
	float theta = acos(dot);
	float sin_theta = sin(theta);

	float s0 = sin((1 - t) * theta) / sin_theta;
	float s1 = sin(t * theta) / sin_theta;

	quaternion q0s = q0 * s0;
	quaternion q1s = q1 * s1;
	return quaternion(
		s0 * q0.x_ + s1 * q1.x_,
		s0 * q0.y_ + s1 * q1.y_,
		s0 * q0.z_ + s1 * q1.z_,
		s0 * q0.w_ + s1 * q1.w_
	).normalize();
}