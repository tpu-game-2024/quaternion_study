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
	quaternion q = *this * quaternion(v.x(), v.y(), v.z(), 1.0f) * this->conjugate();
	return vector3(q.x(), q.y(), q.z());
}

quaternion quaternion::operator*(const quaternion& rhs) const
{
	return quaternion(
		w_ * rhs.x_ + rhs.w_ * x_ + y_ * rhs.z_ - z_ * rhs.y_,
		w_ * rhs.y_ + rhs.w_ * y_ + z_ * rhs.x_ - x_ * rhs.z_,
		w_ * rhs.z_ + rhs.w_ * z_ + x_ * rhs.y_ - y_ * rhs.x_,
		w_* rhs.w_ - x_ * rhs.x_ - y_ * rhs.y_ - z_ * rhs.z_);
}

quaternion quaternion::operator+(const quaternion& rhs) const
{
	return quaternion(x_ + rhs.x_, y_ + rhs.y_, z_ + rhs.z_, w_ + rhs.w_);
}


// 単位元にする
quaternion &quaternion::identity()
{
	this->x_ = 0.0f;
	this->y_ = 0.0f;
	this->z_ = 0.0f;
	this->w_ = 1.0f;
	return *this;
}

// 正規化する
quaternion &quaternion::normalize()
{
	*this = *this / std::sqrt(length_sq());
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
	return quaternion(-x_, -y_, -z_, w_);
}

// 逆元を返す
quaternion quaternion::inverse() const
{
	return this->conjugate() / this->length_sq();
}

// 球面線形補間
quaternion quaternion::slerp(const quaternion& q0, const quaternion& q1, float t)
{
	float theta = acosf(q0.x_ * q1.x_ + q0.y_ * q1.y_ + q0.z_ * q1.z_ + q0.w_ * q1.w_);
	if (theta * theta < 0.00000001f) return q0;// 発散を避ける
	float n = 1.0f / sinf(theta);
	return q0 * (sinf((1.0f - t) * theta) * n) + q1 * (sinf(t * theta) * n);
}

