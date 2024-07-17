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
	// todo: 実装して下さい
	quaternion veq = quaternion(v.x(), v.y(), v.z(), 0);
	quaternion qq = *this * veq;
	quaternion q = qq * this->inverse();
	return vector3(q.x(), q.y(), q.z());
}

quaternion quaternion::operator*(const quaternion& rhs) const
{
	// todo: 実装して下さい
	quaternion q = quaternion(
		 this->w()*rhs.x() - this->z()*rhs.y() + this->y()*rhs.z() + this->x()*rhs.w(),
		 this->z()*rhs.x() + this->w()*rhs.y() - this->x()*rhs.z() + this->y()*rhs.w(),
		-this->y()*rhs.x() + this->x()*rhs.y() + this->w()*rhs.z() + this->z()*rhs.w(),
		-this->x()*rhs.x() - this->y()*rhs.y() - this->z()*rhs.z() + this->w()*rhs.w()
		);
	return q;
}

quaternion quaternion::operator+(const quaternion& rhs) const
{
	return quaternion(x_ + rhs.x_, y_ + rhs.y_, z_ + rhs.z_, w_ + rhs.w_);
}


// 単位元にする
quaternion &quaternion::identity()
{
	// todo: 実装して下さい
	x_ = 0.0f;
	y_ = 0.0f;
	z_ = 0.0f;
	w_ = 1.0f;
	return *this;
}

// 正規化する
quaternion &quaternion::normalize()
{
	// todo: 実装して下さい
	*this = *this / sqrt(length_sq());
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
	// todo: 実装して下さい
	quaternion q = quaternion(this->x() * -1, this->y() * -1, this->z() * -1, this->w());
	return q;
}

// 逆元を返す
quaternion quaternion::inverse() const
{
	// todo: 実装して下さい
	quaternion q = this->conjugate() / length_sq();
	return q;
}

// 球面線形補間
quaternion quaternion::slerp(const quaternion& q0, const quaternion& q1, float t)
{
	// todo: 実装して下さい
	return quaternion();
}

