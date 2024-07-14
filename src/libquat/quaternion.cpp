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
	//項を事前に計算する
	const float tx = 2.0f * v.x();
	const float ty = 2.0f * v.y();
	const float tz = 2.0f * v.z();
	const float ww = w_ * w_;
	const float xx = x_ * x_;
	const float yy = y_ * y_;
	const float zz = z_ * z_;
	const float xy = x_ * y_;
	const float wz = w_ * z_;
	const float wy = w_ * y_;
	const float xz = x_ * z_;
	const float yz = y_ * z_;
	const float wx = w_ * x_;

	//ベクトルとかける
	const float rx = v.x() * (ww + xx - yy - zz) + ty * (xy - wz) + tz * (wy + xz);
	const float ry = tx * (xy + wz) + v.y() * (ww - xx + yy - zz) + tz * (yz - wx);
	const float rz = tx * (xz - wy) + ty * (wx + yz) + v.z() * (ww - xx - yy + zz);

	return vector3(rx, ry, rz);
}

quaternion quaternion::operator*(const quaternion& rhs) const
{
	return quaternion(
		w_ * rhs.w() - x_ * rhs.x() - y_ * rhs.y() - z_ * rhs.z(),
		w_ * rhs.x() + x_ * rhs.w() + y_ * rhs.z() - z_ * rhs.y(),
		w_ * rhs.y() - x_ * rhs.z() + y_ * rhs.w() + z_ * rhs.x(),
		w_ * rhs.z() + x_ * rhs.y() - y_ * rhs.x() + z_ * rhs.w()
	);
}

quaternion quaternion::operator+(const quaternion& rhs) const
{
	return quaternion(x_ + rhs.x_, y_ + rhs.y_, z_ + rhs.z_, w_ + rhs.w_);
}


// 単位元にする
quaternion& quaternion::identity()
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
	return quaternion(-x_, -y_, -z_, w_);
}

// 逆元を返す
quaternion quaternion::inverse() const
{
	const float length = length_sq();

	return conjugate() / length;
}

// 球面線形補間
quaternion quaternion::slerp(const quaternion& q0, const quaternion& q1, float t)
{
	const float dot = q0.x() * q1.x() + q0.y() * q1.y() + q0.z() * q1.z() + q0.w() * q1.w();

	//全てが逆のQuaternionは符号を逆にして扱う
	if (dot < -0.9999)
	{
		const quaternion q1_ = quaternion(-q1.x(), -q1.y(), -q1.z(), -q1.w());
		return slerp(q0, q1_, t);
	}

	//ほぼ同じ方向の場合は近似する
	if (dot > 0.9999)
	{
		return quaternion(q0 * (1.0f - t) + q1 * t).normalize();
	}

	const float theta = acos(dot);
	const float sint = sin(theta);

	const float s0 = sin((1 - t) * theta) / sint;
	const float s1 = sin(t * theta) / sint;

	return q0 * s0 + q1 * s1;

}

