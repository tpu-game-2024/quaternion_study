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
	quaternion V_ = quaternion(v.x(), v.y(), v.z(), 0.0f);
	quaternion qvq_ = *this * V_ * conjugate();
	return vector3(qvq_.x_, qvq_.y_, qvq_.z_);
}

quaternion quaternion::operator*(const quaternion& rhs) const
{
	// todo: 実装して下さい
	float qx = w_* rhs.x_ + x_ * rhs.w_ + y_ * rhs.z_ - z_ * rhs.y_;
	float qy = w_* rhs.y_ + y_ * rhs.w_ + z_ * rhs.x_ - x_ * rhs.z_;
	float qz = w_* rhs.z_ + z_ * rhs.w_ + x_ * rhs.y_ - y_ * rhs.x_;
	float qw = w_* rhs.w_ - x_ * rhs.x_ - y_ * rhs.y_ - z_ * rhs.z_;

	return quaternion(qx,qy,qz,qw);
}

quaternion quaternion::operator+(const quaternion& rhs) const
{
	return quaternion(x_ + rhs.x_, y_ + rhs.y_, z_ + rhs.z_, w_ + rhs.w_);
}


// 単位元にする
quaternion &quaternion::identity()
{
	// todo: 実装して下さい
	return *this = quaternion::quaternion();
}

// 正規化する
quaternion &quaternion::normalize()
{
	// todo: 実装して下さい
	float scalar = sqrt(length_sq());
	return *this = *this / scalar;
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
	return quaternion(-x_,-y_,-z_ ,w_);
}

// 逆元を返す
quaternion quaternion::inverse() const
{
	// todo: 実装して下さい
	float norm = length_sq();
	quaternion con = conjugate();
	return con / norm;
}

//線形補間
quaternion quaternion::lerp(const quaternion& q0, const quaternion& q1, float t)
{
	//LERP(A,B,t) = (1 − t)A + tB = t(-A + B)+ A
	float x = t * (q1.x_ - q0.x_) + q0.x_;
	float y = t * (q1.y_ - q0.y_) + q0.y_;
	float z = t * (q1.z_ - q0.z_) + q0.z_;
	float w = t * (q1.w_ - q0.w_) + q0.w_;
	return quaternion(x, y, z, w).normalize();
}

// 球面線形補間
quaternion quaternion::slerp(const quaternion& q0, const quaternion& q1, float t)
{
	// todo: 実装して下さい
	
	//他のところを修正しているうちに必要なくなった。
	//quaternion q1_conjugate = q1.conjugate();

	// 二つのクォータニオンの内積を返す
	float dot_product = q0.x_ * q1.x_ + q0.y_ * q1.y_ + q0.z_ * q1.z_ + q0.w_ * q1.w_;

	//もし同じ値ならば内積が1,arccosが0,sin(theta)も0になり0で除算することになる

	/*
	if (dot_product == 1.0f) 
	{	
		//slerpの実行による補間の必要がないため、q0を返す 
		return q0;

		// 限りなく1に近い時の対策としてlerp(線形補間)を行う。
		//return lerp(q0,q1,t);
	}
	*/
	//上記の条件分岐はacos(dot_product)を使わなければ必要なくなった。
	float theta = dot_product; //acos(dot_product);

	float sin_theta = sinf(theta);
	quaternion q = q0 * (sinf((1 - t) * theta) / sin_theta) + q1 * (sinf(t * theta) / sin_theta);
	return q.normalize();

	//operatorを見逃していたので変更
	/*
	float w = (sin((1 - t) * theta) / sin_theta) * q0.w_ + (sin(t * theta) / sin_theta) * q1.w_;
	float x = (sin((1 - t) * theta) / sin_theta) * q0.x_ + (sin(t * theta) / sin_theta) * q1.x_;
	float y = (sin((1 - t) * theta) / sin_theta) * q0.y_ + (sin(t * theta) / sin_theta) * q1.y_;
	float z = (sin((1 - t) * theta) / sin_theta) * q0.z_ + (sin(t * theta) / sin_theta) * q1.z_;
	return quaternion(x, y, z, w).normalize();
	*/
}

