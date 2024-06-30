#include "pch.h"
#include <cmath>
#include "../include/libquat.h"


float vector3::dot(const vector3 &lhs, const vector3 &rhs)
{
	return
		lhs.v_[0] * rhs.v_[0] +
		lhs.v_[1] * rhs.v_[1] +
		lhs.v_[2] * rhs.v_[2];
}

vector3& vector3::normalize()
{
	float l = std::sqrt(dot(*this, *this));

	if (l < 0.0000001f) return *this;// ゼロベクトルの際はそのまま返す

	v_[0] /= l;
	v_[1] /= l;
	v_[2] /= l;

	return *this;
}

