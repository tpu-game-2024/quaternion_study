#pragma once

class vector3
{
	float v_[3] = {};
public:
	vector3() {}
	vector3(float x, float y, float z) { v_[0] = x, v_[1] = y, v_[2] = z; }
	~vector3() {}

	vector3 &set(float x, float y, float z) {
		v_[0] = x; v_[1] = y; v_[2] = z;
		return *this;
	}

	float x() const { return v_[0]; }
	float y() const { return v_[1]; }
	float z() const { return v_[2]; }

	static float dot(const vector3 &lhs, const vector3 &rhs);
	
	vector3& normalize();
};


class quaternion
{
private:
	float x_, y_, z_, w_;

	quaternion operator + (const quaternion& rhs) const;

	float length_sq() const; // 大きさの2乗
	quaternion conjugate() const;// 共役を返す
public:
	quaternion();
	quaternion(float x, float y, float z, float w);
	quaternion(const vector3 &v, float arg);
	~quaternion();

	// 要素の取得
	float x() const { return x_; }
	float y() const { return y_; }
	float z() const { return z_; }
	float w() const { return w_; }

	// 演算子の定義
	quaternion operator * (float f) const;
	quaternion operator / (float f) const;
	vector3 operator * (const vector3& rhs) const;
	quaternion operator * (const quaternion& rhs) const;

	// 変換
	quaternion &identity();// 単位元にする（返り値は自分自身）
	quaternion &normalize();// 正規化する（返り値は自分自身）

	// 生成
	quaternion inverse() const; // 逆元を返す
	static quaternion slerp(const quaternion &q0, const quaternion &q1, float t);// 球面線形補間
};


