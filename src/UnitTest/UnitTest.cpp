#include "pch.h"
#include "CppUnitTest.h"
#include "../include/libquat.h"

#pragma comment(lib,"libquat.lib")

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace UnitTest
{
	TEST_CLASS(QuaternionTest)
	{
		const float PI = 3.1415926535f;

	public:
		
		TEST_METHOD(TestIdentity)// まずは、動くかどうか
		{
			quaternion q;
			q.identity();

			Assert::AreEqual(0.0f, q.x(), 0.0001f, L"x");// 期待値、実際の値、誤差、コメント
			Assert::AreEqual(0.0f, q.y(), 0.0001f, L"y");
			Assert::AreEqual(0.0f, q.z(), 0.0001f, L"z");
			Assert::AreEqual(1.0f, q.w(), 0.0001f, L"w");
		}

		TEST_METHOD(TestNormalize) // 長さを1にできるか
		{
			quaternion q(1.0f, 2.0f, 2.0f, 4.0f);// 長さはsqrt(1+4+4+16)=sqrt(25)=5
			q.normalize();

			Assert::AreEqual(1.0f / 5.0f, q.x(), 0.0001f, L"x");
			Assert::AreEqual(2.0f / 5.0f, q.y(), 0.0001f, L"y");
			Assert::AreEqual(2.0f / 5.0f, q.z(), 0.0001f, L"z");
			Assert::AreEqual(4.0f / 5.0f, q.w(), 0.0001f, L"w");
		}

		TEST_METHOD(TestInverse) // 逆元が作れるか?
		{
			// 正規化された quaternion の逆元は共役
			vector3 axis(1.0f, 2.0f, 3.0f);
			quaternion q(axis.normalize(), .1f * PI);
			quaternion c = q.inverse();

			Assert::AreEqual(-q.x(), c.x(), 0.0001f, L"x");
			Assert::AreEqual(-q.y(), c.y(), 0.0001f, L"y");
			Assert::AreEqual(-q.z(), c.z(), 0.0001f, L"z");
			Assert::AreEqual(+q.w(), c.w(), 0.0001f, L"w");
		}

		TEST_METHOD(TestApply)// ベクトルをチキンと回転できるか?
		{
			vector3 axis(1.0f, 1.0f, 1.0f);
			quaternion q(axis.normalize(), 2.0f * PI / 3.0f);// (1,1,1)を軸として1/3回転

			vector3 x2(2.0f, 0.0f, 0.0f);
			vector3 y2 = q * x2;
			Assert::AreEqual(0.0f, y2.x(), 0.0001f, L"y-x");
			Assert::AreEqual(2.0f, y2.y(), 0.0001f, L"y-y");
			Assert::AreEqual(0.0f, y2.z(), 0.0001f, L"y-z");

			vector3 z2 = q * y2;
			Assert::AreEqual(0.0f, z2.x(), 0.0001f, L"z-x");
			Assert::AreEqual(0.0f, z2.y(), 0.0001f, L"z-y");
			Assert::AreEqual(2.0f, z2.z(), 0.0001f, L"z-z");
		}

		TEST_METHOD(TestSlerp) // 球面線形補間の検証
		{
			quaternion q0(vector3(1.0f, 0.0f, 0.0f), 0.5f * PI);// x軸周りの90度回転
			quaternion q1(vector3(0.0f, 1.0f, 0.0f), 0.5f * PI);// y軸周りの90度回転
			float t = 0.5f;// 真ん中
			quaternion qt = quaternion::slerp(q0, q1, t);

			// 生成したクオータニオンで変換
			vector3 z1(0.0f, 0.0f, 1.0f);
			vector3 v = qt * z1;

			Assert::AreEqual(+0.6666f, v.x(), 0.0001f, L"x");
			Assert::AreEqual(-0.6666f, v.y(), 0.0001f, L"y");
			Assert::AreEqual(+0.3333f, v.z(), 0.0001f, L"z");
		}

		TEST_METHOD(TestSlerpParallel) // 同じquaternionでSlerpしてもきちんと動くか?
		{
			quaternion q0(vector3(1.0f, 0.0f, 0.0f), 1.0f * PI);// x軸周りの180度回転
			float t = 0.5f;// 真ん中
			quaternion qt = quaternion::slerp(q0, q0, t);

			// 生成したクオータニオンで変換
			vector3 z1(0.0f, 0.0f, 1.0f);
			vector3 v = qt * z1;

			Assert::AreEqual( 0.0f, v.x(), 0.0001f, L"x");
			Assert::AreEqual( 0.0f, v.y(), 0.0001f, L"y");
			Assert::AreEqual(-1.0f, v.z(), 0.0001f, L"z");
		}
	};
}
