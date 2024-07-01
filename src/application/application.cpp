// application.cpp : このファイルには 'main' 関数が含まれています。プログラム実行の開始と終了がそこで行われます。
//

#include <iostream>
#include "../include/libquat.h"

#pragma comment(lib,"libquat.lib")


int main()
{
	const float PI = 3.1415926535f;
	quaternion q0(vector3(1.0f, 0.0f, 0.0f), 1.0f * PI);// x軸周りの180度回転
	float t = 0.5f;// 真ん中
	quaternion qt = quaternion::slerp(q0, q0, t);

	// 生成したクオータニオンで変換
	vector3 z1(0.0f, 0.0f, 1.0f);
	vector3 v = qt * z1;
    std::cout << "(x,y,z)=(" << v.x() << ", "<< v.y() << ", "<< v.z() << ")" << std::endl;

    system("PAUSE");
}
