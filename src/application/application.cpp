// application.cpp : このファイルには 'main' 関数が含まれています。プログラム実行の開始と終了がそこで行われます。
//

#include <iostream>
#include "../include/libquat.h"

#pragma comment(lib,"libquat.lib")


int main()
{
	const float PI = 3.1415926535f;

	vector3 axis(1.0f, 1.0f, 1.0f);
	quaternion q(axis.normalize(), 2.0f * PI / 3.0f);// 軸の入れ替え

	vector3 v(1.0f, 0.0f, 0.0f);
	v = q * v;

    std::cout << "(x,y,z)=(" << v.x() << ", "<< v.y() << ", "<< v.z() << ")" << std::endl;

    system("PAUSE");
}
