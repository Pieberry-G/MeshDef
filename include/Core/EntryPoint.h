#pragma once

#include <iostream>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using namespace std;

void testEigen(void) {
    MatrixXd matrix1(2, 2);
    matrix1(0, 0) = 3;
    matrix1(1, 0) = 2;
    matrix1(0, 1) = -1;
    matrix1(1, 1) = matrix1(0, 0) + matrix1(0, 1);
    cout << matrix1 << endl;
}

int main(void) {
    cout << "Hello World" << endl;
    testEigen();
    getchar();
    return 0;
}


//int main()
//{
//	MeshDef::Log::Init();
//	MD_CORE_WARN("Log system initialized!");
//
//	MeshDef::Application* app = new MeshDef::Application();
//	app->Run();
//	delete app;
//}