#pragma once

// int main()
// {
// 	MeshDef::Log::Init();
// 	MD_CORE_WARN("Log system initialized!");
//
// 	MeshDef::Application* app = new MeshDef::Application();
// 	app->Run();
// 	delete app;
// }


#include "torch/torch.h"
#include "torch/script.h"
#include <iostream>
 
int main()
{
    // 检查CUDA是否可用
    bool is_cuda_available = torch::cuda::is_available();
    std::cout << "CUDA available: " << (is_cuda_available ? "Yes" : "No") << std::endl;
    
    // 检查可用的GPU数量
    if(is_cuda_available) {
        int device_count = torch::cuda::device_count();
        std::cout << "Available GPU count: " << device_count << std::endl;
        
        // 检查cuDNN是否可用
        bool cudnn_available = torch::cuda::cudnn_is_available();
        std::cout << "cuDNN available: " << (cudnn_available ? "Yes" : "No") << std::endl;
    }

    // 设置设备（如果CUDA可用则用GPU，否则用CPU）
    torch::Device device = is_cuda_available ? torch::kCUDA : torch::kCPU;
    std::cout << "Using device: " << (is_cuda_available ? "GPU" : "CPU") << std::endl;

    // 在选择的设备上创建张量
    torch::Tensor output = torch::randn({5, 3}, device);
    std::cout << "Tensor on " << (is_cuda_available ? "GPU:" : "CPU:") << std::endl;
    std::cout << output << std::endl;
}



// #include "detectIntersections.h"
// #include <iostream>
//
// int main()
// {
//     // 基本使用示例
//     T_MESH::Basic_TMesh* mesh = new T_MESH::Basic_TMesh();
//
//     // 创建6个顶点形成两个明显相交的三角形
//     T_MESH::Vertex* v1 = mesh->newVertex(0.0, 0.0, 0.0);
//     T_MESH::Vertex* v2 = mesh->newVertex(2.0, 2.0, 0.0); 
//     T_MESH::Vertex* v3 = mesh->newVertex(0.0, 2.0, 0.0);
//     
//     T_MESH::Vertex* v4 = mesh->newVertex(2.0, 0.0, 0.0);
//     T_MESH::Vertex* v5 = mesh->newVertex(0.0, 1.0, 0.0);
//     T_MESH::Vertex* v6 = mesh->newVertex(2.0, 1.0, 0.0);
//
//     // 创建第一个三角形 v1-v2-v3
//     T_MESH::Edge* e1 = mesh->CreateEdge(v1, v2);
//     T_MESH::Edge* e2 = mesh->CreateEdge(v2, v3);
//     T_MESH::Edge* e3 = mesh->CreateEdge(v3, v1);
//     T_MESH::Triangle* t1 = mesh->CreateTriangle(e1, e2, e3);
//
//     // 创建第二个三角形 v4-v5-v6 (横穿第一个三角形)
//     T_MESH::Edge* e4 = mesh->CreateEdge(v4, v5);
//     T_MESH::Edge* e5 = mesh->CreateEdge(v5, v6);
//     T_MESH::Edge* e6 = mesh->CreateEdge(v6, v4);
//     T_MESH::Triangle* t2 = mesh->CreateTriangle(e4, e5, e6);
//
//     // 检测自相交
//     int intersectingCount = mesh->selectIntersectingTriangles();
//
//     std::cout << intersectingCount << std::endl;
// }

// // 修复自相交
// bool success = mesh.strongIntersectionRemoval(10); // 最多10次迭代





















// #include <iostream>
// #include <Eigen/Dense>
// #include <OsiClpSolverInterface.hpp>
// #include <CoinPackedVector.hpp>
// #include <CoinPackedMatrix.hpp>
//
// int main() {
//     // 目标函数系数
//     Eigen::VectorXd obj_coeff(3);
//     obj_coeff << 48, 20, 8;
//     
//     // 约束矩阵
//     Eigen::MatrixXd constraint_mat(3, 3);
//     constraint_mat << 8, 4, 2,
//                       6, 2, 1.5,
//                       1, 1.5, 0.5;
//     
//     // 约束下界
//     Eigen::VectorXd lower_bounds(3);
//     lower_bounds << 600, 300, 200;
//     
//     // 创建求解器
//     OsiClpSolverInterface solver;
//     
//     // 设置问题维度
//     int num_cols = 3;  // x, y, z
//     int num_rows = 3;  // 约束数量
//     
//     // 变量下界 (>= 0)
//     double* col_lower = new double[num_cols];
//     double* col_upper = new double[num_cols];
//     for (int i = 0; i < num_cols; i++) {
//         col_lower[i] = 0.0;
//         col_upper[i] = solver.getInfinity();
//     }
//     
//     // 约束边界
//     double* row_lower = new double[num_rows];
//     double* row_upper = new double[num_rows];
//     for (int i = 0; i < num_rows; i++) {
//         row_lower[i] = lower_bounds(i);
//         row_upper[i] = solver.getInfinity();
//     }
//     
//     // 约束矩阵（压缩列格式）
//     CoinPackedMatrix matrix;
//     matrix.setDimensions(0, num_cols);
//     
//     for (int i = 0; i < num_rows; i++) {
//         CoinPackedVector row;
//         for (int j = 0; j < num_cols; j++) {
//             if (constraint_mat(i, j) != 0) {
//                 row.insert(j, constraint_mat(i, j));
//             }
//         }
//         matrix.appendRow(row);
//     }
//     
//     // 加载问题
//     solver.loadProblem(matrix, col_lower, col_upper, 
//                       obj_coeff.data(), row_lower, row_upper);
//     
//     // 求解
//     solver.initialSolve();
//     
//     // 输出结果
//     if (solver.isProvenOptimal()) {
//         std::cout << "最优目标值: " << solver.getObjValue() << std::endl;
//         const double* solution = solver.getColSolution();
//         std::cout << "最优解:" << std::endl;
//         for (int i = 0; i < num_cols; i++) {
//             std::cout << "x" << i+1 << " = " << solution[i] << std::endl;
//         }
//     } else {
//         std::cout << "未找到最优解" << std::endl;
//     }
//     
//     // 清理
//     delete[] col_lower;
//     delete[] col_upper;
//     delete[] row_lower;
//     delete[] row_upper;
//     
//     return 0;
// }