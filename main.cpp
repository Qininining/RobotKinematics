// #define NO_TEST
#define TEST1

#ifdef NO_TEST
#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
#endif

#ifdef TEST1
#include "KinematicsSolver.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip> // 用于 std::setprecision 和 std::fixed

// 定义 M_PI (如果编译器不提供)
#ifndef M_PI
#define M_PI (acos(-1.0))
#endif



// 辅助函数：打印 4x4 矩阵
void printMatrix(const cv::Matx44d& mat, const std::string& name) {
    std::cout << name << " = \n";
    std::cout << std::fixed << std::setprecision(5);
    for (int i = 0; i < 4; ++i) {
        std::cout << "[ ";
        for (int j = 0; j < 4; ++j) {
            std::cout << std::setw(10) << mat(i, j) << (j == 3 ? "" : ", ");
        }
        std::cout << " ]\n";
    }
    std::cout << std::endl;
}

// 辅助函数：比较两个矩阵是否近似相等
bool compareMatrices(const cv::Matx44d& m1, const cv::Matx44d& m2, double tol = 1e-5) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (std::abs(m1(i, j) - m2(i, j)) > tol) {
                std::cerr << "Matrix comparison failed at (" << i << "," << j << "): "
                          << m1(i,j) << " vs " << m2(i,j) << std::endl;
                return false;
            }
        }
    }
    return true;
}

// 辅助函数：打印关节向量
void printJointVector(const std::vector<double>& q, const std::string& name) {
    std::cout << name << " = { ";
    std::cout << std::fixed << std::setprecision(5);
    for (size_t i = 0; i < q.size(); ++i) {
        std::cout << q[i] << (i == q.size() - 1 ? "" : ", ");
    }
    std::cout << " }" << std::endl;
}

// 辅助函数：打印 6D 旋量 (Twist)
void printTwist(const cv::Vec6d& twist, const std::string& name) {
    std::cout << name << " = { ";
    std::cout << std::fixed << std::setprecision(5);
    for (int i = 0; i < 6; ++i) {
        std::cout << twist(i) << (i == 5 ? "" : ", ");
    }
    std::cout << " }" << std::endl;
}

// 辅助函数：比较两个 6D 旋量是否近似相等
bool compareTwists(const cv::Vec6d& t1, const cv::Vec6d& t2, double tol = 1e-5) {
    for (int i = 0; i < 6; ++i) {
        if (std::abs(t1(i) - t2(i)) > tol) {
            std::cerr << "Twist comparison failed at index " << i << ": "
                      << t1(i) << " vs " << t2(i) << std::endl;
            return false;
        }
    }
    return true;
}

// 辅助函数：打印 cv::Mat
void printCvMat(const cv::Mat& mat, const std::string& name) {
    std::cout << name << " = \n" << mat << std::endl;
}

// 新增辅助函数：比较两个 cv::Mat 矩阵
bool compareCvMats(const cv::Mat& m1, const cv::Mat& m2, double tol = 1e-5) {
    if (m1.size() != m2.size() || m1.type() != m2.type()) {
        std::cerr << "Matrix size or type mismatch!" << std::endl;
        return false;
    }
    for (int i = 0; i < m1.rows; ++i) {
        for (int j = 0; j < m1.cols; ++j) {
            if (std::abs(m1.at<double>(i, j) - m2.at<double>(i, j)) > tol) {
                std::cerr << "Matrix comparison failed at (" << i << "," << j << "): "
                          << m1.at<double>(i, j) << " vs " << m2.at<double>(i, j) << std::endl;
                return false;
            }
        }
    }
    return true;
}

// 测试1：单个 prismatic 关节 (沿 X 轴)
void test_single_prismatic_joint() {
    std::cout << "--- Test: Single Prismatic Joint (along X-axis) ---" << std::endl;
    std::vector<cv::Vec6d> screws;
    screws.push_back(cv::Vec6d(0, 0, 0, 1, 0, 0)); // omega=(0,0,0), v=(1,0,0)

    cv::Matx44d M_initial = cv::Matx44d::eye(); // 初始位姿为单位矩阵

    KinematicsSolver solver(screws, M_initial);

    std::vector<double> q = {2.5}; // 沿 X 轴移动 2.5 个单位

    cv::Matx44d T_expected = cv::Matx44d(
        1.0, 0.0, 0.0, 2.5,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
        );

    try {
        cv::Matx44d T_fk = solver.computeFK(q);
        printMatrix(T_fk, "Calculated FK");
        printMatrix(T_expected, "Expected FK");
        if (compareMatrices(T_fk, T_expected)) {
            std::cout << "Prismatic Joint Test: PASSED" << std::endl;
        } else {
            std::cout << "Prismatic Joint Test: FAILED" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception in Prismatic Joint Test: " << e.what() << std::endl;
    }
    std::cout << "-----------------------------------------" << std::endl << std::endl;
}

// 测试2：单个 revolute 关节 (绕 Z 轴)
void test_single_revolute_joint() {
    std::cout << "--- Test: Single Revolute Joint (about Z-axis) ---" << std::endl;
    std::vector<cv::Vec6d> screws;
    screws.push_back(cv::Vec6d(0, 0, 1, 0, 0, 0)); // omega=(0,0,1), v=(0,0,0)

    cv::Matx44d M_initial = cv::Matx44d::eye();
    M_initial(0, 3) = 2.0; // 初始末端执行器在 (2,0,0)

    KinematicsSolver solver(screws, M_initial);

    double theta = M_PI / 2.0; // 旋转 90 度
    std::vector<double> q = {theta};

    cv::Matx44d T_expected = cv::Matx44d(
        std::cos(theta), -std::sin(theta), 0.0, 0.0,
        std::sin(theta),  std::cos(theta), 0.0, 2.0,
        0.0,              0.0,             1.0, 0.0,
        0.0,              0.0,             0.0, 1.0
        );

    try {
        cv::Matx44d T_fk = solver.computeFK(q);
        printMatrix(T_fk, "Calculated FK");
        printMatrix(T_expected, "Expected FK");
        if (compareMatrices(T_fk, T_expected)) {
            std::cout << "Revolute Joint Test: PASSED" << std::endl;
        } else {
            std::cout << "Revolute Joint Test: FAILED" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception in Revolute Joint Test: " << e.what() << std::endl;
    }
    std::cout << "-----------------------------------------" << std::endl << std::endl;
}

// 测试3：2R 平面机械臂
void test_2R_planar_robot() {
    std::cout << "--- Test: 2R Planar Robot ---" << std::endl;
    double L1 = 1.0;
    double L2 = 1.0;

    std::vector<cv::Vec6d> screws;
    screws.push_back(cv::Vec6d(0, 0, 1, 0, 0, 0));
    screws.push_back(cv::Vec6d(0, 0, 1, 0, -L1, 0));

    cv::Matx44d M_initial = cv::Matx44d::eye();
    M_initial(0, 3) = L1 + L2;

    KinematicsSolver solver(screws, M_initial);

    std::vector<double> q1_vals = {M_PI / 2.0, 0.0};
    cv::Matx44d T_expected1 = cv::Matx44d(
        0.0, -1.0, 0.0, 0.0,
        1.0,  0.0, 0.0, L1 + L2,
        0.0,  0.0, 1.0, 0.0,
        0.0,  0.0, 0.0, 1.0
        );
    std::cout << "Subtest 3.1: q = {PI/2, 0}" << std::endl;
    try {
        cv::Matx44d T_fk1 = solver.computeFK(q1_vals);
        printMatrix(T_fk1, "Calculated FK (q1)");
        printMatrix(T_expected1, "Expected FK (q1)");
        if (compareMatrices(T_fk1, T_expected1)) {
            std::cout << "2R Test (q1): PASSED" << std::endl;
        } else {
            std::cout << "2R Test (q1): FAILED" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception in 2R Test (q1): " << e.what() << std::endl;
    }
    std::cout << std::endl;

    std::vector<double> q2_vals = {0.0, M_PI / 2.0};
    cv::Matx44d T_expected2 = cv::Matx44d(
        0.0, -1.0, 0.0, L1,
        1.0,  0.0, 0.0, L2,
        0.0,  0.0, 1.0, 0.0,
        0.0,  0.0, 0.0, 1.0
        );
    std::cout << "Subtest 3.2: q = {0, PI/2}" << std::endl;
    try {
        cv::Matx44d T_fk2 = solver.computeFK(q2_vals);
        printMatrix(T_fk2, "Calculated FK (q2)");
        printMatrix(T_expected2, "Expected FK (q2)");
        if (compareMatrices(T_fk2, T_expected2)) {
            std::cout << "2R Test (q2): PASSED" << std::endl;
        } else {
            std::cout << "2R Test (q2): FAILED" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception in 2R Test (q2): " << e.what() << std::endl;
    }
    std::cout << std::endl;

    std::vector<double> q3_vals = {M_PI / 2.0, M_PI / 2.0};
    cv::Matx44d T_expected3 = cv::Matx44d(
        -1.0,  0.0, 0.0, -L2,
        0.0, -1.0, 0.0,  L1,
        0.0,  0.0, 1.0,  0.0,
        0.0,  0.0, 0.0,  1.0
        );
    std::cout << "Subtest 3.3: q = {PI/2, PI/2}" << std::endl;
    try {
        cv::Matx44d T_fk3 = solver.computeFK(q3_vals);
        printMatrix(T_fk3, "Calculated FK (q3)");
        printMatrix(T_expected3, "Expected FK (q3)");
        if (compareMatrices(T_fk3, T_expected3)) {
            std::cout << "2R Test (q3): PASSED" << std::endl;
        } else {
            std::cout << "2R Test (q3): FAILED" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception in 2R Test (q3): " << e.what() << std::endl;
    }
    std::cout << "-----------------------------------------" << std::endl << std::endl;
}

void test_PPPRRR_robot() {
    std::cout << "--- Test: PPPRRR Robot ---" << std::endl;
    // std::vector<cv::Vec6d> screws;
    // screws.push_back(cv::Vec6d(0.0, 0.0, 0.0, 1.0, 0.0, 0.0));
    // screws.push_back(cv::Vec6d(0.0, 0.0, 0.0, 0.0, 1.0, 0.0));
    // screws.push_back(cv::Vec6d(0.0, 0.0, 0.0, 0.0, 0.0, 1.0));
    // screws.push_back(cv::Vec6d(0.0, 0.0, 1.0, 0.0, 0.0, 0.0));
    // screws.push_back(cv::Vec6d(0.0, 1.0, 0.0, 0.0, 0.0, 0.0));
    // screws.push_back(cv::Vec6d(1.0, 0.0, 0.0, 0.0, 0.0, 0.0));

    // cv::Matx44d M_initial = cv::Matx44d::eye();


    std::vector<cv::Vec6d> screw_axes = {
        {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},  // Shoulder (S1)

        {-0.00010966218804077, 0.999999860344549, 0.000516996214966776,
         -0.089201126124083, -9.73915983020756e-06, -8.28481817600509e-05},  // Upper Arm (S2)

        {-0.000851501411008793, 0.999999608724093, 0.000239785336181799,
         -0.0892010924995179, -0.000177878813172699, 0.425063054601736},  // Forearm (S3)

        {0.000645748395716926, 0.999977787178547, 0.00663386452268502,
         -0.0891645904058137, -0.00536369075116089, 0.817193278688037},  // Wrist 1 (S4)

        {1.07663873727645e-05, 0.0085250671036427, -0.999963660897216,
         -0.111191909169889, 0.817216260164118, 0.00696587945472968},  // Wrist 2 (S5)

        {0.000651627424651037, 0.999975340393563, 0.00699242350521426,
         0.005596450592457, -0.00571659008736621, 0.816998900818537}   // Wrist 3 (S6)
    };

    // UR5 的标准末端执行器初始位姿 M (与 ur5_kinematics_controller.cpp 中一致)
    cv::Matx44d M_home_ = cv::Matx44d(
        -0.999999787643124, -9.76765308370577e-06,  0.000651627424651037,  0.817145243199844,  // Row 1
        0.00065167979368323, -0.00699241862472029,  0.999975340393563,   0.193656522784246,  // Row 2
        -5.21096047671649e-06,  0.999975552694349,   0.00699242350521426,  -0.00424242678717729, // Row 3
        0.0,                  0.0,                  0.0,                  1.0                // Row 4
        );

    KinematicsSolver solver(screw_axes, M_home_);

    std::vector<double> q_vals = {0, 0, 0, 0, 0, 4};

    // cv::Matx44d T_expected = cv::Matx44d(
    //     0.0, -1.0,  0.0, 1.0,
    //     0.0,  0.0,  1.0, 0.5,
    //     -1.0,  0.0,  0.0, 0.2,
    //     0.0,  0.0,  0.0, 1.0
    //     );

    std::cout << "Test Case: q = {1.0, 0.5, 0.2, PI/2, PI/2, 0.0}" << std::endl;
    try {
        cv::Matx44d T_fk = solver.computeFK(q_vals);
        printMatrix(T_fk, "Calculated FK (PPPRRR)");
        // printMatrix(T_expected, "Expected FK (PPPRRR)");
        // if (compareMatrices(T_fk, T_expected)) {
        //     std::cout << "PPPRRR Test: PASSED" << std::endl;
        // } else {
        //     std::cout << "PPPRRR Test: FAILED" << std::endl;
        // }
    } catch (const std::exception& e) {
        std::cerr << "Exception in PPPRRR Test: " << e.what() << std::endl;
    }
    std::cout << "-----------------------------------------" << std::endl << std::endl;
}

void test_PPPRRR_robot_with_offset() {
    std::cout << "--- Test: PPPRRR Robot with Offset Axes ---" << std::endl;

    double L1 = 0.5;
    double L2 = 0.4;
    double L3 = 0.3;
    double L4 = 0.2;

    std::vector<cv::Vec6d> screws;
    screws.push_back(cv::Vec6d(0.0, 0.0, 0.0, 1.0, 0.0, 0.0));
    screws.push_back(cv::Vec6d(0.0, 0.0, 0.0, 0.0, 1.0, 0.0));
    screws.push_back(cv::Vec6d(0.0, 0.0, 0.0, 0.0, 0.0, 1.0));
    screws.push_back(cv::Vec6d(0.0, 0.0, 1.0, 0.0, -L1, 0.0));
    screws.push_back(cv::Vec6d(0.0, 1.0, 0.0, 0.0, 0.0, L1));
    screws.push_back(cv::Vec6d(1.0, 0.0, 0.0, 0.0, L3, -L2));

    cv::Matx44d M_initial = cv::Matx44d::eye();
    M_initial(0, 3) = L1;
    M_initial(1, 3) = L2;
    M_initial(2, 3) = L3 + L4;

    KinematicsSolver solver(screws, M_initial);

    double d1 = 0.1, d2 = 0.2, d3 = 0.3;
    double theta4 = M_PI / 2.0;
    double theta5 = M_PI / 2.0;
    double theta6 = 0.0;
    std::vector<double> q_vals = {d1, d2, d3, theta4, theta5, theta6};

    cv::Matx44d T_expected = cv::Matx44d(
         0.0, -1.0,  0.0,  0.2,
         0.0,  0.0,  1.0,  0.7,
        -1.0,  0.0,  0.0,  0.3,
         0.0,  0.0,  0.0,  1.0
    );

    std::cout << "Test Case: q = {" << d1 << ", " << d2 << ", " << d3 << ", PI/2, PI/2, " << theta6 << "}" << std::endl;
    std::cout << "L1=" << L1 << ", L2=" << L2 << ", L3=" << L3 << ", L4=" << L4 << std::endl;
    try {
        cv::Matx44d T_fk = solver.computeFK(q_vals);
        printMatrix(T_fk, "Calculated FK (PPPRRR Offset)");
        printMatrix(T_expected, "Expected FK (PPPRRR Offset)");
        if (compareMatrices(T_fk, T_expected)) {
            std::cout << "PPPRRR Offset Test: PASSED" << std::endl;
        } else {
            std::cout << "PPPRRR Offset Test: FAILED" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception in PPPRRR Offset Test: " << e.what() << std::endl;
    }
    std::cout << "-----------------------------------------" << std::endl << std::endl;
}

void test_IK_2R_planar_robot() {
    std::cout << "--- Test: IK for 2R Planar Robot ---" << std::endl;
    double L1 = 1.0;
    double L2 = 1.0;

    std::vector<cv::Vec6d> screws;
    screws.push_back(cv::Vec6d(0, 0, 1, 0, 0, 0));
    screws.push_back(cv::Vec6d(0, 0, 1, 0, -L1, 0));
    cv::Matx44d M_initial = cv::Matx44d::eye();
    M_initial(0, 3) = L1 + L2;
    KinematicsSolver solver(screws, M_initial);

    std::vector<double> q_target_fk = {M_PI / 4.0, M_PI / 3.0};
    cv::Matx44d T_target = solver.computeFK(q_target_fk);
    printMatrix(T_target, "Target Pose (from FK)");
    printJointVector(q_target_fk, "Target Joint Angles (for reference)");

    std::vector<double> q_initial_guess = {0.1, 0.1};
    printJointVector(q_initial_guess, "Initial Guess for IK");

    try {
        std::vector<double> q_ik_solution = solver.computeIK(T_target, q_initial_guess, 1e-6, 300);
        printJointVector(q_ik_solution, "Calculated IK Solution");

        cv::Matx44d T_achieved = solver.computeFK(q_ik_solution);
        printMatrix(T_achieved, "Achieved Pose (from IK solution)");

        if (compareMatrices(T_achieved, T_target, 1e-5)) {
            std::cout << "IK Test (2R Planar): PASSED (Achieved pose matches target pose)" << std::endl;
        } else {
            std::cout << "IK Test (2R Planar): FAILED (Achieved pose does NOT match target pose)" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception in IK Test (2R Planar): " << e.what() << std::endl;
    }
    std::cout << "-----------------------------------------" << std::endl << std::endl;
}

void test_IK_PPPRRR_robot() {
    std::cout << "--- Test: IK for PPPRRR Robot ---" << std::endl;
    std::vector<cv::Vec6d> screws;
    screws.push_back(cv::Vec6d(0.0, 0.0, 0.0, 1.0, 0.0, 0.0));
    screws.push_back(cv::Vec6d(0.0, 0.0, 0.0, 0.0, 1.0, 0.0));
    screws.push_back(cv::Vec6d(0.0, 0.0, 0.0, 0.0, 0.0, 1.0));
    screws.push_back(cv::Vec6d(0.0, 0.0, 1.0, 0.0, 0.0, 0.0));
    screws.push_back(cv::Vec6d(0.0, 1.0, 0.0, 0.0, 0.0, 0.0));
    screws.push_back(cv::Vec6d(1.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    cv::Matx44d M_initial = cv::Matx44d::eye();
    KinematicsSolver solver(screws, M_initial);

    std::vector<double> q_target_fk = {0.5, -0.2, 0.8, M_PI / 6.0, M_PI / 4.0, -M_PI / 3.0};
    cv::Matx44d T_target = solver.computeFK(q_target_fk);
    printMatrix(T_target, "Target Pose (PPPRRR, from FK)");
    printJointVector(q_target_fk, "Target Joint Angles (PPPRRR, for reference)");

    std::vector<double> q_initial_guess = {0.4, -0.1, 0.7, 0.5, 0.7, -0.5};
    printJointVector(q_initial_guess, "Initial Guess for IK (PPPRRR)");

    try {
        std::vector<double> q_ik_solution = solver.computeIK(T_target, q_initial_guess, 1e-6, 200);
        printJointVector(q_ik_solution, "Calculated IK Solution (PPPRRR)");

        cv::Matx44d T_achieved = solver.computeFK(q_ik_solution);
        printMatrix(T_achieved, "Achieved Pose (PPPRRR, from IK solution)");

        if (compareMatrices(T_achieved, T_target, 1e-5)) {
            std::cout << "IK Test (PPPRRR): PASSED (Achieved pose matches target pose)" << std::endl;
        } else {
            std::cout << "IK Test (PPPRRR): FAILED (Achieved pose does NOT match target pose)" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception in IK Test (PPPRRR): " << e.what() << std::endl;
    }
    std::cout << "-----------------------------------------" << std::endl << std::endl;
}

void test_IK_PPPRRR_robot_with_offset() {
    std::cout << "--- Test: IK for PPPRRR Robot with Offset Axes ---" << std::endl;
    double L1 = 0.5, L2 = 0.4, L3 = 0.3, L4 = 0.2;
    std::vector<cv::Vec6d> screws;
    screws.push_back(cv::Vec6d(0.0, 0.0, 0.0, 1.0, 0.0, 0.0));
    screws.push_back(cv::Vec6d(0.0, 0.0, 0.0, 0.0, 1.0, 0.0));
    screws.push_back(cv::Vec6d(0.0, 0.0, 0.0, 0.0, 0.0, 1.0));
    screws.push_back(cv::Vec6d(0.0, 0.0, 1.0, 0.0, -L1, 0.0));
    screws.push_back(cv::Vec6d(0.0, 1.0, 0.0, 0.0, 0.0, L1));
    screws.push_back(cv::Vec6d(1.0, 0.0, 0.0, 0.0, L3, -L2));
    cv::Matx44d M_initial = cv::Matx44d::eye();
    M_initial(0, 3) = L1; M_initial(1, 3) = L2; M_initial(2, 3) = L3 + L4;
    KinematicsSolver solver(screws, M_initial);

    std::vector<double> q_target_fk = {0.1, 0.15, 0.25, M_PI / 7.0, -M_PI / 5.0, M_PI / 8.0};
    cv::Matx44d T_target = solver.computeFK(q_target_fk);
    printMatrix(T_target, "Target Pose (PPPRRR Offset, from FK)");
    printJointVector(q_target_fk, "Target Joint Angles (PPPRRR Offset, for reference)");

    std::vector<double> q_initial_guess = {0.05, 0.1, 0.2, 0.4, -0.6, 0.3};
    printJointVector(q_initial_guess, "Initial Guess for IK (PPPRRR Offset)");

    try {
        std::vector<double> q_ik_solution = solver.computeIK(T_target, q_initial_guess, 1e-6, 200);
        printJointVector(q_ik_solution, "Calculated IK Solution (PPPRRR Offset)");

        cv::Matx44d T_achieved = solver.computeFK(q_ik_solution);
        printMatrix(T_achieved, "Achieved Pose (PPPRRR Offset, from IK solution)");

        if (compareMatrices(T_achieved, T_target, 1e-5)) {
            std::cout << "IK Test (PPPRRR Offset): PASSED (Achieved pose matches target pose)" << std::endl;
        } else {
            std::cout << "IK Test (PPPRRR Offset): FAILED (Achieved pose does NOT match target pose)" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception in IK Test (PPPRRR Offset): " << e.what() << std::endl;
    }
    std::cout << "-----------------------------------------" << std::endl << std::endl;
}

// 测试 calculateProductOfExponentials 和 computeJacobianSpace
void test_product_of_exponentials_and_jacobian() {
    std::cout << "--- Test: Product of Exponentials and Jacobian Space ---" << std::endl;

    // 使用与 UR5 相同的螺旋轴和初始位姿
    std::vector<cv::Vec6d> screw_axes = {
        {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},  // Shoulder (S1)
        {-0.00010966218804077, 0.999999860344549, 0.000516996214966776,
         -0.089201126124083, -9.73915983020756e-06, -8.28481817600509e-05},  // Upper Arm (S2)
        {-0.000851501411008793, 0.999999608724093, 0.000239785336181799,
         -0.0892010924995179, -0.000177878813172699, 0.425063054601736},  // Forearm (S3)
        {0.000645748395716926, 0.999977787178547, 0.00663386452268502,
         -0.0891645904058137, -0.00536369075116089, 0.817193278688037},  // Wrist 1 (S4)
        {1.07663873727645e-05, 0.0085250671036427, -0.999963660897216,
         -0.111191909169889, 0.817216260164118, 0.00696587945472968},  // Wrist 2 (S5)
        {0.000651627424651037, 0.999975340393563, 0.00699242350521426,
         0.005596450592457, -0.00571659008736621, 0.816998900818537}   // Wrist 3 (S6)
    };

    cv::Matx44d M_home_ = cv::Matx44d(
        -0.999999787643124, -9.76765308370577e-06,  0.000651627424651037,  0.817145243199844,
        0.00065167979368323, -0.00699241862472029,  0.999975340393563,   0.193656522784246,
        -5.21096047671649e-06,  0.999975552694349,   0.00699242350521426,  -0.00424242678717729,
        0.0,                  0.0,                  0.0,                  1.0
    );

    KinematicsSolver solver(screw_axes, M_home_);

    // 目标位姿：在 UR5 的初始位姿基础上，进行一个已知的变换
    double delta_x = 0.1, delta_y = 0.2, delta_z = 0.3;
    double delta_roll = M_PI / 18.0, delta_pitch = M_PI / 36.0, delta_yaw = M_PI / 72.0; // 注意：roll, pitch, yaw 当前未在 T_target 构建中使用

    // 修正 T_target 初始化以避免 C2679 错误
    // 注意：下方 transform1_val 和 transform2_val 使用逗号初始化的方式以及给定的数值序列，
    // 可能不会产生标准的平移或旋转矩阵。
    // 请确认这些矩阵是否代表了您预期的变换。
    cv::Matx44d transform1_val(
        delta_x, delta_y, delta_z, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0, 1.0
        );

    cv::Matx44d transform2_val(
        0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0, 1.0
        );
    cv::Matx44d T_target = M_home_ * transform1_val * transform2_val;

    // 初始关节角度：与 UR5 的标准姿态一致
    std::vector<double> q_initial = {0.0, -M_PI / 2.0, 0.0, 0.0, 0.0, 0.0};

    // 计算雅可比矩阵
    cv::Matx<double, 6, 6> J = solver.computeJacobianSpace(q_initial);

    // 计算末端执行器在目标位姿下的速度
    cv::Vec6d V_s_target(0.1, 0.1, 0.0, 0.0, 0.0, 0.0); // 期望的末端执行器速度
    std::vector<double> q_dot = solver.computeJointVelocities(V_s_target, q_initial, 0.01); // 计算关节速度

    // 使用雅可比矩阵验证关节速度与末端执行器速度的关系
    cv::Vec6d V_s_calculated = J * cv::Vec6d(q_dot[0], q_dot[1], q_dot[2], q_dot[3], q_dot[4], q_dot[5]);

    std::cout << "Target End Effector Velocity (V_s_target): " << V_s_target.t() << std::endl;
    std::cout << "Calculated End Effector Velocity (V_s_calculated): " << V_s_calculated.t() << std::endl;

    // 检查 V_s_calculated 是否与 V_s_target 近似相等
    if (compareTwists(V_s_target, V_s_calculated, 1e-5)) {
        std::cout << "Product of Exponentials and Jacobian Test: PASSED" << std::endl;
    } else {
        std::cout << "Product of Exponentials and Jacobian Test: FAILED" << std::endl;
    }

    std::cout << "----------------------------------------------------------" << std::endl << std::endl;
}

// 新增测试函数：测试关节速度和末端执行器速度计算 (UR5 参数)
void test_joint_and_effector_velocities_UR5() {
    std::cout << "--- Test: Joint and Effector Velocities (UR5 Parameters) ---" << std::endl;

    std::vector<cv::Vec6d> screw_axes = {
        {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},  // Shoulder (S1)
        {-0.00010966218804077, 0.999999860344549, 0.000516996214966776,
         -0.089201126124083, -9.73915983020756e-06, -8.28481817600509e-05},  // Upper Arm (S2)
        {-0.000851501411008793, 0.999999608724093, 0.000239785336181799,
         -0.0892010924995179, -0.000177878813172699, 0.425063054601736},  // Forearm (S3)
        {0.000645748395716926, 0.999977787178547, 0.00663386452268502,
         -0.0891645904058137, -0.00536369075116089, 0.817193278688037},  // Wrist 1 (S4)
        {1.07663873727645e-05, 0.0085250671036427, -0.999963660897216,
         -0.111191909169889, 0.817216260164118, 0.00696587945472968},  // Wrist 2 (S5)
        {0.000651627424651037, 0.999975340393563, 0.00699242350521426,
         0.005596450592457, -0.00571659008736621, 0.816998900818537}   // Wrist 3 (S6)
    };

    cv::Matx44d M_home_ = cv::Matx44d(
        -0.999999787643124, -9.76765308370577e-06,  0.000651627424651037,  0.817145243199844,
        0.00065167979368323, -0.00699241862472029,  0.999975340393563,   0.193656522784246,
        -5.21096047671649e-06,  0.999975552694349,   0.00699242350521426,  -0.00424242678717729,
        0.0,                  0.0,                  0.0,                  1.0
    );

    KinematicsSolver solver(screw_axes, M_home_);

    std::vector<double> q_current = {-0.272, -2.131, 1.562, -1.033, 4.582, 1.159}; // 示例关节角度
    printJointVector(q_current, "Current Joint Angles (q_current)");

    cv::Vec6d V_s_desired(0, 0.000, 1, 0.000, 0.000, 0.000); // 示例期望空间速度
    printTwist(V_s_desired, "Desired End Effector Velocity (V_s_desired)");

    double lambda_dls = 0.00; // DLS 阻尼因子

    try {
        // 1. 计算关节速度 q_dot
        std::vector<double> q_dot_calculated = solver.computeJointVelocities(V_s_desired, q_current, lambda_dls);
        printJointVector(q_dot_calculated, "Calculated Joint Velocities (q_dot_calculated)");

        // 2. 使用计算出的 q_dot 重新计算末端执行器速度
        //-0.683, -1.859, 0.960, 0.433, 0.004, -0.438
        // 定义q_dot_calculated_new = -0.683, -1.859, 0.960, 0.433, 0.004, -0.438
        // std::vector<double> q_dot_calculated_new = {-0.683, -1.859, 0.960, 0.433, 0.004, -0.438};

        cv::Vec6d V_s_recalculated = solver.computeEndEffectorVelocity(q_current, q_dot_calculated);
        printTwist(V_s_recalculated, "Recalculated End Effector Velocity (V_s_recalculated)");

        // 3. 比较 V_s_desired 和 V_s_recalculated
        // 由于 DLS 的存在，两者可能不会完全相等，尤其是在奇异点附近或 lambda 较大时。
        // 容差可能需要根据 lambda 的值和机器人的具体配置进行调整。
        double comparison_tolerance = 1e-4;
        if (compareTwists(V_s_desired, V_s_recalculated, comparison_tolerance)) {
            std::cout << "Joint/Effector Velocities Test (UR5): PASSED" << std::endl;
        } else {
            std::cout << "Joint/Effector Velocities Test (UR5): FAILED" << std::endl;
            std::cout << "  Note: Discrepancy might be due to DLS damping (lambda=" << lambda_dls << ")." << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception in Joint/Effector Velocities Test (UR5): " << e.what() << std::endl;
    }
    std::cout << "----------------------------------------------------------" << std::endl << std::endl;
}


int main(int argc, char *argv[])
{
    // FK Tests
    // test_single_prismatic_joint();
    // test_single_revolute_joint();
    // test_2R_planar_robot();
    // test_PPPRRR_robot();
    // test_PPPRRR_robot_with_offset();

    // IK Tests
    // test_IK_2R_planar_robot();
    // test_IK_PPPRRR_robot();
    // test_IK_PPPRRR_robot_with_offset();

    // Product of Exponentials and Jacobian Test
    // test_product_of_exponentials_and_jacobian();

    // 新增测试调用
    test_joint_and_effector_velocities_UR5();

    std::cout << "All tests finished." << std::endl;
    return 0;
}
#endif




