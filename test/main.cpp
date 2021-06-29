//
// Created by xuqw on 12/31/19.
//

#include "axxbsolver.h"
#include "conventionalaxxbsvdsolver.h"
#include "extendedaxxbelilambdasvdsolver.h"

#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/SVD>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <unsupported/Eigen/KroneckerProduct>
#include <axxb_utils.h>

//IMPORTANT: THE TRANSFORMATION SHOULD COVER ALL KIND OF MOTION

// return is pose or transformation? transformation
Poses ReadPoses(int start_frame, int end_frame, const std::string &path2poses);

Eigen::Matrix3d SolveR(Poses As, Poses Bs);
Eigen::Matrix3d SolveRConv(Poses As, Poses Bs);

int main(int argc, char **argv) {
    Poses As, Bs;

    // read file for As
    Poses As_rpy1 = ReadPoses(1, 181, std::string(argv[1]));
    Poses As_rpy2 = ReadPoses(21, 121, std::string(argv[2]));
    Poses As_zpitch1 = ReadPoses(21, 121, std::string(argv[3]));
    Poses As_zpitch2 = ReadPoses(21, 101, std::string(argv[4]));
    Poses As_zroll2 = ReadPoses(41, 161, std::string(argv[5]));
    Poses As_zroll3 = ReadPoses(41, 121, std::string(argv[6]));

    // read file for Bs
    Poses Bs_rpy1 = ReadPoses(1, 181, std::string(argv[7]));
    Poses Bs_rpy2 = ReadPoses(21, 121, std::string(argv[8]));
    Poses Bs_zpitch1 = ReadPoses(21, 121, std::string(argv[9]));
    Poses Bs_zpitch2 = ReadPoses(21, 101, std::string(argv[10]));
    Poses Bs_zroll2 = ReadPoses(41, 161, std::string(argv[11]));
    Poses Bs_zroll3 = ReadPoses(41, 121, std::string(argv[12]));

    for (int i = 1; i < As_rpy1.size(); ++i) {
        As.push_back(As_rpy1[i]);
        Bs.push_back(Bs_rpy1[i]);
    }

    for (int i = 1; i < As_rpy2.size(); ++i) {
        As.push_back(As_rpy2[i]);
        Bs.push_back(Bs_rpy2[i]);
    }

    for (int i = 1; i < As_zpitch1.size(); ++i) {
        As.push_back(As_zpitch1[i]);
        Bs.push_back(Bs_zpitch1[i]);
    }

    for (int i = 1; i < As_zpitch2.size(); ++i) {
        As.push_back(As_zpitch2[i]);
        Bs.push_back(Bs_zpitch2[i]);
    }

    for (int i = 1; i < As_zroll2.size(); ++i) {
        As.push_back(As_zroll2[i]);
        Bs.push_back(Bs_zroll2[i]);
    }

    for (int i = 1; i < As_zroll3.size(); ++i) {
        As.push_back(As_zroll3[i]);
        Bs.push_back(Bs_zroll3[i]);
    }

    // simulated As and Bs
//    Eigen::Matrix3d X = Eigen::AngleAxisd(M_PI/2., Eigen::Vector3d::UnitZ()).toRotationMatrix() *
//                        Eigen::AngleAxisd(0., Eigen::Vector3d::UnitY()).toRotationMatrix() *
//                        Eigen::AngleAxisd(0., Eigen::Vector3d::UnitX()).toRotationMatrix();
//    double yaw = M_PI;
//    double pitch = 0.;
//    double roll = M_PI/3;
//    for (int i = 0; i < 2; ++i) {
//        pitch = i*M_PI/1000.;
//        Eigen::Matrix3d Ai = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
//                             Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
//                             Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
//        std::cout << "Ai = " << Ai << std::endl;
//        std::cout << "Bi = " << Ai * X << std::endl;
//        pitch = (i+1)*M_PI/1000.;
//        Eigen::Matrix3d Aj = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
//                             Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
//                             Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
//        std::cout << "Aj = " << Aj << std::endl;
//        std::cout << "Bj = " << Aj * X << std::endl;
//        Pose Aij = Pose::Identity();
//        Pose Bij = Pose::Identity();
//        Aij.block(0, 0, 3, 3) = Ai.inverse() * Aj;
//        Bij.block(0, 0, 3, 3) = (Ai*X).inverse() * (Aj*X);
//        As.push_back(Aij);
//        Bs.push_back(Bij);
//    }
//
//    for (int i = 0; i < 2; ++i) {
//        yaw = M_PI - i*M_PI/1000.;
//        Eigen::Matrix3d Ai = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
//                             Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
//                             Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
//        std::cout << "Ai = " << Ai << std::endl;
//        std::cout << "Bi = " << Ai * X << std::endl;
//        yaw = M_PI - (i+1)*M_PI/1000.;
//        Eigen::Matrix3d Aj = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
//                             Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
//                             Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
//        std::cout << "Aj = " << Aj << std::endl;
//        std::cout << "Bj = " << Aj * X << std::endl;
//        Pose Aij = Pose::Identity();
//        Pose Bij = Pose::Identity();
//        Aij.block(0, 0, 3, 3) = Ai.inverse() * Aj;
//        Bij.block(0, 0, 3, 3) = (Ai*X).inverse() * (Aj*X);
//        As.push_back(Aij);
//        Bs.push_back(Bij);
//    }
//
//    for (int i = 0; i < 2; ++i) {
//        roll = M_PI/3 + i*M_PI/1000.;
//        Eigen::Matrix3d Ai = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
//                             Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
//                             Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
//        std::cout << "Ai = " << Ai << std::endl;
//        std::cout << "Bi = " << Ai * X << std::endl;
//        roll = M_PI/3 + (i+1)*M_PI/1000.;
//        Eigen::Matrix3d Aj = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
//                             Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
//                             Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
//        std::cout << "Aj = " << Aj << std::endl;
//        std::cout << "Bj = " << Aj * X << std::endl;
//        Pose Aij = Pose::Identity();
//        Pose Bij = Pose::Identity();
//        Aij.block(0, 0, 3, 3) = Ai.inverse() * Aj;
//        Bij.block(0, 0, 3, 3) = (Ai*X).inverse() * (Aj*X);
//        As.push_back(Aij);
//        Bs.push_back(Bij);
//    }

    std::cout << "As[0] is " << As[0] << std::endl;
    std::cout << "Bs[0] is " << Bs[0] << std::endl;
    std::cout << "As size is " << As.size() << ", " << Bs.size() << std::endl;

//    ConventionalAXXBSVDSolver solver(As, Bs);
//    Pose T = solver.SolveX();
    Eigen::Matrix3d R = SolveR(As, Bs);
    for (int i = 0; i < 3; ++i) {
        R.col(i) = R.col(i) / R.col(i).norm();
    }

    std::cout << "R is " << R << std::endl;

    for (int j = 0; j < As.size(); ++j) {
        Eigen::Matrix3d Aij = As[j].block(0, 0, 3, 3);
        Eigen::Matrix3d Bij = Bs[j].block(0, 0, 3, 3);
        std::cout << "norm(Aij*R - R*Bij) =  " << (Aij*R - R*Bij).norm() << std::endl;
    }
    return 0;
}

Poses ReadPoses(int start_frame, int end_frame, const std::string &path2poses) {
    Poses poses;
    std::ifstream fposes;
    fposes.open(path2poses.c_str());
    std::string line;

    int cnt = start_frame;
    Eigen::Matrix3d last_R = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d curr_R = Eigen::Matrix3d::Identity();
    while (std::getline(fposes, line)) {
        if(cnt < end_frame){
            std::istringstream iss(line);
            double yaw, pitch, roll;
            if (!(iss >> yaw >> pitch >> roll))
                break;
            Pose pose = Pose::Identity();
            curr_R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
            Eigen::Matrix3d R = last_R.inverse() * curr_R;
            pose.block(0,0,3,3) = R;
            poses.push_back(pose);
            last_R = curr_R;
            ++cnt;
        }
    }
    return poses;
}

Eigen::Matrix3d SolveR(Poses As, Poses Bs){
    // reference: http://www.cs.jhu.edu/~sleonard/lecture03.pdf
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    for (int i = 0; i < As.size(); ++i) {
        Eigen::Matrix3d Ra = As[i].topLeftCorner(3,3);
        double theta_a = acos((Ra.trace()-1)/2);
        Eigen::Matrix3d log_Ra = theta_a/(2*sin(theta_a)) * (Ra - Ra.transpose());
        Eigen::Vector3d ra = Eigen::Vector3d(log_Ra(2,1), log_Ra(0, 2), log_Ra(1,0));

        Eigen::Matrix3d Rb = Bs[i].topLeftCorner(3,3);
        double theta_b = acos((Rb.trace()-1)/2);
        Eigen::Matrix3d log_Rb = theta_b/(2*sin(theta_b)) * (Rb - Rb.transpose());
        Eigen::Vector3d rb = Eigen::Vector3d(log_Rb(2,1), log_Rb(0, 2), log_Rb(1,0));

        M += rb * ra.transpose();
    }

    Eigen::Matrix3d R = (M.transpose()*M).inverse().sqrt()*M.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(R, Eigen::ComputeThinU | Eigen::ComputeThinV);
    auto U = svd.matrixU();
    auto V = svd.matrixV();

    return U *V.transpose();
}

Eigen::Matrix3d SolveRConv(Poses As, Poses Bs){
    Eigen::MatrixXd m = Eigen::MatrixXd::Zero(9*As.size(),9);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(9*As.size());
    for(int i=0;i<As.size();i++)
    {
        //extract R,t from homogophy matrix
        Eigen::Matrix3d Ra = As[i].topLeftCorner(3,3);
        Eigen::Matrix3d Rb = Bs[i].topLeftCorner(3,3);

        m.block<9,9>(9*i,0) = Eigen::MatrixXd::Identity(9,9) - Eigen::kroneckerProduct(Ra,Rb);
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(m, Eigen::ComputeThinV | Eigen::ComputeThinU );

//    Eigen::Matrix<double, 9, 1> x = m.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    Eigen::Matrix<double, 9, 1> x = svd.matrixV().block<9, 1>(0, 8);
    Eigen::Matrix3d R = Eigen::Map< Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(x.data()); //row major

    return R;
}