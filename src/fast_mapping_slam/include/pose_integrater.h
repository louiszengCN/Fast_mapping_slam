#ifndef POSE_FUSION_H
#define POSE_FUSION_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <ros/ros.h>
using namespace std;
class PoseFusion {
public:
    struct Pose {
        Eigen::Matrix3d rotation_matrix;
        Eigen::Vector3d translation_vector;
    };

    PoseFusion();

    vector<double> integratePoses(std::vector<double> pose1, std::vector<double> pose2, double time1, double time2);

    Pose double_2_pose(double x, double y, double theta);




private:
    double timestamp1_;
    double timestamp2_;
};

#endif // POSE_FUSION_H
