#include "../include/pose_integrater.h"
PoseFusion::PoseFusion() {
}


PoseFusion::Pose PoseFusion::double_2_pose(double x, double y, double theta){

    PoseFusion::Pose pose1;
    pose1.rotation_matrix = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose1.translation_vector << x, y, 0.0;
    return pose1;

}

vector<double> PoseFusion::integratePoses(vector<double> pose1, vector<double> pose2, std::chrono::steady_clock::time_point timestamp1, std::chrono::steady_clock::time_point timestamp2) {
    Pose fused_pose;
    Pose pose_front = double_2_pose(pose1[0], pose1[1], pose1[2]);
    Pose pose_backend = double_2_pose(pose2[0], pose2[1], pose2[2]);
    //由时间戳计算权重
    std::chrono::steady_clock::time_point current_timestamp = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used_1 = std::chrono::duration_cast<std::chrono::duration<double>>(current_timestamp - timestamp1);
    std::chrono::duration<double> time_used_2 = std::chrono::duration_cast<std::chrono::duration<double>>(timestamp2 - timestamp1);
    double alpha = time_used_1.count() / time_used_2.count();
    // 旋转部分使用球面线性插值(Slerp)计算
    Eigen::Quaterniond q1(pose_front.rotation_matrix);
    Eigen::Quaterniond q2(pose_backend.rotation_matrix);
    Eigen::Quaterniond interpolated_rotation = q1.slerp(alpha, q2);
    fused_pose.rotation_matrix = interpolated_rotation.toRotationMatrix();

    // 平移部分使用线性插值计算
    fused_pose.translation_vector = pose_front.translation_vector + alpha * (pose_backend.translation_vector - pose_front.translation_vector);

    vector<double> pose_fuse;
    

    // 获取平移向量的 x 和 y 分量
    pose_fuse.push_back(fused_pose.translation_vector[0]); // x
    pose_fuse.push_back(fused_pose.translation_vector[1]); // y

    // 获取旋转矩阵的旋转角度 theta
    double theta = std::atan2(fused_pose.rotation_matrix(1, 0), fused_pose.rotation_matrix(0, 0));
    pose_fuse.push_back(theta);

    return pose_fuse;

}
