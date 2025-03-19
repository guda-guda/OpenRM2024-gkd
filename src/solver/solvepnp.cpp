#include "solver/solvepnp.h"
#include "solver/ternary.hpp"
#include "utils/timer.h"
#include "uniterm/uniterm.h"
#include "structure/slidestd.hpp"
#include <cmath>
using namespace rm;
using namespace std;

static double ANGLE_COST_RATIO = 4.0;

//CHANGELOG
//移除Camera* camera项，直接设置intrinsic_matrix、distortion_coeffs、Rotate_pnp2head、Trans_pnp2head
double rm::solveYawPnP(
    const double yaw,
    cv::Mat intrinsic_matrix,
    cv::Mat distortion_coeffs,
    Eigen::Matrix4d Trans_pnp2head,
    Eigen::Matrix3d Rotate_pnp2head,
    Eigen::Vector4d& ret_pose,
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points,
    const Eigen::Matrix3d& rotate_head2world,
    const Eigen::Matrix4d& trans_head2world,
    rm::ArmorID armor_id,
    bool display_flag
) {
    ret_pose = Eigen::Vector4d(0, 0, 0, 1);
    // if (camera == nullptr) return 0.0;
    YawPnP* yaw_pnp = new YawPnP();

    // 设置yaw
    yaw_pnp->sys_yaw = yaw;

    // 设置装甲板四点坐标
    yaw_pnp->setWorldPoints(object_points);
    yaw_pnp->setImagePoints(image_points);

    cv::Mat rvec, tvec, rotate_cv;
    Eigen::Vector4d pose_pnp;
    Eigen::Matrix3d rotate_pnp, rotate_world;

    // 使用OpenCV求解PnP
    cv::solvePnP(object_points, image_points, 
                 intrinsic_matrix, distortion_coeffs, 
                 rvec, tvec, false, cv::SOLVEPNP_IPPE);

    //DEBUG 调换tvec坐标系  (看起来是对的)
    double pose0 = tvec.at<double>(2);
    double pose1 = tvec.at<double>(0);
    double pose2 = -tvec.at<double>(1);
    tvec.at<double>(0) = pose0;
    tvec.at<double>(1) = pose1;
    tvec.at<double>(2) = pose2;

    //DEBUG 调换rvec坐标系
    cv::Mat R = (cv::Mat_<double>(3,3) <<
    0,  0,  1,
   -1,  0,  0,
    0, -1,  0);
    cv::Mat oldR;
    cv::Rodrigues(rvec, oldR);
    cv::Mat newR = R * oldR;
    cv::Rodrigues(newR, rvec);


    // 计算装甲板位姿，确定返回值
    /*NEED DEBUG*/
    Eigen::Matrix4d trans_pnp2head = Trans_pnp2head;
    yaw_pnp->T = trans_head2world * trans_pnp2head;
    yaw_pnp->T_inv = yaw_pnp->T.inverse();
    
    //先不在这里做坐标系转换，而是直接输出相对相机的位置
    rm::tf_Vec4d(tvec, pose_pnp);
    // ret_pose = yaw_pnp->T * pose_pnp;
    ret_pose = pose_pnp;
    yaw_pnp->pose = ret_pose;

    //DEBUG mm -> m
    // double npose1 = ret_pose[0] / 1000;
    // double npose2 = ret_pose[1] / 1000;
    // double npose3 = ret_pose[2] / 1000;
    // ret_pose[0] = npose1;
    // ret_pose[1] = npose2;
    // ret_pose[2] = npose3;

    
    if(false)
    {
        std::cout << "trans_pnp2head->" << trans_pnp2head << std::endl;
        std::cout << "trans_head2world->" << trans_head2world << std::endl;
        std::cout << "yaw_pnp->T->" << yaw_pnp->T << std::endl;
        std::cout << "yaw_pnp->T_inv->" << yaw_pnp->T_inv << std::endl;
        
        std::cout << "tvec->" << tvec << std::endl;
        std::cout << "pose_pnp->" << pose_pnp << std::endl;
        std::cout << "ret_pose->" << ret_pose << std::endl;
        
        std::cout << "--------------------------------------" << std::endl;
    }

    

    // 计算装甲板仰角
    Eigen::Matrix3d rotate_pnp2head = Rotate_pnp2head;
    cv::Rodrigues(rvec, rotate_cv);
    rm::tf_Mat3d(rotate_cv, rotate_pnp);
    rotate_world = rotate_head2world * rotate_pnp2head * rotate_pnp;
    double armor_pitch_pnp = rm::tf_rotation2armorpitch(rotate_world);
    double armor_yaw_pnp = rm::tf_rotation2armoryaw(rotate_world);

    // 设置装甲板仰角
    if (armor_id == rm::ARMOR_ID_UNKNOWN) {
        yaw_pnp->setElevation(armor_pitch_pnp);
    } else {
        yaw_pnp->setElevation(armor_id);
    }

    // 设置相机内参
    tf_Mat3f(intrinsic_matrix, yaw_pnp->Kc);

    if (display_flag) {
        displayYawPnP(yaw_pnp);
    }

    // 求解yaw
    double angle_yaw = yaw_pnp->getYawByAngleCost(-(M_PI / 2), (M_PI / 2), 0.03);
    double pixel_yaw = yaw_pnp->getYawByPixelCost(-(M_PI / 2), (M_PI / 2), 0.03);
    double append_yaw = yaw_pnp->getYawByMix(pixel_yaw, angle_yaw);

    delete yaw_pnp;
    return append_yaw + yaw;
}

void rm::displayYawPnP(YawPnP* yaw_pnp) {
    cv::Mat img_cost(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));

    double step = M_PI / 250;
    double max = -1.0, min = 1e10;
    vector<double> pixel_cost_list(250);
    vector<double> angle_cost_list(250);
    vector<double> cost_list(250);

    for (int i = 0; i < 250; i++) {
        double app_yaw = i * step - M_PI / 2;

        pixel_cost_list[i] = yaw_pnp->getPixelCost(app_yaw);
        angle_cost_list[i] = yaw_pnp->getAngleCost(app_yaw);

        max = std::max(max, pixel_cost_list[i]);
        min = std::min(min, pixel_cost_list[i]);

        max = std::max(max, angle_cost_list[i]);
        min = std::min(min, angle_cost_list[i]);
    }

    for (int i = 0; i < 250; i++) {
        int pixel_show_cost = (pixel_cost_list[i] - min) / (max - min) * 500;
        int angle_show_cost = (angle_cost_list[i] - min) / (max - min) * 500;
        int cost_show = (cost_list[i] - min) / (max - min) * 500;

        cv::circle(img_cost, cv::Point(i * 2, 499 - pixel_show_cost), 1, cv::Scalar(0, 255, 255), 2);
        cv::circle(img_cost, cv::Point(i * 2, 499 - angle_show_cost), 1, cv::Scalar(255, 255, 0), 2);
    }

    double angle_yaw = yaw_pnp->getYawByAngleCost(-(M_PI / 2), (M_PI / 2), 0.03);
    double pixel_yaw = yaw_pnp->getYawByPixelCost(-(M_PI / 2), (M_PI / 2), 0.03);
    double append_yaw = yaw_pnp->getYawByMix(pixel_yaw, angle_yaw);

    cv::line(img_cost, 
        cv::Point((append_yaw + M_PI / 2) / M_PI * 500, 0), 
        cv::Point((append_yaw + M_PI / 2) / M_PI * 500, 500), 
        cv::Scalar(255, 255, 255), 2);

    switch (yaw_pnp->elevation) {
        case rm::ARMOR_ELEVATION_UP_15:
            cv::putText(img_cost, "UP15", cv::Point(20, 30), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(255, 150, 0), 1);
            break;
        case rm::ARMOR_ELEVATION_UP_75:
            cv::putText(img_cost, "UP75", cv::Point(20, 30), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(255, 150, 0), 1);
            break;
        case rm::ARMOR_ELEVATION_DOWN_15:
            cv::putText(img_cost, "DW15", cv::Point(20, 30), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(255, 150, 0), 1);
            break;
    }

    for (int i = 1; i <= 7; i++) {
        int left_x = 250 - 250 * 0.2 * i / M_PI;
        int right_x = 250 + 250 * 0.2 * i / M_PI;
        cv::line(img_cost, cv::Point(left_x, 486), cv::Point(left_x, 499), cv::Scalar(255, 255, 255), 1);
        cv::line(img_cost, cv::Point(right_x, 486), cv::Point(right_x, 499), cv::Scalar(255, 255, 255), 1);
    }
    cv::line(img_cost, cv::Point(250, 483), cv::Point(250, 499), cv::Scalar(255, 255, 255), 2);

    
    cv::imshow("cost", img_cost);
    cv::waitKey(1);
}

void YawPnP::setWorldPoints(const std::vector<cv::Point3f>& object_points) { 
    P_world.clear();
    for (const auto& p : object_points) {
        P_world.push_back(Eigen::Vector4d(0, -(p.x * 1e-3), -(p.y * 1e-3), 1));
    }
}

void YawPnP::setImagePoints(const std::vector<cv::Point2f>& image_points) { 
    P_pixel.clear();
    for (const auto& p : image_points) {
        P_pixel.push_back(Eigen::Vector2d(p.x, p.y));
    }
}

double YawPnP::operator()(double append_yaw) const {
    std::vector<Eigen::Vector4d> P_mapping = getMapping(append_yaw);
    std::vector<Eigen::Vector2d> P_project = getProject(P_mapping);
    double cost = getCost(P_project, append_yaw);
    return cost;
};

ArmorElevation YawPnP::setElevation(rm::ArmorID armor_id) {
    ArmorElevation elevation;
    switch (armor_id) {
        case rm::ARMOR_ID_SENTRY:
        case rm::ARMOR_ID_HERO:
        case rm::ARMOR_ID_ENGINEER:
        case rm::ARMOR_ID_INFANTRY_3:
        case rm::ARMOR_ID_INFANTRY_4:
        case rm::ARMOR_ID_INFANTRY_5:
            elevation = ARMOR_ELEVATION_UP_15;
            break;
        case rm::ARMOR_ID_TOWER:
            elevation = ARMOR_ELEVATION_DOWN_15;
            break;
        default:
            elevation = ARMOR_ELEVATION_UP_15;
            break;
    }
    this->elevation = elevation;
    return elevation;
}

ArmorElevation YawPnP::setElevation(double pitch) {
    ArmorElevation elevation;
    if (pitch > ANGLE_BOUNDARY_UP) {
        elevation = ARMOR_ELEVATION_UP_75;
    } else if (pitch < ANGLE_BOUNDARY_DOWN) {
        elevation = ARMOR_ELEVATION_DOWN_15;
    } else {
        elevation = ARMOR_ELEVATION_UP_15;
    }
    this->elevation = elevation;
    return elevation;
}

std::vector<Eigen::Vector4d> YawPnP::getMapping(double append_yaw) const {
    Eigen::Matrix4d M;
    std::vector<Eigen::Vector4d> P_mapping;

    double yaw = sys_yaw + append_yaw;
    double pitch;
    switch(elevation) {
        case ARMOR_ELEVATION_UP_15:
            pitch = ANGLE_UP_15;
            break;
        case ARMOR_ELEVATION_UP_75:
            pitch = ANGLE_UP_75;
            break;
        case ARMOR_ELEVATION_DOWN_15:
            pitch = ANGLE_DOWN_15;
            break;
        default:
            pitch = 0;
            break;
    }

    pitch = -pitch;
    M << cos(yaw) * cos(pitch), -sin(yaw), -sin(pitch) * cos(yaw), pose(0),
         sin(yaw) * cos(pitch),  cos(yaw), -sin(pitch) * sin(yaw), pose(1),
                    sin(pitch),         0,             cos(pitch), pose(2),
                             0,         0,                      0,       1;
    
    for (const auto& p : P_world) {
        P_mapping.push_back(M * p);
    }
    return P_mapping;
}

std::vector<Eigen::Vector2d> YawPnP::getProject(const std::vector<Eigen::Vector4d>& P_world) const {
    std::vector<Eigen::Vector2d> P_project;
    for (const auto& p : P_world) {
        Eigen::Vector3d p_camera = (T_inv * p).head(3);
        Eigen::Vector3d p_project = Kc * p_camera;
        P_project.push_back(p_project.head(2) / p_camera(2));
    }
    return P_project;
}

double YawPnP::getCost(const std::vector<Eigen::Vector2d>& P_project, double append_yaw) const {
    if (P_pixel.size() != P_project.size() || P_project.size() < 4) return 0.0;
    
    int map[4] = {0, 1, 3, 2};

    double cost = 0.0;
    for (int i = 0; i < 4; i++) {
        int index_this = map[i];
        int index_next = map[(i + 1) % 4];
        Eigen::Vector2d pixel_line = P_pixel[index_next] - P_pixel[index_this];
        Eigen::Vector2d project_line = P_project[index_next] - P_project[index_this];

        double this_dist = (P_pixel[index_this] - P_project[index_this]).norm();
        double next_dist = (P_pixel[index_next] - P_project[index_next]).norm();
        double line_dist = fabs(pixel_line.norm() - project_line.norm());

        double pixel_dist = (0.5 * (this_dist + next_dist) + line_dist) / pixel_line.norm();


        double cos_angle = pixel_line.dot(project_line) / (pixel_line.norm() * project_line.norm());
        double angle_dist = fabs(acos(cos_angle)) * ANGLE_COST_RATIO;

        double ratio = fabs((1 - exp(-append_yaw)) / (1 + exp(-append_yaw)));
        double cost_i = pow(pixel_dist * ratio, 2) + pow(angle_dist * (1 - ratio), 2);
        cost += sqrt(cost_i);
    }

    return cost; 
}

double YawPnP::getPixelCost(const std::vector<Eigen::Vector2d>& P_project, double append_yaw) const {
    if (P_pixel.size() != P_project.size() || P_project.size() < 4) return 0.0;

    int map[4] = {0, 1, 3, 2};

    double cost = 0.0;
    for (int i = 0; i < 4; i++) {
        int index_this = map[i];
        int index_next = map[(i + 1) % 4];
        Eigen::Vector2d pixel_line = P_pixel[index_next] - P_pixel[index_this];
        Eigen::Vector2d project_line = P_project[index_next] - P_project[index_this];

        double this_dist = (P_pixel[index_this] - P_project[index_this]).norm();
        double next_dist = (P_pixel[index_next] - P_project[index_next]).norm();
        double line_dist = fabs(pixel_line.norm() - project_line.norm());

        double pixel_dist = (0.5 * (this_dist + next_dist) + line_dist) / pixel_line.norm();
        cost += pixel_dist;
    }
    return cost;
}

double YawPnP::getAngleCost(const std::vector<Eigen::Vector2d>& P_project, double append_yaw) const {
    if (P_pixel.size() != P_project.size() || P_project.size() < 4) return 0.0;

    int map[4] = {0, 1, 3, 2};

    double cost = 0.0;
    for (int i = 0; i < 4; i++) {
        int index_this = map[i];
        int index_next = map[(i + 1) % 4];
        Eigen::Vector2d pixel_line = P_pixel[index_next] - P_pixel[index_this];
        Eigen::Vector2d project_line = P_project[index_next] - P_project[index_this];

        double cos_angle = pixel_line.dot(project_line) / (pixel_line.norm() * project_line.norm());
        double angle_dist = fabs(acos(cos_angle));

        cost += angle_dist;
    }
    return cost;
}

double YawPnP::getCost(double append_yaw) const {
    std::vector<Eigen::Vector4d> P_mapping = getMapping(append_yaw);
    std::vector<Eigen::Vector2d> P_project = getProject(P_mapping);
    double cost = getCost(P_project, append_yaw);
    return cost;
}

double YawPnP::getPixelCost(double append_yaw) const {
    std::vector<Eigen::Vector4d> P_mapping = getMapping(append_yaw);
    std::vector<Eigen::Vector2d> P_project = getProject(P_mapping);
    double cost = getPixelCost(P_project, append_yaw);
    return cost;
}

double YawPnP::getAngleCost(double append_yaw) const {
    std::vector<Eigen::Vector4d> P_mapping = getMapping(append_yaw);
    std::vector<Eigen::Vector2d> P_project = getProject(P_mapping);
    double cost = getAngleCost(P_project, append_yaw);
    return cost;
}

double YawPnP::getYawByPixelCost(double left, double right, double epsilon) const {
    while (right - left > epsilon) {
        double mid1 = left + (right - left) / 3;
        double mid2 = right - (right - left) / 3;

        double f1 = getPixelCost(mid1);
        double f2 = getPixelCost(mid2);

        if (f1 < f2) {
            right = mid2;
        } else {
            left = mid1;
        }
    }
    return (left + right) / 2;
}

double YawPnP::getYawByAngleCost(double left, double right, double epsilon) const {
    while (right - left > epsilon) {
        double mid1 = left + (right - left) / 3;
        double mid2 = right - (right - left) / 3;

        double f1 = getAngleCost(mid1);
        double f2 = getAngleCost(mid2);

        if (f1 < f2) {
            right = mid2;
        } else {
            left = mid1;
        }
    }
    return (left + right) / 2;
}

double YawPnP::getYawByMix(double pixel_yaw, double angle_yaw) const {
    double mid = 0.3;
    double len = 0.1;
    
    double ratio = 0.5 + 0.5 * sin(M_PI * (fabs(pixel_yaw) - mid) / len);
    double append_yaw;

    if ((fabs(pixel_yaw) > (mid - len / 2)) && (fabs(pixel_yaw) < (mid + len / 2))) {
        append_yaw = ratio * pixel_yaw + (1 - ratio) * angle_yaw;
    } else if (fabs(pixel_yaw) <= (mid - len / 2)) {
        append_yaw = angle_yaw;
    } else {
        append_yaw = pixel_yaw;
    }
    return append_yaw;
}