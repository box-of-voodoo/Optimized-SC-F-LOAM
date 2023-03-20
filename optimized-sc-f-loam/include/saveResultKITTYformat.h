#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Rot3.h>

#include <gtsam/geometry/Pose3.h>


#include <scancontext/common.h>


// void saveOptimizedVerticesKITTIformat(const gtsam::Values& _estimates, std::string _filename);
// void saveOdometryVerticesKITTIformat(const std::string& _filename);
// void savegtVerticesKITTIformat(const std::string& _filename);

gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& p)
{
    return gtsam::Pose3( gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z) );
} // Pose6DtoGTSAMPose3

void saveOptimizedVerticesKITTIformat(const gtsam::Values& _estimates, std::string _filename)
{
    using namespace gtsam;
    std::cout << "优化后位姿的长度: " << _estimates.size() << std::endl;

    std::fstream stream(_filename.c_str(), std::fstream::out);
    for(const auto& key_value: _estimates) {
        auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
        if (!p) continue;

        const Pose3& pose = p->value();

        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }

}//saveOptimizedVerticesKITTIformat

void saveVerticesKITTIformat(const std::vector<Pose6D> &keyframePoses,const std::string& _filename)
{
    // ref from gtsam's original code "dataset.cpp"
    std::cout << "优化前位姿的长度： " << keyframePoses.size() << std::endl;
    std::fstream stream(_filename.c_str(), std::fstream::out);
    for(const auto& _pose6d: keyframePoses) {
        gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
        gtsam::Point3 t = pose.translation();
        gtsam::Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3
        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}//saveVerticesKITTIformat


void savegtVerticesKITTIformat(const std::vector<Pose6D> &keyframegtPoses,const std::string& _filename)
{
    // ref from gtsam's original code "dataset.cpp"
    std::cout << "gt位姿的长度： " << keyframegtPoses.size() << std::endl;
    std::fstream stream(_filename.c_str(), std::fstream::out);
    for(const auto& _pose6d: keyframegtPoses) {
        gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
        gtsam::Point3 t = pose.translation();
        gtsam::Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}//savegtVerticesKITTIformat