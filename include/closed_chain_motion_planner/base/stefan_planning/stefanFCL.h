#pragma once

#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>


#include <closed_chain_motion_planner/base/stefan_planning/fcl_eigen_utils.h>
#include <closed_chain_motion_planner/base/stefan_planning/vtk_mesh_utils.h>
#include <closed_chain_motion_planner/base/utils.h>

#include <fcl/traversal/traversal_node_bvhs.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/collision_node.h>
#include <fcl/collision.h>
#include <fcl/BV/BV.h>
#include <fcl/BV/OBBRSS.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/narrowphase/narrowphase.h>

#include <fcl/broadphase/broadphase.h>
#include <fcl/collision.h>
#include <fcl/distance.h>

#include <closed_chain_motion_planner/kinematics/grasping_point.h>
typedef fcl::OBBRSS BV;
typedef fcl::BVHModel<BV> BVHM;
typedef std::shared_ptr<BVHM> BVHMPtr;
using fcl::Box;
typedef std::shared_ptr<fcl::Box> BoxPtr;
using fcl::CollisionObject;
typedef std::shared_ptr<fcl::CollisionObject> CollisionObjectPtr;

typedef pcl::PointCloud<pcl::PointXYZRGBNormal> CloudT;
typedef pcl::PointXYZRGBNormal PointT;
typedef std::shared_ptr<PointT> PointTPtr;

using namespace std;
using namespace Eigen;
using namespace pcl;

class stefanFCL
{
public:
    BVHMPtr mesh_model_;
    BoxPtr table_model_[9];
    fcl::Transform3f table_transform[9];
    std::string file_name;
    stefanFCL()
    {
        file_name = "/home/jiyeong/catkin_ws/src/1_assembly/grasping_point/STEFAN/stl/assembly_without_bottom.stl";
        // grasping_point grp;
        // file_name = grp.mesh_file_;
        pcl::PolygonMesh mesh;
        pcl::io::loadPolygonFile(file_name, mesh);
        std::vector<TrianglePlaneData> triangles = buildTriangleData(mesh);
        loadMesh(triangles);

        Eigen::Isometry3d table_t_[9];
        for (int i = 0; i < 9; i++)
            table_t_[i].setIdentity();

        table_t_[0].translation() << 0.69, -0.04, 1.0826;
        table_model_[0] = std::make_shared<Box>(0.36, 0.21, 0.164);

        table_t_[1].translation() << 0.465, -0.505, 1.0826;
        table_model_[1] = std::make_shared<Box>(0.21, 0.16, 0.164);

        table_t_[2].translation() << 0.595, 0.355, 1.0826;
        table_model_[2] = std::make_shared<Box>(0.16, 0.21, 0.164);
            
        table_t_[3].translation() << 0.42, 0.1, 1.0826;
        table_model_[3] = std::make_shared<Box>(0.21, 0.21, 0.164);

        /////////////////////////////////

        table_t_[4].translation() << -0.05, 0.0, 1.0; // 판다 두개 있는쪽
        table_model_[4] = std::make_shared<Box>(0.1, 1.0, 1.0);

        table_t_[5].translation() << 1.35, 0.0, 1.0; //판다 top 있는쪽
        table_model_[5] = std::make_shared<Box>(0.1, 1.0, 1.0);

        table_t_[6].translation() << 0.75, -0.6, 1.0; //right
        table_model_[6] = std::make_shared<Box>(1.0, 0.1, 2.0);

        table_t_[7].translation() << 0.75, 0.6, 1.0; //left
        table_model_[7] = std::make_shared<Box>(1.0, 0.1, 2.0);


        table_t_[8].translation() << 0.95, 0.0, 1.90; // 천장
        table_model_[8] = std::make_shared<Box>(1.0, 0.6, 0.1);

        for (int i = 0; i < 9; i++)
            FCLEigenUtils::convertTransform(table_t_[i], table_transform[i]);
    }
    void loadMesh(const std::vector<TrianglePlaneData> &mesh)
    {
        std::vector<fcl::Vec3f> points;
        std::vector<fcl::Triangle> triangles;
        mesh_model_ = std::make_shared<BVHM>();

        for (const auto &tri_plane : mesh)
        {
            fcl::Triangle tri;

            for (int i = 0; i < 3; i++)
            {
                tri[i] = points.size();
                points.push_back(
                    fcl::Vec3f(
                        tri_plane.points[i](0),
                        tri_plane.points[i](1),
                        tri_plane.points[i](2)));
            }
            triangles.push_back(tri);
        }
        mesh_model_->beginModel();
        mesh_model_->addSubModel(points, triangles);
        mesh_model_->endModel();
    }

    bool isFeasible(Eigen::Isometry3d mesh_transform) 
    {
        fcl::CollisionRequest request;
        // request.enable_contact = true;
        
        fcl::CollisionResult result;
        
        bool is_collided = false;
        fcl::Transform3f fcl_transform;
        FCLEigenUtils::convertTransform(mesh_transform, fcl_transform);

        for (int i = 0; i < 9; i++)
        {
            fcl::collide(mesh_model_.get(), fcl_transform, table_model_[i].get(), table_transform[i], request, result);
            if (result.isCollision())
            {   
                // std::cout << "contact with " << i << "th table" << std::endl;
                return false;
            }
        }
        return true;

        // return !result.isCollision();
    }

    bool isValid(Eigen::Isometry3d base_object) 
    {
        return isFeasible(base_object);
    }

    bool isValid(ob::State *obj_state_) 
    {
        return isFeasible(StateEigenUtils::StateToIsometry(obj_state_));
    }

    bool is_Valid(Eigen::Vector3d pos, Eigen::Vector4d quat) 
    {
        Eigen::Isometry3d transform;
        transform.setIdentity();
        transform.linear() = Eigen::Quaterniond(quat(3), quat(0), quat(1), quat(2)).toRotationMatrix();
        transform.translation() = pos;
        return isFeasible(transform);
    }
};