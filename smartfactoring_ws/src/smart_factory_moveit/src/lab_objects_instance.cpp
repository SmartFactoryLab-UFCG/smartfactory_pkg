#include <memory>

#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

void addPrimitiveBox(
    moveit_msgs::msg::CollisionObject &collision_object,
    const std::vector<double>& dimensions,
    const std::vector<double>& position,
    const std::vector<double>& orientation)
{
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = dimensions[0];
    primitive.dimensions[primitive.BOX_Y] = dimensions[1];
    primitive.dimensions[primitive.BOX_Z] = dimensions[2];

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = orientation[0];
    box_pose.orientation.x = orientation[1];
    box_pose.orientation.y = orientation[2];
    box_pose.orientation.z = orientation[3];
    box_pose.position.x = position[0];
    box_pose.position.y = position[1];
    box_pose.position.z = position[2];

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
}

void addPrimitiveCylinder(
    moveit_msgs::msg::CollisionObject &collision_object,
    const std::vector<double>& dimensions,
    const std::vector<double>& position,
    const std::vector<double>& orientation)
{
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the cylinder in meters
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[primitive.CYLINDER_HEIGHT] = dimensions[0];
    primitive.dimensions[primitive.CYLINDER_RADIUS] = dimensions[1];

    // Define the pose of the cylinder (relative to the frame_id)
    geometry_msgs::msg::Pose cylinder_pose;
    cylinder_pose.orientation.w = orientation[0];
    cylinder_pose.orientation.x = orientation[1];
    cylinder_pose.orientation.y = orientation[2];
    cylinder_pose.orientation.z = orientation[3];
    cylinder_pose.position.x = position[0];
    cylinder_pose.position.y = position[1];
    cylinder_pose.position.z = position[2];

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cylinder_pose);
}

void addPrimitiveSphere(
    moveit_msgs::msg::CollisionObject &collision_object,
    const double& radius,
    const std::vector<double>& position,
    const std::vector<double>& orientation)
{
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the sphere in meters
    primitive.type = primitive.SPHERE;
    primitive.dimensions.resize(1);
    primitive.dimensions[primitive.SPHERE_RADIUS] = radius;

    // Define the pose of the sphere (relative to the frame_id)
    geometry_msgs::msg::Pose sphere_pose;
    sphere_pose.orientation.w = orientation[0];
    sphere_pose.orientation.x = orientation[1];
    sphere_pose.orientation.y = orientation[2];
    sphere_pose.orientation.z = orientation[3];
    sphere_pose.position.x = position[0];
    sphere_pose.position.y = position[1];
    sphere_pose.position.z = position[2];

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(sphere_pose);
}

int main(int argc, char * argv[])
{
    // Initialize ROS and node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "collision_objects",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    node->declare_parameter<std::string>("scene_file", "/workspaces/ind-5/ros2-ws/jazzy_ws/src/smart_factory_moveit/config/scene_objects.yaml");
    std::string scene_file = node->get_parameter("scene_file").as_string();

    // Initialize logger
    auto const logger = rclcpp::get_logger("collision_objects");

    using moveit::planning_interface::PlanningSceneInterface;
    auto planning_scene = PlanningSceneInterface();

    // Create Object Vector
    std::vector<moveit_msgs::msg::CollisionObject> objs;

    // load the Yaml
    YAML::Node config = YAML::LoadFile(scene_file);

    for (const auto & obj_node : config["objects"])
    {
        moveit_msgs::msg::CollisionObject co;
        co.id            = obj_node["id"].as<std::string>();
        co.header.frame_id = obj_node["frame_id"].as<std::string>("world");
        co.operation     = co.ADD;

        for (const auto & prim_node : obj_node["primitives"])
        {
            std::string type = prim_node["type"].as<std::string>();
            auto pose_v      = prim_node["pose"]["position"].as<std::vector<double>>();
            auto orient_v    = prim_node["pose"]["orientation"].as<std::vector<double>>();

            if (type == "box")
            {
                auto dims = prim_node["dimensions"].as<std::vector<double>>();  // size 3
                addPrimitiveBox(co, dims, pose_v, orient_v);
            }
            else if (type == "cylinder")
            {
                auto dims = prim_node["dimensions"].as<std::vector<double>>();  // size 2
                addPrimitiveCylinder(co, dims, pose_v, orient_v);
            }
            else if (type == "sphere")
            {
                double r = prim_node["dimensions"][0].as<double>();
                addPrimitiveSphere(co, r, pose_v, orient_v);
            }
            // …etc…
        }

        RCLCPP_INFO(logger, 
                    "Added object %s to frame %s",
                    co.id.c_str(),
                    co.header.frame_id.c_str());
        objs.push_back(co);
    }

    RCLCPP_INFO(logger, "Adding object collection to Planning Scene Interface");
    planning_scene.addCollisionObjects(objs);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
