#ifndef MADA_ROBOTS_LISTENER_HPP
#define MADA_ROBOTS_LISTENER_HPP

#include <mada/mada_util.h>
#include <mada/util/funciones.hpp>
#include <mada/util/fmm_2.hpp>
#include <mada/util/generic_fmm.hpp>
#include <mada/util/geometry.hpp>
#include <mada/util/graph_functions.hpp>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <random>

namespace RobotsListener{

    struct Positions{
        vector<int> x, y;
        vector<float> o;

        vector<geometry_msgs::Pose> poses;
    };

    Positions toPositions(vector<geometry_msgs::Pose> poses, nav_msgs::OccupancyGrid map)
    {
        Positions res;

        res.poses = poses;

        res.x.resize(poses.size()); res.y.resize(poses.size()); res.o.resize(poses.size());

        for(int i=0; i<poses.size(); i++)
        {
            res.x[i] = (poses[i].position.y - map.info.origin.position.y)/map.info.resolution;
            res.y[i] = (poses[i].position.x - map.info.origin.position.x)/map.info.resolution;
            res.o[i] = tf::getYaw(poses[i].orientation);
        }

        return res;
    }

    Positions getPositions(Positions positions, Poss<int> poss, float radius)
    {
        Positions res;
        vector<bool> in(positions.x.size(), false);
        for(int i=0; i<poss.x.size(); i++){
            for(int j=0; j<positions.x.size(); j++){
                if(!in[j] && hypot(poss.x[i] - positions.x[j], poss.y[i] - positions.y[j]) <= radius){
                    in[j] = true;
                    res.x.push_back(positions.x[j]); res.y.push_back(positions.y[j]);
                    res.o.push_back(positions.o[j]); res.poses.push_back(positions.poses[j]);
                }
            }
        }
        return res;
    }

    class RobotsListener
    {
    private:
        ros::NodeHandle nh_;
        vector<ros::Subscriber> subscribers_; // Lista de suscriptores
        vector<string> robot_names_; // Lista de nombres de robots
        vector<geometry_msgs::Pose> robot_poses_; // Lista de poses de robots (paralela a robot_names_)

    public:
        // Constructor
        RobotsListener(const vector<string>& robot_names, string pose_name) : robot_names_(robot_names), robot_poses_(robot_names.size())
        {
            string topic_name;
            if(pose_name.find("amcl")>=0 && pose_name.find("amcl")<pose_name.size()-1){
                for (size_t i = 0; i < robot_names.size(); ++i) {
                    // Crear un suscriptor para cada robot
                    //string topic_name = "/" + robot_names[i] + "/" + pose_name; // Supongamos que el topic es /robot_name/pose
                    if(robot_names[i].size()) topic_name = "/" + robot_names[i] + "/" + pose_name;
                    else topic_name = "/" + pose_name;
                    subscribers_.emplace_back(nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
                        topic_name, 10,
                        [this, i](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
                            this->amclPoseCallback(msg, i);
                        }
                    ));
                }
            }else if((pose_name.find("odom")>=0 && pose_name.find("odom")<pose_name.size()-1) || (pose_name.find("ground_truth")>=0 && pose_name.find("ground_truth")<pose_name.size()-1)){
                for (size_t i = 0; i < robot_names.size(); ++i) {
                    // Crear un suscriptor para cada robot
                    //string topic_name = "/" + robot_names[i] + "/" + pose_name; // Supongamos que el topic es /robot_name/pose
                    if(robot_names[i].size()) topic_name = "/" + robot_names[i] + "/" + pose_name;
                    else topic_name = "/" + pose_name;
                    subscribers_.emplace_back(nh_.subscribe<nav_msgs::Odometry>(
                        topic_name, 10,
                        [this, i](const nav_msgs::Odometry::ConstPtr& msg) {
                            this->odomPoseCallback(msg, i);
                        }
                    ));
                }
            }
        }

        RobotsListener(const vector<string>& robot_pose_names)
        {
            // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            // NO HAS CHEQUEADO SI ESTO FUNCIONA
            // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            if(robot_pose_names.size() == 0) return;
            int ind;
            string pose_name;
            string topic_name;
            robot_names_.resize(robot_pose_names.size());
            if(ind >= 0){
                pose_name = robot_pose_names[0].substr(ind, robot_pose_names[0].size()-1);
                for(int i=0; i<robot_pose_names.size(); i++){
                    robot_names_[i] = robot_pose_names[0].substr(0, ind);
                }
            }
            if(pose_name.find("amcl")>=0 && pose_name.find("amcl")<pose_name.size()-1){
                for (size_t i = 0; i < robot_names_.size(); ++i) {
                    // Crear un suscriptor para cada robot
                    //string topic_name = "/" + robot_names_[i] + "/" + pose_name;
                    if(robot_names_[i].size()) topic_name = "/" + robot_names_[i] + "/" + pose_name;
                    else topic_name = "/" + pose_name;
                    subscribers_.emplace_back(nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
                        topic_name, 10,
                        [this, i](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
                            this->amclPoseCallback(msg, i);
                        }
                    ));
                }
            }else if((pose_name.find("odom")>=0 && pose_name.find("odom")<pose_name.size()-1) || (pose_name.find("ground_truth")>=0 && pose_name.find("ground_truth")<pose_name.size()-1)){
                for (size_t i = 0; i < robot_names_.size(); ++i) {
                    // Crear un suscriptor para cada robot
                    //string topic_name = "/" + robot_names_[i] + "/" + pose_name;
                    if(robot_names_[i].size()) topic_name = "/" + robot_names_[i] + "/" + pose_name;
                    else topic_name = "/" + pose_name;
                    subscribers_.emplace_back(nh_.subscribe<nav_msgs::Odometry>(
                        topic_name, 10,
                        [this, i](const nav_msgs::Odometry::ConstPtr& msg) {
                            this->odomPoseCallback(msg, i);
                        }
                    ));
                }
            }
        }

        // Callbacks
        void odomPoseCallback(const nav_msgs::Odometry::ConstPtr& msg, size_t index)
        {
            if (index < robot_poses_.size()) {
                robot_poses_[index] = msg->pose.pose;
            } else {
                ROS_WARN_STREAM("Índice de robot inválido: " << index);
            }
        }

        void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg, size_t index)
        {
            if (index < robot_poses_.size()) {
                robot_poses_[index] = msg->pose.pose;
            } else {
                ROS_WARN_STREAM("Índice de robot inválido: " << index);
            }
        }

        // Obtener la pose de un robot específico
        geometry_msgs::Pose getPose(const string& robot_name) const
        {
            auto it = find(robot_names_.begin(), robot_names_.end(), robot_name);
            if (it != robot_names_.end()) {
                size_t index = distance(robot_names_.begin(), it);
                return robot_poses_[index];
            } else {
                ROS_WARN_STREAM("Pose for robot " << robot_name << " not available.");
                return geometry_msgs::Pose(); // Pose vacía si no se encuentra
            }
        }
        
        geometry_msgs::Pose getPose(int ind) const
        {
            return robot_poses_[ind];
        }

        // Obtener todas las poses
        vector<geometry_msgs::Pose> getAllPoses() const
        {
            return robot_poses_;
        }

        // Obtener todas las posiciones
        Positions getPositions(nav_msgs::OccupancyGrid map) const
        {
            return toPositions(robot_poses_, map);
        }

        // Verificar si todas las poses están inicializadas (por defecto, se considera inicializada si x != 0)
        bool areAllPosesInitialized() const
        {
            return all_of(robot_poses_.begin(), robot_poses_.end(), [](const geometry_msgs::Pose& pose) {
                return pose.position.x != 0 || pose.position.y != 0 || pose.position.z != 0;
            });
        }
    };

}

#endif