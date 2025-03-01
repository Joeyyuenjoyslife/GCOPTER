#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"
#include "gcopter/pa_checker.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

struct Config
{
    std::string meshResource;
    std::string mapTopic;
    std::string targetTopic;
    double dilateRadius;
    double voxelWidth;
    std::vector<double> mapBound;
    double timeoutRRT;
    double maxVelMag;
    double maxBdrMag;
    double maxTiltAngle;
    double maxPitch;
    double minThrust;
    double maxThrust;
    double vehicleMass;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;
    double weightT;
    std::vector<double> chiVec;
    double smoothingEps;
    int integralIntervs;
    double relCostTol;
    double meshScale;

    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("MeshResource", meshResource);
        nh_priv.getParam("MapTopic", mapTopic);
        nh_priv.getParam("TargetTopic", targetTopic);
        nh_priv.getParam("DilateRadius", dilateRadius);
        nh_priv.getParam("VoxelWidth", voxelWidth);
        nh_priv.getParam("MapBound", mapBound);
        nh_priv.getParam("TimeoutRRT", timeoutRRT);
        nh_priv.getParam("MaxVelMag", maxVelMag);
        nh_priv.getParam("MaxBdrMag", maxBdrMag);
        nh_priv.getParam("MaxTiltAngle", maxTiltAngle);
        nh_priv.getParam("MaxPitch", maxPitch);
        nh_priv.getParam("MinThrust", minThrust);
        nh_priv.getParam("MaxThrust", maxThrust);
        nh_priv.getParam("VehicleMass", vehicleMass);
        nh_priv.getParam("GravAcc", gravAcc);
        nh_priv.getParam("HorizDrag", horizDrag);
        nh_priv.getParam("VertDrag", vertDrag);
        nh_priv.getParam("ParasDrag", parasDrag);
        nh_priv.getParam("SpeedEps", speedEps);
        nh_priv.getParam("WeightT", weightT);
        nh_priv.getParam("ChiVec", chiVec);
        nh_priv.getParam("SmoothingEps", smoothingEps);
        nh_priv.getParam("IntegralIntervs", integralIntervs);
        nh_priv.getParam("RelCostTol", relCostTol);
        nh_priv.getParam("MeshScale", meshScale);
    }
};

class GlobalPlanner
{
private:
    Config config;

    ros::NodeHandle nh;
    ros::Subscriber mapSub;
    ros::Subscriber targetSub;

    bool mapInitialized;
    voxel_map::VoxelMap voxelMap;
    Visualizer visualizer;
    std::vector<Eigen::Vector3d> startGoal;

    Trajectory<5> traj;
    pa_checker::Pa_checker paChecker;
    double trajStamp;

public:
    GlobalPlanner(const Config &conf,
                  ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          mapInitialized(false),
          visualizer(nh),
          paChecker(0.0, 40.0, 4.0, 4.0, false)
    {
        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);

        voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);

        mapSub = nh.subscribe(config.mapTopic, 1, &GlobalPlanner::mapCallBack, this,
                              ros::TransportHints().tcpNoDelay());

        targetSub = nh.subscribe(config.targetTopic, 1, &GlobalPlanner::targetCallBack, this,
                                 ros::TransportHints().tcpNoDelay());
    }

    inline void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        if (!mapInitialized)
        {
            size_t cur = 0;
            const size_t total = msg->data.size() / msg->point_step;
            float *fdata = (float *)(&msg->data[0]);
            for (size_t i = 0; i < total; i++)
            {
                cur = msg->point_step / sizeof(float) * i;

                if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                    std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                    std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
                {
                    continue;
                }
                voxelMap.setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                                     fdata[cur + 1],
                                                     fdata[cur + 2]));
            }

            voxelMap.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()));

            mapInitialized = true;
        }
    }

    inline void plan()
    {
        if (startGoal.size() == 2)
        {
            std::vector<Eigen::Vector3d> route;
            sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[0],
                                                   startGoal[1],
                                                   voxelMap.getOrigin(),
                                                   voxelMap.getCorner(),
                                                   &voxelMap, 0.01,
                                                   route);
            std::vector<Eigen::MatrixX4d> hPolys;
            std::vector<Eigen::Vector3d> pc;
            voxelMap.getSurf(pc);

            sfc_gen::convexCover(route,
                                 pc,
                                 voxelMap.getOrigin(),
                                 voxelMap.getCorner(),
                                 7.0,
                                 3.0,
                                 hPolys);
            sfc_gen::shortCut(hPolys);

            if (route.size() > 1)
            {
                visualizer.visualizePolytope(hPolys);

                Eigen::Matrix3d iniState;
                Eigen::Matrix3d finState;
                iniState << route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

                gcopter::GCOPTER_PolytopeSFC gcopter;

                // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
                // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
                // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
                //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
                // initialize some constraint parameters
                Eigen::VectorXd magnitudeBounds(6);
                Eigen::VectorXd penaltyWeights(6);
                Eigen::VectorXd physicalParams(6);
                magnitudeBounds(0) = config.maxVelMag;
                magnitudeBounds(1) = config.maxBdrMag;
                magnitudeBounds(2) = config.maxTiltAngle;
                magnitudeBounds(3) = config.minThrust;
                magnitudeBounds(4) = config.maxThrust;
                magnitudeBounds(5) = config.maxPitch;
                penaltyWeights(0) = (config.chiVec)[0];
                penaltyWeights(1) = (config.chiVec)[1];
                penaltyWeights(2) = (config.chiVec)[2];
                penaltyWeights(3) = (config.chiVec)[3];
                penaltyWeights(4) = (config.chiVec)[4];
                penaltyWeights(5) = (config.chiVec)[5];
                physicalParams(0) = config.vehicleMass;
                physicalParams(1) = config.gravAcc;
                physicalParams(2) = config.horizDrag;
                physicalParams(3) = config.vertDrag;
                physicalParams(4) = config.parasDrag;
                physicalParams(5) = config.speedEps;
                const int quadratureRes = config.integralIntervs;

                traj.clear();
                paChecker.clear();

                if (!gcopter.setup(config.weightT,
                                   iniState, finState,
                                   hPolys, INFINITY,
                                   config.smoothingEps,
                                   quadratureRes,
                                   magnitudeBounds,
                                   penaltyWeights,
                                   physicalParams))
                {
                    return;
                }

                if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
                {
                    return;
                }

                if (traj.getPieceNum() > 0)
                {
                    trajStamp = ros::Time::now().toSec();
                    visualizer.visualize(traj, route);
                }
            }
        }
    }

    inline void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (mapInitialized)
        {
            if (startGoal.size() >= 2)
            {
                startGoal.clear();
            }
            const double zGoal = config.mapBound[4] + config.dilateRadius +
                                 fabs(msg->pose.orientation.z) *
                                     (config.mapBound[5] - config.mapBound[4] - 2 * config.dilateRadius);
            const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, zGoal);
            if (voxelMap.query(goal) == 0)
            {
                visualizer.visualizeStartGoal(goal, 0.5, startGoal.size());
                startGoal.emplace_back(goal);
            }
            else
            {
                ROS_WARN("Infeasible Position Selected !!!\n");
            }

            plan();
        }
        return;
    }

    inline void process()
    {
        Eigen::VectorXd physicalParams(6);
        physicalParams(0) = config.vehicleMass;
        physicalParams(1) = config.gravAcc;
        physicalParams(2) = config.horizDrag;
        physicalParams(3) = config.vertDrag;
        physicalParams(4) = config.parasDrag;
        physicalParams(5) = config.speedEps;

        flatness::FlatnessMap flatmap;
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
                      physicalParams(3), physicalParams(4), physicalParams(5));

        if (traj.getPieceNum() > 0)
        {
            const double delta = ros::Time::now().toSec() - trajStamp;
            if (delta > 0.0 && delta < traj.getTotalDuration())
            {
                double thr;
                Eigen::Vector4d quat;
                Eigen::Vector3d omg;

                //Joeyyu: calculate the yaw angle------------------------------------------------------------------------
                Eigen::Vector3d vel = traj.getVel(delta);
                Eigen::Vector3d pos = traj.getPos(delta);
                double Psi = std::atan2(vel(1),vel(0));

                flatmap.forward(traj.getVel(delta),
                                traj.getAcc(delta),
                                traj.getJer(delta),
                                Psi, 0.0,
                                thr, quat, omg);
                double speed = traj.getVel(delta).norm();
                
                //Joeyyu: calculate the yaw angle------------------------------------------------------------------------
                // Eigen::Vector3d vel = traj.getVel(delta);
                // Eigen::Vector3d vel_xy(vel(0),vel(1),0);
                
                // vel_xy.normalize();
                // //std::cout << "vel_xy " << " " << vel_xy.transpose() << std::endl;
                // Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(),vel_xy);

                
                // //q.FromTwoVectors(Eigen::Vector3d::UnitX(),vel_xy);
                // q.normalize();
                // //std::cout<<" q: " << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()<<std::endl;
                // Eigen::Quaterniond q0(quat(0),quat(1),quat(2),quat(3));
                // //std::cout<<" q0: " << " " << q0.x() <<  q0.y() << q0.z() << q0.w()<<std::endl;

                // if(fabs(vel(0)) > 0.01 || fabs(vel(1))> 0.01 )
                // {
                //     q0 = q0 * q;
                //     quat = Eigen::Vector4d(q0.w(),q0.x(),q0.y(),q0.z());
                //     //std::cout<<" quat: " << " " << quat.transpose()<<std::endl;
                // }
                

                


                double bodyratemag = omg.norm();
                double tiltangle = acos(1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2))) / M_PI * 180.0;
                //Joeyyu: add the pitch and roll angle
                double pitchangle = asin(2.0*(quat(0)*quat(2)-quat(3)*quat(1)))/ M_PI * 180.0 ;
                double rollangle = atan2(2.0*(quat(0)*quat(1)+quat(2)*quat(3)),1-2.0*(quat(1)*quat(1)+quat(2)*quat(2))) /M_PI * 180.0;
                double yawangle = atan2(2.0*(quat(0)*quat(3)+quat(1)*quat(2)),1-2.0*(quat(2)*quat(2)+quat(3)*quat(3)));
                //double tilt_from_pr = acos(cos(pitchangle)*cos(rollangle));
                //std::cout<<"[Global_planning]:" << " " << "tilt 1:" << " " << tiltangle  
                //<< " " <<" tilt 1:" <<" " << tilt_from_pr <<std::endl;
                //std::cout<<"[Global_planning]:" << " " << "tilt angle:" << " " << tiltangle/3.14 * 180.0 << std::endl;
                //std::cout<<"[Global_planning]:" << " " << "pitch angle:" << " " << pitchangle/3.14 * 180.0 << std::endl;
                //std::cout<<"[Global_planning]:" << " " << "roll angle:" << " " << rollangle/3.14 * 180.0 << std::endl;
                std_msgs::Float64 speedMsg, thrMsg, tiltMsg, bdrMsg;
                //Joeyyu: add pitch and roll Msg;
                std_msgs::Float64 pitchMsg, rollMsg;
                
                paChecker.check(traj, quat, pos, speed, delta);

                //std::cout<<speed << " " << paChecker.getProgress() << " " << paChecker.getSafeFlag() <<std::endl;

                speedMsg.data = speed;
                thrMsg.data = thr;
                tiltMsg.data = tiltangle;
                bdrMsg.data = bodyratemag;

                pitchMsg.data = pitchangle;
                rollMsg.data = rollangle;
                visualizer.speedPub.publish(speedMsg);
                visualizer.thrPub.publish(thrMsg);
                visualizer.tiltPub.publish(tiltMsg);
                visualizer.pitchPub.publish(pitchMsg);
                visualizer.rollPub.publish(rollMsg);

                visualizer.bdrPub.publish(bdrMsg);
                visualizer.visualizeSphere(traj.getPos(delta),
                                           config.dilateRadius);

                Eigen::Quaterniond q_fov(quat(0),quat(1),quat(2),quat(3));
                visualizer.pub_fov_visual(traj.getPos(delta),q_fov);
                visualizer.pub_mesh_drone(traj.getPos(delta),quat, config.meshScale, config.meshResource);
                visualizer.vistraj_pub(traj, paChecker.getProgress(), delta, paChecker.getSafeFlag());
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planning_node");
    ros::NodeHandle nh_;

    GlobalPlanner global_planner(Config(ros::NodeHandle("~")), nh_);

    ros::Rate lr(1000);
    while (ros::ok())
    {
        global_planner.process();
        ros::spinOnce();
        lr.sleep();
    }

    return 0;
}
