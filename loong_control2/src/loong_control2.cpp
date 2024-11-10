#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "GLFW_callbacks.h"
#include "MJ_interface.h"
#include "PVT_ctrl.h"
#include "data_logger.h"
#include "data_bus.h"
#include "pino_kin_dyn.h"
#include "useful_math.h"
#include "wbc_priority.h"
#include "mpc.h"
#include "gait_scheduler.h"
#include "foot_placement.h"
#include "joystick_interpreter.h"
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <ament_index_cpp/get_package_share_directory.hpp>


const std::vector<std::string> JointName = { "J_arm_l_01", "J_arm_l_02", "J_arm_l_03", "J_arm_l_04", "J_arm_l_05", "J_arm_l_06", "J_arm_l_07",
    "J_arm_r_01", "J_arm_r_02", "J_arm_r_03", "J_arm_r_04", "J_arm_r_05", "J_arm_r_06", "J_arm_r_07",
    "J_head_yaw", "J_head_pitch", "J_waist_pitch", "J_waist_roll", "J_waist_yaw",
    "J_hip_l_roll", "J_hip_l_yaw", "J_hip_l_pitch", "J_knee_l_pitch", "J_ankle_l_pitch", "J_ankle_l_roll",        
    "J_hip_r_roll", "J_hip_r_yaw", "J_hip_r_pitch", "J_knee_r_pitch", "J_ankle_r_pitch", "J_ankle_r_roll"};

const double dt = 0.001;
const double dt_200Hz = 0.005;

class AzureLoongCtrl2 : public rclcpp::Node {
public:
    AzureLoongCtrl2() : Node("azureloong_ctrl") {
        // Initialize ROS 2 components
        vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&AzureLoongCtrl2::velCmdCallback, this, std::placeholders::_1)
        );
        js_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        odom_br = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // MuJoCo load and compile model
        char error[1000] = "Could not load binary model";
        std::string pack_path = ament_index_cpp::get_package_share_directory("loong_control");
        mjModel* mj_model = mj_loadXML((pack_path + "/models/scene.xml").c_str(), 0, error, 1000);
        mjData* mj_data = mj_makeData(mj_model);

        // Initialize classes
        uiController = std::make_unique<UIctr>(mj_model, mj_data);   // UI control for Mujoco
        mj_interface = std::make_unique<MJ_Interface>(mj_model, mj_data); // data interface for Mujoco
        kinDynSolver = std::make_unique<Pin_KinDyn>(pack_path + "/models/AzureLoong.urdf"); // kinematics and dynamics solver
        robotState = std::make_unique<DataBus>(kinDynSolver->model_nv); // data bus
        WBC_solv = std::make_unique<WBC_priority>(kinDynSolver->model_nv, 18, 22, 0.7, mj_model->opt.timestep); // WBC solver
        MPC_solv = std::make_unique<MPC>(dt_200Hz);  // mpc controller
        gaitScheduler = std::make_unique<GaitScheduler>(0.25, mj_model->opt.timestep); // gait scheduler
        pvtCtr = std::make_unique<PVT_Ctr>(mj_model->opt.timestep, (pack_path + "/common/joint_ctrl_config.json").c_str()); // PVT joint control
        footPlacement = std::make_unique<FootPlacement>(); // foot-placement planner
        jsInterp = std::make_unique<JoyStickInterpreter>(mj_model->opt.timestep); // desired baselink velocity generator
        
        // Initialize UI: GLFW
        uiController->iniGLFW();
        uiController->enableTracking(); // enable viewpoint tracking of the body 1 of the robot
        uiController->createWindow("Demo", false); // NOTE: if the saveVideo is set to true, the raw recorded file could be 2.5 GB for 15 seconds!

        // Initialize variables
        double stand_legLength = 1.01; // desired baselink height
        double foot_height = 0.07; // distance between the foot ankel joint and the bottom
        int model_nv = kinDynSolver->model_nv;

        robotState->width_hips = 0.229;
        footPlacement->kp_vx = 0.03;
        footPlacement->kp_vy = 0.03;
        footPlacement->kp_wz = 0.03;
        footPlacement->stepHeight = 0.15;
        footPlacement->legLength = stand_legLength;

        mju_copy(mj_data->qpos, mj_model->key_qpos, mj_model->nq * 1); // set ini pos in Mujoco

        std::vector<double> motors_pos_des(model_nv - 6, 0);
        std::vector<double> motors_pos_cur(model_nv - 6, 0);
        std::vector<double> motors_vel_des(model_nv - 6, 0);
        std::vector<double> motors_vel_cur(model_nv - 6, 0);
        std::vector<double> motors_tau_des(model_nv - 6, 0);
        std::vector<double> motors_tau_cur(model_nv - 6, 0);

        // Initialize position and posture for foot-end and hand
        Eigen::Vector3d fe_l_pos_L_des = {-0.018, 0.113, -stand_legLength};
        Eigen::Vector3d fe_r_pos_L_des = {-0.018, -0.116, -stand_legLength};
        Eigen::Vector3d fe_l_eul_L_des = {-0.000, -0.008, -0.000};
        Eigen::Vector3d fe_r_eul_L_des = {0.000, -0.008, 0.000};
        Eigen::Matrix3d fe_l_rot_des = eul2Rot(fe_l_eul_L_des(0), fe_l_eul_L_des(1), fe_l_eul_L_des(2));
        Eigen::Matrix3d fe_r_rot_des = eul2Rot(fe_r_eul_L_des(0), fe_r_eul_L_des(1), fe_r_eul_L_des(2));

        Eigen::Vector3d hd_l_pos_L_des = {-0.02, 0.32, -0.159};
        Eigen::Vector3d hd_r_pos_L_des = {-0.02, -0.32, -0.159};
        Eigen::Vector3d hd_l_eul_L_des = {-1.7581, 0.2129, 2.9581};
        Eigen::Vector3d hd_r_eul_L_des = {1.7581, 0.2129, -2.9581};
        Eigen::Matrix3d hd_l_rot_des = eul2Rot(hd_l_eul_L_des(0), hd_l_eul_L_des(1), hd_l_eul_L_des(2));
        Eigen::Matrix3d hd_r_rot_des = eul2Rot(hd_r_eul_L_des(0), hd_r_eul_L_des(1), hd_r_eul_L_des(2));

        auto resLeg = kinDynSolver->computeInK_Leg(fe_l_rot_des, fe_l_pos_L_des, fe_r_rot_des, fe_r_pos_L_des);
        auto resHand = kinDynSolver->computeInK_Hand(hd_l_rot_des, hd_l_pos_L_des, hd_r_rot_des, hd_r_pos_L_des);
        Eigen::VectorXd qIniDes = Eigen::VectorXd::Zero(mj_model->nq, 1);
        qIniDes.block(7, 0, mj_model->nq - 7, 1) = resLeg.jointPosRes + resHand.jointPosRes;
        WBC_solv->setQini(qIniDes, robotState->q);

        // Main loop
        int MPC_count = 0; // count for controlling the mpc running period
        double startSteppingTime = 1;
        double startWalkingTime = 2;

        mjtNum simstart = mj_data->time;
        double simTime = mj_data->time;

        while (!glfwWindowShouldClose(uiController->window) && rclcpp::ok()) {
            simstart = mj_data->time;
            while ((mj_data->time - simstart) < 1.0 / 60.0 && uiController->runSim) { // press "1" to pause and resume, "2" to step the simulation

                mj_step(mj_model, mj_data);

                simTime = mj_data->time;

                rclcpp::spin_some(this->get_node_base_interface());
                // rclcpp::spinOnce(this->get_node_base_interface());
                // Read the sensors:

                mj_interface->updateSensorValues();
                mj_interface->dataBusWrite(*robotState);

                sensor_msgs::msg::JointState js;
                js.header.stamp = this->get_clock()->now();
                for (int i = 0; i < JointName.size(); i++) {
                    js.name.push_back(JointName[i]);
                    js.position.push_back(robotState->motors_pos_cur[i]);
                    js.velocity.push_back(robotState->motors_vel_cur[i]);
                }
                js_pub->publish(js);

                sensor_msgs::msg::Imu imu;
                imu.header.stamp = this->get_clock()->now();
                imu.header.frame_id = "base";
                imu.linear_acceleration.x = robotState->baseAcc[0];
                imu.linear_acceleration.y = robotState->baseAcc[1];
                imu.linear_acceleration.z = robotState->baseAcc[2];
                imu.angular_velocity.x = robotState->baseAngVel[0];
                imu.angular_velocity.y = robotState->baseAngVel[1];
                imu.angular_velocity.z = robotState->baseAngVel[2];
                tf2::Quaternion quat;
                quat.setRPY(robotState->rpy[0], robotState->rpy[1], robotState->rpy[2]);
                quat = quat.normalize();
                imu.orientation.x = quat.getX();
                imu.orientation.y = quat.getY();
                imu.orientation.z = quat.getZ();
                imu.orientation.w = quat.getW();
                imu_pub->publish(imu);

                nav_msgs::msg::Odometry odom;
                odom.header.stamp = this->get_clock()->now();
                odom.header.frame_id = "odom";
                odom.pose.pose.position.x = robotState->basePos[0];
                odom.pose.pose.position.y = robotState->basePos[1];
                odom.pose.pose.position.z = robotState->basePos[2];
                geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(quat);
                odom.pose.pose.orientation = odom_quat;
                odom.child_frame_id = "base";
                odom.twist.twist.linear.x = robotState->baseLinVel[0];
                odom.twist.twist.linear.y = robotState->baseLinVel[1];
                odom.twist.twist.linear.z = robotState->baseLinVel[2];
                odom.twist.twist.angular.x = robotState->baseAngVel[0];
                odom.twist.twist.angular.y = robotState->baseAngVel[1];
                odom.twist.twist.angular.z = robotState->baseAngVel[2];
                odom_pub->publish(odom);

                geometry_msgs::msg::TransformStamped odom_trans;
                odom_trans.header.stamp = this->get_clock()->now();
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "base";
                odom_trans.transform.translation.x = robotState->basePos[0];
                odom_trans.transform.translation.y = robotState->basePos[1];
                odom_trans.transform.translation.z = robotState->basePos[2];
                odom_trans.transform.rotation = odom_quat;
                odom_br->sendTransform(odom_trans);

                // Update kinematics and dynamics info
                kinDynSolver->dataBusRead(*robotState);
                kinDynSolver->computeJ_dJ();
                kinDynSolver->computeDyn();
                kinDynSolver->dataBusWrite(*robotState);

                if (simTime > startWalkingTime) {
                    jsInterp->setVxDesLPara(xyt_des[0], 2.0);
                    jsInterp->setVyDesLPara(xyt_des[1], 1.0);
                    jsInterp->setWzDesLPara(xyt_des[2], 1.0);
                    robotState->motionState = DataBus::Walk; // start walking
                } else {
                    jsInterp->setIniPos(robotState->q(0), robotState->q(1), robotState->base_rpy(2));
                }
                jsInterp->step();
                robotState->js_pos_des(2) = stand_legLength + foot_height; // pos z is not assigned in jyInterp
                jsInterp->dataBusWrite(*robotState); // only pos x, pos y, theta z, vel x, vel y , omega z are rewrote.

                if (simTime >= startSteppingTime) {
                    // Gait scheduler
                    gaitScheduler->dataBusRead(*robotState);
                    gaitScheduler->step();
                    gaitScheduler->dataBusWrite(*robotState);

                    footPlacement->dataBusRead(*robotState);
                    footPlacement->getSwingPos();
                    footPlacement->dataBusWrite(*robotState);
                }

                // ------------- MPC ------------
                MPC_count = MPC_count + 1;
                if (MPC_count > ((dt_200Hz / dt) - 1)) {
                    MPC_solv->dataBusRead(*robotState);
                    MPC_solv->cal();
                    MPC_solv->dataBusWrite(*robotState);
                    MPC_count = 0;
                }

                // ------------- WBC ------------
                // WBC Calculation
                WBC_solv->dataBusRead(*robotState);
                WBC_solv->computeDdq(*kinDynSolver);
                WBC_solv->computeTau();
                WBC_solv->dataBusWrite(*robotState);
                // Get the final joint command
                if (simTime <= startSteppingTime) {
                    robotState->motors_pos_des = eigen2std(resLeg.jointPosRes + resHand.jointPosRes);
                    robotState->motors_vel_des = motors_vel_des;
                    robotState->motors_tor_des = motors_tau_des;
                } else {
                    MPC_solv->enable();
                    Eigen::Matrix<double, 1, nx> L_diag;
                    Eigen::Matrix<double, 1, nu> K_diag;
                    L_diag << 1.0, 1.0, 1.0, // eul
                              1.0, 200.0, 1.0, // pCoM
                              1e-7, 1e-7, 1e-7, // w
                              100.0, 10.0, 1.0; // vCoM
                              
                    K_diag << 1.0, 1.0, 1.0,//fl
                        1.0, 1.0, 1.0,
                        1.0, 1.0, 1.0,//fr
                        1.0, 1.0, 1.0,1.0;

                MPC_solv->set_weight(1e-6, L_diag, K_diag);
                
                // Apply the joint commands to the robot
                // for (int i = 0; i < robotState->motors_pos_des.size(); i++) {
                //     mj_data->ctrl[i] = robotState->motors_tor_des[i];
                // }
                Eigen::VectorXd pos_des = kinDynSolver->integrateDIY(robotState->q, robotState->wbc_delta_q_final);
                robotState->motors_pos_des = eigen2std(pos_des.block(7, 0, model_nv - 6, 1));
                robotState->motors_vel_des = eigen2std(robotState->wbc_dq_final);
                robotState->motors_tor_des = eigen2std(robotState->wbc_tauJointRes);
            }

            // joint PVT controller
            pvtCtr->dataBusRead(*robotState);
            if (simTime <= 3) {
                pvtCtr->calMotorsPVT(100.0 / 1000.0 / 180.0 * 3.1415);
            } else {
                pvtCtr->setJointPD(100,10,"J_ankle_l_pitch");
                pvtCtr->setJointPD(100,10,"J_ankle_l_roll");
                pvtCtr->setJointPD(100,10,"J_ankle_r_pitch");
                pvtCtr->setJointPD(100,10,"J_ankle_r_roll");
                pvtCtr->setJointPD(1000,100,"J_knee_l_pitch");
                pvtCtr->setJointPD(1000,100,"J_knee_r_pitch");
                pvtCtr->calMotorsPVT();
            }
            pvtCtr->dataBusWrite(*robotState);

            // give the joint torque command to Webots
            mj_interface->setMotorsTorque(robotState->motors_tor_out);

            
        }
        uiController->updateScene();
        // Clean up
        // mj_deleteData(mj_data);
        // mj_deleteModel(mj_model);
        // glfwDestroyWindow(uiController->window);
        // glfwTerminate();
    };
    uiController->Close();
    }
    // virtual ~AzureLoongCtrl2();
private:
    void velCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        xyt_des[0] = msg->linear.x;
        xyt_des[1] = msg->linear.y;
        xyt_des[2] = msg->angular.z;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_br;

    std::unique_ptr<UIctr> uiController;
    std::unique_ptr<MJ_Interface> mj_interface;
    std::unique_ptr<Pin_KinDyn> kinDynSolver;
    std::unique_ptr<DataBus> robotState;
    std::unique_ptr<WBC_priority> WBC_solv;
    std::unique_ptr<MPC> MPC_solv;
    std::unique_ptr<GaitScheduler> gaitScheduler;
    std::unique_ptr<PVT_Ctr> pvtCtr;
    std::unique_ptr<FootPlacement> footPlacement;
    std::unique_ptr<JoyStickInterpreter> jsInterp;

    std::vector<double> xyt_des = {0.0, 0.0, 0.0};

};

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AzureLoongCtrl2>());
    rclcpp::shutdown();
    return 0;
}
