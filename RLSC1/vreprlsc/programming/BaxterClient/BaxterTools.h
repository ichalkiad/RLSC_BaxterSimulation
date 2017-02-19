/*
 * BaxterTools.h
 *
 *  Created on: 24 Jan 2014
 *      Author: s0972326
 */

#ifndef BAXTERTOOLS_H_
#define BAXTERTOOLS_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <pthread.h>
#include <signal.h>
extern "C" {
    #include "extApi.h"
    #include "extApiCustom.h"
}

#define PRINT_ERROR(e) PrintError(__FILE__,__LINE__,e);
#define CHECK std::cout << "Line " << __LINE__ << " OK\n";

void PrintError(const char* file, int line, int err);

KDL::Frame MakeFrame(simxFloat* val);

class BaxterTools
{
  public:
    BaxterTools();
    virtual ~BaxterTools();

    bool Connect(const char* host);
    bool StartSimulation();
    bool StopSimulation(); 
    bool CheckRunning();
    bool AdvanceSimulation();
    bool SetJointAngles(Eigen::VectorXd q_);
    Eigen::VectorXd GetIK(Eigen::VectorXd q_);
    Eigen::MatrixXd GetJ(Eigen::VectorXd q_);
    bool SetObjectPosition(int handle, Eigen::VectorXd x);
    bool SetObjectPosition(int handle, int rel, Eigen::VectorXd x);
    bool SetObjectOrientation(int handle, Eigen::VectorXd x);
    bool SetObjectOrientation(int handle, int rel, Eigen::VectorXd x);
    bool GetObjectPosition(int handle, Eigen::VectorXd& x);
    bool GetObjectPosition(int handle, int rel, Eigen::VectorXd& x);
    bool GetObjectOrientation(int handle, Eigen::VectorXd& x);
    bool GetObjectOrientation(int handle, int rel, Eigen::VectorXd& x);
    bool GetTargets(Eigen::VectorXd& x);
    void SetTargets();
    int GetObjectHandle(const char* name);
    char GetKey();

    static void* CheckKey(void* ptr);

    int clientID; // Client handle
    bool Err, Sim, Running;
    simxInt err;
    simxInt* q; // Joint IDs
    simxInt* endeff;
    simxFloat* q_trans;
    KDL::Chain* chain;
    KDL::ChainFkSolverPos_recursive* fksolver0;
    KDL::ChainFkSolverPos_recursive* fksolver1;
    KDL::ChainJntToJacSolver* jsolver0;
    KDL::ChainJntToJacSolver* jsolver1;
    KDL::Jacobian J0;
    KDL::Jacobian J1;
    KDL::JntArray* qkdl;
    pthread_t key_thread;
    pthread_mutex_t key_mutex;
    char key;
    Eigen::VectorXd targets;
};

#endif /* BAXTERTOOLS_H_ */
