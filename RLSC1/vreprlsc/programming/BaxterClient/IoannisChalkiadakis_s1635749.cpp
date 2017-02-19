/*
 * Baxter.cpp
 *
 *  Created on: 27 Feb 2017
 *  Author: Ioannis Chalkiadakis
 */

// Make sure to have the server side running in V-REP!

#include "BaxterTools.h"
#include <unistd.h>

// Auxiliary function to compute difference of vectors a-b
Eigen::VectorXd subtractVec(Eigen::VectorXd a, Eigen::VectorXd b) {
  
  int len = a.rows();
  Eigen::VectorXd diff = Eigen::VectorXd::Zero(len);

  for (int i=0; i<len; i++) {
    diff(i) = a(i)-b(i);    
  }

  return diff;  
}


int getInput(Eigen::VectorXd* target, Eigen::VectorXd* targets, Eigen::VectorXd* q_start, Eigen::VectorXd* q, Eigen::VectorXd* q_comf, int* y_init, int* y_end, int* q_init, int* q_end, int* Jac_row, int* Jac_col, BaxterTools *bax, double* dy) {

          char key;
	  int y_initR,y_endR,q_initR,q_endR,y_initL,y_endL,q_initL,q_endL,Jac_rowR,Jac_colR,Jac_rowL,Jac_colL;
	  Eigen::VectorXd qstart1(18); // Starting pose 1
	  Eigen::VectorXd qstart2(18); // Starting pose 2
	  Eigen::VectorXd qstart3(18); // Starting pose 3
	  qstart1 << M_PI/4.0,0,0,M_PI/2.0,0,0,0,0,0,      -M_PI/4.0,0,0,M_PI/2.0,0,0,0,0,0;
	  qstart2 << -M_PI/4.0,0,0,M_PI/2.0,M_PI/2.0,M_PI/2.0,0,0,0,   M_PI/4.0,0,0,M_PI/2.0,-M_PI/2.0,M_PI/2.0,0,0,0;
	  qstart3 << M_PI/4.0,-M_PI/2.0,0,M_PI/2.0,-M_PI/4.0,-M_PI/4.0,0,0,0,      -M_PI/4.0,-M_PI/2.0,0,M_PI/2.0,M_PI/4.0,-M_PI/4.0,0,0,0;
	  Eigen::VectorXd q_comf1(18); // Comfortable pose 1
	  Eigen::VectorXd q_comf2(18); // Comfortable pose 2
	  q_comf1 << 0,-0.5,0,0.5,0,1.5,0,0,0,  0,-0.5,0,0.5,0,1.5,0,0,0 ;
	  q_comf2 << -20.0/180.0*M_PI, 40.0/180.0*M_PI, 70.0/180.0*M_PI, 90.0/180.0*M_PI, 0,0.5,0,0,0, 20.0/180.0*M_PI, 40.0/180.0*M_PI, -70.0/180.0*M_PI, 90.0/180.0*M_PI, 0,0.5,0,0,0;
	  
	  //Start and end positions of right/left arm parameters in position and pose vectors
	  y_initR = 0;
	  y_endR = 2;
	  q_initR = 0;
	  q_endR = 6;
	  y_initL = 6;
	  y_endL = 8;
	  q_initL = 9;
	  q_endL = 15;
	  Jac_rowR = 0;
	  Jac_colR = 0;
	  Jac_rowL = 6;
	  Jac_colL = 7;

	  //Select end-effector target
	  std::cout<<"Select target (0-7):"<<std::endl;
	  while (key!='0' & key!='1' & key!='2' & key!='3' & key!='4' & key!='5' & key!='6' & key!='7') {
	    key=bax->GetKey();
	  }
	  (*target) = (*targets).segment((int)(key-'0')*3,3);

	  //Select which arm to use
  	  key=0;
	  std::cout<<"Select arm (1 for left, 2 for right):"<<std::endl;
  	  while (key!='1' & key!='2') {
	    key=bax->GetKey();
	  }
	  switch (key) {
	      case '1': (*y_init) = y_initL;
		        (*y_end) = y_endL;
			(*q_init) = q_initL;
		        (*q_end) = q_endL;
		        (*Jac_row) = Jac_rowL;
		        (*Jac_col) = Jac_colL;
		        break;
     	      case '2': (*y_init) = y_initR;
		        (*y_end) = y_endR;
			(*q_init) = q_initR;
		        (*q_end) = q_endR;
		        (*Jac_row) = Jac_rowR;
		        (*Jac_col) = Jac_colR;
		        break;
	  }

	  //Select starting pose and compute initial square error between starting position and target
	  key=0;
	  std::cout<<"Select starting pose (1-3):"<<std::endl;
	  while (key!='1' & key!='2' & key!='3') {
	    key=bax->GetKey();
	  }
	  switch (key) {
	    case '1': (*q_start) = qstart1;
 	              (*dy) = subtractVec((*target),(bax->GetIK((*q_start))).segment((*y_init),3)).squaredNorm()/3;
		      (*q) = (*q_start);
		      break;
	    case '2': (*q_start) = qstart2;
	              (*dy) = subtractVec((*target),(bax->GetIK((*q_start))).segment((*y_init),3)).squaredNorm()/3;
		      (*q) = (*q_start);
		      break;
            case '3': (*q_start) = qstart3;
	              (*dy) = subtractVec((*target),(bax->GetIK((*q_start))).segment((*y_init),3)).squaredNorm()/3;
		      (*q) = (*q_start);
		      break;
	  }

	  //Select comfortable pose
	  key=0;
	  std::cout<<"Select comfortable pose (1-2):"<<std::endl;
 	  while (key!='1' & key!='2') {
	    key=bax->GetKey();
	  }
	  switch (key) {
              case '1': (*q_comf) = q_comf1;
		      break;
              case '2': (*q_comf) = q_comf2;
		      break;
	  }
	  bax->SetJointAngles((*q_comf));
	  bax->AdvanceSimulation();
	  sleep(2);
	  
	  return 0;
}



Eigen::MatrixXd getPseudoJacobian(Eigen::MatrixXd W,Eigen::MatrixXd C_inv,Eigen::MatrixXd J_pos) {
  //Calculate pseudoinverse of given Jacobian
  Eigen::MatrixXd W_inv = W.inverse();
  Eigen::MatrixXd J_t = J_pos.transpose();
  Eigen::MatrixXd J_sharp_part = J_pos*W_inv*J_t + C_inv;
 
  return W_inv*J_t*(J_sharp_part.inverse());
  
}


Eigen::VectorXd updatePose(Eigen::MatrixXd J_pseud, Eigen::MatrixXd J_pos, Eigen::VectorXd q, Eigen::VectorXd q_comf, Eigen::VectorXd y_star, Eigen::VectorXd y) {

    Eigen::VectorXd dy = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(7);
    Eigen::MatrixXd I  = Eigen::MatrixXd::Identity(7,7);
    //Compute target-current_position and q_comf-current_pose
    dy = subtractVec(y_star,y);
    dq = subtractVec(q_comf,q);
    //Return pose update
    return J_pseud*dy + (I - J_pseud*J_pos)*dq; 
    
    
}

Eigen::VectorXd getInvKinematics(Eigen::VectorXd q, Eigen::VectorXd target, BaxterTools *bax, Eigen::MatrixXd W, Eigen::MatrixXd C_inv, Eigen::VectorXd q_comf,  int y_initIdx, int y_endIdx, int q_initIdx , int q_endIdx, int Jac_row, int Jac_col) {

  Eigen::VectorXd q_old  = Eigen::VectorXd::Zero(18);
  Eigen::VectorXd q_upd  = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd y      = Eigen::VectorXd::Zero(3);
  double dq;  
  Eigen::MatrixXd J;

  q_old = q;
  //Get end-effector position
  y = (bax->GetIK(q_old)).segment(y_initIdx,y_endIdx-y_initIdx+1);
  //Get Jacobian
  J = bax->GetJ(q_old);
  //Get position Jacobian for selected arm
  Eigen::MatrixXd J_pos = J.block(Jac_row,Jac_col,3,7);
  //Get pseudoinverse of Jacobian
  Eigen::MatrixXd J_pseud = getPseudoJacobian(W,C_inv,J_pos);
  //Get pose update for selected arm
  q_upd = updatePose(J_pseud,J_pos,q_old.segment(q_initIdx,q_endIdx-q_initIdx+1),q_comf.segment(q_initIdx,q_endIdx-q_initIdx+1),target,y);

   //Add the update to current pose
  int j = 0;
  for (int k=q_initIdx; k<q_endIdx+1; k++) {
     q(k) = q_old(k) + q_upd(j);
     j++;
  }

  //Make sure joint position is within joint limits
  if (q(q_initIdx)<-1.7016){
    //std::cout<<"s0 locked min"<<std::endl;
     q(q_initIdx) = -1.7016;
  }
  else if (q(q_initIdx)>1.7016){
    //std::cout<<"s0 locked max"<<std::endl;
     q(q_initIdx) = 1.7016;
  }
  if (q(q_initIdx+1)<-2.147){
    //std::cout<<"s1 locked min"<<std::endl;
    q(q_initIdx+1) = -2.147;
  }
  else if (q(q_initIdx+1)>1.047){
    //std::cout<<"s1 locked max"<<std::endl;
    q(q_initIdx+1) = 1.047;
  }
  if (q(q_initIdx+2)<-3.0541){
    //std::cout<<"e0 locked min"<<std::endl;
    q(q_initIdx+2) = -3.0541;
  }
  else if (q(q_initIdx+2)>3.0541){
    //std::cout<<"e0 locked max"<<std::endl;
    q(q_initIdx+2) = 3.0541;
  }
  if (q(q_initIdx+3)<-0.05){
    //std::cout<<"e1 locked min"<<std::endl;
    q(q_initIdx+3) = -0.05;
  }
  else if (q(q_initIdx+3)>2.618){
    //std::cout<<"e1 locked max"<<std::endl;
    q(q_initIdx+3) = 2.618;
  }
  if (q(q_initIdx+4)<-3.059){
    //std::cout<<"w0 locked min"<<std::endl;
    q(q_initIdx+4) = -3.059;
  }
  else if (q(q_initIdx+4)>3.059){
    //std::cout<<"w0 locked max"<<std::endl;
    q(q_initIdx+4) = 3.059;
  }
  if (q(q_initIdx+5)<-1.5707){
    //std::cout<<"w1 locked min"<<std::endl;
    q(q_initIdx+5) = -1.5707;
  }
  else if (q(q_initIdx+5)>2.094){
    //std::cout<<"w1 locked max"<<std::endl;
    q(q_initIdx+5) = 2.094;
  }
  if (q(q_initIdx+6)<-3.059){
    //std::cout<<"w2 locked min"<<std::endl;
    q(q_initIdx+6) = -3.059;
  }
  else if (q(q_initIdx+6)>3.059){
    //std::cout<<"w2 locked max"<<std::endl;
    q(q_initIdx+6) = 3.059;
  }
  

  return q;
}


int main(int argc,char* argv[]) {
        // Create the robot interface object
        BaxterTools bax;
	// Connect to the simulator
        if (argc==2) {
	  bax.Connect(argv[1]);
        }
        else {
	  bax.Connect("localhost");
        }
        // Start the simulation
	bax.StartSimulation();
        Eigen::VectorXd q = Eigen::VectorXd::Zero(18);     // Joint angles
	Eigen::VectorXd x;                                 // End-effector position
	Eigen::VectorXd target = Eigen::VectorXd::Zero(3); // Target position
	Eigen::VectorXd targets = Eigen::VectorXd::Zero(24); // All target positions
	Eigen::MatrixXd J;                                 // Jacobian matrix
	Eigen::MatrixXd poses_pca = Eigen::MatrixXd::Zero(24,7);
	Eigen::VectorXd q_start = Eigen::VectorXd::Zero(18);
	Eigen::VectorXd q_comf = Eigen::VectorXd::Zero(18); 
	Eigen::VectorXd q_new = Eigen::VectorXd::Zero(18); 
	// Used to pass around vector position according to selected arm
	int y_init,y_end,q_init,q_end,Jac_row,Jac_col;
	double dy;
	char key=' ';
	double epsY; // tolerance of square_error(target,end-eff position)
	epsY = 0.00001;
	//int i = 0;
	//////////////////////////////////////////////////////////////////////
	// Constants for homework
	Eigen::MatrixXd W = Eigen::MatrixXd::Zero(7,7);         // Weighting matrix
	for(int i=0;i<7;i++) W(i,i) = ((double)i)/6.0+0.1;
	
	Eigen::MatrixXd C = Eigen::MatrixXd::Identity(3,3)*1e3; // Regularisation
	Eigen::MatrixXd Cinv = Eigen::MatrixXd::Identity(3,3)*1e-3;
	//////////////////////////////////////////////////////////////////////
	
	//Get target positions
	bax.GetTargets(targets);

	//Select mode
	std::cout<<"Press 'c' to set configuration for one task or 'd' to start demo of reaching all targets"<<std::endl;
        while (key!='c' & key!='d'){
	  key=bax.GetKey();
	}
	
	if (key=='c') {
	  //Ask user for arm,target,comfortable pose,start pose	
	  getInput(&target, &targets, &q_start, &q, &q_comf, &y_init, &y_end, &q_init, &q_end, &Jac_row, &Jac_col, &bax, &dy);
	  //Set Baxter at starting pose
	  bax.SetJointAngles(q_start);
	  bax.AdvanceSimulation();
	  sleep(1);
	
	  // Loop until 'q' gets pressed
	  while(key!='q')  {
	    key=bax.GetKey();
	    while ((dy > epsY) & (key!='q'))  {
	      //Get updated pose
	      q_new = getInvKinematics(q, target, &bax, W, Cinv, q_comf, y_init, y_end, q_init, q_end, Jac_row, Jac_col);
	      //Compute square error between arm end-effector position and target
	      dy = subtractVec(target,(bax.GetIK(q_new)).segment(y_init,3)).squaredNorm()/3;
	      q = q_new;
	      //Set Baxter to new pose
	      bax.SetJointAngles(q);
	      bax.AdvanceSimulation();
	      //sleep(1);
	      key=bax.GetKey();
	    }
	    //std::cout<<"Reached target"<<std::endl;	  
	    /*if (key!='q'){
	      for (int j=0; j<7; j++){
	       poses_pca(i,j) = q(q_init+j);
	      }
	      i++;
	      q = Eigen::VectorXd::Zero(18);
	      q_start = Eigen::VectorXd::Zero(18);
	      q_comf = Eigen::VectorXd::Zero(18);
	      dy=0;
	      getInput(&target, &targets, &q_start, &q, &q_comf, &y_init, &y_end, &q_init, &q_end, &Jac_row, &Jac_col, &bax, &dy);
	      bax.SetJointAngles(q_start);
	      sleep(2);
	      bax.AdvanceSimulation();
	     }*/	 

	  }
	  bax.SetJointAngles(q_new);
	  bax.AdvanceSimulation();
	  //std::cout<<poses_pca<<std::endl;
	  //Uncomment next line for Part B, 2.2, norm in null space
	  //q = q_comf;
	  //Compute weighted distance of q_new and q_start
	  Eigen::VectorXd q_diff = subtractVec(q_start.segment(q_init,q_end-q_init+1),q.segment(q_init,q_end-q_init+1));
	  double q_cost = q_diff.transpose()*W*q_diff;
	  std::cout<<"Weighted cost between start and end poses is: "<<q_cost<<std::endl;
	  // Stop simulation and close connection
	  bax.StopSimulation();
	}
	  else if (key=='d'){  
	    /////////////////////////////////////////////
	    //Reaching all targets with the right arm starting from q_start1, q_comf0
	    q_start << M_PI/4.0,0,0,M_PI/2.0,0,0,0,0,0,      -M_PI/4.0,0,0,M_PI/2.0,0,0,0,0,0;
	    q_comf  << 0,-0.5,0,0.5,0,1.5,0,0,0,  0,-0.5,0,0.5,0,1.5,0,0,0 ;
            bax.SetJointAngles(q_comf);
	    bax.AdvanceSimulation();
	    std::cout<<"This is the comfortable pose."<<std::endl;
	    sleep(5);
	    y_init = 0;
	    y_end = 2;
	    q_init = 0;
	    q_end = 6;
	    Jac_row = 0;
	    Jac_col = 0;
            bax.SetJointAngles(q_start);
	    bax.AdvanceSimulation();
	    std::cout<<"Got starting pose..."<<std::endl;
	    sleep(4);
            std::cout<<"Start sequence reaching targets"<<std::endl;
	    q = q_start;
	    for (int i=0; i<8; i++){
	      dy = subtractVec(targets.segment(i*3,3),(bax.GetIK(q_start)).segment(y_init,3)).squaredNorm()/3;
	      while ((dy > epsY))  {
		//Get updated pose
		q_new = getInvKinematics(q, targets.segment(i*3,3), &bax, W, Cinv, q_comf, y_init, y_end, q_init, q_end, Jac_row, Jac_col);
		//Compute square error between arm end-effector position and target
		dy = subtractVec(targets.segment(i*3,3),(bax.GetIK(q_new)).segment(y_init,3)).squaredNorm()/3;
		q = q_new;
		//Set Baxter to new pose
		bax.SetJointAngles(q);
		bax.AdvanceSimulation();
		sleep(1);
	      }
	      std::cout<<"Reached target "<<i<<std::endl;
	      sleep(1);
	      q_start = q;	  
	    }
	    std::cout<<"Exiting"<<std::endl;
	    bax.StopSimulation();
	  }
	  
	return(0);
}
