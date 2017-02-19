/*
 * BaxterTools.cpp
 *
 *  Created on: 24 Jan 2014
 *      Author: Vladimir Ivan
 */

#include "BaxterTools.h"

void PrintError(const char* file, int line, int err)
{
  switch(err)
  {
    case simx_error_noerror:
        break;
    case simx_error_novalue_flag:
        std::cout << file << "[" << line << "]: No value\n";
        break;
    case simx_error_timeout_flag:
        std::cout << file << "[" << line << "]: Timeout\n";
        break;
    case simx_error_illegal_opmode_flag:
        std::cout << file << "[" << line << "]: Operation mode not supported\n";
        break;
    case simx_error_remote_error_flag:
        std::cout << file << "[" << line << "]: Remote error\n";
        break;
    case simx_error_split_progress_flag:
        std::cout << file << "[" << line << "]: Split command still in progress\n";
        break;
    case simx_error_local_error_flag:
        std::cout << file << "[" << line << "]: Local error\n";
        break;
    case simx_error_initialize_error_flag:
        std::cout << file << "[" << line << "]: Init. error\n";
        break;
    default:
        std::cout << file << "[" << line << "]:" << err <<"\n";
        break;

  }
}

BaxterTools::BaxterTools()
{
  Err=false;
  clientID=-1;
  q = new simxInt[18];
  endeff = new simxInt[2];
  q_trans = new simxFloat[20*7];
  chain = new KDL::Chain[2];
  fksolver0 = NULL;
  fksolver1 = NULL;
  jsolver0 = NULL;
  jsolver1 = NULL;
  qkdl = new KDL::JntArray[2];
  pthread_mutex_init(&key_mutex, NULL);
  Running = true;
  pthread_create( &key_thread, NULL, CheckKey, (void*) this);
}

void* BaxterTools::CheckKey(void* ptr)
{
  BaxterTools* bax = (BaxterTools*)ptr;
  char key;
  bax->Running = true;
  bax->key=0;

  while(bax->Running)
  {
    key=getchar();
    if(key!='\n')
    {
      pthread_mutex_lock( &bax->key_mutex );
      bax->key=key;
      pthread_mutex_unlock( &bax->key_mutex );
    }
  }
}

char BaxterTools::GetKey()
{
  char key_;
  pthread_mutex_lock( &key_mutex );
  key_ = key;
  key = 0;
  pthread_mutex_unlock( &key_mutex );
  return key_;
}

bool BaxterTools::Connect(const char* host)
{
  // Open connection
  clientID=simxStart(host,19998,true,true,2000,5);
  Err=true;
  if (clientID!=-1)
  {
    // Set synchronous mode
    PRINT_ERROR(simxSynchronous(clientID,true));

    err=simxGetObjectHandle(clientID,"right_s0",&q[0],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
    err=simxGetObjectHandle(clientID,"right_s1",&q[1],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
    err=simxGetObjectHandle(clientID,"right_e0",&q[2],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
    err=simxGetObjectHandle(clientID,"right_e1",&q[3],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
    err=simxGetObjectHandle(clientID,"right_w0",&q[4],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
    err=simxGetObjectHandle(clientID,"right_w1",&q[5],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
    err=simxGetObjectHandle(clientID,"right_w2",&q[6],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
    err=simxGetObjectHandle(clientID,"right_g1",&q[7],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
    err=simxGetObjectHandle(clientID,"right_g2",&q[8],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
    err=simxGetObjectHandle(clientID,"left_s0",&q[9],simx_opmode_oneshot_wait);  PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
    err=simxGetObjectHandle(clientID,"left_s1",&q[10],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
    err=simxGetObjectHandle(clientID,"left_e0",&q[11],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
    err=simxGetObjectHandle(clientID,"left_e1",&q[12],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
    err=simxGetObjectHandle(clientID,"left_w0",&q[13],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
    err=simxGetObjectHandle(clientID,"left_w1",&q[14],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
    err=simxGetObjectHandle(clientID,"left_w2",&q[15],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
    err=simxGetObjectHandle(clientID,"left_g1",&q[16],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
    err=simxGetObjectHandle(clientID,"left_g2",&q[17],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
    err=simxGetObjectHandle(clientID,"RightEffector",&endeff[0],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
    err=simxGetObjectHandle(clientID,"LeftEffector",&endeff[1],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;

    simxPauseCommunication(clientID,1);
    for(int i=0;i<18;i++)
    {
      err=simxSetJointPosition(clientID,q[i], 0.0,simx_opmode_oneshot); if(err!=simx_error_novalue_flag) PRINT_ERROR(err);
    }
    simxPauseCommunication(clientID,0);
    err=simxSetJointPosition(clientID,q[0], 0.0,simx_opmode_oneshot_wait); if(err!=simx_error_novalue_flag) PRINT_ERROR(err);

    {
      for(int i=1;i<16;i++)
      {
        if(i!=7 && i!=8 && i!=9)
        {
          err=simxGetObjectQuaternion(clientID,q[i],q[i-1],&q_trans[i*7],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
        }
      }
      err=simxGetObjectQuaternion(clientID,q[0],-1,&q_trans[0*7],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
      err=simxGetObjectQuaternion(clientID,q[9],-1,&q_trans[9*7],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
      err=simxGetObjectQuaternion(clientID,q[7],q[6],&q_trans[7*7],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
      err=simxGetObjectQuaternion(clientID,q[8],q[6],&q_trans[8*7],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
      err=simxGetObjectQuaternion(clientID,q[16],q[15],&q_trans[16*7],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
      err=simxGetObjectQuaternion(clientID,q[17],q[15],&q_trans[17*7],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
      err=simxGetObjectQuaternion(clientID,endeff[0],q[6],&q_trans[18*7],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
      err=simxGetObjectQuaternion(clientID,endeff[1],q[15],&q_trans[19*7],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
    }
    {
      for(int i=1;i<16;i++)
      {
        if(i!=7 && i!=8 && i!=9)
        {
          err=simxGetObjectPosition(clientID,q[i],q[i-1],&q_trans[i*7+4],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
        }
      }
      err=simxGetObjectPosition(clientID,q[0],-1,&q_trans[0*7+4],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
      err=simxGetObjectPosition(clientID,q[9],-1,&q_trans[9*7+4],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
      err=simxGetObjectPosition(clientID,q[7],q[6],&q_trans[7*7+4],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
      err=simxGetObjectPosition(clientID,q[8],q[6],&q_trans[8*7+4],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
      err=simxGetObjectPosition(clientID,q[16],q[15],&q_trans[16*7+4],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
      err=simxGetObjectPosition(clientID,q[17],q[15],&q_trans[17*7+4],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
      err=simxGetObjectPosition(clientID,endeff[0],q[6],&q_trans[18*7+4],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
      err=simxGetObjectPosition(clientID,endeff[1],q[15],&q_trans[19*7+4],simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
    }
    Err=false;
    KDL::Frame T;
    {
      chain[0].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),MakeFrame(&q_trans[0])));
      chain[1].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),MakeFrame(&q_trans[(9)*7])));
    }
    for(int i=1;i<7;i++)
    {
      chain[0].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),MakeFrame(&q_trans[i*7])));
      chain[1].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),MakeFrame(&q_trans[(9+i)*7])));
    }
    chain[0].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),MakeFrame(&q_trans[18*7])));
    chain[1].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),MakeFrame(&q_trans[19*7])));
    fksolver0 = new KDL::ChainFkSolverPos_recursive(chain[0]);
    fksolver1 = new KDL::ChainFkSolverPos_recursive(chain[1]);
    jsolver0 = new KDL::ChainJntToJacSolver(chain[0]);
    jsolver1 = new KDL::ChainJntToJacSolver(chain[1]);
    qkdl[0] = KDL::JntArray(chain[0].getNrOfJoints());
    qkdl[1] = KDL::JntArray(chain[1].getNrOfJoints());
    J0 = KDL::Jacobian(chain[0].getNrOfJoints());
    J1 = KDL::Jacobian(chain[1].getNrOfJoints());

    simxPauseCommunication(clientID,1);
    simxFloat q_[18] = {M_PI/4.0,0,0,M_PI/2.0,0,0,0,0,0,-M_PI/4.0,0,0,M_PI/2.0,0,0,0,0,0};
    for(int i=0;i<18;i++)
    {
      err=simxSetJointPosition(clientID,q[i], q_[i],simx_opmode_oneshot); if(err!=simx_error_novalue_flag) PRINT_ERROR(err);
    }

    simxPauseCommunication(clientID,0);

    SetTargets();

    return true;
  }
  return false;
}

KDL::Frame MakeFrame(simxFloat* val)
{
  KDL::Frame ret(KDL::Rotation().Quaternion(val[0],val[1],val[2],val[3]),KDL::Vector(val[4],val[5],val[6]));
  return ret;
}

Eigen::VectorXd BaxterTools::GetIK(Eigen::VectorXd q_)
{
  Eigen::VectorXd ret(12);
  KDL::Frame cartpos;
  double tmp[3];
  for(int i=0;i<7;i++)
  {
    qkdl[0](i)=q_(i);
    qkdl[1](i)=q_(9+i);
  }
  fksolver0->JntToCart(qkdl[0],cartpos,-1);
  ret(0)=cartpos.p[0]; ret(1)=cartpos.p[1]; ret(2)=cartpos.p[2];
  cartpos.M.GetRPY(tmp[0],tmp[1],tmp[2]); ret(3)=tmp[0];ret(4)=tmp[1];ret(5)=tmp[2];
  fksolver1->JntToCart(qkdl[1],cartpos);
  ret(6)=cartpos.p[0]; ret(7)=cartpos.p[1]; ret(8)=cartpos.p[2];
  cartpos.M.GetRPY(tmp[0],tmp[1],tmp[2]); ret(9)=tmp[0];ret(10)=tmp[1];ret(11)=tmp[2];
  return ret;
}
 
void BaxterTools::SetTargets()
{
  int tar_handle = GetObjectHandle("Centre");
  Eigen::VectorXd tmp, tmp1;
  GetObjectPosition(tar_handle,tmp);
  targets.resize(8*3);
  const char* tars[] = {"Target","Target0","Target1","Target2","Target3","Target4","Target5","Target6"};
  for(int i=0;i<8;i++) 
  {
    tmp1=tmp;
    tmp1(0)=tmp1(0)+(((double)i)/7-0.5)*0.2+0.2;
    tmp1(1)=tmp1(1)+(((double)i)/7-0.5)*0.4 +0.1;
    tmp1(2)=tmp1(2)-(((double)i)/7-0.5)*0.3;
    targets.segment(i*3,3)=tmp1;
    tar_handle = GetObjectHandle(tars[i]);
    SetObjectPosition(tar_handle,tmp1);
  }
}

bool BaxterTools::GetTargets(Eigen::VectorXd& x)
{
  x=targets;
  return true;
}

bool BaxterTools::SetObjectPosition(int handle, Eigen::VectorXd x)
{
  return SetObjectPosition(handle,-1,x);
}

bool BaxterTools::SetObjectPosition(int handle, int rel, Eigen::VectorXd x)
{
  if(x.rows()<3) return false;
  simxFloat tmp[3]; tmp[0]=x(0); tmp[1]=x(1); tmp[2]=x(2);
  err=simxSetObjectPosition(clientID,handle,rel,tmp,simx_opmode_oneshot);
  return true;
}

bool BaxterTools::SetObjectOrientation(int handle, Eigen::VectorXd x)
{
  return SetObjectOrientation(handle,-1,x);
}

bool BaxterTools::SetObjectOrientation(int handle, int rel, Eigen::VectorXd x)
{
  if(x.rows()<3) return false;
  double tmpd[4];
  simxFloat tmp[4];
  KDL::Rotation().RPY(x(0),x(1),x(2)).GetQuaternion(tmpd[0],tmpd[1],tmpd[2],tmpd[3]);
  for(int i=0;i<4;i++) tmp[i]=(simxFloat)tmpd[i];
  err=simxSetObjectQuaternion(clientID,handle,rel,tmp,simx_opmode_oneshot);
  return true;
}

bool BaxterTools::GetObjectPosition(int handle, Eigen::VectorXd& x)
{
  return GetObjectPosition(handle,-1,x);
}

bool BaxterTools::GetObjectPosition(int handle, int rel, Eigen::VectorXd& x)
{
  if(x.rows()!=3) x.resize(3);
  simxFloat tmp[3];
  err=simxGetObjectPosition(clientID,handle,rel,tmp,simx_opmode_oneshot_wait);
  x(0)=tmp[0]; x(1)=tmp[1]; x(2)=tmp[2];
  return true;
}

bool BaxterTools::GetObjectOrientation(int handle, Eigen::VectorXd& x)
{
  return GetObjectOrientation(handle,-1,x);
}

bool BaxterTools::GetObjectOrientation(int handle, int rel, Eigen::VectorXd& x)
{
  if(x.rows()!=3) x.resize(3);
  simxFloat tmp[4];
  err=simxGetObjectQuaternion(clientID,handle,rel,tmp,simx_opmode_oneshot_wait);
  KDL::Rotation().Quaternion(tmp[0],tmp[1],tmp[2],tmp[3]).GetRPY(x(0),x(1),x(2));
  return true;
}

int BaxterTools::GetObjectHandle(const char* name)
{
  simxInt ret;
  if(simxGetObjectHandle(clientID,name,&ret,simx_opmode_oneshot_wait)!=simx_error_noerror)
  {
    return -1;
  }
  return (int)ret;
}

Eigen::MatrixXd BaxterTools::GetJ(Eigen::VectorXd q_)
{
  Eigen::MatrixXd ret=Eigen::MatrixXd::Zero(J0.rows()*2,J0.columns()*2);
  for(int i=0;i<7;i++)
  {
    qkdl[0](i)=q_(i);
    qkdl[1](i)=q_(9+i);
  }
  jsolver0->JntToJac(qkdl[0],J0);
  jsolver1->JntToJac(qkdl[1],J1);
  for(int i=0;i<J0.rows();i++)
  {
    for(int j=0;j<J0.columns();j++)
    {
      ret(i,j)=J0(i,j);
      ret(i+J0.rows(),j+J0.columns())=J1(i,j);
    }
  }
  return ret;
}

bool BaxterTools::StartSimulation()
{
  Err=true;
  if (clientID!=-1)
  {
    err=simxStartSimulation(clientID,simx_opmode_oneshot_wait); PRINT_ERROR(err); if(err!=simx_error_noerror) return false;
    Sim=true;
    Err=false;
    return true;
  }
  return false;
}

bool BaxterTools::StopSimulation()
{
  Err=true;
  if (clientID!=-1)
  {
    err=simxStopSimulation(clientID,simx_opmode_oneshot_wait);
    Sim=false;
    Err=false;
    return true;
  }
  return false;
}

bool BaxterTools::CheckRunning()
{
  if (clientID!=-1)
  {
    if(simxGetConnectionId(clientID)!=-1 && Err==false) return true;
    return false;
  }
  return false;
}

bool BaxterTools::AdvanceSimulation()
{
  if (clientID!=-1 && Sim)
  {
    err=simxSynchronousTrigger(clientID); PRINT_ERROR(err);
    if(err!=simx_error_noerror) Err=true;
  }
  return false;
}

BaxterTools::~BaxterTools()
{
  if (clientID!=-1)
  {
    StopSimulation();
    simxFinish(clientID);
  }
  Running = false;
  pthread_kill(key_thread, SIGUSR1);
  pthread_join( key_thread, NULL);
  delete[] q;
  delete[] endeff;
  delete[] chain;
  delete fksolver0;
  delete fksolver1;
  delete[] qkdl;
}

bool BaxterTools::SetJointAngles(Eigen::VectorXd q_)
{
  if (clientID!=-1 && q_.rows()==18)
  {
    simxPauseCommunication(clientID,1);
    for(int i=0;i<18;i++)
    {
      err=simxSetJointPosition(clientID,q[i], q_[i],simx_opmode_oneshot); if(err!=simx_error_novalue_flag) PRINT_ERROR(err);
    }
    simxPauseCommunication(clientID,0);
    return true;
  }
  return false;
}


