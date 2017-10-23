/********************************************************************************
Copyright (c) 2016, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, 
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software 
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/
#include <vector>
#include <iostream>
#include <fstream>

#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <sensor_msgs/JointState.h>

double fRand(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}


void test(ros::NodeHandle& nh, double num_samples, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param)
{

  double eps = 1e-5;

  // This constructor parses the URDF loaded in rosparm urdf_param into the
  // needed KDL structures.  We then pull these out to compare against the KDL
  // IK solver.
  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits

  bool valid = tracik_solver.getKDLChain(chain);
  
  if (!valid) {
    ROS_ERROR("There was no valid KDL chain found");
    return;
  }

  valid = tracik_solver.getKDLLimits(ll,ul);

  if (!valid) {
    ROS_ERROR("There were no valid KDL joint limits found");
    return;
  }

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());

  ROS_INFO ("Using %d joints",chain.getNrOfJoints());


  // Set up KDL IK
  KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
  KDL::ChainIkSolverVel_pinv vik_solver(chain); // PseudoInverse vel solver
  KDL::ChainIkSolverPos_NR_JL kdl_solver(chain,ll,ul,fk_solver, vik_solver, 1, eps); // Joint Limit Solver
  // 1 iteration per solve (will wrap in timed loop to compare with TRAC-IK) 


  // Create Nominal chain configuration midway between all joint limits
  KDL::JntArray nominal(chain.getNrOfJoints());

  for (uint j=0; j<nominal.data.size(); j++) {
    nominal(j) = (ll(j)+ul(j))/2.0;
  }

  // Create desired number of valid, random joint configurations
  std::vector<KDL::JntArray> JointList;
  KDL::JntArray q(chain.getNrOfJoints());
   
  for (uint i=0; i < num_samples; i++) {
    for (uint j=0; j<ll.data.size(); j++) {
      q(j)=fRand(ll(j), ul(j));
    }    
    JointList.push_back(q);
  }


  boost::posix_time::ptime start_time;
  boost::posix_time::time_duration diff;

  KDL::JntArray result;
  KDL::Frame end_effector_pose;
  int rc;

  double total_time=0;
  uint success=0;

  ROS_INFO_STREAM("*** Testing KDL with "<<num_samples<<" random samples");

  for (uint i=0; i < num_samples; i++) {
    fk_solver.JntToCart(JointList[i],end_effector_pose);
    double elapsed = 0;
    result=nominal; // start with nominal
    start_time = boost::posix_time::microsec_clock::local_time();
    do {
      q=result; // when iterating start with last solution
      rc=kdl_solver.CartToJnt(q,end_effector_pose,result);
      diff = boost::posix_time::microsec_clock::local_time() - start_time;
      elapsed = diff.total_nanoseconds() / 1e9;
    } while (rc < 0 && elapsed < timeout);
    total_time+=elapsed;
    if (rc>=0)
      success++;
    
    if (int((double)i/num_samples*100)%10 == 0)
      ROS_INFO_STREAM_THROTTLE(1,int((i)/num_samples*100)<<"\% done");
  }

  ROS_INFO_STREAM("KDL found "<<success<<" solutions ("<<100.0*success/num_samples<<"\%) with an average of "<<total_time/num_samples<<" secs per sample");


  total_time=0;
  success=0;

  ROS_INFO_STREAM("*** Testing TRAC-IK with "<<num_samples<<" random samples");

  for (uint i=0; i < num_samples; i++) {
    fk_solver.JntToCart(JointList[i],end_effector_pose);
    double elapsed = 0;
    start_time = boost::posix_time::microsec_clock::local_time();
    rc=tracik_solver.CartToJnt(nominal,end_effector_pose,result);
    diff = boost::posix_time::microsec_clock::local_time() - start_time;
    elapsed = diff.total_nanoseconds() / 1e9;
    total_time+=elapsed;
    if (rc>=0) {
      ROS_INFO("%f, %f, %f",end_effector_pose.p.data[0], end_effector_pose.p.data[1], end_effector_pose.p.data[2]);
      success++;
    }
    
    if (int((double)i/num_samples*100)%10 == 0)
      ROS_INFO_STREAM_THROTTLE(1,int((i)/num_samples*100)<<"\% done");
  }

  ROS_INFO_STREAM("TRAC-IK found "<<success<<" solutions ("<<100.0*success/num_samples<<"\%) with an average of "<<total_time/num_samples<<" secs per sample");
}


void transform_to_base(double end[])
{
  double world[] = {0.5, 0.0, 0.0};

  for (size_t i = 0; i < 3; i++)
  {
    end[i] = end[i] - world[i];
    // end[i]  = end[i]/2.0; // use ratio to translation origin data
  }

}


void read_data_from_file(const char filename[], std::vector<KDL::Frame> & end_effector_pose_list)
{
     std::ifstream fin;
     fin.open(filename);

     int i = 0;
     double rotation_temp[9];
     KDL::Frame frame;

     for (size_t it = 0; it < 9; it++)
     {
       rotation_temp[it] = 0.0;
     }
     
     if(fin.is_open()){

        while (!fin.eof()) {
           end_effector_pose_list.push_back(frame);

          for (size_t j = 0; j < 3; j++)
          { 
            fin >> end_effector_pose_list[i].p.data[j];
            end_effector_pose_list[i].p.data[j] = end_effector_pose_list[i].p.data[j]/1000.0;

            // std::cout<<end_effector_pose_list[i].p.data[j]<<" ";
          }

          // we move robot to new point (1.0, 0.1, 0.3) relative the coordinate of the world of tracked data.
          transform_to_base(end_effector_pose_list[i].p.data);

          for (size_t k = 0; k < 9; k++)
          {
            fin>>rotation_temp[k];
            end_effector_pose_list[i].M.data[k] = rotation_temp[k];

            // if (k<=7)
            // {
            //   std::cout<<rotation_temp[k]<<" ";
            // } else
            // {
            //   std::cout<<rotation_temp[k]<<std::endl;
            // }
          }

          // transform rotation matrix data from column to row
          for (size_t k1 = 0; k1 < 3; k1++)
          {
            end_effector_pose_list[i].M.data[k1] = rotation_temp[3*k1];
            end_effector_pose_list[i].M.data[k1+3] = rotation_temp[3*k1+1];
            end_effector_pose_list[i].M.data[k1+6] = rotation_temp[3*k1+2];
          }

          i++;
        }

        i--;
        end_effector_pose_list.pop_back();

        std::cout<<"We read "<<i<<" lines from file, and actually we will use "<< end_effector_pose_list.size() <<std::endl;
        fin.close();
     } else
     {
       std::cout<<"no file "<<filename<<" found"<<std::endl;
     }
}

void save_to_file(const char output_filename[], int rc,  KDL::JntArray result)
{
     std::ofstream fout;
     fout.open(output_filename, std::ios_base::app);

     if (!fout.is_open())
     {
       return;
     }

     unsigned int num_joints = result.rows();

     if (rc >= 0)
     {
       for (size_t i = 0; i < num_joints; i++)
       {
         if (i<num_joints -1)
         {
            fout<< result.data(i)<<" ";
         } else
         {
           fout<< result.data(i)<<std::endl;
         }
       }
     } else {
      //  not save to file
     }
     
     // close file
     fout.close();
}


void wrap_to_joint_state(sensor_msgs::JointState & msg, KDL::JntArray result)
{
      msg.name.resize(6);
      msg.header.stamp = ros::Time::now(); // very important for rviz
      std::ostringstream oss;
      
      for (size_t i = 0; i < 6; i++)
      {
        oss.clear();
        oss<<"joint"<<(i+1);
        msg.name[i] = oss.str();
        oss.str("");
      }

      msg.position.resize(6);

      for (size_t i = 0; i < msg.position.size(); i++)
      {
        msg.position[i] = result.data(i);
      }
            
}


void generateIK(ros::NodeHandle& nh, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param)
{
    ros::Publisher chatter_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1000);

    sensor_msgs::JointState msg;

    ros::Rate loop_rate(10);

    int replay_num = 10;


    const char output_filename[] = "/data/ik-catkin_ws/output.txt";
    const char input_filename[] = "/data/ik-catkin_ws/rotateMaxData.txt";

    double eps = 1e-5;

  // This constructor parses the URDF loaded in rosparm urdf_param into the
  // needed KDL structures.  We then pull these out to compare against the KDL
  // IK solver.
  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits

  bool valid = tracik_solver.getKDLChain(chain);
  
  if (!valid) {
    ROS_ERROR("There was no valid KDL chain found");
    return;
  }

  valid = tracik_solver.getKDLLimits(ll,ul);

  if (!valid) {
    ROS_ERROR("There were no valid KDL joint limits found");
    return;
  }

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());

  ROS_INFO ("Using %d joints",chain.getNrOfJoints());

  KDL::JntArray nominal(chain.getNrOfJoints());

  for (uint j=0; j<nominal.data.size(); j++) {
    nominal(j) = (ll(j)+ul(j))/2.0;
  }

  KDL::JntArray result;
  KDL::Frame end_effector_pose;
  std::vector<KDL::Frame> end_effector_pose_list;
  int rc;

  read_data_from_file(input_filename, end_effector_pose_list);

  for (size_t count = 0; count < replay_num; count++)
  { // replay ten times


  for (size_t i = 0; i < end_effector_pose_list.size(); i++)
  {
      if (!ros::ok())
      {
        return;
      }
      

      end_effector_pose = end_effector_pose_list[i];

      for (size_t j = 0; j < 3; j++)
      {
        std::cout<<end_effector_pose.p.data[j]<<" ";
      }
      

      for (size_t k = 0; k < 9; k++)
      {
        if (k>=8)
        {
          std::cout<<end_effector_pose.M.data[k]<<std::endl;
        } else
        {
          std::cout<<end_effector_pose.M.data[k]<<" ";
        }
        
      }

      rc=tracik_solver.CartToJnt(nominal,end_effector_pose,result);

      if (rc<0) {
        // ROS_INFO(" failed!");
        // save "fail"  to file
        if (count == 0)
        {
         save_to_file(output_filename, rc, result);
        }
      } else {
        // save joint data to file
        ROS_INFO("GET Solution!");
        if (count == 0)
        {
          save_to_file(output_filename, rc, result);
        }

        wrap_to_joint_state(msg, result);
        chatter_pub.publish(msg);
        loop_rate.sleep();
      }
      ros::spinOnce();
  }

  } // end `for` aobut `count`, end replay
}


int main(int argc, char** argv)
{
  srand(1);
  ros::init(argc, argv, "ik_tests");
  ros::NodeHandle nh("~");

  int num_samples;
  std::string chain_start, chain_end, urdf_param;
  double timeout;

  nh.param("num_samples", num_samples, 1000);
  nh.param("chain_start", chain_start, std::string(""));
  nh.param("chain_end", chain_end, std::string(""));
  
  if (chain_start=="" || chain_end=="") {
    ROS_FATAL("Missing chain info in launch file");
    exit (-1);
  }

  nh.param("timeout", timeout, 0.005);
  nh.param("urdf_param", urdf_param, std::string("/robot_description"));

  if (num_samples < 1)
    num_samples = 1;

  // test(nh, num_samples, chain_start, chain_end, timeout, urdf_param);

  generateIK(nh, chain_start, chain_end, timeout, urdf_param);

  // test file path
      // std::ifstream in("/data/ik-catkin_ws/rotateMaxData.txt");
      //  if (! in.is_open())  
      //  { std::cout << "Error opening file"; }
      //  else
      //  {
      //    std::cout<<"opened!"<<std::endl;
      //  }

      // test file path
// std::vector<KDL::Frame>  end_effector_pose_list;
    // read_data_from_file("/data/ik-catkin_ws/rotateMaxData.txt", end_effector_pose_list);


  // Useful when you make a script that loops over multiple launch files that test different robot chains
  // std::vector<char *> commandVector;
  // commandVector.push_back((char*)"killall");
  // commandVector.push_back((char*)"-9");
  // commandVector.push_back((char*)"roslaunch");
  // commandVector.push_back(NULL);  

  // char **command = &commandVector[0];
  // execvp(command[0],command);

  return 0;
}
