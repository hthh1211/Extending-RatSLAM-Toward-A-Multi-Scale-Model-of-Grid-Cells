/*
 * openRatSLAM
 *
 * utils - General purpose utility helper functions mainly for angles and readings settings
 *
 * Copyright (C) 2012
 * David Ball (david.ball@qut.edu.au) (1), Scott Heath (scott.heath@uqconnect.edu.au) (2)
 *
 * RatSLAM algorithm by:
 * Michael Milford (1) and Gordon Wyeth (1) ([michael.milford, gordon.wyeth]@qut.edu.au)
 *
 * 1. Queensland University of Technology, Australia
 * 2. The University of Queensland, Australia
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <iostream>
using namespace std;

#include "utils/utils.h"

#include <boost/property_tree/ini_parser.hpp>

#include <ros/ros.h>

#include "ratslam/posecell_network.h"
#include <ratslam_ros/TopologicalAction.h>
#include <nav_msgs/Odometry.h>
#include <ratslam_ros/ViewTemplate.h>


#if HAVE_IRRLICHT
#include "graphics/posecell_scene.h"
ratslam::PosecellScene *pcs_1;
ratslam::PosecellScene *pcs_2;
ratslam::PosecellScene *pcs_3;
std::vector<ratslam::PosecellScene*> pcss;
bool use_graphics;
#endif

using namespace ratslam;

ratslam_ros::TopologicalAction pc_output;

//int module_display_id=2;

//auto posecell_module(unsigned int index,unsigned int size,std::vector<ratslam::PosecellNetwork*> &pcns,ratslam::MultiplePosecellNetwork * mpc)
//{
//if(index<size){auto& element=pcns[index];};
//if(index=size){auto& element=mpc;};
//return element;
//}

template<typename T> void publish_posecells_message(T& element,ros::Publisher &pub_pc)
{
          pc_output.src_id = element->get_current_exp_id();
          pc_output.action = element->get_action();
          if (pc_output.action != ratslam::PosecellNetwork::NO_ACTION)
          {
            pc_output.header.stamp = ros::Time::now();
            pc_output.header.seq++;
            pc_output.dest_id = element->get_current_exp_id();
            pc_output.relative_rad = element->get_relative_rad();
            //if(index==1){
              //pub_pc.publish(pc_output);
              //}
            //if(index==2){pub_pc_2->publish(pc_output);}
            ROS_DEBUG_STREAM("PC:action_publish{odo}{" << ros::Time::now() << "} action{" << pc_output.header.seq << "}=" <<  pc_output.action << " src=" << pc_output.src_id << " dest=" << pc_output.dest_id);
          }
}

void odo_callback(nav_msgs::OdometryConstPtr odo, std::vector<ratslam::PosecellNetwork*> &pcns, std::vector<ros::Publisher> &pub_pc)
//void odo_callback(nav_msgs::OdometryConstPtr odo, std::vector<ratslam::PosecellNetwork*> &pcns, std::vector<ros::Publisher> &pub_pc)
{
  ROS_DEBUG_STREAM("PC:odo_callback{" << ros::Time::now() << "} seq=" << odo->header.seq << " v=" << odo->twist.twist.linear.x << " r=" << odo->twist.twist.angular.z);

  static ros::Time prev_time(0);
  if (prev_time.toSec() > 0)
  {
    std::vector<std::vector<double>> best_xyth_multiple;
    std::vector<double> best_xyth;
    std::vector<double> th_vt_delta_pc_mul;
    double time_diff = (odo->header.stamp - prev_time).toSec();
    unsigned int index=0;
   volatile  unsigned int size=pcns.size();
    //int pcns_size=pcns.size();
    //for (auto&& element:pcns)
    for(index;index<size;++index)
    {  
      //auto& element=posecell_module(index,pcns.size(),);
           
          //if(index==module_display_id){
            
            //}
            pcns[index]->on_odo(odo->twist.twist.linear.x, odo->twist.twist.angular.z, time_diff);
            publish_posecells_message(pcns[index],pub_pc[index]);
    }
    
//     mpc->read_bestpositions(best_xyth_multiple,th_vt_delta_pc_mul);
     //mpc->get_action();
     //

#ifdef HAVE_IRRLICHT
	if (use_graphics)
	{
//    int index=1;
    for (auto&& element:pcss){
    //if(index==2){
		element->update_scene();
		element->draw_all();//}
//    index++;
    }
	}
#endif
  }
  prev_time = odo->header.stamp;
}

void template_callback(ratslam_ros::ViewTemplateConstPtr vt, std::vector<ratslam::PosecellNetwork*> &pcns, std::vector<ros::Publisher> &pub_pc)
{
  ROS_DEBUG_STREAM("PC:vt_callback{" << ros::Time::now() << "} seq=" << vt->header.seq << " id=" << vt->current_id << " rad=" << vt->relative_rad);
  //int i;
  //unsigned int pcns_size=
  for (auto&& element:pcns) 
  {
    element->on_view_template(vt->current_id, vt->relative_rad);
  }
  
        

#ifdef HAVE_IRRLICHT
	if (use_graphics)
	{
 //   int index=1;
    for (auto&& element:pcss){
    //if(index==2){
		element->update_scene();
		element->draw_all();//}
 //   index++;
    }
	}
#endif
}

int main(int argc, char * argv[])
{
  ROS_INFO_STREAM(argv[0] << " - openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
  ROS_INFO_STREAM("RatSLAM algorithm by Michael Milford and Gordon Wyeth");
  ROS_INFO_STREAM("Distributed under the GNU GPL v3, see the included license file.");

  if (argc < 2)
  {
    ROS_FATAL_STREAM("USAGE: " << argv[0] << " <config_file>");
    exit(-1);
  }
  std::string topic_root = "";
  boost::property_tree::ptree settings, ratslam_settings_1, ratslam_settings_2,general_settings;
  read_ini(argv[1], settings);

  get_setting_child(ratslam_settings_1, settings, "ratslam_1", true);
  get_setting_child(ratslam_settings_2, settings, "ratslam_2", true);
  get_setting_child(general_settings, settings, "general", true);
  get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string) "");

  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "RatSLAMPoseCells");
  }
  ros::NodeHandle node;



  //ratslam::PosecellNetwork * pc_1 = new ratslam::PosecellNetwork(ratslam_settings_1);
  ratslam::PosecellNetwork * pc_2 = new ratslam::PosecellNetwork(ratslam_settings_2);
  std::vector<ratslam::PosecellNetwork*> pcns;
  //pcns.push_back(pc_1);
  pcns.push_back(pc_2);

  //std::vector<double> POSECELL_XSIZE;
  //  std::vector<double> grid_spacing_ratio;
  //double EXP_DELTA_PC_THRESHOLD_MUL;
  //std::vector<int> PC_DIM_XY_MUL;
  //std::vector<int> PC_DIM_TH_MUL;

  //for (auto&& element:pcns)
  //{  
   //   POSECELL_XSIZE.push_back(element->posecell_xsize());
   //   PC_DIM_XY_MUL.push_back(element->pc_dim_xy());
   //   PC_DIM_TH_MUL.push_back(element->pc_dim_th());
   //   EXP_DELTA_PC_THRESHOLD_MUL=element->exp_elta_pc_threshold(); //only the smallest module threshold is used.
 // }
 // double smallest_posecell_xsize=POSECELL_XSIZE[POSECELL_XSIZE.size()-1];
  //for  (auto&& element:POSECELL_XSIZE){grid_spacing_ratio.push_back(element/smallest_posecell_xsize);};

  //ratslam::MultiplePosecellNetwork *mpc=new ratslam::MultiplePosecellNetwork(grid_spacing_ratio,PC_DIM_XY_MUL,PC_DIM_TH_MUL, EXP_DELTA_PC_THRESHOLD_MUL);

  //ros::Publisher pub_pc_1 = node.advertise<ratslam_ros::TopologicalAction>(topic_root + "/PoseCell/TopologicalAction_1", 0);
  ros::Publisher pub_pc_2 = node.advertise<ratslam_ros::TopologicalAction>(topic_root + "/PoseCell/TopologicalAction_2", 0);

  //ros::Publisher pub_pc_4 = node.advertise<ratslam_ros::TopologicalAction>(topic_root + "/PoseCell/TopologicalAction_4", 0);
  std::vector<ros::Publisher> pub_pc;
  //pub_pc.push_back(pub_pc_1);
  pub_pc.push_back(pub_pc_2);

  //pub_pc.push_back(pub_pc_4);

  ros::Subscriber sub_odometry = node.subscribe<nav_msgs::Odometry>(topic_root + "/odom", 0, boost::bind(odo_callback, _1, pcns,pub_pc), ros::VoidConstPtr(),
                                                                    ros::TransportHints().tcpNoDelay());          

//  ros::Subscriber sub_odometry = node.subscribe<nav_msgs::Odometry>(topic_root + "/odom", 0, boost::bind(odo_callback, _1, pcns,pub_pc), ros::VoidConstPtr(),
//                                                                    ros::TransportHints().tcpNoDelay());                                                                      
                                               
  ros::Subscriber sub_template = node.subscribe<ratslam_ros::ViewTemplate>(topic_root + "/LocalView/Template", 0, boost::bind(template_callback, _1, pcns, pub_pc),
                                                                           ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
#ifdef HAVE_IRRLICHT
  boost::property_tree::ptree draw_settings;
  get_setting_child(draw_settings, settings, "draw", true);
  get_setting_from_ptree(use_graphics, draw_settings, "enable", true);
  if (use_graphics)
  {
  //pcs_3 = new ratslam::PosecellScene(draw_settings, pc_3,L"3");  
    pcs_2 = new ratslam::PosecellScene(draw_settings, pc_2,L"2",2);
  //pcs_1 = new ratslam::PosecellScene(draw_settings, pc_1,L"1",1);

  //pcss.push_back(pcs_1);
  pcss.push_back(pcs_2);
  }
#endif

  ros::spin();

  return 0;
}
