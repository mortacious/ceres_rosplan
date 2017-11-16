#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>

#include "rosplan_action_interface/RPActionInterface.h"
#include "actionlib/client/simple_action_client.h"
#include <move_base_flex_msgs/GetPathAction.h>
#include <move_base_flex_msgs/ExePathAction.h>
#include <move_base_flex_msgs/RecoveryAction.h>
#include "mongodb_store/message_store.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_srvs/Empty.h"

#ifndef KCL_movebaseflex
#define KCL_movebaseflex

/**
 * This file defines the RPMoveBase class.
 * RPMoveBase is used to connect ROSPlan to the MoveBase library.
 * PDDL "goto_waypoint" actions become "move_base_msgs::MoveBase" actions.
 * Waypoint goals are fetched by name from the SceneDB (implemented by mongoDB).
 */
namespace KCL_rosplan {

	class RPMoveBaseFlex: public RPActionInterface
	{

	private:

		mongodb_store::MessageStoreProxy message_store; // for waypoint positions
		actionlib::SimpleActionClient<move_base_flex_msgs::GetPathAction> get_path_action_client;
		actionlib::SimpleActionClient<move_base_flex_msgs::ExePathAction> exe_path_action_client;
		actionlib::SimpleActionClient<move_base_flex_msgs::RecoveryAction> recovery_action_client;

		ros::ServiceClient remove_connection_service;
	public:

		/* constructor */
		RPMoveBaseFlex(ros::NodeHandle &nh, std::string &actionserver);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
