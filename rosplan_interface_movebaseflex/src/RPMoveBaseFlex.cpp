#include "rosplan_interface_movebaseflex/RPMoveBaseFlex.h"
#include <move_base_flex_msgs/GetPathResult.h>
#include <move_base_flex_msgs/GetPathGoal.h>
#include <interactive_waypoint_server_msgs/RemoveConnection.h>
/* The implementation of RPMoveBase.h */
using namespace KCL_rosplan;

/* constructor */
RPMoveBaseFlex::RPMoveBaseFlex(ros::NodeHandle &nh, std::string &actionserver)
: message_store(nh),
  get_path_action_client(actionserver + "/get_path", true),
  exe_path_action_client(actionserver + "/exe_path", true),
  recovery_action_client(actionserver + "/recovery", true)
{
    remove_connection_service = nh.serviceClient<interactive_waypoint_server_msgs::RemoveConnection>("/interactive_waypoint_server/removeConnection");
}

/* action dispatch callback */
bool RPMoveBaseFlex::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // get waypoint ID from action dispatch

		std::string wpIDFrom, wpIDTo;
        bool found_from = false;
		bool found_to = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
            if(0==msg->parameters[i].key.compare("to")) {
				wpIDTo = msg->parameters[i].value;
				found_to = true;
			}
            if(0==msg->parameters[i].key.compare("from")) {
                wpIDFrom = msg->parameters[i].value;
                found_from = true;
            }
		}
		if(!found_to) {
			ROS_ERROR("KCL: (%s) aborting action dispatch; PDDL action missing required parameter ?to", params.name.c_str());
			return false;
		}

    if(!found_from) {
        ROS_WARN("KCL (%s parameter ?from not found in PDDL action. Knowledge Database will not be updated on failure", params.name.c_str() );
    }

    // get poses from message store
    std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
    if(message_store.queryNamed<geometry_msgs::PoseStamped>(wpIDTo, results)) {
        if (results.size() < 1) {
            ROS_ERROR("KCL: (%s) aborting action dispatch; no matching wpID for target waypoint %s",
                      params.name.c_str(), wpIDTo.c_str());
            return false;
        }
        if (results.size() > 1) {
            ROS_WARN("KCL: (%s) multiple target waypoints share the same wpID. Using the first result.",
                     params.name.c_str());
        }
    } else {
        // no KMS connection (failed)
        ROS_INFO("KCL: (%s) aborting action dispatch; query to sceneDB failed", params.name.c_str());
        return false;
    }
    std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > resultsFrom;
    if(found_from) {
        if(message_store.queryNamed<geometry_msgs::PoseStamped>(wpIDFrom, resultsFrom)) {
            if (resultsFrom.size() < 1) {
                ROS_WARN(
                        "KCL: (%s no matching wpID for origin waypoint %s. Knowledge database will not be updated on failure.",
                        params.name.c_str(), wpIDFrom.c_str());
                found_from = false;
            }
            if (resultsFrom.size() > 1) {
                ROS_WARN("KCL: (%s) multiple origin waypoints share the same wpID. Using the first result.",
                         params.name.c_str());
            }
        } else {
            found_from = false;
        }
    }

    ROS_INFO("KCL: (%s) waiting for move_base flex action server to start", params.name.c_str());
    get_path_action_client.waitForServer();

    // dispatch MoveBaseflex action

    // Get the Path first
    move_base_flex_msgs::GetPathGoal getPathGoal;
    geometry_msgs::PoseStamped &pose = *results[0];
    getPathGoal.use_start_pose = false;
    getPathGoal.target_pose = pose;

    nav_msgs::Path path;

    get_path_action_client.sendGoal(getPathGoal);

    bool finished_before_timeout = get_path_action_client.waitForResult(ros::Duration(10));
    if (finished_before_timeout) {
        auto result = get_path_action_client.getResult();
        ROS_INFO("KCL: (%s) getPath action finished.", params.name.c_str());
        if(result->outcome != move_base_flex_msgs::GetPathResult::SUCCESS) {
            ROS_WARN("KCL: (%s) getPath action failed with Message: %s", params.name.c_str(), result->message.c_str());

            // just abort if no path has been found (failed)

          /*if(found_from) {
            // change knowledge base
            interactive_waypoint_server::removeConnection request;
            geometry_msgs::PoseStamped &pose = *resultsFrom[0];
            request.first_waypoint = pose;
            pose = *results[0];
            request.second_waypoint = pose;
            }*/
            return false;
        }
        path = result->path; // get path
    } else {
        // timed out (failed)
        get_path_action_client.cancelAllGoals();
        ROS_INFO("KCL: (%s) getPath action timed out", params.name.c_str());
        return false;
    }
      
    // Execute the computed Path
    move_base_flex_msgs::ExePathGoal exePathGoal;
    exePathGoal.path = path;

    exe_path_action_client.sendGoal(exePathGoal);

    finished_before_timeout = exe_path_action_client.waitForResult();
    if(finished_before_timeout) {
        auto result = exe_path_action_client.getResult();

        ROS_INFO("KCL: (%s) exePath action finished.", params.name.c_str());
        if(result->outcome == move_base_flex_msgs::GetPathResult::SUCCESS) {
            // we are finished
            return true;
        } else {
            // timed out (failed)
            exe_path_action_client.cancelAllGoals();
            ROS_INFO("KCL: (%s) exePath action failed with %u", params.name.c_str(), result->outcome);
            return false;
        }
    }

    // exePath was not successful

    // EXECUTE RECOVERY BEHAVIOR FIRST (Rotate / Drive back)
    // IF NOT SUCCESSFUL CHANGE KNOWLEDGE BASE (REMOVE PATH)
    // Return false to request replanning
    return false;
}

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_interface_movebaseflex");
    ros::NodeHandle nh("~");

    std::string actionserver;
    nh.param("action_server", actionserver, std::string("/move_base_flex"));

    // create PDDL action subscriber
    KCL_rosplan::RPMoveBaseFlex rpmbf(nh, actionserver);

    // listen for action dispatch
    ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPActionInterface::dispatchCallback, dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmbf));
    rpmbf.runActionInterface();

    return 0;
}
