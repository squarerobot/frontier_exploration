#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <ros/ros.h>

#include <geometry_msgs/PolygonStamped.h>

#include <frontier_exploration/ExploreTaskAction.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "../../frontier_exploration/include/frontier_exploration/geometry_tools.h"

namespace frontier_exploration {

/**
 * @brief Server for frontier exploration action, runs the state machine associated with a
 * structured frontier exploration task and manages robot movement through move_base.
 */
class FrontierExplorationServer {
public:
  /**
   * @brief Constructor for the server, sets up this node's ActionServer for exploration and ActionClient to move_base
   * for robot movement.
   * @param name Name for SimpleActionServer
   */
  FrontierExplorationServer(std::string name)
    : tf_listener_(tf_buffer_, true)
    , private_nh_("~")
    , as_(nh_, name, boost::bind(&FrontierExplorationServer::executeCb, this, _1), false)
    , move_client_("frontier_move_server", true)
    , retry_(5) {
    tf_buffer_.setUsingDedicatedThread(true);
    private_nh_.param<double>("frequency", frequency_, 0.0);
    private_nh_.param<double>("goal_aliasing", goal_aliasing_, 0.1);

    explore_costmap_ros_ =
        boost::shared_ptr<costmap_2d::Costmap2DROS>(new costmap_2d::Costmap2DROS("explore_costmap", tf_buffer_));

    as_.registerPreemptCallback(boost::bind(&FrontierExplorationServer::preemptCb, this));
    as_.start();
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::Buffer tf_buffer_;
  actionlib::SimpleActionServer<frontier_exploration::ExploreTaskAction> as_;

  boost::shared_ptr<costmap_2d::Costmap2DROS> explore_costmap_ros_;
  double frequency_, goal_aliasing_;
  bool success_, moving_;
  int retry_;

  boost::mutex move_client_lock_;
  frontier_exploration::ExploreTaskFeedback feedback_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_client_;
  move_base_msgs::MoveBaseGoal move_client_goal_;

  /**
   * @brief Execute callback for actionserver, run after accepting a new goal
   * @param goal ActionGoal containing boundary of area to explore, and a valid centerpoint for the area.
   */

  void executeCb(const frontier_exploration::ExploreTaskGoalConstPtr& goal) {
    ROS_INFO("NEW EXPLORE TASK RECIEVED");
    success_ = false;
    moving_ = false;

    explore_costmap_ros_->resetLayers();

    // create costmap services
    ros::ServiceClient updateBoundaryPolygon =
        private_nh_.serviceClient<frontier_exploration::UpdateBoundaryPolygon>("explore_costmap/explore_boundary/"
                                                                               "update_boundary_polygon");
    ros::ServiceClient getNextFrontier = private_nh_.serviceClient<frontier_exploration::GetNextFrontier>("explore_"
                                                                                                          "costmap/"
                                                                                                          "explore_"
                                                                                                          "boundary/"
                                                                                                          "get_next_"
                                                                                                          "frontier");

    // wait for move_base and costmap services
    if (!move_client_.waitForServer() || !updateBoundaryPolygon.waitForExistence() ||
        !getNextFrontier.waitForExistence()) {
      as_.setAborted();
      return;
    }

    // set region boundary on costmap
    if (ros::ok() && as_.isActive()) {
      frontier_exploration::UpdateBoundaryPolygon srv;
      srv.request.explore_boundary = goal->explore_boundary;
      if (updateBoundaryPolygon.call(srv)) {
        ROS_INFO("Region boundary set");
      } else {
        ROS_ERROR("Failed to set region boundary");
        as_.setAborted();
        return;
      }
    }

    // loop until all frontiers are explored
    ros::Rate rate(frequency_);
    while (ros::ok() && as_.isActive()) {
      frontier_exploration::GetNextFrontier srv;

      // placeholder for next goal to be sent to move base
      geometry_msgs::PoseStamped goal_pose;

      // get current robot pose in frame of exploration boundary
      geometry_msgs::PoseStamped robot_pose;
      explore_costmap_ros_->getRobotPose(robot_pose);

      // evaluate if robot is within exploration boundary using robot_pose in boundary frame
      geometry_msgs::PoseStamped eval_pose = srv.request.start_pose;
      if (robot_pose.header.frame_id != goal->explore_boundary.header.frame_id) {
        tf_buffer_.transform(robot_pose, eval_pose, goal->explore_boundary.header.frame_id);
      } else {
        eval_pose = robot_pose;
      }
      srv.request.start_pose = eval_pose;

      // check if robot is not within exploration boundary and needs to return to center of search area
      if (goal->explore_boundary.polygon.points.size() > 0 &&
          !pointInPolygon(eval_pose.pose.position, goal->explore_boundary.polygon)) {
        // check if robot has explored at least one frontier, and promote debug message to warning
        if (success_) {
          ROS_WARN("Robot left exploration boundary, returning to center");
        } else {
          ROS_DEBUG("Robot not initially in exploration boundary, traveling to center");
        }

        // get current robot position in frame of exploration center
        geometry_msgs::PointStamped eval_point;
        eval_point.header = eval_pose.header;
        eval_point.point = eval_pose.pose.position;
        if (eval_point.header.frame_id != goal->explore_center.header.frame_id) {
          geometry_msgs::PointStamped temp = eval_point;
          tf_buffer_.transform(temp, eval_point, goal->explore_center.header.frame_id);
        }

        // set goal pose to exploration center
        goal_pose.header = goal->explore_center.header;
        goal_pose.pose.position = goal->explore_center.point;
        tf2::Quaternion quant;
        quant.setRPY(0, 0, yawOfVector(eval_point.point, goal->explore_center.point));
        tf2::convert(quant, goal_pose.pose.orientation);
      } else if (getNextFrontier.call(srv)) {  // if in boundary, try to find next frontier to search
        ROS_DEBUG("Found frontier to explore");
        success_ = true;
        goal_pose = feedback_.next_frontier = srv.response.next_frontier;
        retry_ = 5;

      } else {  // if no frontier found, check if search is successful
        ROS_DEBUG("Couldn't find a frontier");

        // search is succesful
        if (retry_ == 0 && success_) {
          ROS_WARN("Finished exploring room");
          as_.setSucceeded();
          boost::unique_lock<boost::mutex> lock(move_client_lock_);
          move_client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
          return;

        } else if (retry_ == 0 || !ros::ok()) {  // search is not successful

          ROS_ERROR("Failed exploration");
          as_.setAborted();
          return;
        }

        ROS_DEBUG("Retrying...");
        retry_--;
        // try to find frontier again, without moving robot
        continue;
      }
      // if above conditional does not escape this loop step, search has a valid goal_pose

      // check if new goal is close to old goal, hence no need to resend
      if (!moving_ ||
          !pointsNearby(move_client_goal_.target_pose.pose.position, goal_pose.pose.position, goal_aliasing_ * 0.5)) {
        ROS_DEBUG("New exploration goal");
        move_client_goal_.target_pose = goal_pose;
        boost::unique_lock<boost::mutex> lock(move_client_lock_);
        if (as_.isActive()) {
          move_client_.sendGoal(move_client_goal_, boost::bind(&FrontierExplorationServer::doneMovingCb, this, _1, _2),
                                0, boost::bind(&FrontierExplorationServer::feedbackMovingCb, this, _1));
          moving_ = true;
        }
        lock.unlock();
      }

      // check if continuous goal updating is enabled
      if (frequency_ > 0) {
        // sleep for specified frequency and then continue searching
        rate.sleep();
      } else {
        // wait for movement to finish before continuing
        while (ros::ok() && as_.isActive() && moving_) {
          ros::WallDuration(0.1).sleep();
        }
      }
    }

    // goal should never be active at this point
    ROS_ASSERT(!as_.isActive());
  }

  /**
   * @brief Preempt callback for the server, cancels the current running goal and all associated movement actions.
   */
  void preemptCb() {
    boost::unique_lock<boost::mutex> lock(move_client_lock_);
    move_client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
    ROS_WARN("Current exploration task cancelled");

    if (as_.isActive()) {
      as_.setPreempted();
    }
  }

  /**
   * @brief Feedback callback for the move_base client, republishes as feedback for the exploration server
   * @param feedback Feedback from the move_base client
   */
  void feedbackMovingCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    feedback_.base_position = feedback->base_position;
    as_.publishFeedback(feedback_);
  }

  /**
   * @brief Done callback for the move_base client, checks for errors and aborts exploration task if necessary
   * @param state State from the move_base client
   * @param result Result from the move_base client
   */
  void doneMovingCb(const actionlib::SimpleClientGoalState& state,
                    const move_base_msgs::MoveBaseResultConstPtr& result) {
    if (state == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_ERROR("Failed to move");
      as_.setAborted();
    } else if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      moving_ = false;
    }
  }
};
}  // namespace frontier_exploration

int main(int argc, char** argv) {
  ros::init(argc, argv, "explore_server");

  frontier_exploration::FrontierExplorationServer server(ros::this_node::getName());
  ros::spin();
  return 0;
}
