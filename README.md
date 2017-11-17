# Ceres ROSPlan

This repository provides the Tools and interfaces for using ROSPlan with the Ceres Robot. There are several different Packages that are described below in more detail.

## Ceres Rosplan Domain

The package provides the PDDL-domain for the ceres robot. It does not provide any other nodes or scripts. Currently only a simple example domain for a navigation task is implemented. The package also contains a waypoints.yaml file to load some example waypoints on launch.

## Interactive Waypoint Server

The `interactive_waypoint_server` package provides a python node of the same name that allows a user to place interactive waypoints in rviz as well as connect them to each other. For each waypoint that is managed by this node the ROSPlan knowledge database as well as the scene database is updated automatically.
As the waypoints a fully interactive, all changes in rviz will be updated immediately to the database to be used by the planning system and the action interfaces.

### Subscribed Topics
- `clicked_point` (geometry\_msgs/PointStamped)

  &nbsp;&nbsp;&nbsp;&nbsp; The points published by rviz


### Published Topics
- `interactive_waypoint_server/edges` (visualization\_msgs/MarkerArray)
  
  &nbsp;&nbsp;&nbsp;&nbsp; The waypoint connections as LINE markers in rviz
- `interactive_waypoint_server/update` (visualization\_msgs/InteractiveMarkerUpdate)
 
 &nbsp;&nbsp;&nbsp;&nbsp; The waypoints as interactive markers
 
### Services 

- `interactive_waypoint_server/remove_connection` (interactive\_waypoint\_server\_msgs/RemoveConnection)
  
  &nbsp;&nbsp;&nbsp;&nbsp; Remove a connection between two waypoints and update the knowledge database.
- `interactive_waypoint_server/save_waypoints` (interactive\_waypoint\_server\_msgs/SaveWaypoints)

  &nbsp;&nbsp;&nbsp;&nbsp; Store the waypoints

### Services Called

- `kcl_rosplan/update_knowledge_base_array` (rosplan\_knowledge\_base\_msgs/KnowledgeUpdateServiceArray)

  &nbsp;&nbsp;&nbsp;&nbsp; To update the knowledge database
  
### Parameters

- `~waypoint_file` (string, default: `""`)

  &nbsp;&nbsp;&nbsp;&nbsp; Waypoint file to load at start

## Interactive Waypoint Server Messages

The service definitions for the Interactive Waypoint Server Node.

## Rosplan Interface MoveBaseFlex

This package provides a Node that implements the ROSPlan interface to the [Move Base Flex](https://github.com/magazino/move_base_flex) framework for navigation.
The node provides a simple action server for the *goto-waypoint* action in the planning domain.

## Ceres Rosplan Launch

A collection of launch files as well as an rviz configuration to start the example planning task. Contains the following relevant files:

- `ceres_robot.launch`
 
 &nbsp;&nbsp;&nbsp;&nbsp; Launches Gazebo for the Ceres simulation, the navigation stack and rviz
- `ceres_planning.launch`

  &nbsp;&nbsp;&nbsp;&nbsp; Launches the Planning System with the `ceres_domain.pddl` domain and the `interactive_waypoint_server`

