# OSM navigation
Contains ROS packages for semantic localization and topological navigation for autonomous indoor robots using OpenStreetMap

## OSM map server
Provides server for loading semantic map and geometric map.

#### Services:
* `/get_semantic_map`: takes area id/name and return `SemanticMap`
* `/get_geometric_map`: takes area id/name and return `GeometricMap`
(See `osm_navigation_msgs` for message types)

#### Launch
`roslaunch osm_map_server osm_map_server.launch`

#### Tests
Run the scripts inside the `tests` folder
`python get_geometric_test.py`
`python get_semantic_test.py`


## OSM trajectory planner
Plans waypoint based trajectory from starting area to destination area using behaviour specific local areas modelled in OSM world model 

#### Action servers
* `/osm_trajectory_planner` 

#### Launch
`roslaunch osm_trajectory_planner osm_trajectory_planner.launch`

#### Tests
```
rosrun actionlib axclient.py  "/osm_trajectory_planner" osm_planner_msgs/OSMTrajectoryPlannerAction

start_floor: 'BRSU_L0'
destination_floor: 'BRSU_L0'
start_area: 'BRSU_C_L0_C9'
destination_area: 'BRSU_C_L0_C2'
start_local_area: 'BRSU_C_L0_C9_LA1'
destination_local_area: 'BRSU_C_L0_C2_LA1'
start_position: 
  x: 0.0
  y: 0.0
  z: 0.0
destination_task: ''
```

## OSM topological planner
Generates semantic human understandable navigation plan

#### Action servers
* `/osm_topological_planner` 

#### Launch
`roslaunch osm_topological_planner osm_topological_planner.launch`

#### Tests
```
rosrun actionlib axclient.py  "/osm_topological_planner" osm_planner_msgs/OSMTopologicalPlannerAction

start_floor: 'BRSU_L0'
destination_floor: 'BRSU_L0'
start_area: 'BRSU_C_L0_RoomC022'
destination_area: 'BRSU_C_L0_C2'
start_local_area: 'BRSU_C_L0_RoomC022_LA1'
destination_local_area: 'BRSU_C_L0_C2_LA1'
start_position: 
  x: 0.0
  y: 0.0
  z: 0.0
destination_task: ''

---------------------------------------------------

start_floor: 'BRSU_L0'
destination_floor: 'BRSU_L0'
start_area: 'BRSU_C_L0_RoomC022'
destination_area: 'BRSU_A_L0_A1'
start_local_area: 'BRSU_C_L0_RoomC022_LA1'
destination_local_area: 'BRSU_A_L0_A1_LA1'
start_position: 
  x: 0.0
  y: 0.0
  z: 0.0
destination_task: ''

---------------------------------------------------

start_floor: 'BRSU_L0'
destination_floor: 'BRSU_L0'
start_area: 'BRSU_A_L0_A1'
destination_area: 'BRSU_C_L0_RoomC022'
start_local_area: 'BRSU_A_L0_A1_LA1'
destination_local_area: 'BRSU_C_L0_RoomC022_LA1'
start_position: 
  x: 0.0
  y: 0.0
  z: 0.0
destination_task: ''

---------------------------------------------------
start_floor: 'BRSU_L0'
destination_floor: 'BRSU_L0'
start_area: 'BRSU_C_L0_C3'
destination_area: 'BRSU_C_L0_RoomC022'
start_local_area: 'BRSU_C_L0_C3_LA1'
destination_local_area: 'BRSU_C_L0_RoomC022_LA1'
start_position: 
  x: 0.0
  y: 0.0
  z: 0.0
destination_task: ''

```

## Monte Carlo localization using semantic features 
Implements MCL using semantic features modelled in OSM world model. Assumes that robot position is known at area level and then use semantic features obtained from semantic map of an area for localization.

#### Subscribed topics
* robot odometry
* semantic map
* detected semantic features

#### Published topics
* estimated pose
* point cloud array (pose estimate)

#### Launch
`roslaunch semantic_localization semantic_localization_diff.launch`

#### Tests
Inside test folder, execute
`roslaunch semantic_loclaization_test_HBRS_C022.xml`
(NOTE: `semantic_localization_test_C022.bag` file which is not the part of this repository should be copied in `data` folder inside `test` folder)


## Corridor navigation
Performs heading based navigation w.r.t a certain reference feature

#### Subscribed topics
* gateways
* distance monitor
* heading monitor

#### Published topics
* desired heading
* desired velocity 

#### Required services
* heading control switch
* motion control switch
* heading monitor reset
* distance monitor reset

#### Launch
`roslaunch corridor_navigation corridor_navigation.launch`

#### Tests
```
rosrun actionlib axclient.py  "/corridor_navigation_server" corridor_navigation/CorridorNavigationAction

goal_id: 1
direction: 0.0
distance: 12.0
goal_type: 0
```

## Door passing
Performs pure topological door entering without any robot localization estimate

#### Subscribed topics
* gateways
* distance monitor
* heading monitor

#### Published topics
* desired heading
* desirred velocity 
* motion controller mode

#### Required services
* heading control switch
* motion control switch
* heading monitor reset
* distance monitor reset
* motion control params
* motion control drive mode

#### Launch
`roslaunch door_passing door_passing.launch`

#### Tests
```
rosrun actionlib axclient.py  "/door_passing_server" door_passing/DoorPassingAction

door: 0/1/2    (front/right/left)
distance_inside: 0.0
```

## Junction maneuvering
Performs right, left or straight turn at the junction (X/T junctions)

#### Subscribed topics
* gateways
* distance monitor
* heading monitor

#### Published topics
* desired heading
* desirred velocity

#### Required services
* heading control switch
* motion control switch
* heading monitor reset
* distance monitor reset
* motion control params
* motion control drive mode

#### Launch
`roslaunch junction_maneuvering junction_maneuvering.launch`

#### Tests
```
rosrun actionlib axclient.py  "/junction_maneuvering_server" junction_maneuvering/JunctionManeuveringAction

junction: 0/1    (t/x-junction)
turn_direction: 0/1/2    (right/front/left)
distance: 0.0
```

## Area navigation
Navigating using a traffic/navigation sign

#### Subscribed topics
* navigation signs
* distance monitor
* heading monitor

#### Published topics
* desired heading
* desirred velocity 

#### Required services
* heading control switch
* motion control switch
* heading monitor reset
* distance monitor reset
* motion control params
* motion control drive mode

#### Launch
`roslaunch area_navigation area_navigation.launch`

#### Tests
```
rosrun actionlib axclient.py  "/area_navigation_server" area_navigation/AreaNavigationAction

goal_type: 0    (0 - exit)
```

## Symbolic navigation
Executes symbolic feature based navigation using topological plan & environment specific navigation skills

#### Action servers
* `/symbolic_navigation` 

#### Launch
`roslaunch symbolic_navigation symbolic_navigation.launch`

#### Tests
```
rosrun actionlib axclient.py  "/symbolic_navigation_server" symbolic_navigation/SymbolicNavigationAction

start_floor: 'BRSU_L0'
destination_floor: 'BRSU_L0'
start_area: 'BRSU_C_L0_C3'
destination_area: 'BRSU_C_L0_RoomC022'
start_local_area: 'BRSU_C_L0_C3_LA1'
destination_local_area: 'BRSU_C_L0_RoomC022_LA1'
start_position: 
  x: 0.0
  y: 0.0
  z: 0.0
destination_task: ''

