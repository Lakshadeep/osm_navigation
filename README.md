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
* `/osm_low_level_planner` 

#### Launch
`roslaunch osm_low_level_planner osm_low_level_planner.launch`

#### Tests
```
rosrun actionlib axclient.py  "/osm_low_level_planner" osm_low_level_planner/OSMLowLevelPlannerAction

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

## Semantic Monte Carlo Localization (SMCL)
Implements MCL using semantic features modelled in OSM world model. Assumes that robot position is known at area level and then use semantic features obtained from semantic map of an area for localization.

#### Subscribed topics
* robot odometry
* semantic map
* detected semantic features

#### Published topics
* estimated pose
* point cloud array

#### Launch
`roslaunch smcl smcl_diff.launch`

#### Tests
Inside test folder, execute
`roslaunch smcl_test_HBRS_C022.xml`
(NOTE: `semantic_localization_test_C022.bag` file which is not the part of this repository should be copied in `data` folder inside `test` folder)



