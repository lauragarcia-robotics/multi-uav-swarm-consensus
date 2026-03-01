#ifndef SWARM_H
#define SWARM_H

#include <task_03_swarm/task_03_swarm.h>

namespace task_03_swarm
{

// state machine
typedef enum
{
  INIT_STATE,
  AGREEING_ON_DIRECTION,
  TRAVERSING,
} State_t;

class Swarm : public Task03Swarm {

public:
  /**
   * @brief The swarm controller (single unit) initialization method. This method will be called ONLY ONCE in the lifetime of the controller.
   * Use this method to do any heavy pre-computations.
   *
   * @param visibility radius of the agent
   */
  void init(const double visibility_radius);

  /**
   * @brief This method calculates a next-iteration action of one UAV given relative information of its neighbors and obstacles; and the direction towards the
   *        moving target. This method is supposed to be filled in by the student.
   *
   * @param perception Current perceptual information of this UAV. Defined in perception.h. It contains:
   *  - current time
   *  - target vector: 3D vector towards the moving robot in the UAV body frame
   *  - neighbors defined by:
   *    - their position in the UAV body frame
   *    - the variables shared through the communication network
   *  - obstacles consisting of:
   *    - 3D vector from the body frame to the closest obstacle in the environment
   *    - 4 gates (pairs of 2 gate edges) in the UAV body frame
   * @param user_params user-controllable parameters
   * @param action_handlers functions for visualization and data sharing among the UAVs:
   *  - shareVariables(int, int, double) will share the three basic-type variables among the UAVs
   *  - visualizeArrow() will publish the given arrow in the UAV body frame within the visualization
   *  - visualizeArrowFrom() will publish the given arrow at position given in the UAV body frame within the visualization
   *  - visualizeCube() will publish a cube in the UAV body frame within the visualization
   *
   * @return Next-iteration action for this UAV given as a 3D vector. Zero vector is expected if no action should be performed. Beware that i) the vector z-axis
   * component will be set to 0, ii) the vector vector magnitude will be clamped into <0, v_max> limits, and iii) the vector will be passed to a velocity
   * controller of the UAV.
   *
   *       Example 1: v_max=0.75 -> vector (1, 0, 1) will be saturated to 0.75*(1, 0, 0).
   */
  Eigen::Vector3d updateAction(const Perception_t &perception, const UserParams_t &user_params, const ActionHandlers_t &action_handlers);

  /**
   * @brief Non-linear weighting of forces.
   *
   * The function is to be non-increasing, non-negative, and non-linear (growing to high function values when the distance is approaching the safety distance
   * (lower bound)). Below the lower bound (including), the function is to be undefined. Over the visibility range, the function shall return 0.
   *
   * @param distance to an agent/obstacle
   * @param visibility visibility range of the UAVs
   * @param safety distance: min distance to other UAVs or obstacles
   * @param desired distance: desired distance to other UAVs or obstacles (does not need to be used)
   *
   * @return
   *   bool:   True if function is defined for the given distance, False otherwise
   *   double: Weight for an agent/obstacle at given distance, if function is defined for the given distance.
   */
  std::tuple<bool, double> weightingFunction(const double distance, const double visibility, const double safety_distance, const double desired_distance);

private:
  // | -------------- PUT ANY MEMBER VARIABLES HERE ------------- |
  double idling_time_init;
  Eigen::Vector3d last_action_ = Eigen::Vector3d::Zero();

  bool _go_traversing_next_ = false;

  // Constant variables: just an example, feel free to change them
  const double SAFETY_DISTANCE_UAVS  = 2.5;
  const double DESIRED_DISTANCE_UAVS = 4.0;
  const double SAFETY_DISTANCE_OBS = 0.5;
  const double DESIRED_DISTANCE_OBS = 1.0;
  double _agree_consensus_start_time_ = -1.0;
  int _agree_last_dir_int_ = -1;
  bool _have_traversed_cell_ = false; 
  Direction_t proposeNewDirection(const Perception_t &perception); 
  Direction_t _committed_direction_ = NONE;
  Eigen::Vector3d _last_action_ = Eigen::Vector3d::Zero();
  double _last_time_ = -1.0;
  double _min_gate_dist_ = 1e9;

  // | ------------- SHOULD NOT NEED TO MODIFY BELOW ------------ |

  // | ---- Helper methods which will help you if implemented --- |
private:
  Direction_t targetToDirection(const Eigen::Vector3d &target_bearing);
  bool        robotsInIdenticalStates(const Perception_t &perception);
  bool        anyRobotInState(const Perception_t &perception, const State_t &state);

private:
  State_t     _state_                = INIT_STATE;
  Direction_t _navigation_direction_ = NONE;
  double      _visibility_radius_;
  Eigen::Vector3d _goal_direction_vec_ = Eigen::Vector3d::Zero();

  // | --------------------- Helper methods --------------------- |
private:
  std::string stateToString(const State_t &state);

  int     stateToInt(const State_t &state);
  State_t intToState(const int value);

  unsigned int selectGateInDirection(const Direction_t &direction, const Obstacles_t &obstacles);
  unsigned int selectGateClosest(const Obstacles_t &obstacles);

  std::vector<double> computeMutualDistances(const std::vector<Neighbor_t> &neighbors);

  bool                 integersAreUnique(const std::vector<int> &integers);
  std::map<int, int>   countIntegers(const std::vector<int> &integers);
  std::tuple<int, int> getMajority(const std::vector<int> &integers);
  std::tuple<int, int> getMajority(const std::map<int, int> &integer_counts);
};

}  // namespace task_03_swarm

#endif  // SWARM_H
