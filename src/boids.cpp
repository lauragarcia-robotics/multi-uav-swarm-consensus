#include <task_03_boids/boids.h>

namespace task_03_boids
{

// | ---------- HERE YOU MAY WRITE YOUR OWN FUNCTIONS --------- |

// | ------------- FILL COMPULSORY FUNCTIONS BELOW ------------ |

/* updateAgentState() //{ */

/**
 * @brief Calculate a next-iteration action of one agent given relative information of its neighbors and the direction towards a target.
 *        This method is supposed to be filled in by the student.
 *
 * @param AgentState_t Current state of the agent as defined in agent_state.h.
 * @param user_params user-controllable parameters
 * @param action_handlers functions for visualization
 *  - visualizeArrow() will publish the given arrow in the agent frame within the visualization
 *
 * @return
 *    1) XYZ vector in frame of the agent to be set as velocity command. Beware that i) the vector z-axis component will be set to 0, ii) the vector magnitude
 * will be clamped into <v_min, v_max> limits and iii) azimuth of the vector's XY-projection will be saturated such that the azimuth between the agent's current
 * velocity and the vector does not exceed a maximal change.
 *
 *       Example 1: Maximal change is d=45deg, v_min=0.1, v_max=0.2, and current velocity is (0.2, 0, 0) -> vector (0, 1, 1) will be clamped and saturated to
 * 0.2*(cos(d), sin(d), 0).
 *
 *       Example 2: Maximal change is d=45deg, v_min=0.1, v_max=0.2, and current velocity is (0.2, 0, 0) -> vector (0, -0.05, 1) will
 * be clamped and saturated to 0.1*(cos(-d), sin(-d), 0).
 *
 *    2) Probability distribution of colors to be set to the agent for next iteration. Beware that output distribution.dim() has to equal input
 * state.distribution.dim().
 */
std::tuple<Eigen::Vector3d, Distribution> Boids::updateAgentState(const AgentState_t &state, const UserParams_t &user_params,
                                                                  const ActionHandlers_t &action_handlers) {

  // TODO STUDENTS: Finish this method. The example code below can be removed, it's there just for an inspiration.

  // | ------------------- EXAMPLE CODE START ------------------- |

  // Setup the output action
  Eigen::Vector3d action = Eigen::Vector3d::Zero();
  Eigen::Vector3d target = state.target;

  // Access my own prob. distribution of colors
  Distribution my_distribution = state.distribution;
  int          dim             = my_distribution.dim();

  Eigen::Vector3d coh_sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d sep_sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d ali_sum = Eigen::Vector3d::Zero();

  int neighbour_count = 0;

  Distribution neigh_dist_sum(dim);
  for (int i = 0; i < dim; i++){
    neigh_dist_sum.set(i, my_distribution.get(i));
  }
  int dist_count = 1;

  // Iterate over the states of the visible neighbors
  for (const auto &n_state : state.neighbors_states) {

    auto &[n_pos_rel, n_vel_global, n_distribution] = n_state;

    double dist = n_pos_rel.norm();
    if (dist < 1e-3){
      continue;
    }

    //cohesion
    coh_sum +=  n_pos_rel;
    //separation 
    sep_sum -= n_pos_rel / (dist * dist);
    //alignment
    ali_sum += n_vel_global;

    for (int i = 0; i < dim; i++){
      neigh_dist_sum.set(i, neigh_dist_sum.get(i) + n_distribution.get(i));
    }
    dist_count++;
    neighbour_count++;

    // check if the size of my prob. distribution matches the size of the neighbour's distribution
    if (dim != n_distribution.dim()) {
      std::cout << "This should never happen. If it did, you set the previous distribution wrongly." << std::endl;
    }
  }

  Eigen::Vector3d coh_force = Eigen::Vector3d::Zero();
  Eigen::Vector3d sep_force = Eigen::Vector3d::Zero();
  Eigen::Vector3d ali_force = Eigen::Vector3d::Zero();

  if (neighbour_count){
      coh_force = coh_sum/neighbour_count;
      sep_force = sep_sum/neighbour_count;
      ali_force = ali_sum/neighbour_count;
  }

  /*Eigen::Vector3d target_dir = target;
  if (target_dir.norm() > 1e-3){
    target_dir.normalize();
  } */

  // Example: scale the action by user parameter
  action = user_params.param1 * target + user_params.param2 * coh_force + 
           user_params.param3 * sep_force + user_params.param4 * ali_force;

  double beacon_weight = 3.0;
  //Am I nearby a beacon?
  Distribution beacon_distribution;
  if (state.nearby_beacon) {
    beacon_distribution = state.beacon_distribution;

    for (int i = 0; i <dim; i++){
      neigh_dist_sum.set(i, neigh_dist_sum.get(i) + beacon_weight * beacon_distribution.get(i));
    }

  dist_count += beacon_weight;
  }

  double min_speed=0.05;
  if (action.norm() < min_speed) {
    if(action.norm() > 1e-6) {
      action.normalize();
      action *= min_speed;
    } else {
      action = min_speed * target;
    }
  }

  Distribution avg_dist(dim);
  for (int i = 0; i<dim; i++){
    avg_dist.set(i, neigh_dist_sum.get(i) / dist_count);
  }

  double alpha=0.25;
  for (int i = 0; i<dim; i++){
    double new_val=(1.0 - alpha) * my_distribution.get(i) + alpha * avg_dist.get(i);
    my_distribution.set(i, new_val);
  }
  my_distribution.normalize();

  // Print the output action
  printVector3d(action, "Action: ");

  // Visualize the arrow in RViz
  action_handlers.visualizeArrow("action", action, Color_t{0.0, 0.0, 0.0, 1.0});

  // | -------------------- EXAMPLE CODE END -------------------- |

  return {action, my_distribution};
}

//}

}  // namespace task_03_boids
