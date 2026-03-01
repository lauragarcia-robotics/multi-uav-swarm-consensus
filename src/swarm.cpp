#include <student_headers/swarm.h>

namespace task_03_swarm
{

// --------------------------------------------------------------
// |                    the library interface                   |
// --------------------------------------------------------------

/* init() //{ */

void Swarm::init(const double visibility_radius) {

  _visibility_radius_ = visibility_radius;
}

//}

// | ------ Compulsory functions to be filled by students ----- |

/* updateAction() //{ */

Eigen::Vector3d Swarm::updateAction(const Perception_t &perception, const UserParams_t &user_params, 
                                    const ActionHandlers_t &action_handlers) {

  Eigen::Vector3d vec_action = Eigen::Vector3d::Zero();
  const double current_time  = perception.time;

  const double param1 = user_params.param1;
  const double param2 = user_params.param2;
  const double param3 = user_params.param3;

if (_state_ == AGREEING_ON_DIRECTION && _go_traversing_next_) {
  _state_ = TRAVERSING;
  _navigation_direction_ = _committed_direction_;   // <- fija rumbo estable
  _have_traversed_cell_ = false;
  _go_traversing_next_ = false;
}

  bool compute_action = false;

  // ======================= STATE MACHINE =======================
  switch (_state_) {

    case INIT_STATE: {
      _state_ = AGREEING_ON_DIRECTION;
      idling_time_init = current_time;
      _navigation_direction_ = NONE;

      action_handlers.shareVariables(stateToInt(_state_), directionToInt(_navigation_direction_), 0.0);
      break;
    }

    case AGREEING_ON_DIRECTION: {
    std::cout << "Current state: " << stateToString(AGREEING_ON_DIRECTION) << std::endl;

      const int agree_i = stateToInt(AGREEING_ON_DIRECTION);
      const int trav_i  = stateToInt(TRAVERSING);
      const int NONE_i  = directionToInt(NONE);

      if (perception.neighbors.empty()) {
        _navigation_direction_ = NONE;
        action_handlers.shareVariables(agree_i, NONE_i, 0.0);
        _agree_consensus_start_time_ = -1.0;
        _agree_last_dir_int_ = -1;
        compute_action = true;
        break;
      }

      const Direction_t my_prop_dir = proposeNewDirection(perception);
      const int my_prop_int = directionToInt(my_prop_dir);

      int agreeing_neighbors = 0;
      for (const auto &n : perception.neighbors) {
        if (n.shared_variables.int1 == agree_i) agreeing_neighbors++;
      }
      const bool allow_trav = (agreeing_neighbors == 0);

      std::vector<int> votes;
      votes.reserve(1 + perception.neighbors.size());
      votes.push_back(my_prop_int);

      for (const auto &n : perception.neighbors) {
        const auto &vars = n.shared_variables;
        const bool in_phase = (vars.int1 == agree_i) || (allow_trav && vars.int1 == trav_i);
        if (!in_phase) continue;
        if (vars.int2 == NONE_i) continue;
        votes.push_back(vars.int2);
      }

      if (votes.size() < 2) {
        _navigation_direction_ = intToDirection(my_prop_int);
        action_handlers.shareVariables(agree_i, my_prop_int, 0.0);
        _agree_consensus_start_time_ = -1.0;
        _agree_last_dir_int_ = -1;
        compute_action = true;
        break;
      }

      const auto counts = countIntegers(votes);
      auto [maj_int, maj_freq] = getMajority(counts);

      int best_int = (maj_int == -1) ? my_prop_int : maj_int;
      int best_freq = (maj_int == -1) ? 1 : maj_freq;

      int tie_count = 0;
      for (const auto &kv : counts) if (kv.second == best_freq) tie_count++;
      if (tie_count > 1) best_int = my_prop_int;
      if (best_int == NONE_i) best_int = my_prop_int;

      _navigation_direction_ = intToDirection(best_int);

      action_handlers.shareVariables(agree_i, best_int, 0.0);

      bool consensus = true;
      int total_in_phase = 0;
      for (const auto &n : perception.neighbors) {
        const auto &vars = n.shared_variables;
        const bool in_phase = (vars.int1 == agree_i) || (allow_trav && vars.int1 == trav_i);
        if (!in_phase) continue;
        total_in_phase++;
        if (vars.int2 == NONE_i || vars.int2 != best_int) { consensus = false; break; }
      }

      const double CONSENSUS_HOLD = 0.30;

      if (total_in_phase == 0 || !consensus) {
        _agree_consensus_start_time_ = -1.0;
        _agree_last_dir_int_ = -1;
      } else {
        if (_agree_last_dir_int_ != best_int) {
          _agree_last_dir_int_ = best_int;
          _agree_consensus_start_time_ = current_time;
        } else if (_agree_consensus_start_time_ < 0.0) {
          _agree_consensus_start_time_ = current_time;
        }

        if (_agree_consensus_start_time_ > 0.0 &&
            (current_time - _agree_consensus_start_time_) >= CONSENSUS_HOLD) {

          _committed_direction_ = intToDirection(best_int); 
          _go_traversing_next_ = true;

          _agree_consensus_start_time_ = -1.0;
          _agree_last_dir_int_ = -1;
        }
      }

      compute_action = true;
      break;
    }

    case TRAVERSING: {
      compute_action = true;
      break;
    }

    default:
      break;
  }

  // ===================== STATE MACHINE END =====================

  if (compute_action) {

    // ----------------- 1) NAVEGACIÓN AL GATE / TARGET -----------------
    if (_navigation_direction_ == NONE) {
    _navigation_direction_ = targetToDirection(perception.target_vector);
    }

    const auto &gates = perception.obstacles.gates;
    bool has_gate = !gates.empty();

    Eigen::Vector3d vec_navigation = Eigen::Vector3d::Zero();
    Eigen::Vector3d gate_center    = Eigen::Vector3d::Zero();
    double my_gate_dist = 0.0;

    if (has_gate) {
      unsigned int gate_idx = selectGateInDirection(_navigation_direction_, perception.obstacles);
      if (gate_idx >= gates.size()) gate_idx = selectGateClosest(perception.obstacles);

      const auto gate = gates[gate_idx];
      gate_center = 0.5 * (gate.first + gate.second);

      vec_navigation = gate_center;
      vec_navigation.z() = 0.0;

      my_gate_dist = gate_center.norm();
      if (vec_navigation.norm() > 1e-3) vec_navigation.normalize();

    } else {
      vec_navigation = perception.target_vector;
      vec_navigation.z() = 0.0;
      my_gate_dist = vec_navigation.norm();
      if (vec_navigation.norm() > 1e-3) vec_navigation.normalize();
    }

    // ----------------- 2) SEPARACIÓN + COHESIÓN -----------------
    Eigen::Vector3d vec_separation = Eigen::Vector3d::Zero();

    const double desired  = DESIRED_DISTANCE_UAVS;
    const double far_thr  = 1.5 * DESIRED_DISTANCE_UAVS;
    const double coh_gain = 0.2;

    for (const auto &n : perception.neighbors) {
      const double n_dist = n.position.norm();
      if (n_dist < 1e-3) continue;

      const Eigen::Vector3d dir_to_neighbor = n.position / n_dist;

      bool w_def = false;
      double w = 0.0;
      std::tie(w_def, w) = weightingFunction(n_dist, _visibility_radius_, SAFETY_DISTANCE_UAVS, DESIRED_DISTANCE_UAVS);

      if (w_def && w > 0.0 && n_dist < desired) {
        vec_separation -= w * dir_to_neighbor;
      }
      if (n_dist > far_thr && n_dist < _visibility_radius_) {
        vec_separation += coh_gain * dir_to_neighbor;
      }
    }

    double sep_boost = 1.0;
    if (my_gate_dist < 5.0) sep_boost = 1.2;
    if (my_gate_dist < 3.0) sep_boost = 1.5;

    vec_separation *= sep_boost;

    const double max_sep_norm = 1.0;
    if (vec_separation.norm() > max_sep_norm) {
      vec_separation = vec_separation.normalized() * max_sep_norm;
    }

    // ----------------- 3) EVITAR OBSTÁCULOS -----------------
    bool obs_repulsion_undefined = false;

    Eigen::Vector3d vec_obs = Eigen::Vector3d::Zero();
    const Eigen::Vector3d obs_vec = perception.obstacles.closest;
    const double obs_dist = obs_vec.norm();

    if (obs_dist > 1e-3) {
      bool w_def = false;
      double w = 0.0;
      std::tie(w_def, w) = weightingFunction(obs_dist, _visibility_radius_,
                                            SAFETY_DISTANCE_OBS, DESIRED_DISTANCE_OBS);

      if (!w_def) {
        obs_repulsion_undefined = true;  
      }

      if (w_def && w > 0.0) {
        vec_obs -= w * (obs_vec / obs_dist);
      }
    }

    // ----------------- 4) DETECTAR SI HE TRAVERSADO LA CELDA -----------------
    double nav_weight = param1;

    if (_state_ == TRAVERSING) {


      if (has_gate) {
        Eigen::Vector3d d = Eigen::Vector3d::Zero();
        switch (_navigation_direction_) {
          case UP:    d = Eigen::Vector3d(0, 1, 0); break;
          case DOWN:  d = Eigen::Vector3d(0,-1, 0); break;
          case RIGHT: d = Eigen::Vector3d(1, 0, 0); break;
          case LEFT:  d = Eigen::Vector3d(-1,0, 0); break;
          default:    d = Eigen::Vector3d(0, 1, 0); break;
        }
        d.normalize();

        Eigen::Vector3d c = -gate_center;
        if (c.norm() > 1e-6) c.normalize();

        const double dot_product = d.dot(c);
        const double gate_dist = gate_center.norm();
        const double CELL_RADIUS = 5.0;

        if (gate_dist < CELL_RADIUS) {
          const bool in_new_cell = (dot_product > 0);
          if (!_have_traversed_cell_ && in_new_cell) {
            _have_traversed_cell_ = true;
          }
        }
      }
      // ----------------- 5) SINCRONIZACIÓN EN EL GATE -----------------
      const int my_state_i = stateToInt(_state_);
      const int my_dir_i   = directionToInt(_navigation_direction_);

      const double shared_d = _have_traversed_cell_ ? -1.0 : my_gate_dist;
      action_handlers.shareVariables(my_state_i, my_dir_i, shared_d);

      double min_dist_same_dir  = my_gate_dist;
      bool   any_valid_neighbor = false;
      const int trav_i = stateToInt(TRAVERSING);

      for (const auto &n : perception.neighbors) {
        const auto &vars = n.shared_variables;
        if (vars.int1 != trav_i) continue;
        if (vars.int2 != my_dir_i) continue;
        if (vars.dbl <= 0.0) continue; 

        any_valid_neighbor = true;
        if (vars.dbl < min_dist_same_dir) min_dist_same_dir = vars.dbl;
      }

      const double LEADER_EPS     = 0.05;
      const double GATE_SLOW_DIST = 5.0;
      const double GATE_STOP_DIST = 2.5;

      const bool i_am_leader = (!any_valid_neighbor) ||
                             (std::fabs(my_gate_dist - min_dist_same_dir) < LEADER_EPS);

      if (!i_am_leader) {
        if (my_gate_dist < GATE_STOP_DIST) nav_weight = 0.05 * param1;
        else if (my_gate_dist < GATE_SLOW_DIST) nav_weight = 0.3 * param1;

        nav_weight = std::max(nav_weight, 0.15 * param1);
      }

      // ----------------- 6) DETECTAR SI TODOS HAN TRAVERSADO -----------------
      bool all_traversed = _have_traversed_cell_;

      if (all_traversed) {
        for (const auto &n : perception.neighbors) {
          const auto &vars = n.shared_variables;
          if (vars.int1 == trav_i && vars.int2 == my_dir_i) {
            if (vars.dbl >= 0.0) { all_traversed = false; break; }
          }
        }
      }

      if (all_traversed) {
        _state_ = AGREEING_ON_DIRECTION;
        _navigation_direction_ = NONE;
        _have_traversed_cell_ = false;
        _agree_consensus_start_time_ = -1.0;
        _agree_last_dir_int_ = -1;
        _go_traversing_next_ = false;
      }
    }

const bool in_transition = has_gate && !_have_traversed_cell_ && (my_gate_dist < 6.0);
    if (_state_ == TRAVERSING && in_transition) {
        nav_weight *= 2.0;
        vec_separation *= 0.25;
    }

    // ----------------- 7) COMBINAR SUBVECTORES -----------------
    const Eigen::Vector3d v_nav = nav_weight * vec_navigation;
    const Eigen::Vector3d v_sep = param2    * vec_separation;
    const Eigen::Vector3d v_obs = param3    * vec_obs;

    vec_action = v_nav + v_sep + v_obs;

    double VMAX = 0.8;           

    if (obs_dist < 4.0) VMAX = 0.6;
    if (obs_dist < 3.0) VMAX = 0.45;

    if (my_gate_dist < 4.0) VMAX = std::min(VMAX, 0.6);
    if (my_gate_dist < 3.0) VMAX = std::min(VMAX, 0.45);

    if (vec_action.norm() > VMAX) {
      vec_action = vec_action.normalized() * VMAX;
    }
    double dt = 0.02; 
    if (_last_time_ > 0.0) dt = std::max(1e-3, current_time - _last_time_);
    _last_time_ = current_time;

    const double tau = 0.25;               
    const double alpha = dt / (tau + dt); 

    vec_action = (1.0 - alpha) * _last_action_ + alpha * vec_action;
    _last_action_ = vec_action;

    printVector3d(vec_action, "Action:");

    // ----------------- 8) VISUALIZACIÓN -----------------
    action_handlers.visualizeArrow("nav_dir", vec_navigation, Color_t{0.3, 0.3, 1.0, 0.4});
    action_handlers.visualizeArrow("v_nav",   v_nav,          Color_t{0.0, 0.0, 1.0, 0.9});
    action_handlers.visualizeArrow("separation", vec_separation, Color_t{1.0, 0.0, 0.0, 0.5});
    action_handlers.visualizeArrow("obstacle",   vec_obs,        Color_t{0.0, 1.0, 0.0, 0.5});
    action_handlers.visualizeArrow("v_sep", v_sep, Color_t{1.0, 0.0, 0.0, 0.8});
    action_handlers.visualizeArrow("v_obs", v_obs, Color_t{0.0, 1.0, 0.0, 0.8});
    action_handlers.visualizeArrow("target", perception.target_vector, Color_t{1.0, 1.0, 1.0, 0.5});
    action_handlers.visualizeArrow("action", vec_action, Color_t{0.0, 0.0, 0.0, 1.0});

    return vec_action;
  }

  action_handlers.shareVariables(stateToInt(_state_), directionToInt(_navigation_direction_), 0.0);

  return Eigen::Vector3d::Zero();
}

/* weightingFunction() //{ */

std::tuple<bool, double> Swarm::weightingFunction(const double distance, const double visibility, const double safety_distance,
                                                  [[maybe_unused]] const double desired_distance) {

  if (distance <= safety_distance) {
    return {false, 0.0};
  }

  if (distance >= visibility) {
    return {true, 0.0};
  }

  const double range = visibility - safety_distance;
  if (range <= 0.0){
    return {false, 0.0};
  }

  double diff = distance - safety_distance;
  if (diff < 1e-5) {
    diff = 1e-5;
  }

  double weight = safety_distance / diff;

  return {true, weight};
}

//}

// | -- Helper methods to be filled in by students if needed -- |

/* targetToDirection() //{ */

Direction_t Swarm::targetToDirection(const Eigen::Vector3d &target_vector) {

  const double x = target_vector.x();
  const double y = target_vector.y();

  if (std::abs(x) < 1e-6 && std::abs(y) < 1e-6) {
    return UP;
  }

  if (std::abs(x) > std::abs(y)) {
    return (x > 0.0) ? RIGHT : LEFT;
  } else {
    return (y > 0.0) ? UP : DOWN;
  }
}

//}

// | ------------ Helper methods for data handling ------------ |

/* selectGateInDirection() //{ */

unsigned int Swarm::selectGateInDirection(const Direction_t &direction, const Obstacles_t &obstacles) {
  (void) obstacles;
  switch (direction) {

    case UP: {
      return 1;
      break;
    }

    case DOWN: {
      return 3;
      break;
    }

    case LEFT: {
      return 2;
      break;
    }

    case RIGHT: {
      return 0;
      break;
    }

    case NONE: {
      std::cout << "[ERROR] selectGateInDirection() given direction=NONE. Can't determine the gate, returning G1." << std::endl;
      break;
    }
  }

  return 0;
}

//}

/* selectGateClosest() //{ */

unsigned int Swarm::selectGateClosest(const Obstacles_t &obstacles) {

  unsigned int min_idx  = 0;
  double       min_dist = obstacles.gates[0].first.norm();

  for (unsigned int i = 0; i < obstacles.gates.size(); i++) {

    const auto   G      = obstacles.gates[i];
    const double G_dist = (G.first.norm() < G.second.norm()) ? G.first.norm() : G.second.norm();

    if (G_dist < min_dist) {
      min_idx  = i;
      min_dist = G_dist;
    }
  }

  return min_idx;
}

//}

/* computeMutualDistances() //{ */

std::vector<double> Swarm::computeMutualDistances(const std::vector<Neighbor_t> &neighbors) {

  // All known positions (incl. mine)
  std::vector<Eigen::Vector3d> positions = {Eigen::Vector3d::Zero()};
  for (const auto &n : neighbors) {
    positions.push_back(n.position);
  }

  // Compute all mutual distances
  std::vector<double> distances;
  for (unsigned int i = 0; i < positions.size(); i++) {
    for (unsigned int j = i + 1; j < positions.size(); j++) {
      distances.push_back((positions[j] - positions[i]).norm());
    }
  }

  return distances;
}

//}

/* integersAreUnique() //{ */

bool Swarm::integersAreUnique(const std::vector<int> &integers) {

  const auto count_map = countIntegers(integers);

  return count_map.size() == integers.size();
}

//}

/* countIntegers() //{ */

std::map<int, int> Swarm::countIntegers(const std::vector<int> &integers) {

  std::map<int, int> count_map;

  for (const int i : integers) {
    if (count_map.find(i) == count_map.end()) {
      count_map[i] = 0;
    }
    count_map[i]++;
  }

  return count_map;
}

//}

/* getMajority() //{ */

std::tuple<int, int> Swarm::getMajority(const std::map<int, int> &integer_counts) {

  if (integer_counts.empty()) {
    return {-1, -1};
  }

  int max_idx = 0;
  int max_val = 0;

  for (auto it = integer_counts.begin(); it != integer_counts.end(); ++it) {
    if (it->second > max_val) {
      max_idx = it->first;
      max_val = it->second;
    }
  }

  return {max_idx, max_val};
}

std::tuple<int, int> Swarm::getMajority(const std::vector<int> &integers) {
  return getMajority(countIntegers(integers));
}

//}

// | --------- Helper methods for data-type conversion -------- |

/* stateToInt() //{ */

int Swarm::stateToInt(const State_t &state) {
  return static_cast<int>(state);
}

//}

/* intToState() //{ */

State_t Swarm::intToState(const int value) {
  return static_cast<State_t>(value);
}

//}

// | --------------- Helper methods for printing -------------- |

/* stateToString() //{ */

std::string Swarm::stateToString(const State_t &state) {

  switch (state) {

    case INIT_STATE: {
      return "INIT_STATE";
      break;
    }

    case AGREEING_ON_DIRECTION: {
      return "AGREEING_ON_DIRECTION";
      break;
    }

    case TRAVERSING: {
      return "TRAVERSING";
      break;
    }

    default: {
      break;
    }
  }

  return "UNKNOWN";
}

/* proposeNewDirection() //{ */
Direction_t Swarm::proposeNewDirection(const Perception_t &perception) {

  const double x = perception.target_vector.x();
  const double y = perception.target_vector.y();

  const double switch_eps = 1.0;

  // 1) Si tenemos una dirección comprometida previa, mantener eje
  if (_committed_direction_ != NONE) {

    bool moving_x = (_committed_direction_ == LEFT || _committed_direction_ == RIGHT);
    bool moving_y = (_committed_direction_ == UP   || _committed_direction_ == DOWN);

    if (moving_x) {
      if (std::abs(x) > switch_eps)
        return (x > 0.0) ? RIGHT : LEFT;
      else
        return (y > 0.0) ? UP : DOWN;
    }

    if (moving_y) {
      if (std::abs(y) > switch_eps)
        return (y > 0.0) ? UP : DOWN;
      else
        return (x > 0.0) ? RIGHT : LEFT;
    }
  }

  // 2) Si no hay memoria aún, eje dominante
  if (std::abs(x) > std::abs(y))
    return (x > 0.0) ? RIGHT : LEFT;
  else
    return (y > 0.0) ? UP : DOWN;
}

//}

}  // namespace task_03_swarm
