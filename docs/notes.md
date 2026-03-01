# Implementation Notes – Multi-UAV Swarm Consensus

## 1. System Overview

This project implements a fully decentralized swarm navigation strategy for multiple UAVs.

Each agent operates using:
- Local perception (neighbors and obstacles)
- Shared state variables
- No central coordinator
- No global map

Coordinated behavior emerges through distributed consensus and local interaction rules.

---

## 2. Finite State Machine (FSM)

The swarm operates using three states:

### INIT_STATE
Initial state. Agents transition immediately to agreement phase.

### AGREEING_ON_DIRECTION
- Each UAV proposes a navigation direction based on its target vector.
- Directions are shared among neighbors.
- A local majority voting mechanism is applied.
- Consensus must be maintained for a fixed time window before commitment.
- This prevents oscillatory behavior and unstable switching.

If consensus is reached:
- The direction is committed.
- Agents transition to the TRAVERSING state.

### TRAVERSING
- Agents move toward the selected gate.
- Synchronization occurs via shared distance-to-gate values.
- A dynamic leader emerges:
  - The UAV closest to the gate naturally acts as leader.
  - Other agents adapt speed when approaching the gate.
- Traversal completion is detected collectively.
- Once all agents traverse the cell, the system returns to AGREEING_ON_DIRECTION.

---

## 3. Navigation Vector Composition

The final velocity command is computed as:

v = w_nav * v_navigation  
  + w_sep * v_separation  
  + w_obs * v_obstacle  

Where:

- v_navigation → normalized direction toward selected gate or target
- v_separation → inter-UAV collision avoidance + mild cohesion
- v_obstacle → obstacle repulsion based on inverse-distance weighting

Velocity constraints:
- Saturated with adaptive VMAX
- Reduced near obstacles and gates
- Smoothed using first-order low-pass filtering

---

## 4. Distributed Consensus Mechanism

Consensus is achieved using:

- Local majority voting
- Tie-breaking via self-preference
- Temporal validation window (CONSENSUS_HOLD)
- Shared state broadcasting

This ensures:
- Fully decentralized agreement
- Robust direction selection
- Stability against transient neighbor changes

---

## 5. Separation and Cohesion Strategy

For each visible neighbor:

- Repulsion if distance < desired inter-UAV spacing
- Mild attraction if too far but within visibility radius
- Weighting based on safety and visibility distances

Obstacle avoidance uses a similar inverse-distance weighting function.

---

## 6. Robustness Features

- Direction memory to prevent oscillations
- Dynamic speed scaling near gates
- Leader-free synchronization
- Velocity smoothing
- Safety distance enforcement

---

## 7. Design Goals

- Fully decentralized architecture
- No global coordination
- Stable group motion
- Safe inter-UAV spacing
- Coordinated cell traversal
- Emergent swarm behavior from local rules

---

## Skills Demonstrated

- Distributed multi-agent consensus
- Swarm robotics
- Finite State Machine design
- Decentralized decision-making
- Obstacle avoidance
- Reactive navigation
- Emergent coordination behavior
