# Multi-UAV Swarm Control with Distributed Consensus

Decentralized swarm navigation strategy for multiple UAVs based on consensus, state-machine coordination and reactive obstacle avoidance.

## Overview

This project implements a fully decentralized control strategy for a UAV swarm operating in a structured environment with gates and obstacles.

The system combines:

- Distributed consensus on navigation direction
- Finite State Machine (FSM)
- Dynamic leader emergence
- Inter-UAV separation and cohesion
- Obstacle avoidance
- Smooth velocity filtering

The implementation was developed as part of the Multi-Robot Systems course at CVUT (Czech Technical University in Prague).

---

## Swarm Architecture

Each UAV operates independently using only:

- Local perception (neighbors + obstacles)
- Shared variables exchanged with neighbors
- No central controller
- No global map

Emergent coordinated behaviour is achieved through local interaction rules.

---

## State Machine

The swarm operates using three states:

### 1️⃣ INIT_STATE
Initial transition to agreement phase.

### 2️⃣ AGREEING_ON_DIRECTION
- Each UAV proposes a navigation direction.
- Majority voting among neighbors.
- Consensus must be stable for a short time window.
- Direction is committed only after agreement.

### 3️⃣ TRAVERSING
- Swarm moves through selected gate.
- Dynamic leader selection (closest UAV to gate).
- Speed adaptation near gate.
- Traversal completion detected collectively.

---

## Control Vector Composition

Final velocity command:

v = w_nav * v_navigation  
  + w_sep * v_separation  
  + w_obs * v_obstacle  

Where:

- Navigation → gate/target direction
- Separation → collision avoidance + mild cohesion
- Obstacle → repulsive weighting function

Velocity is saturated and smoothed using first-order filtering.

---

## Boids-Based Behaviour (task extension)

An additional implementation based on Boids rules includes:

- Cohesion
- Separation
- Alignment
- Beacon influence
- Probabilistic state diffusion between agents

---

## Tech Stack

- C++
- Eigen
- Decentralized multi-agent framework
- UAV simulation environment (CVUT MRS)

---

## Key Concepts Demonstrated

- Distributed consensus algorithms
- Multi-agent coordination
- Swarm robotics
- State-machine design
- Reactive navigation
- Leader-free synchronization
- Local-to-global emergent behaviour
