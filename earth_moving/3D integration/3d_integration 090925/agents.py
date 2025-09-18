"""
Multi-Agent System for Earth Moving Simulation

This module provides agent classes for coordinated planning and execution
in the earth moving environment. Supports multi-step lookahead planning
with spillage-based environment prediction.
"""

import math
import random
from typing import List, Dict, Tuple, Optional, Any
from enum import Enum


class ActionType(Enum):
    """Types of actions an agent can perform."""
    MOVE_AND_COLLECT = "move_and_collect"
    WAIT = "wait"
    COORDINATE = "coordinate"


class AgentAction:
    """Represents a single action that an agent can take."""
    
    def __init__(self, action_type: ActionType, target_cell: Optional[Tuple[int, int]] = None, 
                 path_type: str = "target", expected_objects: int = 0, expected_spillage: Dict = None):
        self.action_type = action_type
        self.target_cell = target_cell  # (x, y) coordinates
        self.path_type = path_type  # "target" or "highway"
        self.expected_objects = expected_objects
        self.expected_spillage = expected_spillage or {}
        self.estimated_cost = 0.0  # Time/energy cost
        self.estimated_reward = 0.0  # Expected benefit
    
    def __repr__(self):
        if self.target_cell:
            return f"Action({self.action_type.value} -> {self.target_cell}, objects={self.expected_objects})"
        else:
            return f"Action({self.action_type.value})"


class Agent:
    """
    Intelligent agent for multi-step planning in earth moving simulation.
    
    Features:
    - Multi-step lookahead planning
    - Coordination with other agents
    - Spillage-aware decision making
    - State tracking and management
    """
    
    def __init__(self, agent_id: str, initial_position: Tuple[int, int], 
                 capacity: int = 10, planning_horizon: int = 3):
        # Agent identity and basic properties
        self.agent_id = agent_id
        self.capacity = capacity
        self.planning_horizon = planning_horizon
        
        # Current state
        self.position = initial_position  # (x, y)
        self.current_load = 0
        self.total_collected = 0
        self.total_delivered = 0
        self.energy_used = 0.0
        
        # Planning state
        self.planned_actions = []  # Queue of planned actions
        self.planning_depth = 0
        self.last_evaluation_score = 0.0
        
        # Coordination
        self.known_agents = {}  # Other agents' states
        self.coordination_messages = []
        
        # Performance tracking
        self.action_history = []
        self.planning_time = 0.0
        
    def get_state(self):
        """Get current agent state for coordination and planning."""
        return {
            'agent_id': self.agent_id,
            'position': self.position,
            'current_load': self.current_load,
            'capacity': self.capacity,
            'planned_actions': [str(action) for action in self.planned_actions],
            'total_collected': self.total_collected,
            'total_delivered': self.total_delivered,
            'energy_used': self.energy_used
        }
    
    def update_known_agent(self, other_agent_state: Dict):
        """Update knowledge about another agent's state."""
        agent_id = other_agent_state['agent_id']
        self.known_agents[agent_id] = other_agent_state
    
    def plan_actions(self, env, use_coordination: bool = True):
        """
        Plan a sequence of actions using multi-step lookahead.
        
        :param env: Current environment state
        :param use_coordination: Whether to consider other agents
        :return: List of planned actions
        """
        if not env.cells_with_objects:
            return [AgentAction(ActionType.WAIT)]
        
        # Clear previous plans
        self.planned_actions = []
        
        # Find best action sequence using recursive planning
        best_actions = self._plan_recursive(env, self.planning_horizon, use_coordination)
        
        if best_actions:
            self.planned_actions = best_actions
            print(f"Agent {self.agent_id} planned {len(best_actions)} actions")
        else:
            # Fallback: simple greedy action
            self.planned_actions = [self._get_greedy_action(env)]
            print(f"Agent {self.agent_id} using fallback greedy action")
        
        return self.planned_actions
    
    def _plan_recursive(self, env, depth: int, use_coordination: bool, 
                       current_actions: List[AgentAction] = None):
        """
        Recursive planning with environment simulation.
        
        :param env: Environment state
        :param depth: Remaining planning depth
        :param use_coordination: Consider other agents
        :param current_actions: Actions planned so far
        :return: Best action sequence
        """
        if depth <= 0 or not env.cells_with_objects:
            return current_actions or []
        
        current_actions = current_actions or []
        best_actions = None
        best_score = -float('inf')
        
        # Generate possible actions from current state
        possible_actions = self._generate_possible_actions(env, use_coordination)
        
        for action in possible_actions:
            # Simulate this action
            env_copy = env.copy_state()
            success = self._simulate_action(env_copy, action)
            
            if success:
                # Evaluate resulting state
                metrics = env_copy.evaluate_environment_state()
                immediate_score = metrics['composite_score']
                
                # Plan remaining moves recursively
                extended_actions = current_actions + [action]
                future_actions = self._plan_recursive(
                    env_copy, depth - 1, use_coordination, extended_actions
                )
                
                # Calculate total score (immediate + future potential)
                if len(future_actions) > len(extended_actions):
                    # Discount future rewards
                    future_potential = 0.8 ** (len(future_actions) - len(extended_actions))
                    total_score = immediate_score + future_potential * 0.5
                else:
                    total_score = immediate_score
                
                # Track best sequence
                if total_score > best_score:
                    best_score = total_score
                    best_actions = future_actions if future_actions else extended_actions
        
        return best_actions
    
    def _generate_possible_actions(self, env, use_coordination: bool):
        """Generate possible actions from current state."""
        actions = []
        
        # Get current cell
        current_cell = env.get_cell(self.position[0], self.position[1])
        if not current_cell:
            return actions
        
        # Action 1: Move to collect objects (if not at capacity)
        if self.current_load < self.capacity:
            # Find promising cells to move to
            candidate_cells = self._find_collection_targets(env, max_candidates=5)
            
            for target_cell in candidate_cells:
                if target_cell == current_cell:
                    continue
                
                # Check if another agent is planning to go there (coordination)
                if use_coordination and self._is_cell_contested(target_cell):
                    continue
                
                # Get path preview for this target
                paths = env.get_path_for_preview(target_cell, "target")
                if paths:
                    path_info = paths[0]
                    expected_objects = min(
                        path_info['objects'], 
                        self.capacity - self.current_load
                    )
                    
                    action = AgentAction(
                        ActionType.MOVE_AND_COLLECT,
                        target_cell=(target_cell.x, target_cell.y),
                        path_type="target",
                        expected_objects=expected_objects,
                        expected_spillage=path_info.get('impacted_cells', {})
                    )
                    
                    # Estimate costs and rewards
                    action.estimated_cost = path_info['distance']
                    action.estimated_reward = expected_objects / (path_info['distance'] + 1)
                    
                    actions.append(action)
        
        # Action 2: Wait (if other agents are working or no good options)
        wait_action = AgentAction(ActionType.WAIT)
        wait_action.estimated_cost = 1.0
        wait_action.estimated_reward = 0.1  # Small reward for coordination
        actions.append(wait_action)
        
        # Sort by reward/cost ratio
        actions.sort(key=lambda a: a.estimated_reward / (a.estimated_cost + 0.1), reverse=True)
        
        return actions[:3]  # Return top 3 actions for planning efficiency
    
    def _find_collection_targets(self, env, max_candidates: int = 5):
        """Find the most promising cells to collect objects from."""
        if not env.cells_with_objects:
            return []
        
        # Score cells by collection potential
        scored_cells = []
        
        for cell in env.cells_with_objects:
            if cell.num_objects <= 0:
                continue
            
            # Calculate distance from agent
            distance = math.hypot(
                cell.x - self.position[0], 
                cell.y - self.position[1]
            )
            
            # Score based on objects/distance ratio with some bonuses
            base_score = cell.num_objects / (distance + 1)
            
            # Bonus for cells on highways
            heat_bonus = getattr(cell, 'heat_map', 0) * 0.1
            
            # Bonus for cells closer to target zone
            proximity_bonus = 1.0 / (cell.distance_to_target + 1)
            
            total_score = base_score + heat_bonus + proximity_bonus
            scored_cells.append((total_score, cell))
        
        # Sort and return top candidates
        scored_cells.sort(reverse=True)
        return [cell for _, cell in scored_cells[:max_candidates]]
    
    def _is_cell_contested(self, target_cell):
        """Check if another agent is planning to go to this cell."""
        for agent_id, agent_state in self.known_agents.items():
            if agent_id == self.agent_id:
                continue
            
            # Check if other agent has planned actions targeting this cell
            for action_str in agent_state.get('planned_actions', []):
                if f"-> ({target_cell.x}, {target_cell.y})" in action_str:
                    return True
        
        return False
    
    def _simulate_action(self, env, action: AgentAction):
        """Simulate executing an action in the environment copy."""
        try:
            if action.action_type == ActionType.MOVE_AND_COLLECT:
                if action.target_cell:
                    target_cell = env.get_cell(action.target_cell[0], action.target_cell[1])
                    if target_cell and target_cell.num_objects > 0:
                        # Execute path in environment
                        env.execute_path(
                            target_cell, 
                            action.path_type, 
                            use_spillage=True
                        )
                        
                        # Update agent state
                        self.position = action.target_cell
                        collected = min(action.expected_objects, self.capacity - self.current_load)
                        self.current_load += collected
                        self.total_collected += collected
                        
                        return True
            
            elif action.action_type == ActionType.WAIT:
                # Simple wait - no environment changes
                return True
            
            return False
            
        except Exception as e:
            print(f"Action simulation failed: {e}")
            return False
    
    def _get_greedy_action(self, env):
        """Fallback greedy action selection."""
        targets = self._find_collection_targets(env, max_candidates=1)
        
        if targets and self.current_load < self.capacity:
            target = targets[0]
            paths = env.get_path_for_preview(target, "target")
            if paths:
                return AgentAction(
                    ActionType.MOVE_AND_COLLECT,
                    target_cell=(target.x, target.y),
                    expected_objects=min(target.num_objects, self.capacity - self.current_load)
                )
        
        return AgentAction(ActionType.WAIT)
    
    def execute_next_action(self, env):
        """Execute the next planned action."""
        if not self.planned_actions:
            return False
        
        action = self.planned_actions.pop(0)
        success = self._simulate_action(env, action)
        
        if success:
            self.action_history.append(action)
            print(f"Agent {self.agent_id} executed: {action}")
        else:
            print(f"Agent {self.agent_id} failed to execute: {action}")
        
        return success
    
    def get_performance_summary(self):
        """Get performance summary for analysis."""
        efficiency = self.total_delivered / (self.energy_used + 1)
        return {
            'agent_id': self.agent_id,
            'total_collected': self.total_collected,
            'total_delivered': self.total_delivered,
            'energy_used': self.energy_used,
            'efficiency': efficiency,
            'actions_executed': len(self.action_history),
            'current_load': self.current_load,
            'position': self.position
        }


class MultiAgentCoordinator:
    """Coordinates multiple agents for efficient collaboration."""
    
    def __init__(self, agents: List[Agent]):
        self.agents = agents
        self.coordination_enabled = True
        self.planning_rounds = 0
        
    def coordinate_agents(self, env):
        """Coordinate all agents for the next round of actions."""
        if not self.coordination_enabled:
            # Independent planning
            for agent in self.agents:
                agent.plan_actions(env, use_coordination=False)
            return
        
        # Share agent states
        self._share_agent_states()
        
        # Sequential planning with coordination
        for agent in self.agents:
            agent.plan_actions(env, use_coordination=True)
            self._update_coordination_info()
        
        self.planning_rounds += 1
        print(f"Coordination round {self.planning_rounds} completed for {len(self.agents)} agents")
    
    def _share_agent_states(self):
        """Share all agent states for coordination."""
        agent_states = {agent.agent_id: agent.get_state() for agent in self.agents}
        
        for agent in self.agents:
            for other_id, other_state in agent_states.items():
                if other_id != agent.agent_id:
                    agent.update_known_agent(other_state)
    
    def _update_coordination_info(self):
        """Update coordination information after planning."""
        # Re-share states after planning
        self._share_agent_states()
    
    def execute_agent_actions(self, env):
        """Execute one action for each agent."""
        results = {}
        
        for agent in self.agents:
            success = agent.execute_next_action(env)
            results[agent.agent_id] = success
        
        # Update environment after all actions
        if any(results.values()):
            env.update_environment()
        
        return results
    
    def get_system_performance(self):
        """Get performance summary for all agents."""
        individual_performance = [agent.get_performance_summary() for agent in self.agents]
        
        total_collected = sum(perf['total_collected'] for perf in individual_performance)
        total_delivered = sum(perf['total_delivered'] for perf in individual_performance)
        total_energy = sum(perf['energy_used'] for perf in individual_performance)
        
        system_performance = {
            'total_agents': len(self.agents),
            'planning_rounds': self.planning_rounds,
            'total_collected': total_collected,
            'total_delivered': total_delivered,
            'total_energy_used': total_energy,
            'system_efficiency': total_delivered / (total_energy + 1),
            'individual_performance': individual_performance
        }
        
        return system_performance


# Legacy compatibility classes
class BaseAgent:
    """Legacy base agent class for backward compatibility."""
    def __init__(self, id):
        self.id = id

    def decide_action(self, env):
        raise NotImplementedError("Subclasses must implement this method")


class RandomAgent(BaseAgent):
    """Legacy random agent for backward compatibility."""
    def decide_action(self, env):
        return "random_move"