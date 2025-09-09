import numpy as np
import random
from collections import defaultdict
import math


class POMCPNode:
    def __init__(self, action=None, parent=None):
        self.action = action
        self.parent = parent
        self.children = {}
        self.visit_count = 0
        self.value_sum = 0.0
        self.particles = []
        
    def is_leaf(self):
        return len(self.children) == 0
        
    def ucb_value(self, c=1.4):
        if self.visit_count == 0:
            return float('inf')
        exploitation = self.value_sum / self.visit_count
        exploration = c * math.sqrt(math.log(self.parent.visit_count) / self.visit_count)
        return exploitation + exploration

class ParticleFilter:
    def __init__(self, goals=None, GOAL_SHAPES=None):
        self.goals = goals or list(GOAL_SHAPES.keys())
        self.particles = []
        self.reset()
    
    def reset(self):
        """Reset particles to uniform distribution over all goals"""
        self.particles = [random.choice(self.goals) for _ in range(500)]
    
    def update(self, observation, likelihood_fn):
        """Update particles based on observation using likelihood function"""
        if not self.particles:
            return
            
        # Calculate weights for each particle
        weights = []
        for particle in self.particles:
            weight = likelihood_fn(particle, observation)
            weights.append(weight)
        
        # Normalize weights
        total_weight = sum(weights)
        if total_weight == 0:
            return  # No update if all weights are zero
        
        weights = [w / total_weight for w in weights]
        
        # Resample particles based on weights
        new_particles = []
        for _ in range(len(self.particles)):
            # Weighted random selection
            r = random.random()
            cumsum = 0
            for i, weight in enumerate(weights):
                cumsum += weight
                if r <= cumsum:
                    new_particles.append(self.particles[i])
                    break
        
        self.particles = new_particles
    
    def get_belief(self):
        """Get current belief distribution over goals"""
        if not self.particles:
            return {}
        belief = defaultdict(int)
        for particle in self.particles:
            belief[particle] += 1
        total = len(self.particles)
        return {goal: count/total for goal, count in belief.items()}

class POMCP:
    def __init__(self, game_state, particle_filter, gamma=0.95, c=1.4):
        self.game_state = game_state
        self.particle_filter = particle_filter
        self.gamma = gamma  # discount factor
        self.c = c  # UCB exploration parameter
        self.root = None
        
    def search(self, num_simulations=100):
        """Run POMCP search to find best action"""
        self.root = POMCPNode()
        self.root.particles = self.particle_filter.particles.copy()
        
        for _ in range(num_simulations):
            self.simulate(self.root, 0)
        
        # Return best action based on visit counts
        if not self.root.children:
            return None
            
        best_action = max(self.root.children.keys(), 
                         key=lambda a: self.root.children[a].visit_count)
        return best_action
    
    def simulate(self, node, depth):
        """Simulate one path through the tree"""
        if depth > 10:  # max depth
            return 0
        
        if node.is_leaf():
            return self.rollout(node, depth)
        
        action = self.select_action(node)
        
        if action not in node.children:
            node.children[action] = POMCPNode(action, node)
            return self.rollout(node.children[action], depth)
        
        child = node.children[action]
        reward = self.get_reward(action, node.particles)
        future_value = self.simulate(child, depth + 1)
        total_value = reward + self.gamma * future_value
        
        node.visit_count += 1
        node.value_sum += total_value
        
        return total_value
    
    def select_action(self, node):
        """Select action using UCB"""
        if not node.children:
            # Return random action from available actions
            return random.choice(['suggest', 'wait'])
        
        return max(node.children.keys(), 
                  key=lambda a: node.children[a].ucb_value(self.c))
    
    def rollout(self, node, depth):
        """Random rollout from current node - no threshold-based bias"""
        if depth > 5:  # Shorter rollouts for faster decisions
            return 0
        
        # Pure random choice - let RL learn the optimal policy
        action = np.random.choice(['suggest', 'wait'])
            
        reward = self.get_reward(action, node.particles)
        return reward + self.gamma * self.rollout(node, depth + 1)
    
    def get_reward(self, action, particles):
        """Get expected reward for action given particles - pure RL approach"""
        if action == 'wait':
            return -0.1  # Small negative reward for waiting (time cost)
        elif action == 'suggest':
            # Expected reward based on current belief and game state
            belief = self.get_particle_belief(particles)
            if not belief:
                return -2  # No belief, likely bad suggestion
                
            # Calculate expected reward without any threshold logic
            # Let the RL system learn what constitutes a good suggestion
            expected_reward = 0
            total_prob = 0
            
            for goal, prob in belief.items():
                overlap = self.game_state.calculate_overlap(goal)
                
                # Simple linear model: higher overlap = higher acceptance probability
                # But let RL learn the exact relationship through experience
                p_accept_goal = min(0.9, overlap * 1.5)  # Could be completely wrong - RL will learn
                p_accept_move = min(0.7, overlap * 1.0)  
                p_reject = max(0.1, 1 - p_accept_goal - p_accept_move)
                
                # Normalize probabilities
                total = p_accept_goal + p_accept_move + p_reject
                p_accept_goal /= total
                p_accept_move /= total  
                p_reject /= total
                
                goal_reward = (p_accept_goal * 10 +    # Large reward for goal acceptance
                              p_accept_move * 2 +      # Smaller reward for move acceptance  
                              p_reject * (-1))         # Negative reward for rejection
                
                expected_reward += prob * goal_reward
                total_prob += prob
            
            if total_prob > 0:
                expected_reward /= total_prob
            
            return expected_reward
        
        return 0
    
    def get_particle_belief(self, particles):
        """Convert particle list to belief distribution"""
        if not particles:
            return {}
        belief = defaultdict(int)
        for particle in particles:
            belief[particle] += 1
        total = len(particles)
        return {goal: count/total for goal, count in belief.items()}


class SimpleBlockMover:
    def __init__(self, GOAL_SHAPES, size=0.3):
        self.size = size
        self.blocks = []
                
        self.selected = None  # Keep for visual feedback, but no interaction
        self.goal_shape = None
        self.GOAL_SHAPES = GOAL_SHAPES
        
        # POMCP and particle filter
        self.particle_filter = ParticleFilter(GOAL_SHAPES=GOAL_SHAPES)
        self.pomcp = POMCP(self, self.particle_filter)
        
        # AI suggestion state
        self.suggested_goal = None
        self.suggested_move = None
        self.suggestion_active = False
        self.suggested_goal_name = None
        self.last_red_pos = []
        
        # History for learning
        self.move_history = []
        self.total_reward = 0

        # # Remove mouse event handlers - blocks are no longer interactive
        # self.draw()

    def check_red_block_moved(self):
        """Check if red block moved and reset particle filter if so"""
        # Since blocks can't move interactively anymore, this is mainly for consistency
        return False

    def update_beliefs_from_move(self, moved_block_idx):
        """Update particle filter based on move (kept for compatibility)"""
        def likelihood(goal, observation):
            """Likelihood that human would make this move given this goal"""
            block_idx, _ = observation
            
            # Calculate how much this move improves overlap for this goal
            current_overlap = self.calculate_overlap(goal)
            
            # Humans are more likely to move blocks that improve their suspected goal
            if current_overlap > 0.3:
                return min(1.0, current_overlap * 2)
            else:
                return 0.1  # low but non-zero baseline
        
        observation = (moved_block_idx, self.blocks[moved_block_idx])
        self.particle_filter.update(observation, likelihood)

    def pomcp_decide_action(self):
        """Use POMCP to decide whether to suggest or wait"""
        # Update POMCP with current game state
        self.pomcp.game_state = self
        self.pomcp.particle_filter = self.particle_filter
        
        action = self.pomcp.search(num_simulations=100)
        
        belief = self.particle_filter.get_belief()
        
        # Debug info: calculate expected rewards for both actions
        suggest_reward = self.pomcp.get_reward('suggest', self.particle_filter.particles)
        wait_reward = self.pomcp.get_reward('wait', self.particle_filter.particles)
        
        print(f"Current beliefs: {belief}")
        print(f"Expected rewards - Suggest: {suggest_reward:.2f}, Wait: {wait_reward:.2f}")
        print(f"POMCP decision: {action}")
        
        # Override if suggest reward is clearly better (debugging aid)
        if suggest_reward > wait_reward + 0.5:
            print("(Forcing suggest due to clear advantage)")
            action = 'suggest'
        
        return action

    def make_pomcp_suggestion(self):
        """Use POMCP to decide whether to make a suggestion"""
        action = self.pomcp_decide_action()
        
        if action == 'suggest':
            # Find best goal based on current belief and overlaps
            belief = self.particle_filter.get_belief()
            best_goal = None
            best_score = -1
            
            goal_names_list = ["Goal00", "Goal01", "Goal02", "Goal03", "Goal04","Goal05", "Goal06", "Goal07", "Goal08", "Goal09"]
            # Pure RL approach - no thresholds, just pick the best scoring goal
            for i, goal, prob in enumerate(belief.items()):
                overlap = self.calculate_overlap(goal)
                score = prob * overlap
                if score > best_score:
                    best_score = score
                    best_goal = goal
                    best_goal_name = goal_names_list[i]
            
            # Always suggest if POMCP decided to suggest, regardless of overlap
            if best_goal:
                move = self.find_beneficial_move(best_goal)
                self.suggested_goal = best_goal
                self.suggested_move = move
                self.suggestion_active = True
                self.suggested_goal_name = best_goal_name
                print(f"AI suggests goal '{best_goal}' (belief: {belief.get(best_goal, 0):.1%}, overlap: {self.calculate_overlap(best_goal):.1%})")
                if move:
                    print(f"Suggested move: Block {move['block_idx']} to ({move['target_x']:.0f}, {move['target_y']:.0f})")
                return True
            else:
                print("POMCP decided to suggest but no goal found")
                return False
        else:  # action == 'wait'
            self.suggestion_active = False
            print("POMCP decided to wait for more moves")
            return False

    def process_human_feedback(self, feedback, reward):
        """Process human feedback and update learning"""
        self.total_reward += reward
        print(f"Reward: {reward:+.1f}, Total: {self.total_reward:.1f}")
        
        if feedback == 'accept_goal':
            # Strong positive update for the suggested goal
            def likelihood(goal, observation):
                if goal == self.suggested_goal:
                    return 10.0  # Very high likelihood
                else:
                    return 0.1   # Very low likelihood for other goals
            
            self.particle_filter.update(None, likelihood)
            print(f"Goal '{self.suggested_goal}' accepted - strong belief update")
            
        elif feedback == 'accept_move':
            # Moderate positive update
            def likelihood(goal, observation):
                if goal == self.suggested_goal:
                    return 3.0   # Higher likelihood 
                else:
                    return 0.5   # Lower likelihood for other goals
            
            self.particle_filter.update(None, likelihood)
            
        elif feedback == 'reject':
            # Negative update - reduce belief in suggested goal
            def likelihood(goal, observation):
                if goal == self.suggested_goal:
                    return 0.1   # Much lower likelihood
                else:
                    return 1.0   # Maintain likelihood for other goals
            
            self.particle_filter.update(None, likelihood)

    def calculate_overlap(self, goal_name):
        """Calculate overlap percentage between current blocks and a goal shape"""
        red = [b for b in self.blocks if b['color'] == 'red'][0]
        x0, y0, theta = red['x'], red['y'], red['theta']
        
        goal_positions = []
        for (dx, dy) in self.GOAL_SHAPES[goal_name]:
            c, sn = np.cos(theta), np.sin(theta)
            X = x0 + (dx * self.size * c - dy * self.size * sn)
            Y = y0 + (dx * self.size * sn + dy * self.size * c)
            goal_positions.append((X, Y))
        
        # Check which goal positions have overlapping blocks
        overlapped = 0
        overlap_threshold_dist = self.size * 0.7
        
        for goal_x, goal_y in goal_positions:
            for block in self.blocks:
                dist = np.sqrt((block['x'] - goal_x)**2 + (block['y'] - goal_y)**2)
                if dist <= overlap_threshold_dist:
                    overlapped += 1
                    break
        
        return overlapped / len(goal_positions) if goal_positions else 0

    def find_beneficial_move(self, goal_name):
        """Find a move that would improve overlap for the given goal"""
        red = [b for b in self.blocks if b['color'] == 'red'][0]
        x0, y0, theta = red['x'], red['y'], red['theta']
        
        # Get goal positions
        goal_positions = []
        for (dx, dy) in self.GOAL_SHAPES[goal_name]:
            c, sn = np.cos(theta), np.sin(theta)
            X = x0 + (dx * self.size * c - dy * self.size * sn)
            Y = y0 + (dx * self.size * sn + dy * self.size * c)
            goal_positions.append((X, Y))
        
        # Find unoccupied goal positions
        overlap_threshold_dist = self.size * 0.7
        unoccupied_goals = []
        sub_goal_names = [f"G{i}" for i in range(9)]        
        occupied_goals_names = []

        for i, goal_x, goal_y in enumerate(goal_positions):
            occupied = False
            for block in self.blocks:
                dist = np.sqrt((block['x'] - goal_x)**2 + (block['y'] - goal_y)**2)
                if dist <= overlap_threshold_dist:
                    occupied = True
                    break
            if not occupied:
                unoccupied_goals.append((goal_x, goal_y))
            else:
                occupied_goals_names.append(sub_goal_names[i])
        
        if not unoccupied_goals:
            return None
            
        # Find non-overlapped blocks
        non_overlapped_blocks = []
        for i, block in enumerate(self.blocks):
            is_overlapped = False
            for goal_x, goal_y in goal_positions:
                dist = np.sqrt((block['x'] - goal_x)**2 + (block['y'] - goal_y)**2)
                if dist <= overlap_threshold_dist:
                    is_overlapped = True
                    break
            if not is_overlapped:
                non_overlapped_blocks.append(i)
        
        if not non_overlapped_blocks:
            return None
            
        # Suggest moving a random non-overlapped block to a random unoccupied goal
        block_idx = np.random.choice(non_overlapped_blocks)
        target_x, target_y = unoccupied_goals[np.random.randint(len(unoccupied_goals))]

        return {
            'block_idx': block_idx,
            'target_x': target_x,
            'target_y': target_y,
            'occupied_goals': occupied_goals_names
        }

    def accept_goal(self):
        """Accept the suggested goal - end episode and save goal positions"""
        if self.suggestion_active:
            reward = 10.0  # Large positive reward
            self.process_human_feedback('accept_goal', reward)
            print(f"Episode completed! Goal '{self.suggested_goal}' accepted.")
            
            # Save goal positions to OUTPUT.dat
            self.save_positions_to_file("OUTPUT.dat", "goal_accepted")
            
            self.suggestion_active = False
            self.goal_shape = self.suggested_goal
            self.draw()

    def accept_move(self):
        """Accept and apply the suggested move and save updated positions"""
        if self.suggestion_active and self.suggested_move:
            reward = 2.0  # Moderate positive reward
            self.process_human_feedback('accept_move', reward)
            
            move = self.suggested_move
            self.blocks[move['block_idx']]['x'] = move['target_x']
            self.blocks[move['block_idx']]['y'] = move['target_y']
            print(f"Applied suggested move. Block positions updated.")
            
            # Save updated positions to OUTPUT.dat
            self.save_positions_to_file("OUTPUT.dat", "move_accepted")
            
            self.suggestion_active = False
            self.draw()

    def reject_suggestion(self):
        """Reject the suggestion and continue"""
        if self.suggestion_active:
            reward = -1.0  # Negative reward
            self.process_human_feedback('reject', reward)
            print("Suggestion rejected.")
            self.suggestion_active = False
            self.draw()

class CamillaInference:

    def __init__(self):
        self.GOAL_SHAPES = self._load_goal_shapes()
        self.block_size = 0.30  # meters
        self.inference_class = SimpleBlockMover(self.GOAL_SHAPES, size=self.block_size)

    def _load_goal_shapes(self):            
        GOAL_SHAPES = {
            "Goal00": [(-0.0, 0.0), (-0.3508, -0.0), (0.0, 0.3508), (-0.3508, 0.3508), (0.0, 0.7016), (-0.3508, 0.7016), (-0.7016, 0.0), (-0.7016, 0.3508), (-0.7016, 0.7016)],
            "Goal01": [(-0.0, 0.0), (0.0, 0.3508), (-0.0, 0.7016), (-0.0, 1.0524), (-0.0, 1.4032), (-0.0, 1.754), (-0.0, 2.1048), (0.0, 2.4556), (-0.0, 2.8064)],
            "Goal02": [(-0.0, 0.0), (0.0, 0.3508), (-0.0, 0.7016), (-0.3508, 0.0), (-0.3508, 0.7016), (-0.7016, 0.0), (-0.7016, 0.7016), (-1.0524, 0.0), (-1.0524, 0.7016)],
            "Goal03": [(0.0, 0.0), (-0.0, 0.3508), (-0.3508, 0.3508), (0.0, 0.7016), (-0.3508, 0.7016), (0.0, 1.0524), (-0.3508, 1.0524), (0.0, 1.4032), (-0.7016, 0.7016)],
            "Goal04": [(0.0, 0.0), (-0.0, 0.3508), (-0.0, 0.7016), (0.0, 1.0524), (-0.3508, 1.0524), (-0.7016, 1.0524), (-0.7016, 1.4032), (-0.7016, 1.754), (-0.7016, 2.1048)],
            "Goal05": [(-0.0, 0.0), (-0.0, 0.3508), (-0.3508, -0.0), (-0.0, 0.7016), (-0.7016, 0.0), (-0.0, 1.0524), (-1.0524, 0.0), (-0.0, 1.4032), (-1.4032, 0.0)],
            "Goal06": [(0.0, 0.0), (-0.3508, -0.0), (-0.0, 0.3508), (-0.3508, 0.3508), (-0.0, 0.7016), (-0.3508, 0.7016), (-0.0, 1.0524), (-0.7016, 0.0), (-1.0524, 0.0)],
            "Goal07": [(-0.0, 0.0), (-0.3508, -0.0), (0.0, 0.3508), (-0.3508, 0.3508), (-0.0, 0.7016), (0.0, 1.0524), (-0.3508, 1.0524), (-0.0, 1.4032), (-0.3508, 1.4032)],
            "Goal08": [(-0.0, 0.0), (-0.0, 0.3508), (0.0, 0.7016), (-0.3508, 0.3508), (-0.3508, 0.7016), (-0.3508, 1.0524), (-0.7016, 0.7016), (-0.7016, 1.0524), (-0.7016, 1.4032)],
            "Goal09": [(-0.0, 0.0), (-0.3508, 0.0), (0.0, 0.3508), (-0.7016, 0.0), (-0.0, 0.7016), (-0.0, 1.0524), (-0.0, 1.4032), (-0.3508, 1.4032), (-0.7016, 1.4032)],
        }
        return GOAL_SHAPES

    def perform_inference(self, input_dat):

        # Array of Arrays 9 bolcks with x,y,rot
        self.inference_class.blocks = input_dat
        self.last_red_pos = (self.inference_class.blocks[0]['x'], self.inference_class.blocks[0]['y'], self.inference_class.blocks[0]['theta'])
        goal_suggested = self.inference_class.make_pomcp_suggestion()
        if goal_suggested:
            print ("Goal Suggested")
            goal_name = self.inference_class.suggested_goal_name
            suggested_move = self.inference_class.suggested_move
            if suggested_move:
                target_name = suggested_move['target_idx']
                target_name = f"G{target_name}"
            else:
                target_name = None 
            print(f"Suggested Goal: {goal_name}, Suggested Target: {target_name}")
        else:
            print ("No Goal Suggested")
            goal_name = None
            target_name = None


        # return goal_name Goal09, target_name G1, satisfied_targets List[G0, G1, G2, ...], placed_blocks ["AnchorCube", "Cube01", ...]
        pass

    def process_user_reply(self, reply):
        pass


