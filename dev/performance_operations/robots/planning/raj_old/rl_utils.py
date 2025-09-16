import numpy as np
import pickle
from etils import epath

def load_params(path: str):
    with epath.Path(path).open('rb') as fin:
        buf = fin.read()
    return pickle.loads(buf)

def save_params(path: str, params):
    """Saves parameters in flax format."""
    with epath.Path(path).open('wb') as fout:
        fout.write(pickle.dumps(params))

def truncated_geometric_sample(batch_size, max_steps, gamma=0.99):
    """Vectorized sampling of future timesteps using a truncated geometric distribution."""
    offsets = np.random.geometric(p=1 - gamma, size=batch_size)
    return np.minimum(offsets, max_steps)

class ReplayBuffer:
    def __init__(self, max_capacity, episode_len, obs_dim, act_dim, goalstart, goalend):
        self.max_capacity = max_capacity
        self.episode_len = episode_len
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.goalstart = goalstart
        self.goalend = goalend
        self.observations = np.zeros((max_capacity, episode_len, obs_dim), dtype=np.float32)
        self.actions = np.zeros((max_capacity, episode_len, act_dim), dtype=np.float32)
        
        self.episode_ptr = 0
        self.timestep_ptr = 0
        self.full = False

    def add(self, obs, act):
        """
        Add a single transition (observation, action) at the current timestep within an episode.
        Automatically switches to the next episode once episode_len transitions are added.
        """

        self.observations[self.episode_ptr, self.timestep_ptr] = obs
        self.actions[self.episode_ptr, self.timestep_ptr] = act
        
        self.timestep_ptr += 1
        
        if self.timestep_ptr == self.episode_len:
            self.timestep_ptr = 0
            self.episode_ptr = (self.episode_ptr + 1) % self.max_capacity
            self.full = self.full or self.episode_ptr == 0

    def sample(self, batch_size, gamma=0.99):
        """Sample (s, a, s') tuples with s' drawn from a truncated geometric distribution."""
        num_episodes = len(self)
        episode_idxs = np.random.randint(0, num_episodes, size=batch_size)
        timestep_idxs = np.random.randint(0, self.episode_len - 1, size=batch_size)
        
        offsets = truncated_geometric_sample(batch_size, self.episode_len - 1 - timestep_idxs, gamma)
        future_timestep_idxs = np.minimum(timestep_idxs + offsets, self.episode_len - 1)
        
        obs = self.observations[episode_idxs, timestep_idxs]
        acts = self.actions[episode_idxs, timestep_idxs]
        future_goal = self.observations[episode_idxs, future_timestep_idxs, self.goalstart: self.goalend]
        
        return obs, acts, future_goal
    
    def __len__(self):
        return self.max_capacity if self.full else self.episode_ptr