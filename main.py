import gymnasium as gym
import numpy as np
from scipy.optimize import minimize
from gymnasium.wrappers import RecordVideo

def initStateAndControlMatrices():
    A = np.eye(4)  # State transformational matrix
    B = np.array(
        [[0],
         [1],
         [-1],
         [-0.5]]
    )
    return A, B


def computeCost(U, A, B, observation, target_state):
    cost = 0  # Init cost to 0
    for i in range(N):
        action = U[i].reshape(1,-1)
        C = (A @ observation) + np.hstack(B @ action)  # Output matrix C
        state_error = C - target_state  # Difference in state error
        Jz = state_error.T @ Q @ state_error  # State error weighting
        Ju = action.T @ R @ action  # Control error weighting
        cost += Jz + Ju  # Total cost to minimize
    print("Total state error: ", cost)
    return cost

def initCostMatrices():
    Q = np.eye(4)  # Penalize state deviation
    Q[0,0] = 1500 # Cart position
    Q[1,1] = 0 # Cart velocity
    Q[2,2] = 1000 # Pole Angle
    Q[3,3] = 1000 # Pole Angular Velocity
    R = np.array([[0.1]])  # Penalize large control inputs
    return Q,R

# Uncomment if you want to save the video
# env = gym.make("CartPole-v1", render_mode="rgb_array")
# env = RecordVideo(env, video_folder="videos", episode_trigger=lambda e: e == 0)

env = gym.make("CartPole-v1", render_mode="human")


observation, info = env.reset()

A, B = initStateAndControlMatrices()  # Init State and control matrices
Q, R = initCostMatrices()

episode_over = False

N = 10  # Prediction Horizon

# Target state, Cart position, Cart velocity, Pole Angle, and Pole Angular Velocity
target_state = np.array([0, 0, 0, 0])


iteration_count = 0
while not episode_over:
    action = env.action_space.sample()
    U0 = np.zeros(N)
    U0[0] = action
    result = minimize(computeCost, U0, args=(A, B, observation, target_state), bounds=[(-1, 1)] * N)
    optimal_action = 1 if result.x[0] > 0 else 0  # Convert to a discrete value
    print("Action: ", optimal_action)
    observation, reward, terminated, truncated, info = env.step(optimal_action)
    episode_over = terminated or truncated
    print("Iter: ", iteration_count)
    iteration_count += 1

env.close()
