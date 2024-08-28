import networkx as nx
from lomap import Ts
import networkx as nx
from collections import defaultdict
import networkx as nx
from collections import defaultdict


def find_visited_edges_with_counts(G, sorted_trajs):
    visited_edges = {}

    for capability, traj_list in sorted_trajs[0].items():
        visited_edges[capability] = defaultdict(int)

        for i in range(1, len(traj_list)):
            current_node, current_time, current_agents = traj_list[i]
            current_agents = float(current_agents)  # Convert to float

            # Start from the immediately previous node
            for j in range(i - 1, -1, -1):
                prev_node, prev_time, prev_agents = traj_list[j]
                prev_agents = float(prev_agents)  # Convert to float

                if G.has_edge(prev_node, current_node):
                    # Found a valid edge
                    edge = (prev_node, current_node)  # Keep the direction from prev to current

                    # Calculate the number of agents that moved along this edge
                    agents_moved = min(prev_agents, current_agents)

                    # Add the count to the edge
                    visited_edges[capability][edge] += agents_moved
                    break  # Stop searching for this edge

                # If we've gone too far back in time, stop searching
                if int(current_time) - int(prev_time) > 1:
                    break

    return visited_edges


def sort_trajectories_by_time(trajs):
    sorted_trajs = {}

    for capability, traj_list in trajs[0].items():
        # Sort the trajectory list by the second element (time) of each sublist
        sorted_traj = sorted(traj_list, key=lambda x: int(x[1]))
        sorted_trajs[capability] = sorted_traj

    return [sorted_trajs]


# edges = record_visited_edges(ts.g, trajs)

import numpy as np

def weight_generation(variance_map):
    """
    Generate weights based on variances.
    This function is adapted to work with the edge-based variance_map structure.
    """
    weights = {}
    total_variance = sum(var for var in variance_map.values())
    for edge, var in variance_map.items():
        weights[edge] = var / total_variance
    return weights


def update_edge_weight_list(traj, belief_graph, variance_edge, graph, weights_comb, prior_map, visited_edge):
    '''
    Update beliefs and variances for rewards based on trajectories and observations.

    :param traj: Dictionary of paths for each robot capability
    :param belief_reward: Dictionary of dictionaries, e.g., {'r1': {'q1': 0, 'q2': 0}, 'r2': {'q1': 0, 'q2': 0}}
    :param ground_reward: Similar structure to belief_reward, but with true reward values
    :param variance_map: Dictionary of dictionaries, e.g., {'q1': {'r1': 5.0, 'r2': 5.0}, 'q2': {'r1': 5.0, 'r2': 5.0}}
    :param max_reward: Dictionary of maximum rewards for each resource, e.g., {'r1': 2, 'r2': 20}
    :param graph: Graph representation of the environment
    :return: Updated belief_reward, variance_map, and probability map
    '''
    for cap, path in traj.items():
        if cap == '2':
            for edge in path.keys():
                u, v = edge
                if u != v:
                    if edge not in visited_edge:
                        visited_edge.append(edge)
                    prior = prior_map[edge]
                    observation = generate_observation(graph.edge[u][v]['weight'])
                    lik = likelihood(observation, weights_comb)
                    posterior = bayesian_update(prior, lik)
                    prior_map[edge] = posterior
                    mean_weight = np.sum(weights_comb * prior)
                    variance = np.sum((weights_comb - mean_weight) ** 2 * prior)
                    belief_graph.edge[u][v]['weight'] = weights_comb[np.argmax(posterior)]
                    variance_edge[edge] = variance
        else:
            for edge in path.keys():
                u, v = edge
                if u != v:
                    if edge not in visited_edge:
                        visited_edge.append(edge)
                    prior = prior_map[edge]
                    observation = generate_observation(graph.edge[u][v]['weight'])
                    lik = likelihood(observation, weights_comb)
                    posterior = bayesian_update(prior, lik)
                    prior_map[edge] = posterior
                    mean_weight = np.sum(weights_comb * prior)
                    # print((weights_comb - mean_weight))
                    variance = np.sum((weights_comb - mean_weight) ** 2 * prior)
                    belief_graph.edge[u][v]['weight'] = weights_comb[np.argmax(posterior)]
                    # graph.edge[u][v]['weight'] = prior_map[edge]
                    variance_edge[edge] = variance

    prob_map = weight_generation(variance_edge)
    return variance_edge, prob_map, prior_map


def generate_observation(true_weight, std=0.5):
    while True:
        observation = true_weight + np.random.normal(0, std)
        if observation >= 1:
            return observation


def likelihood(observation, weights):
    sigma = 0.5 # Assume some standard deviation for the noise
    return np.exp(-0.5 * (observation - weights) ** 2 / sigma ** 2) / (sigma * np.sqrt(2 * np.pi))


# Bayesian update function
def bayesian_update(prior, likelihood):
    posterior = likelihood * prior
    posterior /= np.sum(posterior)  # Normalize to sum to 1
    return posterior
