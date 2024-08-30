import re
import time
import numpy as np
import random
import matplotlib.pyplot as plt
from scipy.stats import truncnorm


def update_agent_distribution(variables, specification1, agents1):
    traj = {}
    pattern = r'F\[0,\s*(\d+)\]'
    # Search for the pattern in the string
    match = re.search(pattern, specification1)
    time_bound = int(match.group(1))
    type_robot = len(agents1)
    for t in range(type_robot):
        traj[str(t + 1)] = []

    for var in variables:
        var_name = var.varName
        if var_name.startswith('z_'):
            parts = var_name.split('_')
            if len(parts) == 4:
                _, location, capability, t = parts
                t = int(t)
                value = var.x
                if int(value) != 0:
                    if location not in traj[capability]:
                        traj[capability].append(location)
    return traj


def update_visit_traj(variables, type_robots):
    traj = {}
    for g in range(type_robots):
        traj[str(g + 1)] = set()
    for var in variables:
        var_name = var.varName
        if var_name.startswith('z_'):
            parts = var_name.split('_')
            if len(parts) == 4:
                _, location, capability, _ = parts
                value = var.x
                if int(value) != 0:
                    traj[capability].add(location)
    return traj


def update_visit_list(traj, visit):
    for key, item in traj.items():
        for lo in item:
            if lo not in visit:
                visit.append(lo)
    return visit, len(visit)


def update_agent_traj(variables):
    traj = {}
    full_traj = {}
    for var in variables:
        var_name = var.varName
        if var_name.startswith('z_'):
            parts = var_name.split('_')
            if len(parts) == 4:
                _, location, capability, t = parts
                if location.startswith('q'):
                    value = var.x
                    if int(value) != 0:
                        if capability not in traj:
                            traj[capability] = []
                            full_traj[capability] = []
                            full_traj[capability].append([location, t, value])
                            traj[capability].append(location)
                        else:
                            full_traj[capability].append([location, t, value])
                            if location not in traj[capability]: # depend if you allow mutiple update per iteration
                                traj[capability].append(location)

    return traj, full_traj


def computed_resources(belief_resource, ground_resource, actual_resources, max_resource):
    resources = list(max_resource.keys())
    for resource in resources:
        for location in belief_resource[resource]:
            ub = min(belief_resource[resource][location], ground_resource[resource][location])
            lb = actual_resources[resource][location]
            actual_resources[resource][location] = np.random.uniform(lb, ub)
    return actual_resources


def tic():
    # Homemade version of matlab tic and toc functions
    global startTime_for_tictoc
    startTime_for_tictoc = time.time()


def toc():
    if 'startTime_for_tictoc' in globals():
        print("Elapsed time is " + str(time.time() - startTime_for_tictoc) + " seconds.")
    else:
        print("Toc: start time not set")


def weight_generation(variance_map):
    """
    Generate weights based on variances.
    such that high variance corresponding to higher weight for exploration.
    """
    weights = {}
    total_variance = sum(sum(resource_vars.values()) for resource_vars in variance_map.values())

    for location, resource_vars in variance_map.items():
        weights[location] = sum(resource_vars.values()) / total_variance

    return weights


def kalman_filter_1d(z, x, P, R, Q=0):
    # Update
    x_pred = x
    P_pred = P + Q
    # Predict
    K = P_pred / (P_pred + R)
    x = x_pred + K * (z - x_pred)
    P = (1 - K) * P_pred
    return x, P


def update_resource_list(traj, belief_resource, ground_resource, variance_map, max_resource, graph):
    '''
    Update beliefs and variances for resources based on trajectories and observations.

    :param traj: Dictionary of paths for each robot capability
    :param belief_resource: Dictionary of dictionaries, e.g., {'r1': {'q1': 0, 'q2': 0}, 'r2': {'q1': 0, 'q2': 0}}
    :param ground_resource: Similar structure to belief_resource, but with true resource values
    :param variance_map: Dictionary of dictionaries, e.g., {'q1': {'r1': 5.0, 'r2': 5.0}, 'q2': {'r1': 5.0, 'r2': 5.0}}
    :param max_resource: Dictionary of maximum resources for each resource, e.g., {'r1': 2, 'r2': 20}
    :param graph: Graph representation of the environment
    :return: Updated belief_resource, variance_map, and prob map
    '''
    std1, std2 = calculate_std_values(max_resource)
    resources = list(max_resource.keys())

    for cap, path in traj.items():
        if cap == '2':
            check_lo = set()
            for location in path:
                neighbors = set(graph.neighbors(location)) | {location}
                # neighbors = {location}
                for c_lo in neighbors - check_lo:
                    check_lo.add(c_lo)
                    for resource in resources:
                        R = std2[resource] ** 2
                        z = np.random.normal(ground_resource[resource][c_lo], std2[resource])
                        if ground_resource[resource][c_lo] == 0:
                            z = 0
                        belief_resource[resource][c_lo], variance_map[c_lo][resource] = kalman_filter_1d(
                            z, belief_resource[resource][c_lo], variance_map[c_lo][resource], R
                        )
        else:
            for location in path:
                for resource in resources:
                    R = std1[resource] ** 2
                    z = np.random.normal(ground_resource[resource][location], std1[resource])
                    if ground_resource[resource][location] == 0:
                        z = 0
                    belief_resource[resource][location], variance_map[location][resource] = kalman_filter_1d(
                        z, belief_resource[resource][location], variance_map[location][resource], R
                    )

    prob_map = weight_generation(variance_map)
    return belief_resource, variance_map, prob_map


def calculate_std_values(max_resource, baseline_std1=0.5, baseline_std2=0.8, baseline_resource=5):
    """
    Calculate std values for each sensor based on max_resource.

    :param max_resource: Dictionary of maximum resources for each resource
                       e.g., {'r1': 2, 'r2': 20}
    :param baseline_std1: Baseline std for sensor 1 (default 0.5)
    :param baseline_std2: Baseline std for sensor 2 (default 0.8)
    :param baseline_resource: Baseline resource value (default 5)
    :return: Two dictionaries of std values, one for each sensor
             e.g., ({'r1': 0.12, 'r2': 1.2}, {'r1': 0.32, 'r2': 3.2})
    """
    std1 = {resource: baseline_std1 * (max_val / baseline_resource)
            for resource, max_val in max_resource.items()}
    std2 = {resource: baseline_std2 * (max_val / baseline_resource)
            for resource, max_val in max_resource.items()}
    return std1, std2


def average_error(visit, ground_truth, belief_resource):
    """
    Calculate the average error between ground truth and belief resource for each resource,
    considering only locations with non-zero ground truth values.

    :param visit: List of visited locations
    :param ground_truth: Dictionary of ground truth values, e.g., {'r1': {'q1': 5, 'q2': 3}, 'r2': {'q1': 2, 'q2': 4}}
    :param belief_resource: Dictionary of belief resource values, similar structure to ground_truth
    :return: Dictionary of average errors for each resource
    """
    error = {resource: 0 for resource in ground_truth}
    count = {resource: 0 for resource in ground_truth}

    for location in visit:
        for resource in ground_truth:
            if (location in ground_truth[resource] and
                    location in belief_resource[resource] and
                    ground_truth[resource][location] != 0):
                error[resource] += abs(ground_truth[resource][location] - belief_resource[resource][location])
                count[resource] += 1

    avg_error = {resource: error[resource] / count[resource] if count[resource] > 0 else 0
                 for resource in error}

    return avg_error


def generate_normalized_inverse_weights(max_resource):
    """
    Generate weights inversely proportional to max resources and normalize them.

    :param max_resource: Dictionary of maximum resources for each resource
                       e.g., {'r1': 5, 'r2': 10, 'r3': 20, 'r4': 15}
    :return: Dictionary of normalized weights that sum to 1
             e.g., {'r1': 0.48, 'r2': 0.24, 'r3': 0.12, 'r4': 0.16}
    """
    # Calculate inverse of max resources
    inverse_resources = {resource: 1.0 / resource for resource, resource in max_resource.items()}

    # Calculate total of inverse resources
    total_inverse = sum(inverse_resources.values())

    # Normalize the weights
    normalized_weights = {resource: inv / total_inverse for resource, inv in inverse_resources.items()}

    return normalized_weights
