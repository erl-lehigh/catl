"""
 Explainable Robotics Lab, Lehigh University
 See license.txt file for license information.
 @author: Gustavo Cardona, Cristian-Ioan Vasile
"""

from gurobipy import GRB
import numpy as np
from route_planning_exploration import route_planning
from utility_nodes import *
import copy
from utility_edges import *

def update_agent_traj(variables, agent_classes):
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
                            if location not in traj[capability]:
                                traj[capability].append(location)

    return traj, full_traj

# ts_filename='10_by_15_grid_new.yaml'
ts_filename='10_by_15_grid_new.yaml'
'''Simple example of planning with resource constraints.'''
# setup_logging()
ts = Ts.load(ts_filename)

real_graph = copy.deepcopy(ts)
agents = [('q1', {'a'}), ('q1', {'a'}), ('q1', {'a'}), ('q136', {'b'}), ('q136', {'b'}), ('q136', {'b'}), ('q15', {'c'}), ('q15', {'c'}), ('q15', {'c'})
          ]

specification = ('F[0, 30] T(2, red, {(a, 1), (b, 1)}, {(r1, 25), (r1, 15)})')
specification += ' && F[0, 30] T(1, yellow, {(c, 2), (a, 1), (b, 1)}, {(r2, 15), (r2, 25)})'
specification += ' && F[0, 30] T(1, green, {(c, 1), (a, 1), (b, 1)}, {(r2, 10), (r2, 10)})'

type_resources = 2
min_re = {'r1': 2, 'r2': 2}
max_re = {'r1': 15, 'r2': 15}
# for re in range(type_resources):
weights = generate_normalized_inverse_weights(max_re) # normalize weights
resources = {}
belief_resources = {}

non_zero = {
    'r1': {'q9': 10, 'q76': 5, 'q114': 5, 'q73': 15, 'q144': 5, 'q75': 5, 'q140': 5},
    'r2': {'q10': 5, 'q145': 10, 'q125': 10, 'q80': 15, 'q73': 5, 'q53': 5}
}


for re in range(type_resources):
    resource_key = 'r{}'.format(re + 1)
    resources[resource_key] = {}
    belief_resources[resource_key] = {}
    min_value = min_re[resource_key]
    max_value = max_re[resource_key]
    # Calculate mean and standard deviation for the truncated normal distribution
    mean = (min_value + max_value) / 2
    sd = (max_value - min_value) / 6
    for i, (u, _) in enumerate(ts.g.nodes(data=True)):
        # Assign the generated value to the resource
        # Initialize belief to 0
        belief_resources[resource_key][u] = 0
        if u in non_zero[resource_key]:
            resources[resource_key][u] = non_zero[resource_key][u]
        else:
            resources[resource_key][u] = 0


alpha = 1  # exploration weight
decay_factor = 0.7
prob_map = {}
true_prob_map = {}
variance_map = {}
prior_map = {}
for u, _ in ts.g.nodes(data=True):
    prob_map[u] = 1.0 / float(ts.g.number_of_nodes())
    true_prob_map[u] = 0.0
    variance_map[u] = {}
    for re in range(type_resources):
        variance_map[u]['r{}'.format(re + 1)] = 3.0

# edge update
variance_edge = {}
maxiumum_weight = 5
weights_comb = np.arange(1, maxiumum_weight + 1)
prob_map_edge = {}
num_edges_exclude_self = 0
for u, v in ts.g.edges():
    if u != v:
        ts.g.edge[u][v]['weight'] = 2 # initial guess for each edge
        variance_edge[(u, v)] = real_graph.g.edge[u][v]['weight'] # variance proprotional to the actual weight
        prior_map[(u,v)] = np.ones_like(weights_comb) / len(weights_comb) # uncertainty of the edge weight
        num_edges_exclude_self += 1

for u, v in ts.g.edges():
    if u != v:
        prob_map_edge[(u, v)] = 1.0 / num_edges_exclude_self

storage_type = 'uniform' # choices: comparmental, uniform
capacities = {
    frozenset({'a'}): 50,
    frozenset({'b'}): 50,
    frozenset({'c'}): 50
}

initial_locations, capabilities = zip(*agents)
agent_classes = set(map(frozenset, capabilities))
initial_locations = set(initial_locations)
capabilities = set.union(*capabilities)

# make sure all agents' initial states are in the TS
for state, _ in agents:
    assert state in ts.g, 'State "{}" not in TS!'.format(state)



visit = []
# ''' Run optimization iteratively'''
num_itr = 20
his_reward = []
his_explore = []
his_explore_edge = []
his_itr = []
his_satis = []
his_avg_error = []
his_prob_map = [copy.deepcopy(prob_map)]
full_trajs = []
history_belief_resources = [copy.deepcopy(belief_resources)]
his_variance_map = [copy.deepcopy(variance_map)]
his_avg_edge_var = [np.mean(list(variance_edge.values()))]
visited_edge = []


for i in range(num_itr + 1):
    if i == num_itr:
        print ('--------------------Start Optimal Solution--------------------')
        m = route_planning(real_graph, agents, specification, resources, true_prob_map, 0, weights, prob_map_edge,
                           storage_type=storage_type,
                           capacities=capacities, resource_distribution=resources,
                           resource_type='divisible', travel_time_weight=0,
                           resources_weight=0, transportation=True,
                           partial_satis=True)
        variables1 = m.getVars()
        trajs, temp_full = update_agent_traj(variables1, agent_classes)
    else:
        print ('--------------------Start Iteration {}--------------------'.format(i))
        m = route_planning(ts, agents, specification, belief_resources, prob_map, alpha, weights, prob_map_edge,
                           storage_type=storage_type,
                           capacities=capacities, resource_distribution=belief_resources,
                           resource_type='divisible', travel_time_weight=0,
                           resources_weight=0, transportation=True,
                           partial_satis=True)
        alpha *= decay_factor
        variables1 = m.getVars()
        trajs, temp_full = update_agent_traj(variables1, agent_classes)
        full_trajs.append(temp_full)
        trajs_ordered = sort_trajectories_by_time([temp_full])
        edges = find_visited_edges_with_counts(ts.g, trajs_ordered)
        variance_edge, prob_map_edge, prior_map = update_edge_weight_list(edges, ts.g, variance_edge, real_graph.g, weights_comb, prior_map, visited_edge)

        belief_resources, variance_map, prob_map = update_reward_list(trajs, belief_resources, resources,
                                                                             variance_map, max_re, ts.g)
        visit, num_visit = update_visit_list(trajs, visit)
        his_prob_map.append(copy.deepcopy(prob_map))
        # his_avg_error.append(average_error(ts.g.nodes(), resources, belief_resources))
        history_belief_resources.append(copy.deepcopy(belief_resources))
        his_variance_map.append(copy.deepcopy(variance_map))

        temp_edge = []
        for key, value in variance_edge.items():
            if key in visited_edge:
                temp_edge.append(value)
        his_avg_edge_var.append(np.mean(temp_edge))
        print('Average edge variance:', np.mean(temp_edge))
    obj = 0
    n_objectives = m.NumObj
    for o in range(n_objectives):
        m.params.ObjNumber = o
        if m.status != GRB.Status.OPTIMAL:
            print('Model is infeasible')
        else:
            if m.ObjNName == 'satisfaction_percentage':
                obj += m.ObjNVal
                # print 'travel_time:', m.ObjNVal
                his_satis.append(m.ObjNVal)
                his_itr.append(i)
            elif m.ObjNName == 'reward':
                obj += m.ObjNVal
                his_reward.append(m.ObjNVal * ts.g.number_of_nodes() * 10)
                # print 'reward',  m.ObjNVal
            elif m.ObjNName == 'exploration':
                obj += m.ObjNVal
                his_explore.append(m.ObjNVal)
            elif m.ObjNName == 'exploration_edge':
                obj += m.ObjNVal
                his_explore_edge.append(m.ObjNVal)
    if i == num_itr:
        print ('Optimal solution: satisfaction_percentage:{}'.format(his_satis[i]))
    else:
        print ('Itr:{} node exp {}, edge exp {}, satisfaction_percentage:{}'.format(i,
                                                                                              his_explore[i],
                                                                                              his_explore_edge[i],
                                                                                              his_satis[i]))

import pickle
with open('history_belief_resources.pkl', 'wb') as f:
    pickle.dump(history_belief_resources, f)
with open('his_variance_map.pkl', 'wb') as f:
    pickle.dump(his_variance_map, f)
with open('full_trajs.pkl', 'wb') as f:
    pickle.dump(full_trajs, f)

plt.figure(2, figsize=(10, 4))
plt.subplot(1, 3, 1)
plt.plot(his_itr[:-1], his_satis[:-1], marker='o')
plt.axhline(his_satis[-1], color='r', linestyle='--')
plt.title('satisfaction_percentage')

plt.subplot(1, 3, 2)
plt.plot(his_avg_edge_var, marker='o')
plt.title('avg edge variance')

plt.subplot(1, 3, 3)
num_error_types = len(his_avg_error[0])
for i in range(num_error_types):
    plt.plot(his_itr[:-1], [errors['r' + str(i+1)] for errors in his_avg_error], marker='o', label='resource {} error'.format(i + 1))
plt.xlabel('Iteration')
plt.xticks(range(min(his_itr[:-1]), max(his_itr[:-1]) + 1, 1))
plt.ylabel('Average Resources Error')
plt.legend()

plt.tight_layout()
plt.show()
