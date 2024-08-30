"""
 Explainable Robotics Lab, Lehigh University
 See license.txt file for license information.
 @author: Gustavo Cardona, Cristian-Ioan Vasile
"""

from lomap import Ts
from gurobipy import GRB
from route_planning_exploration_node_only import route_planning
from utility_resources import *
import copy

ts_filename='10_by_15_grid_new.yaml'
'''Simple example of planning with resource constraints.'''
# setup_logging()
ts = Ts.load(ts_filename)
agents = [('q1', {'a'}), ('q1', {'a'}), ('q1', {'a'}), ('q136', {'b'}), ('q136', {'b'}), ('q136', {'b'}), ('q15', {'c'}), ('q15', {'c'}), ('q15', {'c'})
          ]

specification = ('F[0, 30] T(2, red, {(a, 1), (b, 1)}, {(r1, 25), (r1, 15)})')
specification += ' && F[0, 30] T(1, yellow, {(c, 2), (a, 1), (b, 1)}, {(r2, 15), (r2, 25)})'
specification += ' && F[0, 30] T(1, green, {(c, 1), (a, 1), (b, 1)}, {(r2, 10), (r2, 10)})'

type_resources = 2
min_re = {'r1': 2, 'r2': 2}
max_re = {'r1': 15, 'r2': 15}

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
    mean = (min_value + max_value) / 2
    sd = (max_value - min_value) / 6
    for i, (u, _) in enumerate(ts.g.nodes(data=True)):
        belief_resources[resource_key][u] = 0
        if u in non_zero[resource_key]:
            resources[resource_key][u] = non_zero[resource_key][u]
        else:
            resources[resource_key][u] = 0
actual_resources = copy.deepcopy(belief_resources) # actual resources for optimization


alpha = 1  # exploration weight
decay_factor = 0.8
prob_map = {} # weight for each node, value between 0 and 1, 0 means not worth to visit (high confidence)
true_prob_map = {}
variance_map = {}
for u, _ in ts.g.nodes(data=True):
    prob_map[u] = 1.0 / float(ts.g.number_of_nodes())
    true_prob_map[u] = 0.0 # setting 0 to all nodes for optimal run (confident for all nodes)
    variance_map[u] = {}
    for re in range(type_resources):
        variance_map[u]['r{}'.format(re + 1)] = 3.0
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
num_itr = 6
his_explore = []
his_itr = []
his_satis = []
his_avg_error = []
his_variance_map = [copy.deepcopy(variance_map)]
his_prob_map = [copy.deepcopy(prob_map)] # normalized variance
history_belief_resources = [copy.deepcopy(belief_resources)]

for i in range(num_itr + 1):
    tic()
    if i == num_itr:
        print ('--------------------Start Optimal Solution--------------------')
        m = route_planning(ts, agents, specification, resources, true_prob_map, 0,
                           storage_type=storage_type,
                           capacities=capacities, resource_distribution=resources,
                           resource_type='divisible', travel_time_weight=0,
                           resources_weight=0, transportation=True,
                           partial_satis=True)
    else:
        print ('--------------------Start Iteration {}--------------------'.format(i))
        actual_resources = computed_resources(belief_resources, resources, actual_resources, max_re)
        m = route_planning(ts, agents, specification, actual_resources, prob_map, alpha,
                           storage_type=storage_type,
                           capacities=capacities, resource_distribution=belief_resources,
                           resource_type='divisible', travel_time_weight=0,
                           resources_weight=0, transportation=True,
                           partial_satis=True)
        alpha *= decay_factor
        variables1 = m.getVars()
        trajs, _ = update_agent_traj(variables1)
        belief_resources, variance_map, prob_map = update_resource_list(trajs, belief_resources, resources,
                                                                             variance_map, max_re, ts.g)
        his_prob_map.append(copy.deepcopy(prob_map))
        his_avg_error.append(average_error(ts.g.nodes(), resources, belief_resources))
        history_belief_resources.append(copy.deepcopy(belief_resources))
        his_variance_map.append(copy.deepcopy(variance_map))
    toc()
    n_objectives = m.NumObj
    for o in range(n_objectives):
        m.params.ObjNumber = o
        if m.status != GRB.Status.OPTIMAL:
            print('Model is infeasible')
        else:
            if m.ObjNName == 'satisfaction_percentage':
                his_satis.append(m.ObjNVal)
                his_itr.append(i)
            elif m.ObjNName == 'exploration':
                his_explore.append(m.ObjNVal)
    if i == num_itr:
        print ('Optimal solution: satisfaction_percentage:{}'.format(his_satis[i]))
    else:
        print ('Itr:{} exploration {}, satisfaction_percentage:{}'.format(i,
                                                                                              his_explore[i],
                                                                                              his_satis[i]))

plt.figure(2, figsize=(10, 4))
plt.subplot(1, 2, 1)
plt.plot(his_itr[:-1], his_satis[:-1], marker='o')
plt.axhline(his_satis[-1], color='r', linestyle='--')
plt.title('satisfaction_percentage')

plt.subplot(1, 2, 2)
num_error_types = len(his_avg_error[0])
for i in range(num_error_types):
    plt.plot(his_itr[:-1], [errors['r' + str(i+1)] for errors in his_avg_error], marker='o', label='resource {} error'.format(i + 1))
plt.xlabel('Iteration')
plt.xticks(range(min(his_itr[:-1]), max(his_itr[:-1]) + 1, 1))
plt.ylabel('Average Resources Error')
plt.legend()

plt.tight_layout()
plt.show()