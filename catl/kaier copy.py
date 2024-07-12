"""
 Explainable Robotics Lab, Lehigh University
 See license.txt file for license information.
 @author: Gustavo Cardona, Cristian-Ioan Vasile
"""

import sys
import logging
import time
from lomap import Ts
from gurobipy import GRB
from route_planning import route_planning
from visualization import show_environment
from check_system_constraints import check_initial_states, check_flow_constraints

def update_agent_traj(variables, agent_classes):
    traj = {}
    for t in range(len(agent_classes)):
        traj[str(t + 1)] = []
    for var in variables:
        var_name = var.varName
        if var_name.startswith('z_'):
            parts = var_name.split('_')
            if len(parts) == 4:
                _, location, capability, t = parts
                if location.startswith('q'):
                    value = var.x
                    if int(value) != 0:
                        if location not in traj[capability]:
                            traj[capability].append([location, t, value])
    return traj


def setup_logging(logfile='uni_div.log', loglevel=logging.DEBUG,
                 fs='%(asctime)s | %(name)s | %(levelname)s | {%(message)s}',
                 dfs='%m/%d/%Y %I:%M:%S %p'):

    if logfile is not None:
        logging.basicConfig(filename=logfile, level=loglevel,
                            format=fs, datefmt=dfs)

    root = logging.getLogger()
    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(loglevel)
    ch.setFormatter(logging.Formatter(fs, dfs))
    root.addHandler(ch)

ts_filename='5_by_5_grid.yaml'
'''Simple example of planning with resource constraints.'''
setup_logging()
ts = Ts.load(ts_filename)
# for u, v in ts.g.edges():
#     assert ts.g.has_edge(v, u)
# show_environment(ts)
agents = [('q1', {'a'}), ('q1', {'b'}),('q1', {'a'}), ('q1', {'b'})
          ]

resources = {'r1': {'q1': 10},
             'r2': {'q1': 10},
            }

storage_type = 'uniform' # choices: comparmental, uniform
capacities = {
    frozenset({'a'}): 5,
    frozenset({'b'}): 5
}

initial_locations, capabilities = zip(*agents)
agent_classes = set(map(frozenset, capabilities))
initial_locations = set(initial_locations)
capabilities = set.union(*capabilities)

# make sure all agents' initial states are in the TS
for state, _ in agents:
    assert state in ts.g, 'State "{}" not in TS!'.format(state)

specification = ('F[0, 8] T(2, yellow, {(a, 1), (b, 1)}, {(r1, 1.4), (r2, 1.4)})')
# specification = ('F[0, 8] T(2, yellow, {(a, 1), (b, 1)}')
# specification = ('G[8,8] T(2, yellow, {(a, 1), (b, 1)}')
# specification = ('G[8, 8] T(2, yellow, {(a, 1), (b, 1)}, {(r1, 1.4), (r2, 1.4)})')
# specification += ' && F[0, 2] T(2, cyan, {(a, 2)})'
start = time.time()
m = route_planning(ts, agents, specification, storage_type=storage_type,
                   capacities=capacities, resource_distribution=resources,
                   resource_type='divisible', travel_time_weight=0.1, 
                   resources_weight=0.1, transportation=True, 
                   partial_satis=False)

variables1 = m.getVars()
traj = update_agent_traj(variables1, agent_classes)
print('traj:', traj, type(traj))
n_objectives = m.NumObj
for o in range(n_objectives):
    # Set which objective we will query
    m.params.ObjNumber = o
    # Query the o-th objective value
    if m.status == GRB.Status.INFEASIBLE:
        print('Model is infeasible')
    else:
        print('Objectives:', m.ObjNName, ':', m.ObjNVal)
print(specification)
