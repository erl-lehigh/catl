'''
 Copyright (C) 2020 Cristian Ioan Vasile <cvasile@lehigh.edu>
 Explainable Robotics Lab (ERL), Autonomous and Intelligent Robotics (AIR) Lab,
 Lehigh University
 See license.txt file for license information.
'''

import sys
import logging
import time
from lomap import Ts
from gurobipy import GRB
from route_planning import route_planning
from visualization import show_environment
from check_system_constraints import check_initial_states, check_flow_constraints


def setup_logging(logfile='comp_indiv.log', loglevel=logging.DEBUG,
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


def case_simple(ts_filename='construction.yaml'):
    '''Simple example of planning with resource constraints.'''
    setup_logging()

    ts = Ts.load(ts_filename)
    for u, v in ts.g.edges():
        assert ts.g.has_edge(v, u)
    show_environment(ts)

    for u in ts.g:
        logging.debug('State: %s, Data: %s', u, str(ts.g.node[u]))

    # agents = [('q1', {'a','b'}), ('q1', {'e','f'}), ('q1', {'c','d'}), 
    #           ('q1', {'e','f'}), ('q2', {'c','d'}), ('q2', {'c','d'}),
    #           ('q2', {'a','c'}), ('q3', {'a','f'}), ('q3', {'a','f'}),
    #           ('q3', {'e','f'}), ('q3', {'a','c'}), ('q4', {'a','f'}),
    #           ('q5', {'a','b'}), ('q5', {'e','f'}), ('q6', {'c','d'}), 
    #           ('q7', {'d','b'}), ('q7', {'d','b'}), ('q8', {'d','b'}),
    #           ('q9', {'e','f'})]
    agents = [('q1', {'a','b'}), ('q1', {'c', 'd'}), ('q1', {'e','f'}),
              ('q1', {'c','d'}), ('q1', {'a', 'f'}), ('q1', {'d','b'}),
              ('q1', {'a','b'}), ('q1', {'a', 'f'}), ('q1', {'d','b'}),
              ('q1', {'e','f'}), ('q1', {'c', 'd'}), ('q1', {'e','f'}),
              ('q1', {'a','c'}), ('q1', {'a', 'c'}), ('q1', {'d','b'}),
              ('q1', {'e','f'}), ('q1', {'c', 'd'}), ('q1', {'e','f'}),
              ]
    resources = {'r1': {'q1': 10, 'q5': 10},
                 'r2': {'q2': 10, 'q6': 10},
                 'r3': {'q3': 10, 'q7': 10},
                 'r4': {'q4': 10, 'q8': 10}
                }

    storage_type='comparmental'
    capacities = {
        frozenset({'a', 'b'}): {'r1': 4, 'r2': 4, 'r3': 4, 'r4': 3},
        frozenset({'c', 'd'}): {'r1': 2, 'r2': 2, 'r3': 2, 'r4': 2},
        frozenset({'e', 'f'}): {'r1': 2, 'r2': 2, 'r3': 2, 'r4': 2},
        frozenset({'a', 'c'}): {'r1': 2, 'r2': 2, 'r3': 2, 'r4': 2},
        frozenset({'d', 'b'}): {'r1': 2, 'r2': 2, 'r3': 2, 'r4': 2},
        frozenset({'a', 'f'}): {'r1': 2, 'r2': 2, 'r3': 2, 'r4': 2}
    }       

    initial_locations, capabilities = zip(*agents)
    agent_classes = set(map(frozenset, capabilities))
    initial_locations = set(initial_locations)
    capabilities = set.union(*capabilities)

    logging.debug('Initial locations: %s', initial_locations)
    logging.debug('Capabilities: %s', capabilities)
    logging.debug('Agent classes: %s', agent_classes)

    # make sure all agents' initial states are in the TS
    for state, _ in agents:
        assert state in ts.g, 'State "{}" not in TS!'.format(state)

    # specification = ('F[0, 5] T(2, green, {(a, 2), (b, 2)}, {(r1, 1), (r2, 1)})')
    # specification += ' && G[20, 24] T(2, red, {(c, 2), (d, 2)}, {(r3, 1), (r2, 1)})'
    # specification += ' && G[10, 14] T(2, yellow, {(e, 2), (f, 2)}, {(r2, 1), (r3, 1)})'
    # specification += ' && F[0, 20] T(2, blue, {(a, 1), (d, 2)}, {(r4, 1), (r3, 1)})'
    # specification += ' && F[0, 25] T(2, purple, {(c, 2), (f, 2)}, {(r1, 1), (r3, 1)})'
    # specification += ' && F[0, 25] T(2, orange, {(a, 2), (b, 2)}, {(r1, 1), (r2, 1)})'
    specification = ('F[0,5] T(1, cyan, {(a, 2), (b, 2)}, {(r1, 1), (r2, 1)})')
    specification += ' && G[20, 22] T(1, gray, {(c, 1), (d, 2)}, {(r3, 1), (r2, 1)})'
    specification += ' && G[10, 12] T(1, yellow, {(e, 2), (f, 1)}, {(r2, 2), (r3, 1)})'
    specification += ' && G[20, 22] T(1, pink, {(e, 2), (f, 1)}, {(r4, 2), (r3, 1)})'
    specification += ' && F[10, 14] T(1, purple, {(a, 2), (f, 1)}, {(r1, 2), (r3, 1)})'
    specification += ' && F[10, 14] T(1, orange, {(d, 3), (b, 2)}, {(r4, 2), (r2, 1)})'

    start = time.time()
    m = route_planning(ts, agents, specification, storage_type=storage_type,
                       capacities=capacities, resource_distribution=resources,
                       resource_type='indivisible', travel_time_weight=0.1, resources_weight=0.1,
                       transportation=True)     
    end = time.time()                  
    time_ = end - start
    time_bound = len(ts.g.nodes(data=True)[0][1]['vars']) - 1

    logging.error('"Planning horizon": %d', time_bound,)

    # check_initial_states(ts, agents)
    # check_flow_constraints(ts, agents, time_bound)

    print('Compartmental-indivisible')
    print('Time needed for Compartmental-indivisible method: ', time_)
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

if __name__ == '__main__':
    case_simple()
