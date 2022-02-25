'''
 Copyright (C) 2020 Cristian Ioan Vasile <cvasile@lehigh.edu>
 Explainable Robotics Lab (ERL), Autonomous and Intelligent Robotics (AIR) Lab,
 Lehigh University
 See license.txt file for license information.
'''
import sys
import logging
import time
import numpy as np
from lomap import Ts
from gurobipy import GRB
from route_planning import route_planning
from visualization import show_environment
from check_system_constraints import check_initial_states, check_flow_constraints


def setup_logging(logfile='methods_comparison.log', loglevel=logging.DEBUG,
                 fs='%(asctime)s | %(name)s | %(levelname)s | %(message)s',
                 dfs='%m/%d/%Y %I:%M:%S %p'):

    if logfile is not None:
        logging.basicConfig(filename=logfile, level=loglevel,
                            format=fs, datefmt=dfs)

    root = logging.getLogger()
    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(loglevel)
    ch.setFormatter(logging.Formatter(fs, dfs))
    root.addHandler(ch)




def baseline_vs_transportation(ts_filename='simple2.yaml', storage_type='uniform'): #storage_type uniform or comparmental

    '''Simple example of planning with resource constraints.'''
    setup_logging()

    ts = Ts.load(ts_filename)
    for u, v in ts.g.edges():
        assert ts.g.has_edge(v, u)
    show_environment(ts)

    for u in ts.g:
        logging.debug('State: %s, Data: %s', u, str(ts.g.node[u]))

    agents = [('q1', {'a','b'}), ('q6', {'c', 'd'}), ('q9', {'e','f'}),
              ('q2', {'a','c'}), ('q3', {'a', 'f'}), ('q7', {'d','b'}),
              ('q5', {'a','b'}), ('q4', {'a', 'f'}), ('q8', {'d','b'}),
              ('q5', {'a','f'}), ('q2', {'c', 'd'}), ('q3', {'e','f'}),
              ('q2', {'a','c'}), ('q3', {'a', 'c'}), ('q7', {'d','b'}),
            #   ('q5', {'a','b'}), ('q1', {'e', 'f'}), ('q8', {'d','b'}),
            #   ('q5', {'a','f'}), ('q4', {'c', 'd'}), ('q3', {'e','f'}),
            #   ('q5', {'a','b'}), ('q6', {'e', 'f'}), ('q6', {'d','b'}),
            #   ('q1', {'d','b'})
            ]

    resources = {'r1': {'q1': 6., 'q5': 6.},
                 'r2': {'q2': 6., 'q6': 6.},
                 'r3': {'q3': 6., 'q7': 6.},
                 'r4': {'q4': 6., 'q8': 6.}
                }

    storage_type_u = 'uniform'
    capacities_u = {
        frozenset({'a', 'b'}): 5,
        frozenset({'c', 'd'}): 8,
        frozenset({'e', 'f'}): 5,
        frozenset({'a', 'c'}): 7,
        frozenset({'d', 'b'}): 6,
        frozenset({'a', 'f'}): 4
        }
    storage_type_c='comparmental'
    capacities_c = {
        frozenset({'a', 'b'}): {'r1': 4, 'r2': 4, 'r3': 4, 'r4': 4},
        frozenset({'c', 'd'}): {'r1': 1, 'r2': 1, 'r3': 1, 'r4': 1},
        frozenset({'e', 'f'}): {'r1': 3, 'r2': 2, 'r3': 1, 'r4': 2},
        frozenset({'a', 'c'}): {'r1': 1, 'r2': 2, 'r3': 3, 'r4': 2},
        frozenset({'d', 'b'}): {'r1': 2, 'r2': 1, 'r3': 1, 'r4': 2},
        frozenset({'a', 'f'}): {'r1': 3, 'r2': 2, 'r3': 1, 'r4': 3}
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

    spec_transportation = ('F[0, 5] T(2, green, {(a, 2), (b, 2)}, {(r1, 1), (r2, 1)})')
    spec_transportation += ' && G[20, 24] T(2, red, {(c, 2), (d, 2)}, {(r3, 1), (r4, 1)})'
    spec_transportation += ' && G[10, 14] T(2, yellow, {(e, 1), (f, 2)}, {(r1, 1), (r3, 1)})'
    spec_transportation += ' && G[8, 12] T(2, pink, {(a, 3), (c, 2)}, {(r4, 1), (r2, 1)})'
    # spec_transportation += (' || F[0, 10] T(2, blue, {(f, 1), (d, 1)}, {(r3, 1)})')
    spec_CaTL = ('F[0, 5] T(2, green, {(a, 1), (b, 1)})')
    spec_CaTL += (' && G[20, 24] T(2, red, {(c, 1), (d, 1)})')
    # agents = agents[0:10]

    # Transportation_blended method multiobjective: Uniform-divisible
    start_uniform_divisible = time.time()
    m_uniform_divisible = route_planning(ts, agents, spec_transportation, storage_type=storage_type_u,
                       capacities=capacities_u, resource_distribution=resources,
                       resource_type='divisible', travel_time_weight=0.2,
                       resources_weight=0.2, transportation=True)
    end_uniform_divisible = time.time()
    time_uniform_divisible = end_uniform_divisible - start_uniform_divisible 
    
    # Transportation_blended method multiobjective: Uniform-indivisible
    start_uniform_indivisible = time.time()
    m_uniform_indivisible = route_planning(ts, agents, spec_transportation, storage_type=storage_type_u,
                       capacities=capacities_u, resource_distribution=resources,
                       resource_type='indivisible', travel_time_weight=0.2,
                       resources_weight=0.2, transportation=True)
    end_uniform_indivisible = time.time()
    time_uniform_indivisible = end_uniform_indivisible - start_uniform_indivisible

    # Transportation_blended method multiobjective: Comparmental-divisible
    start_comp_divisible = time.time()
    m_comp_divisible = route_planning(ts, agents, spec_transportation, storage_type=storage_type_c,
                       capacities=capacities_c, resource_distribution=resources,
                       resource_type='divisible', travel_time_weight=0.2,
                       resources_weight=0.2, transportation=True)
    end_comp_divisible = time.time()
    time_comp_divisible = end_comp_divisible - start_comp_divisible

    # Transportation_blended method multiobjective: Comparmental-indivisible
    start_comp_indivisible = time.time()
    m_comp_indivisible = route_planning(ts, agents, spec_transportation, storage_type=storage_type_c,
                       capacities=capacities_c, resource_distribution=resources,
                       resource_type='indivisible', travel_time_weight=0.2,
                       resources_weight=0.2, transportation=True)
    end_comp_indivisible = time.time()
    time_comp_indivisible = end_comp_indivisible - start_comp_indivisible

    #CaTL-Baseline with time regularization obejective
    start_CaTL = time.time() 
    m_CaTL = route_planning(ts, agents, spec_CaTL, travel_time_weight=0.2)       
    end_CaTL = time.time()
    time_CaTL = end_CaTL - start_CaTL



    # time_bound = len(ts.g.nodes(data=True)[0][1]['vars']) - 1

    # logging.debug('Planning horizon: %d', time_bound)

    # check_initial_states(ts, agents)
    # check_flow_constraints(ts, agents, time_bound)


    # Transportation_blended method multiobjective: Uniform-divisible
    print('Uniform-divisible')
    print('Time needed for Uniform-divisible method: ', time_uniform_divisible)
    n_objectives = m_uniform_divisible.NumObj
    for o in range(n_objectives):
        # Set which objective we will query
        m_uniform_divisible.params.ObjNumber = o
        # Query the o-th objective value
        if m_uniform_divisible.status == GRB.Status.INFEASIBLE:
            print('Model is infeasible')
        else:
            print('Objectives:', m_uniform_divisible.ObjNName, ':', m_uniform_divisible.ObjNVal)

    # Transportation_blended method multiobjective: Uniform-indivisible
    print(' Uniform-indivisible')
    print('Time needed for  Uniform-indivisible method: ', time_uniform_indivisible)
    n_objectives = m_uniform_indivisible.NumObj
    for o in range(n_objectives):
        # Set which objective we will query
        m_uniform_indivisible.params.ObjNumber = o
        # Query the o-th objective value
        if m_uniform_indivisible.status == GRB.Status.INFEASIBLE:
            print('Model is infeasible')
        else:
            print('Objectives:', m_uniform_indivisible.ObjNName, ':', m_uniform_indivisible.ObjNVal)
    

    # Transportation_blended method multiobjective: Comparmental-divisible
    print('Comparmental-divisible')
    print('Time needed for  Comparmental-divisible method: ', time_comp_divisible)
    n_objectives = m_comp_divisible.NumObj
    for o in range(n_objectives):
        # Set which objective we will query
        m_comp_divisible.params.ObjNumber = o
        # Query the o-th objective value
        if m_comp_divisible.status == GRB.Status.INFEASIBLE:
            print('Model is infeasible')
        else:
            print('Objectives:', m_comp_divisible.ObjNName, ':', m_comp_divisible.ObjNVal)

    # Transportation_blended method multiobjective: Comparmental-indivisible
    print('Comparmental-indivisible')
    print('Time needed for  Comparmental-indivisible method: ', time_comp_indivisible)
    n_objectives = m_comp_indivisible.NumObj
    for o in range(n_objectives):
        # Set which objective we will query
        m_comp_indivisible.params.ObjNumber = o
        # Query the o-th objective value
        if m_comp_indivisible.status == GRB.Status.INFEASIBLE:
            print('Model is infeasible')
        else:
            print('Objectives:', m_comp_indivisible.ObjNName, ':', m_comp_indivisible.ObjNVal)        

    #CaTL-Baseline
    print('CaTL')
    print('Time needed for Baseline: ', time_CaTL)
    obj = m_CaTL.getObjective()
    print( 'Objective:', str(obj), ':', -obj.getValue())
    print
    
if __name__ == '__main__':
    baseline_vs_transportation()
