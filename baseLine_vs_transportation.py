'''
 Copyright (C) 2020 Cristian Ioan Vasile <cvasile@lehigh.edu>
 Explainable Robotics Lab (ERL), Autonomous and Intelligent Robotics (AIR) Lab,
 Lehigh University
 See license.txt file for license information.
'''

import sys
import logging
import time
import copy
from matplotlib.pyplot import flag
import numpy as np
from lomap import Ts
from gurobipy import GRB
from route_planning import route_planning
from visualization import show_environment
from check_system_constraints import check_initial_states, check_flow_constraints


def setup_logging(logfile='test_simple.log', loglevel=logging.DEBUG,
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

def generateRandomAgents(stateList,capabilityList,numCapabilitiesPerAgent,
                        numAgentsPerClass, numClasses):
    agents = []
    # print
    for j  in range(numClasses):
        modCapList =  copy.copy(capabilityList)
        #Ensure capability coverage
        modCapList.pop(j%len(capabilityList))
        randClass = set({capabilityList[j%len(capabilityList)]}) | set(np.random.choice(modCapList,size=numCapabilitiesPerAgent-1,replace=False))
        agents += [(stateList[np.random.choice(range(len(stateList)))], randClass) for k in range(numAgentsPerClass)]
    return agents


def baseline_vs_transportation(ts_filename='simple2.yaml'):

    '''Simple example of planning with resource constraints.'''
    setup_logging()

    ts = Ts.load(ts_filename)
    for u, v in ts.g.edges():
        assert ts.g.has_edge(v, u)
    show_environment(ts)

    for u in ts.g:
        logging.debug('State: %s, Data: %s', u, str(ts.g.node[u]))

    agents = [('q1', {'a','b'}), ('q6', {'c', 'd'}), ('q9', {'e','f'}),
              ('q2', {'a','c'}), ('q3', {'a', 'c'}), ('q7', {'d','b'}),
              ('q5', {'a','b'}), ('q4', {'a', 'f'}), ('q8', {'d','b'}),
              ('q5', {'a','f'}), ('q4', {'c', 'd'}), ('q8', {'e','f'})
             ]
    resources = {'r1': {'q1': 6.1, 'q5': 6.2},
                 'r2': {'q2': 6.2, 'q6': 6.2},
                 'r3': {'q3': 6.2, 'q7': 6.2},
                 'r4': {'q4': 6.2, 'q8': 6.2}
                }
    storage_type = 'comparmental' # choices: comparmental, uniform
    capacities = {
        frozenset({'a', 'b'}): {'r1': 2, 'r2': 2, 'r3': 2, 'r4': 2},
        frozenset({'c', 'd'}): {'r1': 2, 'r2': 2, 'r3': 1, 'r4': 2},
        frozenset({'e', 'f'}): {'r1': 2, 'r2': 2, 'r3': 1, 'r4': 2},
        frozenset({'a', 'c'}): {'r1': 2, 'r2': 2, 'r3': 1, 'r4': 2},
        frozenset({'b', 'd'}): {'r1': 2, 'r2': 2, 'r3': 1, 'r4': 2},
        frozenset({'a', 'f'}): {'r1': 2, 'r2': 2, 'r3': 1, 'r4': 2},
        # frozenset({'b'}): {'r1': 1, 'r2': 1, 'r3': 2, 'r4': 2}
    }
    # storage_type = 'uniform' # choices: comparmental, uniform
    # capacities = {
    #     frozenset({'a'}): 2
    # }                                                 

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

    spec_transportation = ('F[0, 10] T(2, orange, {(a, 2)}, {(r1, 3.5), (r2, 0.5)})')
    spec_CaTL = ('F[0, 10] T(2, orange, {(a, 1)})')
                    # 'F[0, 2] T(1, green, {(a, 1), (b, 1)})'
                     # '&& G[1, 4] T(2, green, {(IR, 1), (Vis, 3)})'
                     # '&& F[3, 4] T(3, yellow, {(IR, 3), (Vis, 2), (UV, 3), (Mo, 4)})'

    numCapabilitiesPerAgent = 2
    numAgentsPerClass = 1
    numClasses = len(agent_classes)
    agentsTrial = generateRandomAgents(ts.g.nodes(),list(capabilities),numCapabilitiesPerAgent,numAgentsPerClass, numClasses)
           

    # Transportation_blended method multiobjective
    start_transportation = time.time()
    m_transportation_blend = route_planning(ts, agents, spec_transportation, storage_type=storage_type,
                       capacities=capacities, resource_distribution=resources,
                       resource_type='divisible', travel_time_weight=0.1,
                       resources_weight=0.1, transportation=True)
    end_transportation = time.time()
    time_transportation = end_transportation - start_transportation
    
    # Transportation reward method single objective
    start_transportation_so = time.time()
    m_transportation_so = route_planning(ts, agents, spec_transportation, storage_type=storage_type,
                       capacities=capacities, resource_distribution=resources,
                       resource_type='divisible', travel_time_weight=0.1,
                       resources_weight=0.1, transportation=True, flag=False)
    end_transportation_so = time.time()
    time_transportation_so = end_transportation_so - start_transportation_so

    #CaTL-Baseline
    start_CaTL = time.time() 
    m_CaTL = route_planning(ts, agents, spec_CaTL)       
    end_CaTL = time.time()
    time_CaTL = end_CaTL - start_CaTL



    time_bound = len(ts.g.nodes(data=True)[0][1]['vars']) - 1

    logging.debug('Planning horizon: %d', time_bound)

    check_initial_states(ts, agents)
    check_flow_constraints(ts, agents, time_bound)
    
    print('CaTL')
    print('Time needed for Baseline: ', time_CaTL)
    obj = m_CaTL.getObjective()
    print( 'Objective:', str(obj), ':', -obj.getValue())
    print

    print('Transportation Blended')
    print('Time needed for Transportation blended method: ', time_transportation)
    n_objectives = m_transportation_blend.NumObj
    for o in range(n_objectives):
        # Set which objective we will query
        m_transportation_blend.params.ObjNumber = o
        # Query the o-th objective value
        if m_transportation_blend.status == GRB.Status.INFEASIBLE:
            print('Model is infeasible')
        else:
            print('Objectives:', m_transportation_blend.ObjNName, ':', m_transportation_blend.ObjNVal)
    
    print
    print('Transportation Single Objective')
    print('Time needed for Transportation single objective: ', time_transportation_so)
    obj3 = m_transportation_so.getObjective()
    if m_transportation_so.status == GRB.Status.INFEASIBLE:
            print('Model is infeasible')
    else:
        print( 'Objective:', str(obj3), ':', obj3.getValue())
    print
    
    print(agentsTrial)
    # print(agents)
    print(len(agent_classes), agent_classes, type(capacities))
    
if __name__ == '__main__':
    baseline_vs_transportation()
