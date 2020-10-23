'''
 Copyright (C) 2020 Cristian Ioan Vasile <cvasile@lehigh.edu>
 Explainable Robotics Lab (ERL), Autonomous and Intelligent Robotics (AIR) Lab,
 Lehigh University
 See license.txt file for license information.
'''

import sys
import logging

from lomap import Ts

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


def mars_partial(ts_filename='mars_ps.yaml'):
    '''TODO:
    '''
    setup_logging()

    ts = Ts.load(ts_filename)
    show_environment(ts)
    for u, v in ts.g.edges():
        print v,u
        assert ts.g.has_edge(v, u)
    #show_environment(ts)

    for u in ts.g:
        logging.debug('State: %s, Data: %s', u, str(ts.g.node[u]))
    #ca = {'Ma', 'Me', 'Mo', 'Pi', 'Ri', 'Sh', 'Su', 'Dr'} from q1-q9
    #AP = {q1:blue, q2:pink, q3:cyan, q4:green, q5:yellow, q6:purple, q7:gray, q8:red, q9:orange} 
    # 15 agents just 5 with Dr which the one need it to get samples

    agents = [('q1', {'Ma', 'Mo', 'Ri', 'Dr'}), ('q2', {'Ma', 'Mo', 'Ri', 'Dr'}), ('q4', {'Me', 'Pi', 'Sh', 'Su'})
            , ('q3', {'Me', 'Pi', 'Sh', 'Su'}), ('q5', {'Ma', 'Me', 'Mo', 'Pi'}), ('q6', {'Ma', 'Me', 'Mo', 'Pi'})
            , ('q9', {'Ri', 'Sh', 'Su', 'Dr'}), ('q8', {'Ri', 'Sh', 'Su', 'Dr'}), ('q7', {'Ma', 'Mo', 'Ri', 'Su'})
            , ('q9', {'Ri', 'Mo', 'Sh', 'Su'}), ('q8', {'Pi', 'Me', 'Ri', 'Su'}), ('q7', {'Mo', 'Sh', 'Ri', 'Su'})
            , ('q6', {'Ma', 'Me', 'Mo', 'Su'}), ('q5', {'Sh', 'Su', 'Pi', 'Ri'}), ('q4', {'Ma', 'Mo', 'Ri', 'Dr'})]

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
    
    #Simplest specification to test robustness
    #specification = ('F[0, 5] T(1, green, {(Ma, 1)})')
    specification = ('F[0,10] T(3, blue, {(Dr,3)}) && F[0,10] T(3, green, {(Dr,1)}) && F[0,10] T(3, pink, {(Dr,1)})')
    
    #specification = ('F[0,10] T(7, blue, {(Dr,3)}) || F[0,10] T(7, green, {(Dr,1)}) || F[0,10] T(6, pink, {(Dr,1)})')

                     

    m = route_planning(ts, agents, specification)
    print('final_opti')
    time_bound = len(ts.g.nodes(data=True)[0][1]['vars']) - 1

    logging.debug('Planning horizon: %d', time_bound)

    check_initial_states(ts, agents)
    check_flow_constraints(ts, agents, time_bound)

if __name__ == '__main__':
    mars_partial()

    #formula = F[0,10](T(3,A,{(C1,2)}) || T(3,B,{(C1,2),(C2,2)})) && F[20,40](T(3,C,{(C2,1)}) U[5,10] (T(3, A, {(C1, 1),(C2,1)}) && T(3,B,{(C1,1),(C2,1)})))