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


def case_simple(ts_filename='simple.yaml'):
    '''TODO:
    '''
    setup_logging()

    ts = Ts.load(ts_filename)
    for u, v in ts.g.edges():
        assert ts.g.has_edge(v, u)
    show_environment(ts)

    for u in ts.g:
        logging.debug('State: %s, Data: %s', u, str(ts.g.node[u]))

    agents = [('q1', {'a'}), ('q2', {'a'}), ('q3', {'a'}), ('q1', {'a'}), ('q2', {'a'}), ('q3', {'a'}),]
             

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

    specification = ('T(1, green, {(a, 1)})')
    #('F[0, 6] T(1, green, {(a, 1)}) && F[0, 6] T(3, blue, {(a, 1)}) && F[6, 12] T(1, blue, {(a, 1)}) && F[7, 12] T(2, orange, {(a, 1)})')
    #('T(1, blue, {(a, 1)}) || T(1, orange, {(a,1)})' )
    #('F[0, 6] T(1, green, {(a, 1)}) || F[0, 6] T(3, blue, {(a, 1)}) && F[6, 12] T(1, blue, {(a, 1)}) || F[7, 12] T(2, orange, {(a, 1)})')
    #'F[0, 20] T(1, green {(a, 2)}) &&  G[20,40] F[0,10] T(1, blue, {(a, 1)})'
                     # '&& G[1, 4] T(2, green, {(IR, 1), (Vis, 3)})'
                     # '&& F[3, 4] T(3, yellow, {(IR, 3), (Vis, 2), (UV, 3), (Mo, 4)})'
                     


    m, n = route_planning(ts, agents, specification)

    time_bound = len(ts.g.nodes(data=True)[0][1]['vars']) - 1

    logging.debug('Planning horizon: %d', time_bound)

    check_initial_states(ts, agents)
    check_flow_constraints(ts, agents, time_bound)
    print(n.x)
if __name__ == '__main__':
    case_simple()