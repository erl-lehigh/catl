'''
 Copyright (C) 2018-2020 Cristian Ioan Vasile <cvasile@lehigh.edu>
 Explainable Robotics Lab (ERL), Autonomous and Intelligent Robotics (AIR) Lab,
 Lehigh University
 Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Lab, Boston University
 See license.txt file for license information.
'''

import sys
import logging

from lomap import Ts

from route_planning import route_planning
from visualization import show_environment
from check_system_constraints import check_initial_states, check_flow_constraints


def setup_logging(logfile='test_farm.log', loglevel=logging.DEBUG):
    fs, dfs = '%(asctime)s %(levelname)s %(message)s', '%m/%d/%Y %I:%M:%S %p'

    if logfile is not None:
        logging.basicConfig(filename=logfile, level=loglevel,
                            format=fs, datefmt=dfs)

    root = logging.getLogger()
    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(loglevel)
    ch.setFormatter(logging.Formatter(fs, dfs))
    root.addHandler(ch)


def case_farm(ts_filename='farm.yaml'):
    '''TODO:
    '''
    setup_logging()

    ts = Ts.load(ts_filename)
    for u, v in ts.g.edges():
        assert ts.g.has_edge(v, u)
    show_environment(ts)

    for u in ts.g:
        print(u, ts.g.node[u])

    agents = [('q1', {'UV', 'Mo'}), ('q1', {'UV', 'Mo'}),
              ('q2', {'Vis', 'Mo'}),
              ('q4', {'IR', 'UV'}), ('q4', {'Vis', 'UV'}),
              ('q5', {'Vis', 'Mo'}),
              ('q6', {'IR', 'UV'}),
              ('q8', {'Vis', 'IR'}),
              ('q9', {'Vis', 'UV'}),
              ('q10', {'Vis', 'IR'}), ('q10', {'Vis', 'IR'})]

    initial_locations, capabilities = zip(*agents)
    agent_classes = set(map(frozenset, capabilities))
    initial_locations = set(initial_locations)
    capabilities = set.union(*capabilities)

    logging.debug('Initial locations:', initial_locations)
    logging.debug('Capabilities:', capabilities)
    logging.debug('Agent classes:', agent_classes)

    # make sure all agents' initial states are in the TS
    for state, _ in agents:
        assert state in ts.g, 'State "{}" not in TS!'.format(state)

    specification = ('F[0, 1] T(1, blue, {(UV, 2), (Mo, 1)})'
                     # '&& G[1, 4] T(2, green, {(IR, 1), (Vis, 3)})'
                     # '&& F[3, 4] T(3, yellow, {(IR, 3), (Vis, 2), (UV, 3), (Mo, 4)})'
                     )

    m = route_planning(ts, agents, specification)
    time_bound = len(ts.g.nodes(data=True)[0][1]['vars']) - 1

    logging.debug('Planning horizon: %d', time_bound)

    check_initial_states(ts, agents)
    check_flow_constraints(ts, agents, time_bound)


if __name__ == '__main__':
    case_farm()
