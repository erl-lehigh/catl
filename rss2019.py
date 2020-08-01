'''
 Copyright (C) 2018-2020 Cristian Ioan Vasile <cvasile@lehigh.edu>
 Explainable Robotics Lab (ERL), Autonomous and Intelligent Robotics (AIR) Lab,
 Lehigh University
 Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Lab, Boston University
 See license.txt file for license information.
'''

from lomap import Ts

from route_planning import route_planning
from visualization import show_environment


def case_rss2019(ts_filename='farm.yaml'):
    '''TODO:
    '''

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
    # make sure all agents' initial states are in the TS
    for state, _ in agents:
        assert state in ts.g, 'State "{}" not in TS!'.format(state)

    specification ='F[0, 2] T(4, blue, {(a, 2), (b, 3)})'\
                    '&& G[1, 7] T(2, orange, {(a, 1), (c, 4)})'\
                    '&& F[3, 5] T(3, red, {(b, 1), (d, 2)})'
    

    route_planning(ts, agents, specification)

if __name__ == '__main__':
    case_rss2019()
