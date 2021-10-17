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
import time

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


def mars_partial(ts_filename='m_ps.yaml'):

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

    agents = [('q1', {'vis', 'mo', 'uv'}), ('q1', {'vis', 'ir'}), ('q2', {'ir', 'uv'})
            , ('q2', {'mo', 'vis'}), ('q2', {'vis', 'ir', 'uv'}), ('q3', {'ir', 'uv'})
            , ('q3', {'mo', 'vis'}), ('q4', {'ir', 'uv'}), ('q4', {'mo', 'ir'})
            , ('q5', {'vis', 'ir', 'uv'}), ('q6', {'vis', 'ir'}), ('q7', {'ir', 'mo'})
            , ('q8', {'mo', 'vis'}), ('q9', {'vis', 'mo', 'uv'})]

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
    T1 = " T(3, pink, {(ir,3)}) "
    T2 = " T(3, green, {(vis,4)}) "
    T3 = " T(6, orange, {(uv,7)}) "
    T4 = " T(5, blue, {(mo,3)}) "
    T5 = " T(3, purple, {(ir,1), (mo,1)}) "
    T6 = " T(6, red, {(mo,4)}) "

    #Simplest specification to test robustness
    #specification = ('F[0, 5] T(1, green, {(Ma, 1)})')
    # specification = ('F[10, 15] T(3, green, {(Ma, 9)})')
    #('G[0,10] T(1, blue, {(Dr,3)}) && G[0,10] T(1, green, {(Dr,4)})')
    #('F[0,10] T(3, blue, {(Dr,3)}) && F[10,20] T(2, green, {(Dr,1)}) && F[10,20] T(2, pink, {(Dr,1)})')

    # specification = ('G[0,20]' + T1 + '&& G[0,20]' + T3+ '&& G[0,20]' + T5)
    # specification = ('G[0,20]' + T2 + '&& G[0,20]' + T4+ '&& G[0,20]' + T6)
    specification = ('F[0,20]' + T1 + '&& F[0,20]' + T3+ '&& F[0,20]' + T5 + 'F[0,20]' + T2 + '&& F[0,20]' + T4+ '&& F[0,20]' + T6)

                     

    m = route_planning(ts, agents, specification)
    print('final_opti')
    time_bound = len(ts.g.nodes(data=True)[0][1]['vars']) - 1

    # logging.debug('Planning horizon: %d', time_bound)

    # check_initial_states(ts, agents)
    # check_flow_constraints(ts, agents, time_bound)
    # print(n.x)
if __name__ == '__main__':

    start = time.time()
    mars_partial()
    end = time.time()
    final_time = end - start 
    print("Time Nedeed: ", final_time)
    #formula = F[0,10](T(3,A,{(C1,2)}) || T(3,B,{(C1,2),(C2,2)})) && F[20,40](T(3,C,{(C2,1)}) U[5,10] (T(3, A, {(C1, 1),(C2,1)}) && T(3,B,{(C1,1),(C2,1)})))