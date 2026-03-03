'''
 Copyright (C) 2018-2020 Cristian Ioan Vasile <cvasile@lehigh.edu>
 Explainable Robotics Lab (ERL), Autonomous and Intelligent Robotics (AIR) Lab,
 Lehigh University
 Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Lab, Boston University
 See license.txt file for license information.
'''

from lomap import Ts
import time
from route_planning import route_planning
from visualization import show_environment
from stl import Operation


def case_wafr2026(ts_filename='/home/erl/PyProj/catl/farm.yaml'):
    '''TODO:
    '''

    ts = Ts.load(ts_filename)
    for u, v in ts.g.edges():
        assert ts.g.has_edge(v, u)
    show_environment(ts)

    for u in ts.g:
        print(u, ts.g.nodes[u])

    agents = [('q9', {'UV', 'IR', 'Mo', 'Vis'}),
              ('q9', {'UV', 'IR', 'Mo', 'Vis'}),
              ('q9', {'UV', 'IR', 'Mo', 'Vis'}),
              ('q9', {'UV', 'IR', 'Mo', 'Vis'}),
              ('q9', {'UV', 'IR', 'Mo', 'Vis'}),
              ('q9', {'UV', 'IR', 'Mo', 'Vis'}),
              ('q9', {'UV', 'Vis'}),
              ('q9', {'UV', 'Vis'}),
              ('q9', {'UV', 'Vis'}),
              ('q9', {'IR', 'Vis'}),
              ('q9', {'IR', 'Vis'}),
              ('q9', {'IR', 'Vis'}),
              ('q9', {'Mo', 'Vis'}),
              ('q9', {'Mo', 'Vis'}),
              ('q9', {'Mo', 'Vis'}),]
    # make sure all agents' initial states are in the TS
    for state, _ in agents:
        assert state in ts.g, 'State "{}" not in TS!'.format(state)

    specification = """(G[0, 10] T(3, yellow, {(Vis, 3), (IR, 3), (UV, 3), (Mo, 3)}))
                    && ((F[0, 10] T(5, blue, {(Vis, 1)}))
                    || (F[0, 10] T(5, green, {(Mo, 1)}))
                    || (F[0, 10] T(5, red, {(UV, 1)}))
                    || (F[0, 10] T(5, orange, {(IR, 1)})))"""
    start_time = time.time()
    model, stl_milp = route_planning(ts, agents, specification, maximalSatisfaction=True, balance=True, decouple=False)
    elapsedTime = time.time() - start_time
    print('Route planning took {:.3f} seconds.'.format(elapsedTime))
    u = 'q1'
    v = 'q2'
    g = frozenset(['Vis', 'UV'])
    for child in stl_milp.formula.children:
        print(child)
        var = stl_milp.variables[child][0]
        name = var.VarName
        print(name, var.X)
    for child in stl_milp.formula.children[1].children:
        print(child)
        var = stl_milp.variables[child][0]
        name = var.VarName
        print(name, var.X)
    #print(stl_milp.rhoVariables[stl_milp.formula.children[0]][0])
    #print(stl_milp.rhoVariables[stl_milp.formula.children[1]][0])
    for t in range(11):
        print([stl_milp.variables[child.variable][t] for child in stl_milp.formula.children[0].child.child.children if child.variable in stl_milp.variables])
    for k in range(4):
        for t in range(15):
            print([stl_milp.variables[child.variable][t] for child in stl_milp.formula.children[1].children[k].child.child.children if child.variable in stl_milp.variables])
    print(sum(stl_milp.balanceRobustnessObjectives[1][i].getValue() for i in range(4)))
    print(stl_milp.rho)
    #print(ts.g.nodes[u]['vars'][0])

    #print ('Node:', ts.g.nodes[u]['vars'][0][g].x)
    #print ('Edge:', ts.g[u][v]['vars'][3][g].x)


if __name__ == '__main__':
    case_wafr2026()
