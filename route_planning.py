'''
 Copyright (C) 2018 Cristian Ioan Vasile <cvasile@bu.edu>
 Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Lab, Boston University
 See license.txt file for license information.
'''

import logging

from gurobipy import Model as GRBModel
from gurobipy import GRB

from lomap import Timer

from stl import stl2milp
from cmtl import CapabilityRequest, CMTLFormula
from cmtl import cmtl2stl
from visualization import show_environment


def compute_capability_bitmap(agents):
    '''TODO:
    '''
    capabilities = set.union(*[cap for _, cap in agents])
    capabilities = {c: 1<<k for k, c in enumerate(capabilities)}
    print capabilities
    return capabilities

def compute_initial_capability_distribution(ts, agents, capabilities):
    '''TODO:
    '''
    nc = len(capabilities)
    capability_distribution = {u: [0]*nc for u in ts.g}
    for state, cap in agents:
        g = sum([capabilities[c] for c in cap])
        capability_distribution[state][g] += 1
    return capability_distribution

def create_variables(m, ts, number_of_capabilities, bound, vtype=GRB.INTEGER):
    '''TODO:
    '''
    nc = number_of_capabilities
    # node variables
    for u, d in ts.g.nodes(data=True):
        d['vars'] = []
        for k in range(bound+1):
            name = 'z_' + u + '_{cap}_' + k
            d['vars'].append([m.addVar(vtype=vtype, name=name.format(c))
                                                        for c in range(1<<nc)])
    # edge variables
    for u, v, d in ts.g.edges(data=True):
        d['vars'] = []
        for k in range(bound+1):
            name = 'z_' + u + '_' + v + '_{cap}_' + k
            d['vars'].append([m.addVar(vtype=vtype, name=name.format(c))
                                                        for c in range(1<<nc)])

def add_system_constraints(m, ts, number_of_capabilities,
                           capability_distribution, bound):
    '''TODO:
    '''
    nc = number_of_capabilities
    # initial time constraints
    for u in ts.g.nodes():
        for g in range(1<<nc):
            conserve = sum([d['vars'][d['weight']][g]
                            for _, _, d in ts.g.out_edges_iter(u, data=True)
                                if d['weight'] <= bound])
            conserve = (conserve == capability_distribution[u][g])
            m.addConstr(conserve, 'init_distrib_{}_{}'.format(u, g))

    # edge conservation constraints
    for u, ud in ts.g.nodes(data=True):
        for k in range(bound):
            for g in range(1<<nc):
                conserve = sum([d['vars'][k+d['weight']][g]
                            for _, _, d in ts.g.out_edges_iter(u, data=True)
                                if k + d['weight'] <= bound])

                # node constraint: team state # TODO: Needed?
                team_state_eq = (ud['vars'][k][g] == conserve)
                m.addConstr(team_state_eq, 'team_{}_{}_{}'.format(u, g, k))

                # flow balancing constraint
                conserve -= sum([d['vars'][k][g]
                            for _, _, d in ts.g.in_edges_iter(u, data=True)])
                conserve = (conserve == 0)
                m.addConstr(conserve, 'conserve_{}_{}_{}'.format(u, g, k))

def route_planning(ts, agents, formula, bound=None):
    '''TODO:
    '''
    ast = CMTLFormula.from_formula(formula)
    if bound is None:
        bound = formula.bound()

    # create MILP
    m = GRBModel('milp')

    # create system variables
    capabilities = compute_capability_bitmap(agents)
    number_of_capabilities = len(capabilities)
    create_variables(m, ts, number_of_capabilities, bound)

    # add system constraints
    capability_distribution = compute_initial_capability_distribution(ts,
                                                           agents, capabilities)
    add_system_constraints(m, ts, number_of_capabilities,
                           capability_distribution, bound)

    # add CMTL formula constraints
    stl = cmtl2stl(ast)
    stl_milp = stl2milp(stl, model=m, robust=True)
    z_formula = stl_milp.to_milp()

    # run optimizer
    m.optimize()

    if m.status == GRB.Status.OPTIMAL:
        logging.info('"Optimal objective LP": %f', m.objVal)
    elif m.status == GRB.Status.INF_OR_UNBD:
        logging.error('Model is infeasible or unbounded')
    elif m.status == GRB.Status.INFEASIBLE:
        logging.error('Model is infeasible')
    elif m.status == GRB.Status.UNBOUNDED:
        logging.error('Model is unbounded')
    else:
        logging.error('Optimization ended with status %s', m.status)
