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
    '''Computes a bitmap encoding of agents' capabilities. Each capability is
    associated with a bit in a binary word of length equal to the number of
    capabilities.
    Input
    -----
    List of agents, where agents are tuples (q, cap), q is the initial state of
    the agent, and cap is the set of capabilities. Agents' identifiers are their
    indices in the list.

    Output
    ------
    The output is a dictionary from capabilities to integers representing the
    capabilitie's binary word.
    '''
    capabilities = set.union(*[cap for _, cap in agents])
    capabilities = {c: 1<<k for k, c in enumerate(capabilities)}
    logging.debug('Capabilities bitmap: %s', capabilities)
    return capabilities

def compute_agent_classes(agents, capabilities):
    '''Computes the set of agent types w.r.t. capabilities.
    Input
    -----
    - List of agents, where agents are tuples (q, cap), q is the initial state of
    the agent, and cap is the set of capabilities. Agents' identifiers are their
    indices in the list.
    - Dictionary of capability encoding that maps capabilities to binary words
    represented as integers.
    Output
    ------
    Dictionary of agent classes that maps frozen (immutable) sets of
    capabilities to the binary words encoding the sets.
    '''
    return {frozenset(caps): sum([capabilities[c] for c in caps])
                                                          for _, caps in agents}

def compute_initial_capability_distribution(ts, agents, capabilities):
    '''Computes the initial number of agents of each class at each state.
    Input
    -----
    - The transition system specifying the environment.
    - List of agents, where agents are tuples (q, cap), q is the initial state of
    the agent, and cap is the set of capabilities. Agents' identifiers are their
    indices in the list.
    - Dictionary of capability encoding that maps capabilities to binary words
    represented as integers.
    Output
    ------
    Dictionary from states to distribution of agents from each class. The
    distribution is a list of length equal to the number of capabilities, and
    each element is the number of agents of having those capabilities (a class).
    '''
    nc = len(capabilities)
    capability_distribution = {u: [0]*nc for u in ts.g}
    for state, cap in agents:
        g = sum([capabilities[c] for c in cap])
        capability_distribution[state][g] += 1
    return capability_distribution

def create_system_variables(m, ts, agent_classes, bound, vtype=GRB.INTEGER):
    '''Creates the state and transition variables associated with the given
    transition system.

    The state variables are z_{state}_{cap}_k, where {state} is a node in the TS
    graph, {cap} is a capability class encoded as an integer, and k is the time
    step.

    The transition variables are z_{state1}_{state2}_{cap}_k, where {state1} and
    {state2} define the transition, {cap} is a capability class encoded as an
    integer, and k is the time step.

    Input
    -----
    - The Gurobi model variable.
    - The transition system specifying the environment.
    - The agent classes given as a dictionary from frozen sets of capabilities
    to bitmaps (integers).
    - Time bound.
    - Variable type (default: integer)

    Note
    ----
    Data structure holding the variables is a list of list of variables, e.g.,

        d['vars'][k][g] is the z_{q/e}_bitmap(g)_k

    where d is the dictionary of attributes for a node q or an edge e in the TS,
    g is an agent class (frozen set of capabilities), bitmap(g) is the binary
    encoding of g as an integer, and k is the time step.
    Also, d['vars'] is a list of length `bound+1', d['vars'][k] is a dictionary
    from frozen sets to gurobi variables.
    '''
    # node variables
    for u, d in ts.g.nodes(data=True):
        d['vars'] = [] # initialize node variables list
        for k in range(bound+1):
            name = 'z_' + u + '_{cap}_' + k
            d['vars'].append({g: m.addVar(vtype=vtype, name=name.format(enc))
                                          for g, enc in agent_classes.items()})
    # edge variables
    for u, v, d in ts.g.edges(data=True):
        d['vars'] = [] # initialize edge variables list
        for k in range(bound+1):
            name = 'z_' + u + '_' + v + '_{cap}_' + k
            d['vars'].append({g: m.addVar(vtype=vtype, name=name.format(enc))
                                        for g, enc in agent_classes.items()})

def add_system_constraints(m, ts, agent_classes, capability_distribution,
                           bound):
    '''Computes the constraints that capture the system dynamics.

    Note
    ----
    The initial time constraints

        z_{state}_g_0 = \eta_{state}_g

    is equivalent to

        \sum_{e=(u, v) \in T} z_e_g_W(e) = \eta_{state}_g

    because of the definition of the team state at TS states,
    where \eta_{state}_g is the number of agents of class g at state {state} at
    time 0.
    '''
    # initial time constraints
    for u in ts.g.nodes():
        for g, g_enc in agent_classes.items():
            conserve = sum([d['vars'][d['weight']][g]
                            for _, _, d in ts.g.out_edges_iter(u, data=True)
                                if d['weight'] <= bound])
            conserve = (conserve == capability_distribution[u][g])
            m.addConstr(conserve, 'init_distrib_{}_{}'.format(u, g_enc))

    # edge conservation constraints
    for u, ud in ts.g.nodes(data=True):
        for k in range(bound):
            for g, g_enc in agent_classes.items():
                conserve = sum([d['vars'][k+d['weight']][g]
                            for _, _, d in ts.g.out_edges_iter(u, data=True)
                                if k + d['weight'] <= bound])

                # node constraint: team state
                team_state_eq = (ud['vars'][k][g] == conserve)
                m.addConstr(team_state_eq, 'team_{}_{}_{}'.format(u, g_enc, k))

                # flow balancing constraint
                conserve -= sum([d['vars'][k][g]
                            for _, _, d in ts.g.in_edges_iter(u, data=True)])
                conserve = (conserve == 0)
                m.addConstr(conserve, 'conserve_{}_{}_{}'.format(u, g_enc, k))

def add_proposition_constraints(m, ts):
    '''TODO:
    '''
    # proposition variables
#     for
#

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
    agent_classes = compute_agent_classes(agents, capabilities)
    create_system_variables(m, ts, agent_classes, bound)

    # add system constraints
    capability_distribution = compute_initial_capability_distribution(ts,
                                                           agents, capabilities)
    add_system_constraints(m, ts, agent_classes, capability_distribution, bound)

    # add proposition constraints


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
