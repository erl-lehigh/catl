'''
 Copyright (C) 2018-2020 Cristian Ioan Vasile <cvasile@lehigh.edu>
 Explainable Robotics Lab (ERL), Autonomous and Intelligent Robotics (AIR) Lab,
 Lehigh University
 Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Lab, Boston University
 See license.txt file for license information.
'''

from collections import defaultdict
import logging

from gurobipy import Model as GRBModel
from gurobipy import GRB

from lomap import Timer

from stl.stl2milp import stl2milp
from catl import CATLFormula
from catl import catl2stl
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
    return {frozenset(g): sum([capabilities[c] for c in g]) for _, g in agents}

def compute_initial_capability_distribution(ts, agents, agent_classes):
    '''Computes the initial number of agents of each class at each state.
    Input
    -----
    - The transition system specifying the environment.
    - List of agents, where agents are tuples (q, cap), q is the initial state of
    the agent, and cap is the set of capabilities. Agents' identifiers are their
    indices in the list.
    - Dictionary of capability class encoding that maps capabilities to binary
    words represented as integers.
    Output
    ------
    Dictionary from states to distribution of agents from each class. The
    distribution is a list of length equal to the number of classes, and
    each element is the number of agents of having those capabilities (a class).
    '''
    nc = len(agent_classes)
    capability_distribution = {u: defaultdict(int) for u in ts.g}
    for state, g in agents:
        g_enc = agent_classes[frozenset(g)]
        capability_distribution[state][g_enc] += 1
    return capability_distribution

def create_system_variables(m, ts, agent_classes, time_bound, variable_bound,
                            vtype=GRB.INTEGER):
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
    - The upper bound for variables.
    - Variable type (default: integer).

    Note
    ----
    Data structure holding the variables is a list of list of variables, e.g.,

        d['vars'][k][g] is the z_{q/e}_bitmap(g)_k

    where d is the dictionary of attributes for a node q or an edge e in the TS,
    g is an agent class (frozen set of capabilities), bitmap(g) is the binary
    encoding of g as an integer, and k is the time step.
    Also, d['vars'] is a list of length `time_bound+1', d['vars'][k] is a
    dictionary from frozen sets to gurobi variables.
    '''
    # node variables
    for u, d in ts.g.nodes(data=True):
        d['vars'] = [] # initialize node variables list
        for k in range(time_bound+1):
            name = 'z_{state}_{{}}_{time}'.format(state=u, time=k)
            d['vars'].append({g: m.addVar(vtype=vtype, name=name.format(enc),
                                          lb=0, ub=variable_bound)
                                          for g, enc in agent_classes.items()})
    # edge variables
    for u, v, d in ts.g.edges(data=True):
        d['vars'] = [] # initialize edge variables list
        for k in range(time_bound+1):
            name = 'z_{src}_{dest}_{{}}_{time}'.format(src=u, dest=v, time=k)
            d['vars'].append({g: m.addVar(vtype=vtype, name=name.format(enc),
                                          lb=0, ub=variable_bound)
                                        for g, enc in agent_classes.items()})

def add_system_constraints(m, ts, agent_classes, capability_distribution,
                           time_bound):
    '''Computes the constraints that capture the system dynamics.

    Input
    -----
    - The Gurobi model variable.
    - The transition system specifying the environment.
    - The agent classes given as a dictionary from frozen sets of capabilities
    to bitmaps (integers).
    - The initial distribution of capabilities at each state.
    - Time bound.

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
    # edge conservation constraints
    for u, ud in ts.g.nodes(data=True):
        for k in range(time_bound):
            for g, g_enc in agent_classes.items():
                conserve = sum([d['vars'][k+d['weight']][g]
                            for _, _, d in ts.g.out_edges_iter(u, data=True)
                                if k + d['weight'] <= time_bound])

                # node constraint: team state
                team_state_eq = (ud['vars'][k][g] == conserve)
                m.addConstr(team_state_eq, 'team_{}_{}_{}'.format(u, g_enc, k))

                # flow balancing constraint
                conserve -= sum([d['vars'][k][g]
                            for _, _, d in ts.g.in_edges_iter(u, data=True)])
                conserve = (conserve == 0)
                m.addConstr(conserve, 'conserve_{}_{}_{}'.format(u, g_enc, k))

#     # initial time constraints - encoding using transition variables
#     for u in ts.g.nodes():
#         for g, g_enc in agent_classes.items():
#             conserve = sum([d['vars'][d['weight']][g]
#                             for _, _, d in ts.g.out_edges_iter(u, data=True)
#                                 if d['weight'] <= time_bound])
#             conserve = (conserve == capability_distribution[u][g])
#             m.addConstr(conserve, 'init_distrib_{}_{}'.format(u, g_enc))

    # initial time constraints - encoding using state variables
    for u, ud in ts.g.nodes(data=True):
        for g, g_enc in agent_classes.items():
            conserve = (ud['vars'][0][g] == capability_distribution[u][g])
            m.addConstr(conserve, 'init_distrib_{}_{}'.format(u, g_enc))

def extract_propositions(ts, ast):
    '''Returns the set of propositions in the formula, and checks that it is
    included in the transitions system.

    Input
    -----
    - The transition system specifying the environment.
    - The AST of the CaTL specification formula.

    Output
    ------
    Set of propositions in the specification formula.
    '''
    ts_propositions = set.union(*[d['prop'] for _, d in ts.g.nodes(data=True)])
    formula_propositions = set(ast.propositions())
    assert formula_propositions <= ts_propositions, \
                                'There are unknown propositions in the formula!'
    return formula_propositions

def add_proposition_constraints(m, stl_milp, ts, ast, capabilities,
                                agent_classes, time_bound, variable_bound,
                                vtype=GRB.INTEGER):
    '''Adds the proposition constraints. First, the proposition-state variables
    are defined such that capabilities are not double booked. Second, contraints
    are added such that proposition are satisfied as best as possible. The
    variables in the MILP encoding of the STL formula are used for the encoding
    as the minimizers of over proposition-state variables.

    Input
    -----
    - The Gurobi model variable.
    - The MILP encoding of the STL formula obtained from the CaTL specification.
    - The transition system specifying the environment.
    - The AST of the CaTL specification formula.
    - Dictionary of capability encoding that maps capabilities to binary words
    represented as integers.
    - The agent classes given as a dictionary from frozen sets of capabilities
    to bitmaps (integers).
    - Time bound.
    - The upper bound for variables.
    - Variable type (default: integer).
    '''
    props = extract_propositions(ts, ast)

    # add proposition-state variables
    for u, ud in ts.g.nodes(data=True):
        ud['prop_vars'] = dict()
        for c in capabilities:
            ud['prop_vars'][c] = []
            for k in range(time_bound+1):
                ud['prop_vars'][c].append(dict())
                for prop in ud['prop']:
                    name = 'z_{prop}_{state}_{cap}_{time}'.format(
                        prop=prop, state=u, cap=c, time=k)
                    ud['prop_vars'][c][k][prop] = m.addVar(
                        vtype=vtype, name=name, lb=0, ub=variable_bound)

    # constraints for relating (proposition, state) pairs to system states
    for u, ud in ts.g.nodes(data=True):
        for c in capabilities:
            for k in range(time_bound+1):
                equality = sum([ud['prop_vars'][c][k][prop]
                                                    for prop in ud['prop']])
                equality -= sum([ud['vars'][k][g] for g in agent_classes
                                                                    if c in g])
                equality = (equality == 0)
                m.addConstr(equality, 'prop_state_{}_{}_{}'.format(u, c, k))

    # add propositions constraints for only those variables appearing in the
    # MILP encoding of the formula
    for prop in props:
        for c in capabilities:
            for k in range(time_bound+1):
                variable = '{prop}_{cap}'.format(prop=prop, cap=c)
                if (variable in stl_milp.variables
                                        and k in stl_milp.variables[variable]):
                    for u, ud in ts.g.nodes(data=True):
                        if prop in ud['prop']:
                            min_prop = (stl_milp.variables[variable][k]
                                                <= ud['prop_vars'][c][k][prop])
                            m.addConstr(min_prop, 'min_prop_{}_{}_{}_{}'.format(
                                                                prop, c, k, u))

def add_travel_time_objective(m, ts, weight, time_bound, variable_bound):
    '''Adds the total travel time of all agents as an objective.

    Input
    -----
    - The Gurobi model variable.
    - The transition system specifying the environment.
    - The objective's weight.
    - Time bound.
    - The upper bound for variables.
    '''
    travel_time = sum(sum(d['vars'].values()) * d['weight']
                      for _, _, d in ts.g.edges(data=True))
    travel_time /= (time_bound * variable_bound)
    m.setObjectiveN(travel_time, m.NumObj, weight=weight)

def extract_trajetories(m, ts, agents, time_bound):
    '''TODO:
    '''
    raise NotImplementedError
    # initialize trajectories for each agent
    trajectories = [[(state, 0)] for state, _ in agents]

    for k in range(1, time_bound+1):
        # setup matching problem
        prev = [trajectories[agent][-1] for agent in len(agents)]

        matching = dict()
        constraints = {} # active transitions whose constraints have to be met
        for a, (state, time) in enumerate(prev):
            assert k-1 <= time
            if time == k-1: # check if vehicle needs to be assigned next state
                matching[prev] = [] # initialize possible future states
                agent_class = agents[a][1] # extract agent class
                # loop over outgoing transitions that have agents of the given
                # class traversing them; add outgoing neighbors
                for _, next_state, d in ts.g.out_edges_iter(state, data=True):
                    if d['vars'][time+d['weight']][agent_class] > 0:
                        matching[prev].append((next_state, time+d['weight']))
                        constraints.add((state, next_state, time+d['weight']))

        # create constraint matching problem
        # TODO: each agent in `matching' has to be matched to a future state
        # such that each active transition in `constraints' has exactly the
        # the correct amount of agents traversing it
        # NOTE: It seems to me to be an instance of the knapsack problem, but I
        # am not sure. It can be posed as an ILP.

    return trajectories

def route_planning(ts, agents, formula, time_bound=None, variable_bound=None,
                   robust=True, travel_time_weight=0):
    '''Performs route planning for agents `agents' moving in a transition system
    `ts' such that the CaTL specification `formula' is satisfied.

    Input
    -----
    - The transition system specifying the environment.
    - List of agents, where agents are tuples (q, cap), q is the initial state
    of the agent, and cap is the set of capabilities. Agents' identifiers are
    their indices in the list.
    - The CaTL specification formula.
    - The time bound used in the encoding (default: computed from CaTL formula).
    - The upper bound for variables.
    - Flag indicating whether to solve the robust or feasibility problem.
    - The weight of the total travel time objective used for regularization.

    Output
    ------
    TODO: TBD
    '''
    ast = CATLFormula.from_formula(formula)
    if time_bound is None:
        time_bound = int(ast.bound())

    if variable_bound is None:
        variable_bound = len(agents)

    # create MILP
    m = GRBModel('milp')

    # create system variables
    capabilities = compute_capability_bitmap(agents)
    agent_classes = compute_agent_classes(agents, capabilities)
    create_system_variables(m, ts, agent_classes, time_bound)

    # add system constraints
    capability_distribution = compute_initial_capability_distribution(ts,
                                                          agents, agent_classes)
    add_system_constraints(m, ts, agent_classes, capability_distribution,
                           time_bound, variable_bound)

    # add CATL formula constraints
    stl = catl2stl(ast)
    ranges = {variable: (0, len(agents)) for variable in stl.variables()}
    stl_milp = stl2milp(stl, ranges=ranges, model=m, robust=robust)
    stl_milp.translate()

    # add proposition constraints
    add_proposition_constraints(m, stl_milp, ts, ast, capabilities,
                                agent_classes, time_bound, variable_bound)

    # add travel time regularization
    if travel_time_weight > 0:
        add_travel_time_objective(m, ts, travel_time_weight, time_bound,
                                  variable_bound)

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

#     return extract_trajetories(m, ts, agents, time_bound) #TODO:
    return m
