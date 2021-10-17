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
import gurobipy as grb

from lomap import Timer

from stl.stl2milp import stl2milp
from stl.pstl2milp import pstl2milp
from stl import Operation
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
        for k in range(time_bound):
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
        for k in range(time_bound+1):
            for g, g_enc in agent_classes.items():
                departing = sum([d['vars'][k][g]
                            for _, _, d in ts.g.out_edges_iter(u, data=True)
                                if k + d['weight'] <= time_bound])
                arriving = sum([d['vars'][k - d['weight']][g]
                            for _, _, d in ts.g.in_edges_iter(u, data=True)
                                if k - d['weight'] >= 0])

                if 0 < k < time_bound:
                    # flow balancing constraint
                    m.addConstr(departing == arriving,
                                'conserve_{}_{}_{}'.format(u, g_enc, k))

                # node constraint: team state
                if k < time_bound:
                    team_state_eq = (ud['vars'][k][g] == departing)
                else:
                    team_state_eq = (ud['vars'][k][g] == arriving)
                m.addConstr(team_state_eq, 'team_{}_{}_{}'.format(u, g_enc, k))

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
            conserve = (ud['vars'][0][g] == capability_distribution[u][g_enc])
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


def nsub_formulae_satisfied(stl, stl_milp, t=0):
    '''TODO:
    Returns Gurobi variable 
    '''
    m = stl_milp.model 
    satis = stl_milp.variables[stl][t]
    
    if stl.op == Operation.PRED:
        return satis
    
    elif stl.op == Operation.AND:
        for child in stl.children:
            satis += nsub_formulae_satisfied(child, stl_milp, t)
        n = m.addVar(name="n_{}_{}".format(stl.identifier(), t))
        m.addConstr(n == satis)
        return n

    elif stl.op == Operation.OR:
        p = m.addVar();
        m.addConstr(p == grb.max_([nsub_formulae_satisfied(child, stl_milp, t)
                                    for child in stl.children]))
        n = m.addVar(name="n_{}_{}".format(stl.identifier(), t))
        m.addConstr(n == satis + p)
        return n

    elif stl.op == Operation.UNTIL:
        a, b = int(stl.low), int(stl.high)
        satis_until=[]
        sum_left = sum([nsub_formulae_satisfied(stl.left, stl_milp, tau)
                        for tau in range(t, t+a)])
        for t_prime in range(a,b+1):
            sum_left += nsub_formulae_satisfied(stl.left, stl_milp, t + t_prime)
            aux = m.addVar()
            m.addConstr(aux == nsub_formulae_satisfied(stl.right, stl_milp, t + t_prime)
                             + sum_left)
            satis_until.append(aux)
        p = m.addVar();
        m.addConstr(p == grb.max_(satis_until))
        n = m.addVar(name="n_{}_{}".format(stl.identifier(), t))
        m.addConstr(n == satis + p)
        return n

    elif stl.op == Operation.EVENT:
        a, b = int(stl.low), int(stl.high)
        child = stl.child
        p = m.addVar()
        m.addConstr(p == grb.max_([nsub_formulae_satisfied(child, stl_milp, t+tau)
                                   for tau in range(a, b+1)]))
        n = m.addVar(name="n_{}_{}".format(stl.identifier(), t))
        m.addConstr(n == satis + p)
        return n 

    elif stl.op == Operation.ALWAYS:
        a, b = int(stl.low), int(stl.high)
        child = stl.child
        p = m.addVar()
        m.addConstr(p == sum([nsub_formulae_satisfied(child, stl_milp, t+tau)
                              for tau in range(a, b+1)]))
        n = m.addVar(name="n_{}_{}".format(stl.identifier(), t))
        m.addConstr(n == satis + p)
        return n 

def partial_robustness(stl, stl_milp, t=0, max_robustness=1000):
 
    m = stl_milp.model 

    if stl.op == Operation.PRED:
        print(stl.variable)
        r = m.addVar(name='ro_{}_{}'.format(stl.identifier, t)) 
        term = m.addVar(name = 'term_{}_{}'.format(stl.identifier, t)) 
        m.addConstr(r == (stl_milp.variables[stl.variable][t] - stl.threshold) / max_robustness) 
        m.addConstr(term == grb.min_(r, stl_milp.variables[stl.variable][t])) 
        return term, r, 1

    elif stl.op == Operation.AND:
        r = m.addVar(name = 'ro_{}_{}'.format(stl.identifier, t))
        term = m.addVar(name = 'term_{}_{}'.format(stl.identifier, t))
        term_children, r_children, n_term_children = zip(*[partial_robustness(ch, stl_milp, t, max_robustness) for ch in stl.children])
        m.addConstr(r == grb.min_(r_children)) 
        m.addConstr(term == grb.min_(r, stl_milp.variables[stl][t]))
        return term + sum(term_children), r, 1 + sum(n_term_children)

    elif stl.op == Operation.OR:
        r = m.addVar(name = 'ro_{}_{}'.format(stl.identifier, t))
        term = m.addVar(name = 'term_{}_{}'.format(stl.identifier, t))
        term_children, r_children, n_term_children = zip(*[partial_robustness(ch, stl_milp, t, max_robustness) for ch in stl.children])
        m.addConstr(r == grb.max_(r_children))
        m.addConstr(term == grb.min_(r, stl_milp.variables[stl][t]))
        return term + sum(term_children), r, 1 + sum(n_term_children)

    elif stl.op == Operation.UNTIL:
        r = m.addVar(name = 'ro_{}_{}'.format(stl.identifier, t))
        term = m.addVar(name = 'term_{}_{}'.format(stl.identifier, t))
        a, b = int(stl.low), int(stl.high)
        r_until=[]
        n_terms = 0
        terms_children = 0
        for t_ in range(a,b+1):
            term_left, r_left, n_terms_left =  zip(*[partial_robustness(stl.left, stl_milp, t+t__, max_robustness) for t__ in range(0,t_)])
            term_right, r_right, n_terms_right = partial_robustness(stl.rigth, stl_milp, t+t_)
            terms_children += term_right + sum(term_left)
            r_until.append(grb.min_(r_right, grb.min_(r_left)))
            n_terms += n_terms_right + sum(n_terms_left)
        m.addConstr(r == grb.max_(r_until))
        m.addConstr(term == grb.min_(r, stl_milp.variables[stl][t]))
        return term + terms_children, r, n_terms

    elif stl.op == Operation.EVENT:
        r = m.addVar(name = 'ro_{}_{}'.format(stl.identifier, t))
        term = m.addVar(name = 'term_{}_{}'.format(stl.identifier, t))
        a, b = int(stl.low), int(stl.high)
        child = stl.child
        term_child, r_child, n_term_child = zip(*[partial_robustness(child, stl_milp, t+tau, max_robustness) for tau in range(a, b+1)])
        m.addConstr(r == grb.max_(r_child))
        m.addConstr(term == grb.min_(r, stl_milp.variables[stl][t]))
        return term + sum(term_child), r, 1 + sum(n_term_child)

    elif stl.op == Operation.ALWAYS:
        r = m.addVar(name = 'ro_{}_{}'.format(stl.identifier, t))
        term = m.addVar(name = 'term_{}_{}'.format(stl.identifier, t))
        a, b = int(stl.low), int(stl.high)
        child = stl.child
        term_child, r_child, n_term_child = zip(*[partial_robustness(child, stl_milp, t+tau, max_robustness) for tau in range(a, b+1)])
        m.addConstr(r == grb.min_(r_child))
        m.addConstr(term == grb.min_(r, stl_milp.variables[stl][t]))
        return term + sum(term_child), r, 1 + sum(n_term_child)


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
    create_system_variables(m, ts, agent_classes, time_bound, variable_bound)

    # add system constraints
    capability_distribution = compute_initial_capability_distribution(ts,
                                                          agents, agent_classes)
    add_system_constraints(m, ts, agent_classes, capability_distribution,
                           time_bound)

    # add CATL formula constraints
    stl = catl2stl(ast)
    ranges = {variable: (0, len(agents)) for variable in stl.variables()}
    stl_milp = pstl2milp(stl, ranges=ranges, model=m, robust=robust)
    zi = stl_milp.translate()

    #stl_milp.translate(satisfaction=False) 
    # add proposition constraints
    add_proposition_constraints(m, stl_milp, ts, ast, capabilities,
                                agent_classes, time_bound, variable_bound)

    # add travel time regularization
    if travel_time_weight > 0:
        add_travel_time_objective(m, ts, travel_time_weight, time_bound,
                                  variable_bound)

    
    method = 3

    if method == 1:
        d = stl_milp.method_1()
        obj = [stl_milp.model.getObjective(objectives) for objectives in range(d+1)]
        print(str(obj), ':', [obj[i].getValue() for i in range(d+1)], "MILP")
    elif method == 2: 
        stl_milp.method_2()
        # print('Objective')
        obj = stl_milp.model.getObjective()
        # print(str(obj), obj.getValue(), "MILP")
    elif method == 3:
        stl_milp.method_3(zi)
        # print('Objective')
        obj = stl_milp.model.getObjective()
        # print(str(obj), obj.getValue(), "MILP")
    
    

    # stl_milp.pstl2lp(ast)
    # run optimizer
    # m.optimize()    


    if m.status == GRB.Status.OPTIMAL:
        logging.info('"Optimal objective MILP": %f', m.objVal)
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
