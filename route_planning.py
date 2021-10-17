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
from catl import extract_stl_task_formulae
from catl import stl_predicate_variables
from visualization import show_environment


resource_variable_types = {
    'divisible': GRB.CONTINUOUS,
    'packets': GRB.INTEGER,
    'indivisible': GRB.BINARY
}


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

def compute_resource_quantities(resource_distribution):
    '''Computes the initial total quantity of each resource.
    Input
    -----
    - Dictionary of resources to quantity distributions over the states of the
    transition system.  Each distribution is a dictionary from states of the TS
    to positive numbers.
    Output
    ------
    Dictionary from resources to total quantities at initial time.
    '''
    return {h: sum(distrib.values())
            for h, distrib in resource_distribution.items()}

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

def create_resource_variables(m, ts, resource_quantities, time_bound,
                              vtype=GRB.CONTINUOUS):
    '''Creates the resource state and transition variables associated with the
    given transition system.

    The resource state variables are y_{state}_{res}_k, where {state} is a node
    in the TS graph, {res} is a resource encoded as a string, and k is the time
    step.

    The resource transition variables are y_{state1}_{state2}_{res}_k, where
    {state1} and {state2} define the transition, {res} is a resource encoded as
    a string, and k is the time step.

    Input
    -----
    - The Gurobi model variable.
    - The transition system specifying the environment.
    - The resources quantities dictionary.
    - Time bound.
    - Variable type (default: real).

    Note
    ----
    Data structure holding the variables is a list of list of variables, e.g.,

        d['res'][k][h] is the y_{q/e}_h_k

    where d is the dictionary of attributes for a node q or an edge e in the TS,
    h is a resource encoded as a string, and k is the time step.
    Also, d['res'] is a list of length `time_bound+1', d['res'][k] is a
    dictionary from strings to gurobi variables.
    '''
    # node variables
    for u, d in ts.g.nodes(data=True):
        d['res'] = [] # initialize node variables list
        for k in range(time_bound+1):
            name = 'y_{state}_{{}}_{time}'.format(state=u, time=k)
            d['res'].append({h: m.addVar(vtype=vtype, name=name.format(h),
                                          lb=0, ub=quantity)
                             for h, quantity in resource_quantities.items()})
    # edge variables
    for u, v, d in ts.g.edges(data=True):
        d['res'] = [] # initialize edge variables list
        for k in range(time_bound):
            name = 'y_{src}_{dest}_{{}}_{time}'.format(src=u, dest=v, time=k)
            d['res'].append({h: m.addVar(vtype=vtype, name=name.format(h),
                                          lb=0, ub=quantity)
                             for h, quantity in resource_quantities.items()})

def extract_task_variables(ts, stl, stl_milp):
    '''TODO:
    task_stl_vars[u][h][k]

    '''
    tasks = extract_stl_task_formulae(stl)

    task_stl_vars = {}
    for u, ud in ts.g.nodes(data=True):
        tasks_w_prop = [task for task in tasks
                        if task[1].proposition in ud['prop']]
        task_stl_vars[u] = [(stl_milp.variables[stl_formula],
                             dict(task.resource_requests))
                            for stl_formula, task in tasks_w_prop]
    return task_stl_vars

def add_resource_constraints(m, ts, resource_distribution, capacities,
                             time_bound, task_stl_vars,
                             storage_type='comparmental'):
    '''Computes the constraints that capture the resource dynamics.

    Input
    -----
    - The Gurobi model variable.
    - The transition system specifying the environment.
    - The initial distribution of resources at each state.
    - The capacities of each agent class.
    - Time bound.
    - The variables associated with each task.
    - The storage type.

    Note
    ----
    The initial time constraints

        y_{state}_h_0 = \delta_{state}_h

    where \delta_{state}_h is the amount of resource h at state {state} at time
    0.

    The storage type can either be general, in which case the `capacities` maps
    agent class g and resource type h to a upper bound.  In case the storage
    type is uniform then the overall carying capacity of each agent class g is
    given and applies to across resource types.
    '''
    # edge conservation constraints
    for u, ud in ts.g.nodes(data=True):
        for k in range(time_bound):
            for h in resource_distribution:
                departing = sum([d['res'][k][h]
                            for _, _, d in ts.g.out_edges_iter(u, data=True)
                                if k + d['weight'] <= time_bound])
                # substract consumption of resources
                departing += sum([task_var.get(k, 0) * quantities.get(h, 0)
                                 for task_var, quantities in task_stl_vars[u]
                                 if h in quantities])

                arriving = sum([d['res'][k - d['weight']][h]
                                for _, _, d in ts.g.in_edges_iter(u, data=True)
                                    if k - d['weight'] >= 0])

                if 0 < k < time_bound:
                    # flow balancing constraint
                    m.addConstr(arriving == departing,
                                'conserve_{}_{}_{}'.format(u, h, k))

                # node constraint: team state
                if k < time_bound:
                    res_state_eq = (ud['res'][k][h] == departing)
                else:
                    res_state_eq = (ud['res'][k][h] == arriving)
                m.addConstr(res_state_eq, 'res_{}_{}_{}'.format(u, h, k))

    for u, v, d in ts.g.out_edges_iter(data=True):
        if u != v:
            for k in range(time_bound):
                if storage_type == 'comparmental':
                    for h in resource_distribution:
                        resource_capacity = sum(capacities[g][h] * var
                                             for g, var in d['vars'][k].items())
                        resource_bound = d['res'][k][h] <= resource_capacity
                        m.addConstr(resource_bound,
                                    'res_bound_{}_{}_{}'.format(u, h, k))
                elif storage_type == 'uniform':
                    total_resources = sum(d['res'][k][h]
                                          for h in resource_distribution)
                    total_capacity = sum(capacities[g] * var
                                         for g, var in d['vars'][k].items())
                    resource_bound = total_resources <= total_capacity
                    m.addConstr(resource_bound, 'res_bound_{}_{}'.format(u, k))

    # initial time constraints - encoding using state variables
    for u, ud in ts.g.nodes(data=True):
        for h in resource_distribution:
            conserve = (ud['res'][0][h] == resource_distribution[h].get(u, 0))
            m.addConstr(conserve, 'res_init_distrib_{}_{}'.format(u, h))

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

def compute_catl_variables_bounds(ast, variable_bound, resource_quantities):
    '''TODO:
    '''
    capability_variables, resource_variables = stl_predicate_variables(ast)
    ranges = {}
    for capability, variables in capability_variables.items():
        ranges.update({variable: (0, variable_bound) for variable in variables})
    for resource, variables in resource_variables.items():
        ranges.update({variable: (0, resource_quantities.get(resource,
                                                             float('inf')))
                       for variable in variables})
    return ranges

def add_proposition_constraints(m, stl_milp, ts, ast, capabilities,
                                agent_classes, time_bound, variable_bound,
                                vtype=GRB.CONTINUOUS):
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
    - Variable type (default: real).
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

def add_proposition_resource_constraints(m, stl_milp, ts, ast, time_bound,
                                         resource_bounds, vtype=GRB.CONTINUOUS):
    '''Adds the proposition resource constraints. First, the proposition-state
    variables are defined. Second, contraints are added such that proposition
    are satisfied as best as possible. The variables in the MILP encoding of the
    STL formula are used for the encoding as the minimizers of over
    proposition-state variables.

    Input
    -----
    - The Gurobi model variable.
    - The MILP encoding of the STL formula obtained from the CaTL specification.
    - The transition system specifying the environment.
    - The AST of the CaTL specification formula.
    - Time bound.
    - The upper bounds for resource variables.
    - Variable type (default: real).
    '''
    props = extract_propositions(ts, ast)

    # add proposition-state variables
    for u, ud in ts.g.nodes(data=True):
        ud['prop_vars'] = dict()
        for h in resource_bounds:
            ud['prop_vars'][h] = []
            for k in range(time_bound+1):
                ud['prop_vars'][h].append(dict())
                for prop in ud['prop']:
                    name = 'z_{prop}_{state}_{res}_{time}'.format(
                        prop=prop, state=u, res=h, time=k)
                    ud['prop_vars'][h][k][prop] = m.addVar(
                        vtype=vtype, name=name, lb=0, ub=resource_bounds[h])

    # constraints for relating (proposition, state) pairs to resource states
    for u, ud in ts.g.nodes(data=True):
        for h in resource_bounds:
            for k in range(time_bound+1):
                equality = sum([ud['prop_vars'][h][k][prop]
                                                    for prop in ud['prop']])
                equality -= ud['res'][k][h]
                equality = (equality == 0)
                m.addConstr(equality, 'prop_state_{}_{}_{}'.format(u, h, k))

    # add propositions constraints for only those variables appearing in the
    # MILP encoding of the formula
    for prop in props:
        for h in resource_bounds:
            variable = '{prop}_{res}'.format(prop=prop, res=h)
            for k in range(time_bound+1):
                if (variable in stl_milp.variables
                                        and k in stl_milp.variables[variable]):
                    for u, ud in ts.g.nodes(data=True):
                        if prop in ud['prop']:
                            min_prop = (stl_milp.variables[variable][k]
                                                <= ud['prop_vars'][h][k][prop])
                            m.addConstr(min_prop, 'min_prop_{}_{}_{}_{}'.format(
                                                                prop, h, k, u))

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
                   storage_type=None, capacities=None,
                   resource_distribution=None,  resource_type='divisible',
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
    - The agents' storage type (options: comparmental or uniform).
    - The resource carrying capacities of agent classes.
    - The initial distribution of resources over states of the transition
    system.
    - The resource type (options: divisible, packets or indivisible).
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
    # bounds for capability and resource variables
    if storage_type is not None:
        resource_quantities = compute_resource_quantities(resource_distribution)
    else:
        resource_quantities = dict()
    ranges = compute_catl_variables_bounds(ast, variable_bound,
                                           resource_quantities)
    stl_milp = stl2milp(stl, ranges=ranges, model=m, robust=robust)
    stl_milp.translate()

    # add proposition constraints
    add_proposition_constraints(m, stl_milp, ts, ast, capabilities,
                                agent_classes, time_bound, variable_bound)

    if storage_type is not None:
        resource_var_type = resource_variable_types[resource_type]
        create_resource_variables(m, ts, resource_quantities, time_bound,
                                  resource_var_type)
        task_stl_vars = extract_task_variables(ts, stl, stl_milp)
        add_resource_constraints(m, ts, resource_distribution, capacities,
                                 time_bound, task_stl_vars, storage_type)
        add_proposition_resource_constraints(m, stl_milp, ts, ast,
                                             time_bound, resource_quantities,
                                             resource_var_type)

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
