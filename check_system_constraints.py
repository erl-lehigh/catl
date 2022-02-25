'''
 Copyright (C) 2020 Cristian Ioan Vasile <cvasile@lehigh.edu>
 Explainable Robotics Lab (ERL), Autonomous and Intelligent Robotics (AIR) Lab,
 Lehigh University
 See license.txt file for license information.
'''

import logging
logger = logging.getLogger(__name__)
logger.addHandler(logging.NullHandler())


def check_initial_states(ts, agents):
    '''Checks if the initial states of the nodes of the environmental graph are
    satisfied in the MILP solution.  The functions assumes that the gurobipy
    model was solved, and the variables are stored in the node and edge
    attributes of the transition system graph `ts.g`.
    Input
    -----
    - The transition system specifying the environment.
    - The list of agents given as a list of pairs of locations (states), and
    capability sets.
    '''
    initial_locations, _ = zip(*agents)
    capability_distribution = {state: dict() for state in initial_locations}
    for agent in agents:
        state, agent_class = agent
        agent_class = frozenset(agent_class)
        if agent_class not in capability_distribution[state]:
            capability_distribution[state][agent_class] = 0
        capability_distribution[state][agent_class] += 1

    for node, data in ts.g.nodes(data=True):
        for agent_class, var in data['vars'][0].items():
            if (node in capability_distribution
                and agent_class in capability_distribution[node]):
                value = capability_distribution[node][agent_class]
                assert var.x == value, (node, agent_class, var, value)
            else:
                assert var.x == 0, (node, agent_class, var, 0)

def check_flow_constraints(ts, agents, time_bound):
    '''Checks if the flow constraints at each state and each time are satisfied
    in the MILP solution.  The functions assumes that the gurobipy model was
    solved, and the variables are stored in the node and edge attributes of the
    transition system graph `ts.g`.
    Input
    -----
    - The transition system specifying the environment.
    - The list of agents given as a list of pairs of locations (states), and
    capability sets.
    '''
    _, agent_classes = zip(*agents)
    agent_classes = set([frozenset(agent_class)
                         for agent_class in agent_classes])

    for t in range(time_bound+1):
        for node, node_data in ts.g.nodes(data=True):
            assert ts.g.has_edge(node, node)
            for agent_class in agent_classes:
                departing = 0
                for _, _, edge_data in ts.g.out_edges([node], data=True):
                    if t < time_bound:
                        departing += edge_data['vars'][t][agent_class].x

                arriving = 0
                for _, _, edge_data in ts.g.in_edges([node], data=True):
                    past = t - edge_data['weight']
                    if past >= 0:
                        arriving += edge_data['vars'][past][agent_class].x
                
                logger.error('"time": %d, "node": "%s", "agent_class": %s, "value": %s',
                             t, node, agent_class,
                             node_data['vars'][t][agent_class].x)
                dep = [(u, v, t, edge_data['vars'][t][agent_class].x)
                       for u, v, edge_data in ts.g.out_edges([node], data=True)
                           if t < time_bound]
                logger.error('"departing": %d, "info": %s', departing, dep)
                arv = [(u, v, t - edge_data['weight'],
                        edge_data['vars'][t - edge_data['weight']][agent_class].x)
                       for u, v, edge_data in ts.g.in_edges([node], data=True)
                           if t - edge_data['weight'] >= 0]
                logger.error('"arriving": %d, "info": %s', arriving, arv)

                if 0 < t < time_bound:
                    assert departing == arriving
   