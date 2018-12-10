"""
Multitask Linear Sum Minimization Test

@author: Zachary Serlin

"""

import matplotlib.pyplot as plt
import random
from scipy.optimize import linear_sum_assignment
import numpy as np
import time

class MLSM():
    """
    Class for Multitask Linear Sum Minimization
    """

    def __init__(self, agent, ts, task,caps,num_agents,num_regions,num_capabilities):
        """
        Setting Parameter

        agent = position of agents
        ts = transition system of allowable moves between regions
        task = desired configuration of agents - next step in trajectory

        """
        self.agent = agent
        self.ts = ts
        self.task = task
        self.caps = caps
        self.num_agents = num_agents
        self.num_regions = num_regions
        self.num_caps = num_capabilities

    def assign(self, task, path = []):
        """
        Task Assignment for next time step
        This function takes in the current agent positions, and the new task
        assignments, and returns the appended path of the agents on a region/task level
        """
        self.task = task
        to_assign = task
        num_tasks = np.sum(task)
        cost = np.ones((self.num_agents,num_tasks))
        region_idx = 0
        task_idx = 0
        exit_var = 0
        task_regions = np.zeros(num_tasks)

        for i in range(0,num_tasks):

            if to_assign[region_idx][task_idx] == 0 and exit_var == 0:
                while(to_assign[region_idx][task_idx]==0 or exit_var == 1):
                    if task_idx < self.num_caps-1 and region_idx < self.num_regions:
                        task_idx += 1
                    elif task_idx == self.num_caps-1 and region_idx < self.num_regions-1:
                        task_idx = 0
                        region_idx += 1
                    elif task_idx == self.num_caps-1 and region_idx == self.num_regions-1:
                        exit_var == 1
                        print('here')
            task_regions[i] = region_idx
            for j in range(0,self.num_agents):
                if self.ts[self.agent[j]][region_idx] == 1:
                    if self.caps[j] == task_idx:
                        cost[j][i] = 0
            if task_idx < self.num_caps-1 and region_idx < self.num_regions:
                task_idx += 1
            elif task_idx == self.num_caps-1 and region_idx < self.num_regions-1:
                task_idx = 0
                region_idx += 1

        print('cost: ',cost)
        row_ind, col_ind = linear_sum_assignment(cost)
        path = self.agent
        for i in range(0,num_tasks):
            path[row_ind[i]] = task_regions[col_ind[i]]

        cost_val = cost[row_ind, col_ind].sum()

        print('cost: ',cost_val)

        if cost_val > 0:
            path = self.agents
            print("NO SOLUTION FOUND")

        return path

def main():
    t = time.time()
    # ====Assign n agents to m tasks with transition system considerations====
    # Test_Case
    agent = [1, 1, 1, 2, 3, 3]
    ts = [[1,1,1,0],[1,1,0,1],[1,0,1,1],[0,1,1,1]]
    task = [[1,0,0],[1,1,1],[0,1,1],[0,0,0]]
    #task = [[1,0,0],[1,1,1],[0,1,1],[0,0,0]]
    caps = [0,0,1,1,2,2]

    num_agents = len(agent)
    num_regions = len(ts[0])
    num_capabilities = len(np.unique(caps))
    mlsm = MLSM(agent, ts, task,caps,num_agents,num_regions,num_capabilities)
    path = mlsm.assign(task)

    print('time: ',(time.time()-t))

    print('path: ',path)


if __name__ == '__main__':
    main()
