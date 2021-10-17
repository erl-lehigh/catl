

import numpy as np
import os
from lomap import Ts


##########################################################
# File Names
##########################################################
ts_filename='../TransitionSystems/MonteCarloExample/TS'
if not os.path.exists('../TransitionSystems/MonteCarloExample'):
    os.makedirs('../TransitionSystems/MonteCarloExample')
save_filename = '../OutputFiles/MonteCarloExample/Solution'
if not os.path.exists('../OutputFiles/MonteCarloExample/'):
    os.makedirs('../OutputFiles/MonteCarloExample/')
experiment_name = '../OutputFiles/MonteCarloExample/Results'
#This reloads prior constraints or solutions.
load_old_files = False
reload_file = None #Would be a string file name





##########################################################
# Specification
##########################################################


formula = '(G[0,30] F[0,15] T(3,red,{(cap1,1),(cap2,2)}) ) && (G[0,30] F[0,15] T(3,blue,{(cap1,1),(cap3,2)}))'#' && (F[0,60] ( T(4,red,{(cap1,1),(cap2,1)}) && T(4,blue,{(cap1,1),(cap3,1)})))'


##########################################################
# Environment Options
##########################################################
edge_prob = 1.0
label_names = ['red','blue','white']
label_probs= [0.1,0.1,0.8]
edge_weight_range = [1.0,1.0]
capability_list = ['cap1','cap2','cap3']
num_capabilities_per_agent = 2
num_classes = 4


##########################################################
# Monte Carlo Options
##########################################################
verbose=True
num_trials=25
dim_list = [[3,3],[3,4]]
num_agents_per_class_list = [4,5]
#solver optionts tuple: robust (True/False), regularize (True/False), alpha (between 0 and 1), upper bound (True/False)
solver_options_list=[(False, False, 0.5,False),(True, False, 0.5,False),(True, False, 0.5,True), (False, True, 0.5,False), (True, True, 0.5,True)]
