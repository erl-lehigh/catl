\ Model milp
\ LP format - for model browsing. Use MPS format to capture full model detail.
Maximize
  rho0 + 0.5 rho1
Subject To
 team_q1_1_0: z_q1_1_0 - z_q1_q1_1_0 - z_q1_q3_1_0 - z_q1_q2_1_0 = 0
 conserve_q1_1_1: - z_q1_q1_1_0 + z_q1_q1_1_1 + z_q1_q3_1_1 + z_q1_q2_1_1
   - z_q3_q1_1_0 - z_q2_q1_1_0 = 0
 team_q1_1_1: z_q1_1_1 - z_q1_q1_1_1 - z_q1_q3_1_1 - z_q1_q2_1_1 = 0
 conserve_q1_1_2: - z_q1_q1_1_1 + z_q1_q1_1_2 + z_q1_q3_1_2 + z_q1_q2_1_2
   - z_q3_q1_1_1 - z_q2_q1_1_1 = 0
 team_q1_1_2: z_q1_1_2 - z_q1_q1_1_2 - z_q1_q3_1_2 - z_q1_q2_1_2 = 0
 conserve_q1_1_3: - z_q1_q1_1_2 + z_q1_q1_1_3 + z_q1_q3_1_3 + z_q1_q2_1_3
   - z_q3_q1_1_2 - z_q2_q1_1_2 = 0
 team_q1_1_3: z_q1_1_3 - z_q1_q1_1_3 - z_q1_q3_1_3 - z_q1_q2_1_3 = 0
 team_q1_1_4: z_q1_1_4 - z_q1_q1_1_3 - z_q3_q1_1_3 - z_q2_q1_1_3 = 0
 team_q3_1_0: z_q3_1_0 - z_q3_q1_1_0 - z_q3_q3_1_0 - z_q3_q4_1_0 = 0
 conserve_q3_1_1: - z_q1_q3_1_0 + z_q3_q1_1_1 - z_q3_q3_1_0 + z_q3_q3_1_1
   + z_q3_q4_1_1 - z_q4_q3_1_0 = 0
 team_q3_1_1: z_q3_1_1 - z_q3_q1_1_1 - z_q3_q3_1_1 - z_q3_q4_1_1 = 0
 conserve_q3_1_2: - z_q1_q3_1_1 + z_q3_q1_1_2 - z_q3_q3_1_1 + z_q3_q3_1_2
   + z_q3_q4_1_2 - z_q4_q3_1_1 = 0
 team_q3_1_2: z_q3_1_2 - z_q3_q1_1_2 - z_q3_q3_1_2 - z_q3_q4_1_2 = 0
 conserve_q3_1_3: - z_q1_q3_1_2 + z_q3_q1_1_3 - z_q3_q3_1_2 + z_q3_q3_1_3
   + z_q3_q4_1_3 - z_q4_q3_1_2 = 0
 team_q3_1_3: z_q3_1_3 - z_q3_q1_1_3 - z_q3_q3_1_3 - z_q3_q4_1_3 = 0
 team_q3_1_4: z_q3_1_4 - z_q1_q3_1_3 - z_q3_q3_1_3 - z_q4_q3_1_3 = 0
 team_q2_1_0: z_q2_1_0 - z_q2_q1_1_0 - z_q2_q2_1_0 - z_q2_q4_1_0 = 0
 conserve_q2_1_1: - z_q1_q2_1_0 + z_q2_q1_1_1 - z_q2_q2_1_0 + z_q2_q2_1_1
   + z_q2_q4_1_1 - z_q4_q2_1_0 = 0
 team_q2_1_1: z_q2_1_1 - z_q2_q1_1_1 - z_q2_q2_1_1 - z_q2_q4_1_1 = 0
 conserve_q2_1_2: - z_q1_q2_1_1 + z_q2_q1_1_2 - z_q2_q2_1_1 + z_q2_q2_1_2
   + z_q2_q4_1_2 - z_q4_q2_1_1 = 0
 team_q2_1_2: z_q2_1_2 - z_q2_q1_1_2 - z_q2_q2_1_2 - z_q2_q4_1_2 = 0
 conserve_q2_1_3: - z_q1_q2_1_2 + z_q2_q1_1_3 - z_q2_q2_1_2 + z_q2_q2_1_3
   + z_q2_q4_1_3 - z_q4_q2_1_2 = 0
 team_q2_1_3: z_q2_1_3 - z_q2_q1_1_3 - z_q2_q2_1_3 - z_q2_q4_1_3 = 0
 team_q2_1_4: z_q2_1_4 - z_q1_q2_1_3 - z_q2_q2_1_3 - z_q4_q2_1_3 = 0
 team_q4_1_0: z_q4_1_0 - z_q4_q3_1_0 - z_q4_q2_1_0 - z_q4_q4_1_0 = 0
 conserve_q4_1_1: - z_q3_q4_1_0 - z_q2_q4_1_0 + z_q4_q3_1_1 + z_q4_q2_1_1
   - z_q4_q4_1_0 + z_q4_q4_1_1 = 0
 team_q4_1_1: z_q4_1_1 - z_q4_q3_1_1 - z_q4_q2_1_1 - z_q4_q4_1_1 = 0
 conserve_q4_1_2: - z_q3_q4_1_1 - z_q2_q4_1_1 + z_q4_q3_1_2 + z_q4_q2_1_2
   - z_q4_q4_1_1 + z_q4_q4_1_2 = 0
 team_q4_1_2: z_q4_1_2 - z_q4_q3_1_2 - z_q4_q2_1_2 - z_q4_q4_1_2 = 0
 conserve_q4_1_3: - z_q3_q4_1_2 - z_q2_q4_1_2 + z_q4_q3_1_3 + z_q4_q2_1_3
   - z_q4_q4_1_2 + z_q4_q4_1_3 = 0
 team_q4_1_3: z_q4_1_3 - z_q4_q3_1_3 - z_q4_q2_1_3 - z_q4_q4_1_3 = 0
 team_q4_1_4: z_q4_1_4 - z_q3_q4_1_3 - z_q2_q4_1_3 - z_q4_q4_1_3 = 0
 init_distrib_q1_1: z_q1_1_0 = 2
 init_distrib_q3_1: z_q3_1_0 = 0
 init_distrib_q2_1: z_q2_1_0 = 0
 init_distrib_q4_1: z_q4_1_0 = 0
 R36: - rho0 - 1000 predicate_2dx2de4eb3ae5fa39f0_0 + green_a_0 <= 1
 R37: - rho0 - 1000 predicate_2dx2de4eb3ae5fa39f0_0 + green_a_0 >= -999
 R38: and_2dx80e20ade56d99f5_0 - predicate_2dx2de4eb3ae5fa39f0_0 <= 0
 R39: and_2dx80e20ade56d99f5_0 - predicate_2dx2de4eb3ae5fa39f0_0 >= 0
 R40: - rho0 - 1000 predicate_2dx2de4eb3ae5fa39f0_1 + green_a_1 <= 1
 R41: - rho0 - 1000 predicate_2dx2de4eb3ae5fa39f0_1 + green_a_1 >= -999
 R42: and_2dx80e20ade56d99f5_1 - predicate_2dx2de4eb3ae5fa39f0_1 <= 0
 R43: and_2dx80e20ade56d99f5_1 - predicate_2dx2de4eb3ae5fa39f0_1 >= 0
 R44: - rho0 - 1000 predicate_2dx2de4eb3ae5fa39f0_2 + green_a_2 <= 1
 R45: - rho0 - 1000 predicate_2dx2de4eb3ae5fa39f0_2 + green_a_2 >= -999
 R46: and_2dx80e20ade56d99f5_2 - predicate_2dx2de4eb3ae5fa39f0_2 <= 0
 R47: and_2dx80e20ade56d99f5_2 - predicate_2dx2de4eb3ae5fa39f0_2 >= 0
 R48: always_2dx61a27e0667b35d05_0 - and_2dx80e20ade56d99f5_0 <= 0
 R49: always_2dx61a27e0667b35d05_0 - and_2dx80e20ade56d99f5_1 <= 0
 R50: always_2dx61a27e0667b35d05_0 - and_2dx80e20ade56d99f5_2 <= 0
 R51: always_2dx61a27e0667b35d05_0 - and_2dx80e20ade56d99f5_0
   - and_2dx80e20ade56d99f5_1 - and_2dx80e20ade56d99f5_2 >= -2
 R52: - rho1 - 1000 predicate_2bx5985614b5977fb2b_0 + green_r2_0 <= 1.5
 R53: - rho1 - 1000 predicate_2bx5985614b5977fb2b_0 + green_r2_0 >= -998.5
 R54: - rho1 - 1000 predicate_2bx31f83dca6953bbc0_0 + green_r1_0 <= 1.5
 R55: - rho1 - 1000 predicate_2bx31f83dca6953bbc0_0 + green_r1_0 >= -998.5
 R56: and_2bx6329f7ce2399b141_0 - always_2dx61a27e0667b35d05_0 <= 0
 R57: and_2bx6329f7ce2399b141_0 - predicate_2bx5985614b5977fb2b_0 <= 0
 R58: and_2bx6329f7ce2399b141_0 - predicate_2bx31f83dca6953bbc0_0 <= 0
 R59: and_2bx6329f7ce2399b141_0 - always_2dx61a27e0667b35d05_0
   - predicate_2bx5985614b5977fb2b_0 - predicate_2bx31f83dca6953bbc0_0
   >= -2
 R60: - rho0 - 1000 predicate_2dx2de4eb3ae5fa39f0_3 + green_a_3 <= 1
 R61: - rho0 - 1000 predicate_2dx2de4eb3ae5fa39f0_3 + green_a_3 >= -999
 R62: and_2dx80e20ade56d99f5_3 - predicate_2dx2de4eb3ae5fa39f0_3 <= 0
 R63: and_2dx80e20ade56d99f5_3 - predicate_2dx2de4eb3ae5fa39f0_3 >= 0
 R64: - and_2dx80e20ade56d99f5_1 + always_2dx61a27e0667b35d05_1 <= 0
 R65: - and_2dx80e20ade56d99f5_2 + always_2dx61a27e0667b35d05_1 <= 0
 R66: always_2dx61a27e0667b35d05_1 - and_2dx80e20ade56d99f5_3 <= 0
 R67: - and_2dx80e20ade56d99f5_1 - and_2dx80e20ade56d99f5_2
   + always_2dx61a27e0667b35d05_1 - and_2dx80e20ade56d99f5_3 >= -2
 R68: - rho1 - 1000 predicate_2bx5985614b5977fb2b_1 + green_r2_1 <= 1.5
 R69: - rho1 - 1000 predicate_2bx5985614b5977fb2b_1 + green_r2_1 >= -998.5
 R70: - rho1 - 1000 predicate_2bx31f83dca6953bbc0_1 + green_r1_1 <= 1.5
 R71: - rho1 - 1000 predicate_2bx31f83dca6953bbc0_1 + green_r1_1 >= -998.5
 R72: and_2bx6329f7ce2399b141_1 - always_2dx61a27e0667b35d05_1 <= 0
 R73: and_2bx6329f7ce2399b141_1 - predicate_2bx5985614b5977fb2b_1 <= 0
 R74: and_2bx6329f7ce2399b141_1 - predicate_2bx31f83dca6953bbc0_1 <= 0
 R75: and_2bx6329f7ce2399b141_1 - always_2dx61a27e0667b35d05_1
   - predicate_2bx5985614b5977fb2b_1 - predicate_2bx31f83dca6953bbc0_1
   >= -2
 R76: - rho0 - 1000 predicate_2dx2de4eb3ae5fa39f0_4 + green_a_4 <= 1
 R77: - rho0 - 1000 predicate_2dx2de4eb3ae5fa39f0_4 + green_a_4 >= -999
 R78: and_2dx80e20ade56d99f5_4 - predicate_2dx2de4eb3ae5fa39f0_4 <= 0
 R79: and_2dx80e20ade56d99f5_4 - predicate_2dx2de4eb3ae5fa39f0_4 >= 0
 R80: - and_2dx80e20ade56d99f5_2 + always_2dx61a27e0667b35d05_2 <= 0
 R81: - and_2dx80e20ade56d99f5_3 + always_2dx61a27e0667b35d05_2 <= 0
 R82: always_2dx61a27e0667b35d05_2 - and_2dx80e20ade56d99f5_4 <= 0
 R83: - and_2dx80e20ade56d99f5_2 - and_2dx80e20ade56d99f5_3
   + always_2dx61a27e0667b35d05_2 - and_2dx80e20ade56d99f5_4 >= -2
 R84: - rho1 - 1000 predicate_2bx5985614b5977fb2b_2 + green_r2_2 <= 1.5
 R85: - rho1 - 1000 predicate_2bx5985614b5977fb2b_2 + green_r2_2 >= -998.5
 R86: - rho1 - 1000 predicate_2bx31f83dca6953bbc0_2 + green_r1_2 <= 1.5
 R87: - rho1 - 1000 predicate_2bx31f83dca6953bbc0_2 + green_r1_2 >= -998.5
 R88: and_2bx6329f7ce2399b141_2 - always_2dx61a27e0667b35d05_2 <= 0
 R89: and_2bx6329f7ce2399b141_2 - predicate_2bx5985614b5977fb2b_2 <= 0
 R90: and_2bx6329f7ce2399b141_2 - predicate_2bx31f83dca6953bbc0_2 <= 0
 R91: and_2bx6329f7ce2399b141_2 - always_2dx61a27e0667b35d05_2
   - predicate_2bx5985614b5977fb2b_2 - predicate_2bx31f83dca6953bbc0_2
   >= -2
 R92: event_2dx15bd35de9a722238_0 - and_2bx6329f7ce2399b141_0 >= 0
 R93: event_2dx15bd35de9a722238_0 - and_2bx6329f7ce2399b141_1 >= 0
 R94: event_2dx15bd35de9a722238_0 - and_2bx6329f7ce2399b141_2 >= 0
 R95: event_2dx15bd35de9a722238_0 - and_2bx6329f7ce2399b141_0
   - and_2bx6329f7ce2399b141_1 - and_2bx6329f7ce2399b141_2 <= 0
 formula_satisfaction: event_2dx15bd35de9a722238_0 = 1
 prop_state_q1_a_0: - z_q1_1_0 + z_blue_q1_a_0 = 0
 prop_state_q1_a_1: - z_q1_1_1 + z_blue_q1_a_1 = 0
 prop_state_q1_a_2: - z_q1_1_2 + z_blue_q1_a_2 = 0
 prop_state_q1_a_3: - z_q1_1_3 + z_blue_q1_a_3 = 0
 prop_state_q1_a_4: - z_q1_1_4 + z_blue_q1_a_4 = 0
 prop_state_q3_a_0: - z_q3_1_0 + z_red_q3_a_0 = 0
 prop_state_q3_a_1: - z_q3_1_1 + z_red_q3_a_1 = 0
 prop_state_q3_a_2: - z_q3_1_2 + z_red_q3_a_2 = 0
 prop_state_q3_a_3: - z_q3_1_3 + z_red_q3_a_3 = 0
 prop_state_q3_a_4: - z_q3_1_4 + z_red_q3_a_4 = 0
 prop_state_q2_a_0: - z_q2_1_0 + z_orange_q2_a_0 = 0
 prop_state_q2_a_1: - z_q2_1_1 + z_orange_q2_a_1 = 0
 prop_state_q2_a_2: - z_q2_1_2 + z_orange_q2_a_2 = 0
 prop_state_q2_a_3: - z_q2_1_3 + z_orange_q2_a_3 = 0
 prop_state_q2_a_4: - z_q2_1_4 + z_orange_q2_a_4 = 0
 prop_state_q4_a_0: - z_q4_1_0 + z_green_q4_a_0 = 0
 prop_state_q4_a_1: - z_q4_1_1 + z_green_q4_a_1 = 0
 prop_state_q4_a_2: - z_q4_1_2 + z_green_q4_a_2 = 0
 prop_state_q4_a_3: - z_q4_1_3 + z_green_q4_a_3 = 0
 prop_state_q4_a_4: - z_q4_1_4 + z_green_q4_a_4 = 0
 min_prop_green_a_0_q4: green_a_0 - z_green_q4_a_0 <= 0
 min_prop_green_a_1_q4: green_a_1 - z_green_q4_a_1 <= 0
 min_prop_green_a_2_q4: green_a_2 - z_green_q4_a_2 <= 0
 min_prop_green_a_3_q4: green_a_3 - z_green_q4_a_3 <= 0
 min_prop_green_a_4_q4: green_a_4 - z_green_q4_a_4 <= 0
 res_q1_r1_0: y_q1_r1_0 - y_q1_q1_r1_0 - y_q1_q3_r1_0 - y_q1_q2_r1_0 = 0
 res_q1_r2_0: y_q1_r2_0 - y_q1_q1_r2_0 - y_q1_q3_r2_0 - y_q1_q2_r2_0 = 0
 conserve_q1_r1_1: y_q1_q1_r1_0 - y_q1_q1_r1_1 - y_q1_q3_r1_1
   - y_q1_q2_r1_1 + y_q3_q1_r1_0 + y_q2_q1_r1_0 = 0
 res_q1_r1_1: y_q1_r1_1 - y_q1_q1_r1_1 - y_q1_q3_r1_1 - y_q1_q2_r1_1 = 0
 conserve_q1_r2_1: y_q1_q1_r2_0 - y_q1_q1_r2_1 - y_q1_q3_r2_1
   - y_q1_q2_r2_1 + y_q3_q1_r2_0 + y_q2_q1_r2_0 = 0
 res_q1_r2_1: y_q1_r2_1 - y_q1_q1_r2_1 - y_q1_q3_r2_1 - y_q1_q2_r2_1 = 0
 conserve_q1_r1_2: y_q1_q1_r1_1 - y_q1_q1_r1_2 - y_q1_q3_r1_2
   - y_q1_q2_r1_2 + y_q3_q1_r1_1 + y_q2_q1_r1_1 = 0
 res_q1_r1_2: y_q1_r1_2 - y_q1_q1_r1_2 - y_q1_q3_r1_2 - y_q1_q2_r1_2 = 0
 conserve_q1_r2_2: y_q1_q1_r2_1 - y_q1_q1_r2_2 - y_q1_q3_r2_2
   - y_q1_q2_r2_2 + y_q3_q1_r2_1 + y_q2_q1_r2_1 = 0
 res_q1_r2_2: y_q1_r2_2 - y_q1_q1_r2_2 - y_q1_q3_r2_2 - y_q1_q2_r2_2 = 0
 conserve_q1_r1_3: y_q1_q1_r1_2 - y_q1_q1_r1_3 - y_q1_q3_r1_3
   - y_q1_q2_r1_3 + y_q3_q1_r1_2 + y_q2_q1_r1_2 = 0
 res_q1_r1_3: y_q1_r1_3 - y_q1_q1_r1_3 - y_q1_q3_r1_3 - y_q1_q2_r1_3 = 0
 conserve_q1_r2_3: y_q1_q1_r2_2 - y_q1_q1_r2_3 - y_q1_q3_r2_3
   - y_q1_q2_r2_3 + y_q3_q1_r2_2 + y_q2_q1_r2_2 = 0
 res_q1_r2_3: y_q1_r2_3 - y_q1_q1_r2_3 - y_q1_q3_r2_3 - y_q1_q2_r2_3 = 0
 res_q3_r1_0: y_q3_r1_0 - y_q3_q1_r1_0 - y_q3_q3_r1_0 - y_q3_q4_r1_0 = 0
 res_q3_r2_0: y_q3_r2_0 - y_q3_q1_r2_0 - y_q3_q3_r2_0 - y_q3_q4_r2_0 = 0
 conserve_q3_r1_1: y_q1_q3_r1_0 - y_q3_q1_r1_1 + y_q3_q3_r1_0
   - y_q3_q3_r1_1 - y_q3_q4_r1_1 + y_q4_q3_r1_0 = 0
 res_q3_r1_1: y_q3_r1_1 - y_q3_q1_r1_1 - y_q3_q3_r1_1 - y_q3_q4_r1_1 = 0
 conserve_q3_r2_1: y_q1_q3_r2_0 - y_q3_q1_r2_1 + y_q3_q3_r2_0
   - y_q3_q3_r2_1 - y_q3_q4_r2_1 + y_q4_q3_r2_0 = 0
 res_q3_r2_1: y_q3_r2_1 - y_q3_q1_r2_1 - y_q3_q3_r2_1 - y_q3_q4_r2_1 = 0
 conserve_q3_r1_2: y_q1_q3_r1_1 - y_q3_q1_r1_2 + y_q3_q3_r1_1
   - y_q3_q3_r1_2 - y_q3_q4_r1_2 + y_q4_q3_r1_1 = 0
 res_q3_r1_2: y_q3_r1_2 - y_q3_q1_r1_2 - y_q3_q3_r1_2 - y_q3_q4_r1_2 = 0
 conserve_q3_r2_2: y_q1_q3_r2_1 - y_q3_q1_r2_2 + y_q3_q3_r2_1
   - y_q3_q3_r2_2 - y_q3_q4_r2_2 + y_q4_q3_r2_1 = 0
 res_q3_r2_2: y_q3_r2_2 - y_q3_q1_r2_2 - y_q3_q3_r2_2 - y_q3_q4_r2_2 = 0
 conserve_q3_r1_3: y_q1_q3_r1_2 - y_q3_q1_r1_3 + y_q3_q3_r1_2
   - y_q3_q3_r1_3 - y_q3_q4_r1_3 + y_q4_q3_r1_2 = 0
 res_q3_r1_3: y_q3_r1_3 - y_q3_q1_r1_3 - y_q3_q3_r1_3 - y_q3_q4_r1_3 = 0
 conserve_q3_r2_3: y_q1_q3_r2_2 - y_q3_q1_r2_3 + y_q3_q3_r2_2
   - y_q3_q3_r2_3 - y_q3_q4_r2_3 + y_q4_q3_r2_2 = 0
 res_q3_r2_3: y_q3_r2_3 - y_q3_q1_r2_3 - y_q3_q3_r2_3 - y_q3_q4_r2_3 = 0
 res_q2_r1_0: y_q2_r1_0 - y_q2_q1_r1_0 - y_q2_q2_r1_0 - y_q2_q4_r1_0 = 0
 res_q2_r2_0: y_q2_r2_0 - y_q2_q1_r2_0 - y_q2_q2_r2_0 - y_q2_q4_r2_0 = 0
 conserve_q2_r1_1: y_q1_q2_r1_0 - y_q2_q1_r1_1 + y_q2_q2_r1_0
   - y_q2_q2_r1_1 - y_q2_q4_r1_1 + y_q4_q2_r1_0 = 0
 res_q2_r1_1: y_q2_r1_1 - y_q2_q1_r1_1 - y_q2_q2_r1_1 - y_q2_q4_r1_1 = 0
 conserve_q2_r2_1: y_q1_q2_r2_0 - y_q2_q1_r2_1 + y_q2_q2_r2_0
   - y_q2_q2_r2_1 - y_q2_q4_r2_1 + y_q4_q2_r2_0 = 0
 res_q2_r2_1: y_q2_r2_1 - y_q2_q1_r2_1 - y_q2_q2_r2_1 - y_q2_q4_r2_1 = 0
 conserve_q2_r1_2: y_q1_q2_r1_1 - y_q2_q1_r1_2 + y_q2_q2_r1_1
   - y_q2_q2_r1_2 - y_q2_q4_r1_2 + y_q4_q2_r1_1 = 0
 res_q2_r1_2: y_q2_r1_2 - y_q2_q1_r1_2 - y_q2_q2_r1_2 - y_q2_q4_r1_2 = 0
 conserve_q2_r2_2: y_q1_q2_r2_1 - y_q2_q1_r2_2 + y_q2_q2_r2_1
   - y_q2_q2_r2_2 - y_q2_q4_r2_2 + y_q4_q2_r2_1 = 0
 res_q2_r2_2: y_q2_r2_2 - y_q2_q1_r2_2 - y_q2_q2_r2_2 - y_q2_q4_r2_2 = 0
 conserve_q2_r1_3: y_q1_q2_r1_2 - y_q2_q1_r1_3 + y_q2_q2_r1_2
   - y_q2_q2_r1_3 - y_q2_q4_r1_3 + y_q4_q2_r1_2 = 0
 res_q2_r1_3: y_q2_r1_3 - y_q2_q1_r1_3 - y_q2_q2_r1_3 - y_q2_q4_r1_3 = 0
 conserve_q2_r2_3: y_q1_q2_r2_2 - y_q2_q1_r2_3 + y_q2_q2_r2_2
   - y_q2_q2_r2_3 - y_q2_q4_r2_3 + y_q4_q2_r2_2 = 0
 res_q2_r2_3: y_q2_r2_3 - y_q2_q1_r2_3 - y_q2_q2_r2_3 - y_q2_q4_r2_3 = 0
 res_q4_r1_0: - 1.5 and_2bx6329f7ce2399b141_0 + y_q4_r1_0 - y_q4_q3_r1_0
   - y_q4_q2_r1_0 - y_q4_q4_r1_0 = 0
 res_q4_r2_0: - 1.5 and_2bx6329f7ce2399b141_0 + y_q4_r2_0 - y_q4_q3_r2_0
   - y_q4_q2_r2_0 - y_q4_q4_r2_0 = 0
 conserve_q4_r1_1: - 1.5 and_2bx6329f7ce2399b141_1 + y_q3_q4_r1_0
   + y_q2_q4_r1_0 - y_q4_q3_r1_1 - y_q4_q2_r1_1 + y_q4_q4_r1_0
   - y_q4_q4_r1_1 = 0
 res_q4_r1_1: - 1.5 and_2bx6329f7ce2399b141_1 + y_q4_r1_1 - y_q4_q3_r1_1
   - y_q4_q2_r1_1 - y_q4_q4_r1_1 = 0
 conserve_q4_r2_1: - 1.5 and_2bx6329f7ce2399b141_1 + y_q3_q4_r2_0
   + y_q2_q4_r2_0 - y_q4_q3_r2_1 - y_q4_q2_r2_1 + y_q4_q4_r2_0
   - y_q4_q4_r2_1 = 0
 res_q4_r2_1: - 1.5 and_2bx6329f7ce2399b141_1 + y_q4_r2_1 - y_q4_q3_r2_1
   - y_q4_q2_r2_1 - y_q4_q4_r2_1 = 0
 conserve_q4_r1_2: - 1.5 and_2bx6329f7ce2399b141_2 + y_q3_q4_r1_1
   + y_q2_q4_r1_1 - y_q4_q3_r1_2 - y_q4_q2_r1_2 + y_q4_q4_r1_1
   - y_q4_q4_r1_2 = 0
 res_q4_r1_2: - 1.5 and_2bx6329f7ce2399b141_2 + y_q4_r1_2 - y_q4_q3_r1_2
   - y_q4_q2_r1_2 - y_q4_q4_r1_2 = 0
 conserve_q4_r2_2: - 1.5 and_2bx6329f7ce2399b141_2 + y_q3_q4_r2_1
   + y_q2_q4_r2_1 - y_q4_q3_r2_2 - y_q4_q2_r2_2 + y_q4_q4_r2_1
   - y_q4_q4_r2_2 = 0
 res_q4_r2_2: - 1.5 and_2bx6329f7ce2399b141_2 + y_q4_r2_2 - y_q4_q3_r2_2
   - y_q4_q2_r2_2 - y_q4_q4_r2_2 = 0
 conserve_q4_r1_3: y_q3_q4_r1_2 + y_q2_q4_r1_2 - y_q4_q3_r1_3
   - y_q4_q2_r1_3 + y_q4_q4_r1_2 - y_q4_q4_r1_3 = 0
 res_q4_r1_3: y_q4_r1_3 - y_q4_q3_r1_3 - y_q4_q2_r1_3 - y_q4_q4_r1_3 = 0
 conserve_q4_r2_3: y_q3_q4_r2_2 + y_q2_q4_r2_2 - y_q4_q3_r2_3
   - y_q4_q2_r2_3 + y_q4_q4_r2_2 - y_q4_q4_r2_3 = 0
 res_q4_r2_3: y_q4_r2_3 - y_q4_q3_r2_3 - y_q4_q2_r2_3 - y_q4_q4_r2_3 = 0
 res_bound_q1_r1_0: - 2 z_q1_q3_1_0 + y_q1_q3_r1_0 <= 0
 res_bound_q1_r2_0: - 2 z_q1_q3_1_0 + y_q1_q3_r2_0 <= 0
 res_bound_q1_r1_1: - 2 z_q1_q3_1_1 + y_q1_q3_r1_1 <= 0
 res_bound_q1_r2_1: - 2 z_q1_q3_1_1 + y_q1_q3_r2_1 <= 0
 res_bound_q1_r1_2: - 2 z_q1_q3_1_2 + y_q1_q3_r1_2 <= 0
 res_bound_q1_r2_2: - 2 z_q1_q3_1_2 + y_q1_q3_r2_2 <= 0
 res_bound_q1_r1_3: - 2 z_q1_q3_1_3 + y_q1_q3_r1_3 <= 0
 res_bound_q1_r2_3: - 2 z_q1_q3_1_3 + y_q1_q3_r2_3 <= 0
 res_bound_q1_r1_0: - 2 z_q1_q2_1_0 + y_q1_q2_r1_0 <= 0
 res_bound_q1_r2_0: - 2 z_q1_q2_1_0 + y_q1_q2_r2_0 <= 0
 res_bound_q1_r1_1: - 2 z_q1_q2_1_1 + y_q1_q2_r1_1 <= 0
 res_bound_q1_r2_1: - 2 z_q1_q2_1_1 + y_q1_q2_r2_1 <= 0
 res_bound_q1_r1_2: - 2 z_q1_q2_1_2 + y_q1_q2_r1_2 <= 0
 res_bound_q1_r2_2: - 2 z_q1_q2_1_2 + y_q1_q2_r2_2 <= 0
 res_bound_q1_r1_3: - 2 z_q1_q2_1_3 + y_q1_q2_r1_3 <= 0
 res_bound_q1_r2_3: - 2 z_q1_q2_1_3 + y_q1_q2_r2_3 <= 0
 res_bound_q3_r1_0: - 2 z_q3_q1_1_0 + y_q3_q1_r1_0 <= 0
 res_bound_q3_r2_0: - 2 z_q3_q1_1_0 + y_q3_q1_r2_0 <= 0
 res_bound_q3_r1_1: - 2 z_q3_q1_1_1 + y_q3_q1_r1_1 <= 0
 res_bound_q3_r2_1: - 2 z_q3_q1_1_1 + y_q3_q1_r2_1 <= 0
 res_bound_q3_r1_2: - 2 z_q3_q1_1_2 + y_q3_q1_r1_2 <= 0
 res_bound_q3_r2_2: - 2 z_q3_q1_1_2 + y_q3_q1_r2_2 <= 0
 res_bound_q3_r1_3: - 2 z_q3_q1_1_3 + y_q3_q1_r1_3 <= 0
 res_bound_q3_r2_3: - 2 z_q3_q1_1_3 + y_q3_q1_r2_3 <= 0
 res_bound_q3_r1_0: - 2 z_q3_q4_1_0 + y_q3_q4_r1_0 <= 0
 res_bound_q3_r2_0: - 2 z_q3_q4_1_0 + y_q3_q4_r2_0 <= 0
 res_bound_q3_r1_1: - 2 z_q3_q4_1_1 + y_q3_q4_r1_1 <= 0
 res_bound_q3_r2_1: - 2 z_q3_q4_1_1 + y_q3_q4_r2_1 <= 0
 res_bound_q3_r1_2: - 2 z_q3_q4_1_2 + y_q3_q4_r1_2 <= 0
 res_bound_q3_r2_2: - 2 z_q3_q4_1_2 + y_q3_q4_r2_2 <= 0
 res_bound_q3_r1_3: - 2 z_q3_q4_1_3 + y_q3_q4_r1_3 <= 0
 res_bound_q3_r2_3: - 2 z_q3_q4_1_3 + y_q3_q4_r2_3 <= 0
 res_bound_q2_r1_0: - 2 z_q2_q1_1_0 + y_q2_q1_r1_0 <= 0
 res_bound_q2_r2_0: - 2 z_q2_q1_1_0 + y_q2_q1_r2_0 <= 0
 res_bound_q2_r1_1: - 2 z_q2_q1_1_1 + y_q2_q1_r1_1 <= 0
 res_bound_q2_r2_1: - 2 z_q2_q1_1_1 + y_q2_q1_r2_1 <= 0
 res_bound_q2_r1_2: - 2 z_q2_q1_1_2 + y_q2_q1_r1_2 <= 0
 res_bound_q2_r2_2: - 2 z_q2_q1_1_2 + y_q2_q1_r2_2 <= 0
 res_bound_q2_r1_3: - 2 z_q2_q1_1_3 + y_q2_q1_r1_3 <= 0
 res_bound_q2_r2_3: - 2 z_q2_q1_1_3 + y_q2_q1_r2_3 <= 0
 res_bound_q2_r1_0: - 2 z_q2_q4_1_0 + y_q2_q4_r1_0 <= 0
 res_bound_q2_r2_0: - 2 z_q2_q4_1_0 + y_q2_q4_r2_0 <= 0
 res_bound_q2_r1_1: - 2 z_q2_q4_1_1 + y_q2_q4_r1_1 <= 0
 res_bound_q2_r2_1: - 2 z_q2_q4_1_1 + y_q2_q4_r2_1 <= 0
 res_bound_q2_r1_2: - 2 z_q2_q4_1_2 + y_q2_q4_r1_2 <= 0
 res_bound_q2_r2_2: - 2 z_q2_q4_1_2 + y_q2_q4_r2_2 <= 0
 res_bound_q2_r1_3: - 2 z_q2_q4_1_3 + y_q2_q4_r1_3 <= 0
 res_bound_q2_r2_3: - 2 z_q2_q4_1_3 + y_q2_q4_r2_3 <= 0
 res_bound_q4_r1_0: - 2 z_q4_q3_1_0 + y_q4_q3_r1_0 <= 0
 res_bound_q4_r2_0: - 2 z_q4_q3_1_0 + y_q4_q3_r2_0 <= 0
 res_bound_q4_r1_1: - 2 z_q4_q3_1_1 + y_q4_q3_r1_1 <= 0
 res_bound_q4_r2_1: - 2 z_q4_q3_1_1 + y_q4_q3_r2_1 <= 0
 res_bound_q4_r1_2: - 2 z_q4_q3_1_2 + y_q4_q3_r1_2 <= 0
 res_bound_q4_r2_2: - 2 z_q4_q3_1_2 + y_q4_q3_r2_2 <= 0
 res_bound_q4_r1_3: - 2 z_q4_q3_1_3 + y_q4_q3_r1_3 <= 0
 res_bound_q4_r2_3: - 2 z_q4_q3_1_3 + y_q4_q3_r2_3 <= 0
 res_bound_q4_r1_0: - 2 z_q4_q2_1_0 + y_q4_q2_r1_0 <= 0
 res_bound_q4_r2_0: - 2 z_q4_q2_1_0 + y_q4_q2_r2_0 <= 0
 res_bound_q4_r1_1: - 2 z_q4_q2_1_1 + y_q4_q2_r1_1 <= 0
 res_bound_q4_r2_1: - 2 z_q4_q2_1_1 + y_q4_q2_r2_1 <= 0
 res_bound_q4_r1_2: - 2 z_q4_q2_1_2 + y_q4_q2_r1_2 <= 0
 res_bound_q4_r2_2: - 2 z_q4_q2_1_2 + y_q4_q2_r2_2 <= 0
 res_bound_q4_r1_3: - 2 z_q4_q2_1_3 + y_q4_q2_r1_3 <= 0
 res_bound_q4_r2_3: - 2 z_q4_q2_1_3 + y_q4_q2_r2_3 <= 0
 res_init_distrib_q1_r1: y_q1_r1_0 = 0
 res_init_distrib_q1_r2: y_q1_r2_0 = 0
 res_init_distrib_q3_r1: y_q3_r1_0 = 0
 res_init_distrib_q3_r2: y_q3_r2_0 = 2.2
 res_init_distrib_q2_r1: y_q2_r1_0 = 2.1
 res_init_distrib_q2_r2: y_q2_r2_0 = 0
 res_init_distrib_q4_r1: y_q4_r1_0 = 1.2
 res_init_distrib_q4_r2: y_q4_r2_0 = 0
 prop_state_q1_r1_0: - y_q1_r1_0 + z_blue_q1_r1_0 = 0
 prop_state_q1_r1_1: - y_q1_r1_1 + z_blue_q1_r1_1 = 0
 prop_state_q1_r1_2: - y_q1_r1_2 + z_blue_q1_r1_2 = 0
 prop_state_q1_r1_3: - y_q1_r1_3 + z_blue_q1_r1_3 = 0
 prop_state_q1_r1_4: - y_q1_r1_4 + z_blue_q1_r1_4 = 0
 prop_state_q1_r2_0: - y_q1_r2_0 + z_blue_q1_r2_0 = 0
 prop_state_q1_r2_1: - y_q1_r2_1 + z_blue_q1_r2_1 = 0
 prop_state_q1_r2_2: - y_q1_r2_2 + z_blue_q1_r2_2 = 0
 prop_state_q1_r2_3: - y_q1_r2_3 + z_blue_q1_r2_3 = 0
 prop_state_q1_r2_4: - y_q1_r2_4 + z_blue_q1_r2_4 = 0
 prop_state_q3_r1_0: - y_q3_r1_0 + z_red_q3_r1_0 = 0
 prop_state_q3_r1_1: - y_q3_r1_1 + z_red_q3_r1_1 = 0
 prop_state_q3_r1_2: - y_q3_r1_2 + z_red_q3_r1_2 = 0
 prop_state_q3_r1_3: - y_q3_r1_3 + z_red_q3_r1_3 = 0
 prop_state_q3_r1_4: - y_q3_r1_4 + z_red_q3_r1_4 = 0
 prop_state_q3_r2_0: - y_q3_r2_0 + z_red_q3_r2_0 = 0
 prop_state_q3_r2_1: - y_q3_r2_1 + z_red_q3_r2_1 = 0
 prop_state_q3_r2_2: - y_q3_r2_2 + z_red_q3_r2_2 = 0
 prop_state_q3_r2_3: - y_q3_r2_3 + z_red_q3_r2_3 = 0
 prop_state_q3_r2_4: - y_q3_r2_4 + z_red_q3_r2_4 = 0
 prop_state_q2_r1_0: - y_q2_r1_0 + z_orange_q2_r1_0 = 0
 prop_state_q2_r1_1: - y_q2_r1_1 + z_orange_q2_r1_1 = 0
 prop_state_q2_r1_2: - y_q2_r1_2 + z_orange_q2_r1_2 = 0
 prop_state_q2_r1_3: - y_q2_r1_3 + z_orange_q2_r1_3 = 0
 prop_state_q2_r1_4: - y_q2_r1_4 + z_orange_q2_r1_4 = 0
 prop_state_q2_r2_0: - y_q2_r2_0 + z_orange_q2_r2_0 = 0
 prop_state_q2_r2_1: - y_q2_r2_1 + z_orange_q2_r2_1 = 0
 prop_state_q2_r2_2: - y_q2_r2_2 + z_orange_q2_r2_2 = 0
 prop_state_q2_r2_3: - y_q2_r2_3 + z_orange_q2_r2_3 = 0
 prop_state_q2_r2_4: - y_q2_r2_4 + z_orange_q2_r2_4 = 0
 prop_state_q4_r1_0: - y_q4_r1_0 + z_green_q4_r1_0 = 0
 prop_state_q4_r1_1: - y_q4_r1_1 + z_green_q4_r1_1 = 0
 prop_state_q4_r1_2: - y_q4_r1_2 + z_green_q4_r1_2 = 0
 prop_state_q4_r1_3: - y_q4_r1_3 + z_green_q4_r1_3 = 0
 prop_state_q4_r1_4: - y_q4_r1_4 + z_green_q4_r1_4 = 0
 prop_state_q4_r2_0: - y_q4_r2_0 + z_green_q4_r2_0 = 0
 prop_state_q4_r2_1: - y_q4_r2_1 + z_green_q4_r2_1 = 0
 prop_state_q4_r2_2: - y_q4_r2_2 + z_green_q4_r2_2 = 0
 prop_state_q4_r2_3: - y_q4_r2_3 + z_green_q4_r2_3 = 0
 prop_state_q4_r2_4: - y_q4_r2_4 + z_green_q4_r2_4 = 0
 min_prop_green_r1_0_q4: green_r1_0 - z_green_q4_r1_0 <= 0
 min_prop_green_r1_1_q4: green_r1_1 - z_green_q4_r1_1 <= 0
 min_prop_green_r1_2_q4: green_r1_2 - z_green_q4_r1_2 <= 0
 min_prop_green_r2_0_q4: green_r2_0 - z_green_q4_r2_0 <= 0
 min_prop_green_r2_1_q4: green_r2_1 - z_green_q4_r2_1 <= 0
 min_prop_green_r2_2_q4: green_r2_2 - z_green_q4_r2_2 <= 0
Bounds
 z_q1_1_0 <= 2
 z_q1_1_1 <= 2
 z_q1_1_2 <= 2
 z_q1_1_3 <= 2
 z_q1_1_4 <= 2
 z_q3_1_0 <= 2
 z_q3_1_1 <= 2
 z_q3_1_2 <= 2
 z_q3_1_3 <= 2
 z_q3_1_4 <= 2
 z_q2_1_0 <= 2
 z_q2_1_1 <= 2
 z_q2_1_2 <= 2
 z_q2_1_3 <= 2
 z_q2_1_4 <= 2
 z_q4_1_0 <= 2
 z_q4_1_1 <= 2
 z_q4_1_2 <= 2
 z_q4_1_3 <= 2
 z_q4_1_4 <= 2
 z_q1_q1_1_0 <= 2
 z_q1_q1_1_1 <= 2
 z_q1_q1_1_2 <= 2
 z_q1_q1_1_3 <= 2
 z_q1_q3_1_0 <= 2
 z_q1_q3_1_1 <= 2
 z_q1_q3_1_2 <= 2
 z_q1_q3_1_3 <= 2
 z_q1_q2_1_0 <= 2
 z_q1_q2_1_1 <= 2
 z_q1_q2_1_2 <= 2
 z_q1_q2_1_3 <= 2
 z_q3_q1_1_0 <= 2
 z_q3_q1_1_1 <= 2
 z_q3_q1_1_2 <= 2
 z_q3_q1_1_3 <= 2
 z_q3_q3_1_0 <= 2
 z_q3_q3_1_1 <= 2
 z_q3_q3_1_2 <= 2
 z_q3_q3_1_3 <= 2
 z_q3_q4_1_0 <= 2
 z_q3_q4_1_1 <= 2
 z_q3_q4_1_2 <= 2
 z_q3_q4_1_3 <= 2
 z_q2_q1_1_0 <= 2
 z_q2_q1_1_1 <= 2
 z_q2_q1_1_2 <= 2
 z_q2_q1_1_3 <= 2
 z_q2_q2_1_0 <= 2
 z_q2_q2_1_1 <= 2
 z_q2_q2_1_2 <= 2
 z_q2_q2_1_3 <= 2
 z_q2_q4_1_0 <= 2
 z_q2_q4_1_1 <= 2
 z_q2_q4_1_2 <= 2
 z_q2_q4_1_3 <= 2
 z_q4_q3_1_0 <= 2
 z_q4_q3_1_1 <= 2
 z_q4_q3_1_2 <= 2
 z_q4_q3_1_3 <= 2
 z_q4_q2_1_0 <= 2
 z_q4_q2_1_1 <= 2
 z_q4_q2_1_2 <= 2
 z_q4_q2_1_3 <= 2
 z_q4_q4_1_0 <= 2
 z_q4_q4_1_1 <= 2
 z_q4_q4_1_2 <= 2
 z_q4_q4_1_3 <= 2
 -infinity <= rho0 <= 999
 -infinity <= rho1 <= 999
 green_a_0 <= 2
 green_a_1 <= 2
 green_a_2 <= 2
 green_r2_0 <= 2.2
 green_r1_0 <= 3.3
 green_a_3 <= 2
 green_r2_1 <= 2.2
 green_r1_1 <= 3.3
 green_a_4 <= 2
 green_r2_2 <= 2.2
 green_r1_2 <= 3.3
 z_blue_q1_a_0 <= 2
 z_blue_q1_a_1 <= 2
 z_blue_q1_a_2 <= 2
 z_blue_q1_a_3 <= 2
 z_blue_q1_a_4 <= 2
 z_red_q3_a_0 <= 2
 z_red_q3_a_1 <= 2
 z_red_q3_a_2 <= 2
 z_red_q3_a_3 <= 2
 z_red_q3_a_4 <= 2
 z_orange_q2_a_0 <= 2
 z_orange_q2_a_1 <= 2
 z_orange_q2_a_2 <= 2
 z_orange_q2_a_3 <= 2
 z_orange_q2_a_4 <= 2
 z_green_q4_a_0 <= 2
 z_green_q4_a_1 <= 2
 z_green_q4_a_2 <= 2
 z_green_q4_a_3 <= 2
 z_green_q4_a_4 <= 2
 y_q1_r1_0 <= 3.3
 y_q1_r2_0 <= 2.2
 y_q1_r1_1 <= 3.3
 y_q1_r2_1 <= 2.2
 y_q1_r1_2 <= 3.3
 y_q1_r2_2 <= 2.2
 y_q1_r1_3 <= 3.3
 y_q1_r2_3 <= 2.2
 y_q1_r1_4 <= 3.3
 y_q1_r2_4 <= 2.2
 y_q3_r1_0 <= 3.3
 y_q3_r2_0 <= 2.2
 y_q3_r1_1 <= 3.3
 y_q3_r2_1 <= 2.2
 y_q3_r1_2 <= 3.3
 y_q3_r2_2 <= 2.2
 y_q3_r1_3 <= 3.3
 y_q3_r2_3 <= 2.2
 y_q3_r1_4 <= 3.3
 y_q3_r2_4 <= 2.2
 y_q2_r1_0 <= 3.3
 y_q2_r2_0 <= 2.2
 y_q2_r1_1 <= 3.3
 y_q2_r2_1 <= 2.2
 y_q2_r1_2 <= 3.3
 y_q2_r2_2 <= 2.2
 y_q2_r1_3 <= 3.3
 y_q2_r2_3 <= 2.2
 y_q2_r1_4 <= 3.3
 y_q2_r2_4 <= 2.2
 y_q4_r1_0 <= 3.3
 y_q4_r2_0 <= 2.2
 y_q4_r1_1 <= 3.3
 y_q4_r2_1 <= 2.2
 y_q4_r1_2 <= 3.3
 y_q4_r2_2 <= 2.2
 y_q4_r1_3 <= 3.3
 y_q4_r2_3 <= 2.2
 y_q4_r1_4 <= 3.3
 y_q4_r2_4 <= 2.2
 y_q1_q1_r1_0 <= 3.3
 y_q1_q1_r2_0 <= 2.2
 y_q1_q1_r1_1 <= 3.3
 y_q1_q1_r2_1 <= 2.2
 y_q1_q1_r1_2 <= 3.3
 y_q1_q1_r2_2 <= 2.2
 y_q1_q1_r1_3 <= 3.3
 y_q1_q1_r2_3 <= 2.2
 y_q1_q3_r1_0 <= 3.3
 y_q1_q3_r2_0 <= 2.2
 y_q1_q3_r1_1 <= 3.3
 y_q1_q3_r2_1 <= 2.2
 y_q1_q3_r1_2 <= 3.3
 y_q1_q3_r2_2 <= 2.2
 y_q1_q3_r1_3 <= 3.3
 y_q1_q3_r2_3 <= 2.2
 y_q1_q2_r1_0 <= 3.3
 y_q1_q2_r2_0 <= 2.2
 y_q1_q2_r1_1 <= 3.3
 y_q1_q2_r2_1 <= 2.2
 y_q1_q2_r1_2 <= 3.3
 y_q1_q2_r2_2 <= 2.2
 y_q1_q2_r1_3 <= 3.3
 y_q1_q2_r2_3 <= 2.2
 y_q3_q1_r1_0 <= 3.3
 y_q3_q1_r2_0 <= 2.2
 y_q3_q1_r1_1 <= 3.3
 y_q3_q1_r2_1 <= 2.2
 y_q3_q1_r1_2 <= 3.3
 y_q3_q1_r2_2 <= 2.2
 y_q3_q1_r1_3 <= 3.3
 y_q3_q1_r2_3 <= 2.2
 y_q3_q3_r1_0 <= 3.3
 y_q3_q3_r2_0 <= 2.2
 y_q3_q3_r1_1 <= 3.3
 y_q3_q3_r2_1 <= 2.2
 y_q3_q3_r1_2 <= 3.3
 y_q3_q3_r2_2 <= 2.2
 y_q3_q3_r1_3 <= 3.3
 y_q3_q3_r2_3 <= 2.2
 y_q3_q4_r1_0 <= 3.3
 y_q3_q4_r2_0 <= 2.2
 y_q3_q4_r1_1 <= 3.3
 y_q3_q4_r2_1 <= 2.2
 y_q3_q4_r1_2 <= 3.3
 y_q3_q4_r2_2 <= 2.2
 y_q3_q4_r1_3 <= 3.3
 y_q3_q4_r2_3 <= 2.2
 y_q2_q1_r1_0 <= 3.3
 y_q2_q1_r2_0 <= 2.2
 y_q2_q1_r1_1 <= 3.3
 y_q2_q1_r2_1 <= 2.2
 y_q2_q1_r1_2 <= 3.3
 y_q2_q1_r2_2 <= 2.2
 y_q2_q1_r1_3 <= 3.3
 y_q2_q1_r2_3 <= 2.2
 y_q2_q2_r1_0 <= 3.3
 y_q2_q2_r2_0 <= 2.2
 y_q2_q2_r1_1 <= 3.3
 y_q2_q2_r2_1 <= 2.2
 y_q2_q2_r1_2 <= 3.3
 y_q2_q2_r2_2 <= 2.2
 y_q2_q2_r1_3 <= 3.3
 y_q2_q2_r2_3 <= 2.2
 y_q2_q4_r1_0 <= 3.3
 y_q2_q4_r2_0 <= 2.2
 y_q2_q4_r1_1 <= 3.3
 y_q2_q4_r2_1 <= 2.2
 y_q2_q4_r1_2 <= 3.3
 y_q2_q4_r2_2 <= 2.2
 y_q2_q4_r1_3 <= 3.3
 y_q2_q4_r2_3 <= 2.2
 y_q4_q3_r1_0 <= 3.3
 y_q4_q3_r2_0 <= 2.2
 y_q4_q3_r1_1 <= 3.3
 y_q4_q3_r2_1 <= 2.2
 y_q4_q3_r1_2 <= 3.3
 y_q4_q3_r2_2 <= 2.2
 y_q4_q3_r1_3 <= 3.3
 y_q4_q3_r2_3 <= 2.2
 y_q4_q2_r1_0 <= 3.3
 y_q4_q2_r2_0 <= 2.2
 y_q4_q2_r1_1 <= 3.3
 y_q4_q2_r2_1 <= 2.2
 y_q4_q2_r1_2 <= 3.3
 y_q4_q2_r2_2 <= 2.2
 y_q4_q2_r1_3 <= 3.3
 y_q4_q2_r2_3 <= 2.2
 y_q4_q4_r1_0 <= 3.3
 y_q4_q4_r2_0 <= 2.2
 y_q4_q4_r1_1 <= 3.3
 y_q4_q4_r2_1 <= 2.2
 y_q4_q4_r1_2 <= 3.3
 y_q4_q4_r2_2 <= 2.2
 y_q4_q4_r1_3 <= 3.3
 y_q4_q4_r2_3 <= 2.2
 z_blue_q1_r1_0 <= 3.3
 z_blue_q1_r1_1 <= 3.3
 z_blue_q1_r1_2 <= 3.3
 z_blue_q1_r1_3 <= 3.3
 z_blue_q1_r1_4 <= 3.3
 z_blue_q1_r2_0 <= 2.2
 z_blue_q1_r2_1 <= 2.2
 z_blue_q1_r2_2 <= 2.2
 z_blue_q1_r2_3 <= 2.2
 z_blue_q1_r2_4 <= 2.2
 z_red_q3_r1_0 <= 3.3
 z_red_q3_r1_1 <= 3.3
 z_red_q3_r1_2 <= 3.3
 z_red_q3_r1_3 <= 3.3
 z_red_q3_r1_4 <= 3.3
 z_red_q3_r2_0 <= 2.2
 z_red_q3_r2_1 <= 2.2
 z_red_q3_r2_2 <= 2.2
 z_red_q3_r2_3 <= 2.2
 z_red_q3_r2_4 <= 2.2
 z_orange_q2_r1_0 <= 3.3
 z_orange_q2_r1_1 <= 3.3
 z_orange_q2_r1_2 <= 3.3
 z_orange_q2_r1_3 <= 3.3
 z_orange_q2_r1_4 <= 3.3
 z_orange_q2_r2_0 <= 2.2
 z_orange_q2_r2_1 <= 2.2
 z_orange_q2_r2_2 <= 2.2
 z_orange_q2_r2_3 <= 2.2
 z_orange_q2_r2_4 <= 2.2
 z_green_q4_r1_0 <= 3.3
 z_green_q4_r1_1 <= 3.3
 z_green_q4_r1_2 <= 3.3
 z_green_q4_r1_3 <= 3.3
 z_green_q4_r1_4 <= 3.3
 z_green_q4_r2_0 <= 2.2
 z_green_q4_r2_1 <= 2.2
 z_green_q4_r2_2 <= 2.2
 z_green_q4_r2_3 <= 2.2
 z_green_q4_r2_4 <= 2.2
Binaries
 event_2dx15bd35de9a722238_0 and_2bx6329f7ce2399b141_0
 always_2dx61a27e0667b35d05_0 and_2dx80e20ade56d99f5_0
 predicate_2dx2de4eb3ae5fa39f0_0 and_2dx80e20ade56d99f5_1
 predicate_2dx2de4eb3ae5fa39f0_1 and_2dx80e20ade56d99f5_2
 predicate_2dx2de4eb3ae5fa39f0_2 predicate_2bx5985614b5977fb2b_0
 predicate_2bx31f83dca6953bbc0_0 and_2bx6329f7ce2399b141_1
 always_2dx61a27e0667b35d05_1 and_2dx80e20ade56d99f5_3
 predicate_2dx2de4eb3ae5fa39f0_3 predicate_2bx5985614b5977fb2b_1
 predicate_2bx31f83dca6953bbc0_1 and_2bx6329f7ce2399b141_2
 always_2dx61a27e0667b35d05_2 and_2dx80e20ade56d99f5_4
 predicate_2dx2de4eb3ae5fa39f0_4 predicate_2bx5985614b5977fb2b_2
 predicate_2bx31f83dca6953bbc0_2
Generals
 z_q1_1_0 z_q1_1_1 z_q1_1_2 z_q1_1_3 z_q1_1_4 z_q3_1_0 z_q3_1_1 z_q3_1_2
 z_q3_1_3 z_q3_1_4 z_q2_1_0 z_q2_1_1 z_q2_1_2 z_q2_1_3 z_q2_1_4 z_q4_1_0
 z_q4_1_1 z_q4_1_2 z_q4_1_3 z_q4_1_4 z_q1_q1_1_0 z_q1_q1_1_1 z_q1_q1_1_2
 z_q1_q1_1_3 z_q1_q3_1_0 z_q1_q3_1_1 z_q1_q3_1_2 z_q1_q3_1_3 z_q1_q2_1_0
 z_q1_q2_1_1 z_q1_q2_1_2 z_q1_q2_1_3 z_q3_q1_1_0 z_q3_q1_1_1 z_q3_q1_1_2
 z_q3_q1_1_3 z_q3_q3_1_0 z_q3_q3_1_1 z_q3_q3_1_2 z_q3_q3_1_3 z_q3_q4_1_0
 z_q3_q4_1_1 z_q3_q4_1_2 z_q3_q4_1_3 z_q2_q1_1_0 z_q2_q1_1_1 z_q2_q1_1_2
 z_q2_q1_1_3 z_q2_q2_1_0 z_q2_q2_1_1 z_q2_q2_1_2 z_q2_q2_1_3 z_q2_q4_1_0
 z_q2_q4_1_1 z_q2_q4_1_2 z_q2_q4_1_3 z_q4_q3_1_0 z_q4_q3_1_1 z_q4_q3_1_2
 z_q4_q3_1_3 z_q4_q2_1_0 z_q4_q2_1_1 z_q4_q2_1_2 z_q4_q2_1_3 z_q4_q4_1_0
 z_q4_q4_1_1 z_q4_q4_1_2 z_q4_q4_1_3
End
