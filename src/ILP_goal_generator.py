#!/usr/bin/env python

# Gerard Canal <gerard.canal@kcl.ac.uk>
# King's College London, 2025

from datetime import datetime
from abstract_goal_generator import AbstractGoalGenerator
from ortools.linear_solver import pywraplp
import sys, os
import numpy as np
import uuid
import random

def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

class ILPGoalGenerator:
    def __init__(self, domain_file, problem_file, n_goals=15, current_time=(datetime.now().hour + datetime.now().minute/60.0), check_unreachable=True, check_mutexes=True, uuid=uuid.uuid4().hex, debug=True):
        self.agg = AbstractGoalGenerator(domain_file, problem_file, n_goals, current_time, check_unreachable, check_mutexes, uuid=uuid)
        self.potential_goals = self.agg.get_grounded_predicates(self.agg.potential_goals)
        self.solver = None
        self.random_benefits = False  # If True, use random benefits instead of the ones from the config file
        if self.random_benefits:
            print('\033[93m'+'\033[1m'+"[WARNING] Using random benefits for goals! This is for testing purposes only."+'\033[0m')
        self.debug = debug

    def load_cfg(self, cfg_file):
        self.agg.load_cfg(cfg_file)

    def make_ILP(self, solver_id="GLOP"):
        # solver_id is case insensitive, and the following names are supported:
        #   - CLP_LINEAR_PROGRAMMING or CLP
        #   - CBC_MIXED_INTEGER_PROGRAMMING or CBC
        #   - GLOP_LINEAR_PROGRAMMING or GLOP
        #   - BOP_INTEGER_PROGRAMMING or BOP
        #   - SAT_INTEGER_PROGRAMMING or SAT or CP_SAT
        #   - SCIP_MIXED_INTEGER_PROGRAMMING or SCIP
        #   - GUROBI_LINEAR_PROGRAMMING or GUROBI_LP
        #   - GUROBI_MIXED_INTEGER_PROGRAMMING or GUROBI or GUROBI_MIP
        #   - CPLEX_LINEAR_PROGRAMMING or CPLEX_LP
        #   - CPLEX_MIXED_INTEGER_PROGRAMMING or CPLEX or CPLEX_MIP
        #   - XPRESS_LINEAR_PROGRAMMING or XPRESS_LP
        #   - XPRESS_MIXED_INTEGER_PROGRAMMING or XPRESS or XPRESS_MIP
        #   - GLPK_LINEAR_PROGRAMMING or GLPK_LP
        #   - GLPK_MIXED_INTEGER_PROGRAMMING or GLPK or GLPK_MIP

        self.solver = pywraplp.Solver.CreateSolver(solver_id)
        if not self.solver:
            raise Exception(f"Could not create solver {solver_id}.")
        
        goal2idx = {}

        # Define variables
        # x and y are integer non-negative variables.
        #infinity = self.solver.infinity()
        g = []
        b = []
        for i, pg in enumerate(self.potential_goals):
            g.append(self.solver.BoolVar(f"g{i} - {pg}"))  # Boolean variable for goal satisfaction 
            goal2idx[str(pg)] = i
            #b[i] = self.solver.IntVar(0.0, infinity, f'b{i} - {pg}') # Benefit variable for goal i
            if self.random_benefits:
                b.append(random.randint(-500, 500))
            else:
                b.append(self.agg.get_grounded_goal_benefit(str(pg)[1:-1].split(), self.agg.get_last_goal_time(pg)))  # Get the benefit of the goal
        print("Number of variables =", self.solver.NumVariables())

        # Add constraints
        # C1 : sum_i{g_i} <= N
        n_goals = self.agg.n_goals if self.agg.n_goals > 0 else len(self.potential_goals)  # If n_goals is not set, use the number of potential goals
        self.solver.Add(sum([g[i] for i in range(len(g))]) <= n_goals)
        #self.solver.Add(sum([g[i] for i in range(len(g))]) == n_goals)

        # C2 : g_i + g_j <= 1 for all mutex pairs (i,j)
        for instance, mutex_group in self.agg.mutex_groups:
            self.solver.Add(sum([g[goal2idx['(' + mgi + ')']] for mgi in mutex_group if ('(' + mgi + ')') in goal2idx]) <= 1)

        # C3 : g_i = 0 if goal i is unreachable
        for ug in self.agg.unreachable_goals:
            idx = goal2idx[str(ug)]
            self.solver.Add(g[idx] == 0)  # If a goal is unreachable, its variable must be 0
        print("Number of constraints =", self.solver.NumConstraints())

        # C4 : g_i = 0 if goal i is already achieved in initial state?!
        # initial_state = self.agg.problem.get_initial_state()
        # initial_state = [str(self.agg.problem.grounding.get_proposition_from_id(p)) for p in np.nonzero(initial_state.logical)[0]]
        # for pg in self.potential_goals:
        #     pg = str(pg)
        #     if pg in initial_state:
        #         idx = goal2idx[pg]
        #         self.solver.Add(g[idx] == 0)  # If a goal is already achieved, its variable must be 0
        
        # Objective
        objective = self.solver.Objective()
        for i in range(len(self.potential_goals)):
            objective.SetCoefficient(g[i], b[i])
        objective.SetMaximization()
        self.g = g  # Store the goal variables for later use
        self.b = b  # Store the benefit variables for later use

    def solve(self):
        assert self.solver is not None, "Solver not initialized. Call make_ILP() first."
        print(f"Solving with {self.solver.SolverVersion()}")
        status = self.solver.Solve()

        if status == pywraplp.Solver.OPTIMAL or status == pywraplp.Solver.FEASIBLE: # TODO print somewhere if feasible but not optimal
            if status == pywraplp.Solver.FEASIBLE:
                eprint("The problem has a feasible solution, but it is not optimal. Check output!")
            goals = []
            print("Objective value =", self.solver.Objective().Value())
            print("Selected goals:")
            for i in range(len(self.potential_goals)):
                #print(g[i].name(), " = ", g[i].solution_value())
                if (self.g[i].solution_value()):
                    assert str(self.potential_goals[i]) in self.g[i].name(), f"Error: Goal variable {i} name {self.g[i].name()} does not match potential goal {self.potential_goals[i]}"
                    print(f"  Goal {i}: {self.potential_goals[i]} is selected with benefit {self.b[i]}.")
                    goals.append((self.potential_goals[i], self.b[i]))  # Remove parentheses and split into list
            print()
            print(f"Problem solved in {self.solver.wall_time():d} milliseconds")
            print(f"Problem solved in {self.solver.iterations():d} iterations")
            print(f"Problem solved in {self.solver.nodes():d} branch-and-bound nodes")
            print(f'Generated {len(goals)} goals')
            return goals, self.solver.Objective().Value(), self.solver.wall_time()
        else:
            eprint("The problem does not have an optimal nor feasible solution.")
            return [], float('-inf'), float('-inf')

    def dprint(self, *args, **kwargs):
        if self.debug:
            print(*args, **kwargs)

if __name__ == '__main__':
    domain_file = os.path.dirname(__file__) + '/../testdomains/domain.pddl'
    problem_file = os.path.dirname(__file__) + '/../testdomains/problem.pddl'
    domain_file = os.path.dirname(__file__) + '/../domains/parking/domain.pddl'
    problem_file = os.path.dirname(__file__) + '/../domains/parking/p26-15-1.pddl'
    # domain_file = os.path.dirname(__file__) + '/../domains/rovers-time-simple-automatic/domain.pddl'
    # problem_file = os.path.dirname(__file__) + '/../domains/rovers-time-simple-automatic/instance-20.pddl'
    generator = ILPGoalGenerator(domain_file, problem_file)
    #generator.load_cfg(os.path.dirname(__file__) + '/../cfg/cfg.yaml')
    generator.load_cfg(os.path.dirname(__file__) + '/../cfg/blank_benefit.yaml')
    #generator.load_cfg('/mnt/Data/grap_results/parking_results/L2_n60_lb5_ub5/cfg.yaml')
    #generator.set_state(['not_holding_object robot_assistant', 'robot_at robot_assistant wp0'])
    
    generator.make_ILP(solver_id="GLOP")
    generator.solve()