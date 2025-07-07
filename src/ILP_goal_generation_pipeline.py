#!/usr/bin/env python
import argparse
import os
from ILP_goal_generator import ILPGoalGenerator
import sys
import time
import subprocess
import re
import copy
import random
import uuid
from pddl.goal_descriptor import GoalConjunction, GoalSimple
from datetime import datetime

def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

PLANNER = "OPTIC"  # Default planner to use for planning the goals

class GoalGenerationPipeline:
    def __init__(self, args):
        current_time = (datetime.now().hour + datetime.now().minute/60.0) # Will be overwritten by the config file if it exists
        n_goals = -1  # Default value, will be overwritten by the config file if it exists. -1 means all goals
        self.goal_generator = ILPGoalGenerator(args.domain, args.problem, n_goals, current_time, check_unreachable=True, check_mutexes=True, uuid=args.uuid, debug=False)
        self.goal_generator.load_cfg(args.cfg_path)
        self.solver_id = args.solver_id
        self.planner_timeout = args.planner_timeout
        self.remove_percent = args.remove_percent  # Percentage of goals to remove from the end of the list if no plan is found
        self.removal_strategy = args.removal_strategy  # Strategy to remove goals if no plan is found: "worst_n" or "random"
        self.uuid = args.uuid # TODO get from arg with uuidgen from bash to keep logs together?
        #self.uuid = self.uuid.replace('-', '')  # Remove dashes
        print('Starting Goal Generation pipeline with UUID:', self.uuid)


    def generate_goals(self, decimal=3):
        start_time = time.time()
        self.goal_generator.make_ILP(solver_id=self.solver_id)
        goals, solver_benefit, solver_time = self.goal_generator.solve()
        # Goals is a list of tuples (goal, benefit)
        return goals, round(solver_benefit, decimal), round(solver_time, decimal), round(time.time() - start_time, decimal+1)

    def goal_generation_pipeline(self):
        goals, solver_benefit, solver_time, total_time = self.generate_goals()
        if not goals:
            eprint('No goals found!')
            return

        last_times_map = self.get_last_time_map(goals)
        goals.sort(key=lambda x: x[1], reverse=True)
        print('Sorted goals:')
        total_benefit = self.compute_benefit_goal_list(goals, last_times_map, True)
        print('\tTotal benefit:', total_benefit, '(Solver benefit:', str(solver_benefit) +'). Solver time: ' + str(solver_time) + ' milliseconds, total time: ' + str(total_time) + ' seconds)')
        assert abs(total_benefit-solver_benefit) < 0.15, 'Solver benefit is different to the planner\'s benefit!'

        plan = self.run_planner(goals)
        if plan:
            planning_time = float(plan[0])  # Time taken by the planner to find the
            print('Goal Generation completed. Plan to goals was found in ' + str(planning_time) + ' seconds.')
            return True
        else:
            original_goals = copy.deepcopy(goals)
            n_trials = 0
            while goals:
                # Remove last X% of goals
                n_to_remove = int(round(len(goals)*(self.remove_percent/100.0)+0.00001))  # delta added to round up if it falls in 0.5
                if n_to_remove > 0:
                    if self.removal_strategy == "worst_n": 
                        del goals[-n_to_remove:] # goals = goals[:-n_to_remove]
                        print(f'Removing a {self.remove_percent}% of the goals ({n_to_remove} goals) from the end of the list, trying to plan with {len(goals)} goals...')
                    elif self.removal_strategy == "random":
                        to_delete = random.sample(range(len(goals)), n_to_remove)
                        goals = [g for i, g in enumerate(goals) if i not in to_delete]
                        print(f'Removing a {self.remove_percent}% of the goals ({n_to_remove} goals) at random, trying to plan with {len(goals)} goals...')
                    else:
                        eprint('Unknown removal strategy ' + self.removal_strategy)
                        return False
                    
                    n_trials += 1

                    print('Goals after removal:')
                    total_benefit = self.compute_benefit_goal_list(goals, last_times_map, True)
                    print('\tTotal benefit:', total_benefit)
                
                    plan = self.run_planner(goals)
               
                if n_to_remove == 0 and not plan:
                    goals = []  # We couldn't solve any goal!
                    print('############### All goals were removed! Problem is unsolvable.')
                    return False

                if plan:
                    print(f'Goal Generation completed after {n_trials} trials with {len(goals)} goals out of the original {len(original_goals)} and a total benefit of {total_benefit}.')
                    return True

    def run_planner(self, goals, planner=PLANNER): # popf or rewrite-no-lp
        # Generate the domain and problem files for the planner with the new goals
        problem = copy.deepcopy(self.goal_generator.agg.problem)
        problem.goal = GoalConjunction([GoalSimple(g[0]) for g in goals])
        goals_problem_path = f'/tmp/goal_generation/{self.uuid}_problem_with_generated_goals.pddl'
        with open(goals_problem_path, 'w') as f:
            f.write(str(problem))

        # Try to solve the plan to those goals
        if planner == "POPF":
            cmd = f'timeout {self.planner_timeout} {os.path.dirname(__file__)}/../bin/popf -n '+ \
                f'{self.goal_generator.agg.pddl_extractor.domain_file} {goals_problem_path}' + \
                f' | tee -a /tmp/goal_generation/{self.uuid}_gg_planner_output.txt' # TODO change tee path to avoid clashes?
        elif planner == "OPTIC":
            cmd = f'timeout {self.planner_timeout} {os.path.dirname(__file__)}/../bin/optic-clp -n '+ \
                  f'{self.goal_generator.agg.pddl_extractor.domain_file} {goals_problem_path}' + \
                  f' | tee -a /tmp/goal_generation/{self.uuid}_gg_planner_output.txt' # TODO change tee path to avoid clashes?
        else:
            cmd = f'timeout {self.planner_timeout} {os.path.dirname(__file__)}/../bin/optic ' + \
                   f'{self.goal_generator.agg.pddl_extractor.domain_file} {goals_problem_path}' + \
                   f' | tee -a /tmp/goal_generation/{self.uuid}_gg_planner_output.txt' # TODO change tee path to avoid clashes?
            # '--optimise --use-rpg Distance --pseudo-ehc '
        print(cmd)
        print(f'Running planner to solve generated goals for {self.planner_timeout} seconds...')
        try:
            output = subprocess.check_output(cmd, shell=True, stderr=subprocess.STDOUT).decode('utf-8')  # also stderr=subprocess.DEVNULL
        except subprocess.CalledProcessError as e:
            if e.returncode != 124:  # 124 is timeout's return code when killed successfully
                eprint('Planner failed with error code ' + str(e.returncode))
            output = e.output.decode('utf-8')
        if planner == "POPF" or planner == "OPTIC":
            plan_re = re.compile(r';.+with metric (\d+.?\d+)$\s+(?:\s*;.*$\n)+; Time (\d+.?\d+)\n((?:.*\(.*\).*$\n)+)', re.MULTILINE)  # Metric, time, plan POPF!
        else:
            #plan_re = re.compile(r';+ Solution.+$(?:\s*;.*$\n)+; Metric value (.+)$\n((?:.*\(.*\).*$\n)+)', re.MULTILINE) # Metric and plan only
            plan_re = re.compile(r';+ Solution.+$\s+; Time (\d+.?\d+)(?:\s*;.*$\n)+; Metric value (.+)$\n((?:.*\(.*\).*$\n)+)', re.MULTILINE) # Time, metric, plan
        plan_match = plan_re.findall(output)  # This has all the plans with its metric. Best one is the last one (plan_match[-1])
        if plan_match:
            if planner == "POPF" or planner == "OPTIC":
                plan_match[-1] = [plan_match[-1][1], plan_match[-1][0], plan_match[-1][2]]  # [time, metric, plan]
            print(f'Plan found with metric value {plan_match[-1][1]} after {plan_match[-1][0]} seconds.')
            return plan_match[-1]  # Return the last found plan as [time, metric, plan]
        eprint('No plan found')
        return None

    def compute_benefit_goal_list(self, goals, last_times_map, print_goals=False, decimals=3):
        total_benefit = 0
        for g in goals:
            gstr = str(g[0])[1:-1].split(' ')
            b = self.goal_generator.agg.get_grounded_goal_benefit(gstr, last_times_map[str(g[0])])
            if print_goals: print('\t\t', str(g[0]), '- computed: ', round(b, decimals), '- solver:', g[1])
            if abs(b-g[1]) > 0.01:
                eprint(f'Warning: Computed benefit {b} for goal {g[0]} does not match solver benefit {g[1]}.')
            total_benefit += b
        return round(total_benefit, decimals)

    def get_last_time_map(self, goals):
        last_time_map = {}
        for goal in goals:
            last_time = self.goal_generator.agg.get_last_goal_time(goal[0])
            if last_time is not None:
                last_time_map[str(goal[0])] = last_time
            else:
                last_time_map[str(goal[0])] = self.goal_generator.agg.current_time  # If no last time is found, use the current time
        return last_time_map

def check_args():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-d", '--domain', type=str,  help="Domain to be evaluated", default=os.path.dirname(__file__) + '/../testdomains/domain.pddl')
    parser.add_argument("-p", '--problem', type=str,  help="Problem to be evaluated", default=os.path.dirname(__file__) + '/../testdomains/problem.pddl')
    parser.add_argument("-g", '--cfg_path', type=str,  help="Config YAML file with benefits", default=os.path.dirname(__file__) + '/../cfg/cfg.yaml')
    
    parser.add_argument("-s", '--solver_id', type=str,  help="Solver ID", default="GLOP")  # GLOP, CLP, CPLEX, CPSAT
    parser.add_argument("-u", '--uuid', type=str,  help="UUID for this run", default=uuid.uuid4().hex)

    #parser.add_argument("-o", '--output', type=str,  help="Output csv file", default='/tmp/pddl3_test.csv')

    parser.add_argument("-t", '--planner_timeout', type=float,  help="Planner timeout", default=30)
    parser.add_argument("-r", '--remove_percent', type=int,  help="Percentage of goals to remove if no plan is found", default=15)
    parser.add_argument("-m", '--removal_strategy', type=str,  help="Removal strategy for goals if no plan is found", default="worst_n", choices=["worst_n", "random"])

    #parser.add_argument("-e", '--planner_exec', type=str,  help="Planner executable path", default=PLANNER_EXEC)
    
    args = parser.parse_args()
    return args

if __name__ == '__main__':
   args = check_args()
   #print(args)
   gpp = GoalGenerationPipeline(args)
   gpp.goal_generation_pipeline()