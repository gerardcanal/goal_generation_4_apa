#!/usr/bin/env python

# Gerard Canal <gerard.canal@kcl.ac.uk>
# King's College London, 2025

import re
import argparse
import subprocess
import time
import os.path
import numpy as np
from make_pddl3_domain import MakePDDL3GRAP
from evaluation.plan_utils import PlanUtils
from eval_utils import EvalUtils
import copy

USE_BENEFIT = True
USE_CONDITIONAL_TIMED_EFFECTS = True
BENEFIT_FACTOR = -1 #-0.01
TIME_FACTOR = 0.05 # 0.03
VIOLATED_PREFS_FACTOR = 0
PLANNER_EXEC = "/home/gerard/Nextcloud/optic-clp/tuned_tautologous/optic-cplex"
PLANNER_EXEC = "/home/gerard/Nextcloud/optic-clp/optic-cplex"
PLANNER_EXEC = "/home/gerard/Nextcloud/optic-clp/contradictory_tuned/optic-cplex"
PLANNER_EXEC = "/home/gerard/Nextcloud/optic-clp/optic-clp"
PLANNER_TIMEOUT = 600
CHECK_OUTPUT = True

class PDDL3GRAPEVAL:

    def __init__(self, domain, problem, cfg_path, csv_file, planner_exec=PLANNER_EXEC, planner_timeout=PLANNER_TIMEOUT,
                 use_benefit=USE_BENEFIT, use_conditionals=USE_CONDITIONAL_TIMED_EFFECTS, benefit_factor=BENEFIT_FACTOR, time_factor=TIME_FACTOR, 
                 violated_prefs_factor=VIOLATED_PREFS_FACTOR, args=None):
        self.csv_line = {}
        self.start_time = time.time()
        should_write_header = not os.path.exists(csv_file)
        self.csv_file = open(csv_file, 'a')
        self.args = args
        self.pddl3 = MakePDDL3GRAP(domain, problem, cfg_path, use_benefit, use_conditionals, benefit_factor, time_factor, violated_prefs_factor, out_folder=args.output_domains, postfix=args.postfix)
        self.pddl3.make_pddl3_domain()
        self.pddl3.make_pddl3_problem()
        self.csv_line['init_time'] = time.time() - self.start_time
        self.domain_file = self.pddl3.domain_file
        self.problem_file = self.pddl3.problem_file
        self.planner_exec = planner_exec
        self.planner_timeout = planner_timeout
        print('Opened CSV file ' + self.csv_file.name + ' for writing')
        self.csv_header(should_write_header)

    def run_planner(self):
        # Runs the planner and returns the plan
        self.csv_line['max_benefit'] = self.pddl3.adg.max_gainable_benefit[0]
        self.csv_line['max_positive_benefit'] = self.pddl3.adg.max_gainable_benefit[1]
        planning_start_time = time.time()
        cmd = f'timeout {self.planner_timeout} ' + self.planner_exec + ' -n ' + self.domain_file + ' ' + self.problem_file + ' | grep -v \\(G\\)'
        if CHECK_OUTPUT:
            cmd += ' | tee /tmp/planner_output.txt'
        print(cmd)
        print('Running planner for ' + str(self.planner_timeout) + ' seconds...')
        try:
            stderr_f = open("/tmp/planner_stderr.txt", "w")
            output = subprocess.check_output(cmd, shell=True, stderr=stderr_f).decode('utf-8')  # also stderr=subprocess.DEVNULL or subprocess.STDOUT
            stderr_f.close()
        except subprocess.CalledProcessError as e:
            if e.returncode != 124:  # 124 is timeout's return code when killed successfully
                print('Planner failed with error code ' + str(e.returncode))
            output = e.output.decode('utf-8')
        self.csv_line['planning_time'] = time.time() - planning_start_time
        #plan_re = re.compile(r';+ Solution.+$(?:\s*;.*$\n)+; Metric value (.+)$\n((?:.*\(.*\).*$\n)+)', re.MULTILINE) # rewrite
        #plan_re = re.compile(r';+ Plan found with metric\s+(.*)$\n(?:\s*;.*$\n)+((?:.*\(.*\).*$\n?)+)', re.MULTILINE) # optic no time groups: [metric, plan]
        plan_re = re.compile(r';+ Plan found with metric\s+(.*)$\n(?:[\t ]*;.*$\n)+; Time (\d+.?\d*)\s(?:[\t ]*;.*$\n)*((?:.*\(.*\).*$\n?)+)', re.MULTILINE) # optic with time groups: [metric, time, plan]
        plan_match = plan_re.findall(output)  # This has all the plans with its metric. Best one is the last one (plan_match[-1])
        self.csv_line['total_time'] = time.time() - self.start_time
        self.csv_line['n_unsolvable_plans'] = 0
        self.csv_line['n_empty_plans'] = 0
        if plan_match:
            print('Plan found with metric value ' + plan_match[-1][0] + ' after ' + plan_match[-1][1] + ' seconds.') 
            self.csv_line['planning_reported_real_time'] = plan_match[-1][1]
            return plan_match[-1]  # Return the last found plan
        elif 'Plan found' in output:
            self.csv_line['n_empty_plans'] += 1
            plan_re = re.compile(r';+ Plan found with metric\s+(.*)$\n(?:\s*;.*$\n)+?; Time (\d+.?\d*)\s', re.MULTILINE)  # optic for empty plan groups [metric, time]
            plan_match = plan_re.findall(output)
            self.csv_line['planning_reported_real_time'] = plan_match[-1][1]
            self.csv_line['plan_metric'] = plan_match[-1][0]
            print('Plan found but empty after ' + plan_match[-1][1] + ' seconds.')
            return None
        self.csv_line['n_unsolvable_plans'] += 1
        self.csv_line['planning_reported_real_time'] = 0
        print('No plan found')
        return None

    def find_achieved_goals(self, plan):
        # Returns a list of goals that are achieved by the plan
        achieved_goals = []


        # Note: the original domain should also achieve the actions as we do not modify the domain's effects!
        domain = self.pddl3.adg.pddl_extractor.domain
        problem = self.pddl3.adg.pddl_extractor.problem
        if plan:
            # Some domains may have uppercase action names, that optic makes lower case. This reverts it
            lower_case_op_map = {op_name.lower(): op_name for op_name in domain.operators.keys()}
            lower_case_instance_map = {inst.lower(): inst for inst in problem.objects_type_map.keys()}
            plan = [(a[0], [lower_case_op_map[a[1][0]]] + [lower_case_instance_map[v] for v in a[1][1:]], a[2]) for a in plan]  # Convert plan to original case

            final_state, _ = PlanUtils.simulate_plan_execution(domain, problem, plan)
        else:
            final_state = problem.get_initial_state()
        final_state = [problem.grounding.get_proposition_from_id(p) for p in np.nonzero(final_state.logical)[0]]
        # Not considering negative goals -- fornow # FIXME
        for g in self.pddl3.all_goals:
            for p in final_state:
                if str(g) == str(p): 
                    achieved_goals.append(g)
                    break
        return achieved_goals

    def csv_header(self, write_header):
        self.csv_fields = ['init_time', 'planning_time',  'planning_reported_real_time', 'total_time', 'n_unsolvable_plans', 'n_empty_plans',
                           # 'requested_goals',  # FIXME in case I try -nP-Goals
                           'initial_solved_goals', 'final_solved_goals', 'final_minus_initial_goals', 'new_goals',
                           'final_n_goals_in_window',
                           'final_n_goals_after_window',
                           'final_n_goals_before_window',
                           'final_n_goals_in_cycle',
                           'final_n_goals_over_cycle',
                           'final_n_goals_before_cycle', 
                           'final_n_pos_goals_benefit',
                           'final_n_neg_goals_benefit',
                           'max_n_pos_goals_benefit', 'max_n_neg_goals_benefit',
                           'plan_length', 'plan_makespan', 'plan_metric',
                           'initial_state_benefit', 'final_minus_initial_benefit',
                           'max_benefit', 'max_positive_benefit', 'final_benefit',
                           'new_goals_benefit'
                          ]
        if write_header:
            self.csv_file.write(','.join(self.csv_fields) + '\n') # Write header

    def __del__(self):
        self.csv_file.close()
        print(f'Closed CSV file {self.csv_file.name}')
    
    def write_csv_line(self, line):
        # Count number of positive and negative specific goal benefits
        self.csv_line['max_n_pos_goals_benefit'] = self.csv_line['max_n_neg_goals_benefit'] = 0
        for g, params in self.pddl3.adg.goal_benefits.items():
            for p, v in params.items():
                if v > 0:
                    self.csv_line['max_n_pos_goals_benefit'] += 1
                else:
                    self.csv_line['max_n_neg_goals_benefit'] += 1
        for field in self.csv_fields:
            assert field in line, f'Field {field} not in line {line}'
        for field in line:
            assert field in self.csv_fields, f'Field {field} not in csv fields'
        self.csv_file.write(','.join([str(line[field]) for field in self.csv_fields]) + '\n')
        self.csv_file.flush()

    def get_last_time_map(self, goals):
        last_time_map = {}
        for goal in goals:
            last_time = self.pddl3.adg.get_last_goal_time(goal)
            if last_time is not None:
                last_time_map[str(goal)] = last_time
            else:
                last_time_map[str(goal)] = self.pddl3.adg.current_time  # If no last time is found, use the current time
        return last_time_map

        # Below is for when the PDDL3 domain doesn't have precomputed benefits
        with open(self.problem_file, 'r') as f:
            problem_file = f.read()
        last_times_goals = {}
        for g in goals:
            goal_name = g.name
            key = str(g) # goal_name + '-' + '-'.join([t.value for t in g.typed_parameters])
            if '__' in goal_name:
                goal_name = goal_name[:goal_name.find('__')]  # in case there was as supertype
            if goal_name in self.pddl3.adg.timing_cyclics:
                regex = re.compile(rf'\({goal_name}__last_time {" ".join([t.value for t in g.typed_parameters])}\) (-?\d+.?\d*)')

                re_result = regex.findall(problem_file)
                assert len(re_result) <= 1, 'Error parsing last time'
                last_times_goals[key] = float(re_result[0]) if re_result else self.pddl3.adg.current_time 
            else:# Default value
                last_times_goals[key] = self.pddl3.adg.current_time 
        return last_times_goals

    def compute_benefit_goal_list(self, goals, last_times_map, print_goals=False):
        total_benefit = 0
        for g in goals:
            key = str(g)
            g = [g.name] + [t.value for t in g.typed_parameters]
            b = self.pddl3.adg.get_grounded_goal_benefit(g, last_times_map[key])
            if print_goals: print('\t\t', g[0] + '(' + ', '.join(g[1:]) + ')', b)
            total_benefit += b
        return total_benefit

    def run_eval(self):
        # Find initial benefit
        goals = eval.find_achieved_goals([])
        last_times_map = self.get_last_time_map(self.pddl3.all_goals)
        self.csv_line['initial_state_benefit'] = self.compute_benefit_goal_list(goals, last_times_map, print_goals=False)
        self.csv_line['initial_solved_goals'] = len(goals)
        initial_goals = [str(g) for g in goals]  # Keep the initial goals for later
        
        # Now plan
        plan = eval.run_planner()
        if plan:
            self.csv_line['plan_metric'] = plan[0]
            plan = PlanUtils.get_plan_as_list(plan[2])
            goals = eval.find_achieved_goals(plan)
            self.csv_line['final_solved_goals'] = len(goals)
            self.csv_line['final_minus_initial_goals'] = self.csv_line['final_solved_goals'] - self.csv_line['initial_solved_goals']
            new_goals = [g for g in goals if str(g) not in initial_goals]
            self.csv_line['new_goals'] = len(new_goals)
            print('Found ' + str(len(goals)) + ' goals:')
            for g in goals:
                print('   ', g)
            print('Found plan:')
            for a in plan:
                print('   ' + str(a[0]) + ': (' + ' '.join(a[1]) + ') [' + str(a[2]) + ']')
            mk = PlanUtils.compute_plan_makespan(plan)
            print('Makespan: ' + str(mk))
            print('Metric: ' + str(self.csv_line['plan_metric']))
            assert (mk - PlanUtils.compute_plan_makespan_v2(plan)) < 0.01, 'Makespan computation error'
            self.csv_line['plan_makespan'] = mk

            last_times_map = self.get_last_time_map(goals)
            total_benefit = self.compute_benefit_goal_list(goals, last_times_map, print_goals=False)
            print('Benefit: ' + str(total_benefit))
            new_goals_benefit = self.compute_benefit_goal_list(new_goals, last_times_map, print_goals=False)
            print('Benefit of new goals:', new_goals_benefit)
            self.csv_line['new_goals_benefit'] = new_goals_benefit
            self.csv_line['final_benefit'] = total_benefit
            self.csv_line['final_minus_initial_benefit'] = total_benefit - self.csv_line['initial_state_benefit']
            self.csv_line['plan_length'] = len(plan)

            if self.args.violated_prefs_factor > 0: 
                violated_prefs = -((self.args.time_factor * mk) + (total_benefit * self.args.benefit_factor)-float(self.csv_line['plan_metric']))/self.args.violated_prefs_factor
            else:
                violated_prefs = 0
            print('Violated preferences: ' + str(violated_prefs))
            metric = (self.args.time_factor * mk) + (total_benefit * self.args.benefit_factor) + (self.args.violated_prefs_factor*violated_prefs)
            print('Computed metric = ' + str(metric))
            print(f"Factors: ({self.args.time_factor}, {self.args.benefit_factor}, {self.args.violated_prefs_factor})")

        else:
            self.csv_line['plan_makespan'] = 0
            self.csv_line['final_benefit'] = 0
            self.csv_line['final_minus_initial_benefit'] = 0 - self.csv_line['initial_state_benefit']
            if not 'plan_metric' in self.csv_line: self.csv_line['plan_metric'] = 0
            self.csv_line['plan_length'] = 0
            self.csv_line['final_solved_goals'] = 0
            self.csv_line['final_minus_initial_goals'] = self.csv_line['final_solved_goals'] - self.csv_line['initial_solved_goals']
            self.csv_line['new_goals'] = self.csv_line['new_goals_benefit'] = 0
            last_times_map = {}
            goals = []

        EvalUtils.count_goals_in_windows(self.csv_line, goals, last_times_map, self.pddl3.adg)
        self.write_csv_line(self.csv_line)


def check_args():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-d", '--domain', type=str,  help="Domain to be evaluated", default=os.path.dirname(__file__) + '/../../domains/rovers-time-simple-automatic/domain.pddl')
    parser.add_argument("-p", '--problem', type=str,  help="Problem to be evaluated", default=os.path.dirname(__file__) + '/../../domains/rovers-time-simple-automatic/instance-20.pddl')
    parser.add_argument("-g", '--cfg_path', type=str,  help="Config YAML file with benefits", default=os.path.dirname(__file__) + '/../../evaluation_cfgs/rovers/LP2_n60_lb-500_ub5/cfg.yaml')

    parser.add_argument("-o", '--output', type=str,  help="Output csv file", default='/tmp/pddl3_test.csv')

    parser.add_argument("-t", '--planner_timeout', type=float,  help="Planner timeout", default=PLANNER_TIMEOUT)
    parser.add_argument("-c", '--use_conditional_eff', type=bool,  help="Use conditional effects for timed benefits", default=USE_CONDITIONAL_TIMED_EFFECTS)
    parser.add_argument("-b", '--benefit_factor', type=float,  help="Benefit factor (that multiplies benefit in the metric)", default=BENEFIT_FACTOR)
    parser.add_argument("-f", '--time_factor', type=float,  help="Time factor (that multiplies total-time in the metric)", default=TIME_FACTOR)
    parser.add_argument("-v", '--violated_prefs_factor', type=float,  help="Violated preferences factor (that multiplies violated-prefs in the metric)", default=VIOLATED_PREFS_FACTOR)
    
    parser.add_argument("-od", '--output_domains', type=str,  help="Where to store the generated domains", default='')
    parser.add_argument("-post", '--postfix', type=str,  help="Postfix to the domain filename", default='')

    parser.add_argument("-e", '--planner_exec', type=str,  help="Planner executable path", default=PLANNER_EXEC)
    global CHECK_OUTPUT
    parser.add_argument('--check_output', action='store_true',  help="Check planner output to file", default=CHECK_OUTPUT)

    args = parser.parse_args()
    CHECK_OUTPUT = args.check_output
    if args.output_domains and args.output_domains[-1] != '/':
        args.output_domains += '/'
    return args

if __name__ == '__main__':
    args = check_args()
    eval = PDDL3GRAPEVAL(args.domain, args.problem, args.cfg_path, csv_file=args.output, planner_exec=args.planner_exec, 
                         planner_timeout=args.planner_timeout, use_conditionals=args.use_conditional_eff,
                         benefit_factor=args.benefit_factor, time_factor=args.time_factor,
                         violated_prefs_factor=args.violated_prefs_factor, args=args)
    
    eval.run_eval()
    print('Done')
