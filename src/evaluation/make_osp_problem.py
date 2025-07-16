#!/usr/bin/env python

# Gerard Canal <gerard.canal@kcl.ac.uk>
# King's College London, 2025

import sys; import os; sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))  # Hack to add src to path

from pddl_goal_domain_generator import PDDLDomainGenerator
from pddl_goal_domain_generator import *
from pddl.goal_descriptor import GoalDescriptor
from pddl.effect import EffectType
from pddl.parser import Parser
import copy
import re

INFINITE_BOUND = True
# TODO: parse output from planner

class MakeOSPGoalGenerator:
    def __init__(self, domain, problem, cfg_path, out_folder=None, postfix=''):
        problem = self.clear_problem(problem)
        self.adg = PDDLDomainGenerator(domain, problem, check_unreachable=False)
        self.all_goals = self.adg.get_grounded_predicates(self.adg.potential_goals)
        self.adg.load_cfg(cfg_path) 
        self.is_domain_durative = any([op.durative for op in self.adg.pddl_extractor.domain.operators.values()])
        self.postfix = postfix
        self.out_folder = out_folder

    def make_pddl_osp_problem(self):
        pddl_goal_metric = '(:utility\n'

        for g in self.all_goals:
            # Compute goal benefit of g
            params = ', '.join([t.value for t in g.typed_parameters])
            goal_last_time = None
            if g.name in self.adg.last_goal_times and params in self.adg.last_goal_times[g.name]:
                goal_last_time = self.adg.last_goal_times[g.name][params]
            g_str = str(g)[1:-1].split()
            b = self.adg.get_grounded_goal_benefit(g_str, goal_last_time)
            b = int(b)
            if b > 0: pddl_goal_metric += f'    (= {g} {b})\n'
        pddl_goal_metric += ')\n'
        #b = 999999999999999 if INFINITE_BOUND else self.problem_bound # FIXME bound is the number of actions in the plan, not the benefit bound!!
        b = int(self.adg.max_gainable_benefit[0]) if INFINITE_BOUND else self.problem_bound
        pddl_goal_metric += f'(:bound {b})\n' # TODO: BOUND
        if self.use_cost_metric:
            pddl_goal_metric += '(:use-cost-metric)\n'


        problem = copy.deepcopy(self.adg.problem)

        # Add initial state metrics
        problem.goal = GoalDescriptor()
        problem.metric = None
        prob_str = str(problem)
        prob_str = prob_str.replace('(:goal ())', pddl_goal_metric)
        self.problem_file = self.adg.pddl_extractor.problem_file.replace('.pddl', '_osp.pddl')
        if self.out_folder:
            self.problem_file = self.out_folder + self.problem_file.split('/')[-1]
        if self.postfix:
            self.problem_file = self.problem_file.replace('.pddl', '_' + self.postfix + '.pddl')
        with open(self.problem_file, 'w') as f:
            f.write(prob_str)

    def clear_problem(self, problem):
        # Remove utility and bound from problem, add empty goal so OTPL and mutex finder can parse it
        self.use_cost_metric = False
        with open(problem, 'r') as f:
            p = f.read()
            p = p.lower()  # TODO may not work if domain is not lowercase!
            p = re.sub(r'\(:utility.*\s(?:.*?\s)+?\)', '', p)
            bound_re = re.compile(r'\(:bound (\d+)\)')
            self.problem_bound = bound_re.findall(p)[0]
            p = bound_re.sub('(:goal ())', p)
            if ':use-cost-metric' in p:
                self.use_cost_metric = True
                p = re.sub(r'\(:use-cost-metric.*\)', '', p)
        with open('/tmp/osp_problem.pddl', 'w') as f:
            f.write(p)
        return '/tmp/osp_problem.pddl'

if __name__ == '__main__':
    #domain = '/tmp/domai.pddl'
    #domain  = '/home/gerard/code/ROSPlan_ws/src/goalgeneration_as_planning/testdomains/domain.pddl'
    #problem = '/tmp/transport_instance-1.pddl'
    #problem = sys.argv[2]
    #problem = '/home/gerard/code/ROSPlan_ws/src/goalgeneration_as_planning/testdomains/problem.pddl'
    #domain = '/home/gerard/code/ROSPlan_ws/src/goalgeneration_as_planning/domains/rovers-time-simple-automatic/domain.pddl'
    #problem = '/home/gerard/code/ROSPlan_ws/src/goalgeneration_as_planning/domains/rovers-time-simple-automatic/instance-20.pddl'
    domain = '/home/gerard/Downloads/pddl-benchmarks-general-utility/barman-opt11-strips-75/domain.pddl'
    domain = '/home/gerard/Downloads/pddl-benchmarks-general-utility/rovers-100/domain.pddl'
    problem = '/home/gerard/Downloads/pddl-benchmarks-general-utility/barman-opt11-strips-75/pfile03-009.pddl'
    problem = '/home/gerard/Downloads/pddl-benchmarks-general-utility/blocks-100/probBLOCKS-15-0.pddl'
    problem = '/home/gerard/Downloads/pddl-benchmarks-general-utility/rovers-100/p12.pddl'
    #python3.9 make_eval_cfg.py -d /home/gerard/Downloads/pddl-benchmarks-general-utility/blocks-100/domain.pddl -p /home/gerard/Downloads/pddl-benchmarks-general-utility/blocks-100/probBLOCKS-15-0.pddl
    #python3.9 make_eval_cfg.py -d /home/gerard/Downloads/pddl-benchmarks-general-utility/rovers-100/domain.pddl -p /home/gerard/Downloads/pddl-benchmarks-general-utility/rovers-100/p22.pddl

    
    if len(sys.argv) > 1:
        domain = sys.argv[1]
        problem = sys.argv[2]
    if len(sys.argv) > 2:
        cfg = sys.argv[3]
    else:
        cfg = '/home/gerard/code/ROSPlan_ws/src/goalgeneration_as_planning/cfg/blank_benefit.yaml'
        cfg = '/home/gerard/Nextcloud/goal_generation_4_apa/evaluation_cfgs/rovers/L1_n-1_lb5_ub500/cfg.yaml'
        #cfg = '/tmp/cfg.yaml'

    domain_maker = MakeOSPGoalGenerator(domain, problem, cfg_path=cfg)
    domain_maker.make_pddl_osp_problem()

