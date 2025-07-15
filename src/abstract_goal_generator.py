#!/usr/bin/env python

# Gerard Canal <gerard.canal@kcl.ac.uk>
# King's College London, 2025

from domain_problem_parser import PDDLExtractor
from datetime import datetime
from mutex_parser import PDDLMutexParser
import yaml
from copy import deepcopy
from pddl.goal_descriptor import GoalConjunction, GoalSimple, AtomicFormula, TypedParameter
from plan_graphs.relaxed_plan_graph import RelaxedPlanGraph
import numpy as np
import time
import subprocess
import re
import os
import uuid

DEBUG = True

class AbstractGoalGenerator:
    def __init__(self, domain_file, problem_file, n_goals=15, current_time=(datetime.now().hour + datetime.now().minute/60.0), check_unreachable=True, check_mutexes=True, uuid=uuid.uuid4().hex):
        self.pddl_extractor = PDDLExtractor(domain_file, problem_file)
        self.domain_name = self.pddl_extractor.get_domain_name()

        self.problem = self.pddl_extractor.problem

        self.uuid = uuid  # Unique identifier for the goal generation instance

        self.n_goals = n_goals  # Horizon in RDDL
        self.current_time = current_time
        self.instance_benefits = {}
        self.grounded_state_fluents = []

        self.init_times = {}
        start_time = time.time()
        self.potential_goals = self.pddl_extractor.find_potential_goals()
        if DEBUG or self.init_times:
            self.init_times['potential_goals_time'] = time.time() - start_time 
            print(f'Finding potential goals took {self.init_times["potential_goals_time"]} seconds')

        self.unreachable_goals = []
        if check_unreachable:
            start_time = time.time()
            self.unreachable_goals = self.find_unachievable_goals()
            if DEBUG or self.init_times: 
                self.init_times['unreachable_goals_time'] = time.time() - start_time
                print(f'Finding unreachable goals took {self.init_times["unreachable_goals_time"]} seconds')

        if check_mutexes:
            start_time = time.time()
            self.mutex_groups = self.find_mutexes()
            self.ungrounded_mutex_groups = self.pddl_mutex_manager._unground_mutex_groups(self.mutex_groups)
            if DEBUG or self.init_times:
                self.init_times['mutexes_time'] = time.time() - start_time
                print(f'Finding mutexes took {self.init_times["mutexes_time"]} seconds')
        else:
            self.ungrounded_mutex_groups = []

        self._state_fluents_from_mutexes()
        self.load_sf_from_problem()
        self.inwindow_reward = 500
        self.outwindow_penalty = 50
        self.no_action_penalty = 50

    def load_cfg(self, yaml_path):
        with open(yaml_path, 'r') as f:
            yaml_cfg = yaml.load(f, Loader=yaml.FullLoader)
            if yaml_cfg:
                if 'current_time' in yaml_cfg:
                    self.current_time = float(yaml_cfg['current_time'])
                if 'last_goal_times' in yaml_cfg:
                    self.last_goal_times = yaml_cfg['last_goal_times']
                else:
                    self.last_goal_times = {}
                if 'instance_benefit' in yaml_cfg:
                    self.instance_benefits = yaml_cfg['instance_benefit']
                else:
                    print(f'No benefit field in {yaml_path}')
                    self.instance_benefits = {}
                if 'goal_benefit' in yaml_cfg:
                        self.goal_benefits = yaml_cfg['goal_benefit']
                else:
                    self.goal_benefits = {}
                if 'timing_windows' in yaml_cfg: # time windows
                    self.timing_windows = yaml_cfg['timing_windows']
                else:
                    self.timing_windows = {}
                if 'timing_cycles' in yaml_cfg:
                    self.timing_cyclics = yaml_cfg['timing_cycles']
                else:
                    self.timing_cyclics = {}
                if 'n_goals' in yaml_cfg:
                    self.n_goals = yaml_cfg['n_goals']
                self.max_gainable_benefit = yaml_cfg['max_benefit'] if 'max_benefit' in yaml_cfg else [-1, -1]

    def find_mutexes(self):
        self.pddl_mutex_manager = PDDLMutexParser(PDDLExtractor(self.pddl_extractor.domain_file, self.pddl_extractor.problem_file))
        output = self.pddl_mutex_manager._run_mutex_finder()
        return self.pddl_mutex_manager._parse_mutex_groups(output)

    # Mutexes may include state variables that are not goals
    def _state_fluents_from_mutexes(self):
        self._sf_mutex = set()
        goal_predicates = [goal.name for goal in self.potential_goals]
        for mutex_group in self.ungrounded_mutex_groups:
            #if mutex_group[0][0] in 
            for i, pred in enumerate(mutex_group[1]):
                if pred[0] not in goal_predicates:
                    self._sf_mutex.add((self.pddl_extractor.domain.predicates[pred[0]], mutex_group, i))

    def set_state(self, state):
        self.grounded_state_fluents += state

    def load_sf_from_problem(self):
        state = []
        for pred in self.problem.propositions:
            for sfm in self._sf_mutex:
                if sfm[0].name == pred.name:
                    state.append(str(pred)[1:-1])
        self.set_state(state)

    def pred_to_pvariable(self, pred, action):
        if any(map(lambda x: pred == x[0].name, self._sf_mutex)):
            return 'sf__' + pred
        if action:
            return 'goal_' + pred
        return 'goal_sel__' + pred

    def generate_domain(self, ddl_path):
        raise NotImplementedError
    
    def generate_instance(self, ddl_path, domain=None):
        raise NotImplementedError
        
    def set_n_goals(self, n_goals):
        self.n_goals = n_goals

    def get_last_goal_time(self, grounded_goal):
        params = ', '.join([x.value for x in grounded_goal.typed_parameters])
        if grounded_goal.name in self.last_goal_times and params in self.last_goal_times[grounded_goal.name]:
            return self.last_goal_times[grounded_goal.name][params]
        return None

    def get_grounded_goal_benefit(self, grounded_goal, goal_last_time=None):
        goal_name = grounded_goal[0].replace('goal_', '')
        if '__' in goal_name:
            goal_name = goal_name[:goal_name.find('__')]
        goal_params = grounded_goal[1:]
        params_str = ', '.join(goal_params)
        n_params = len(goal_params)
        benefit = 0
        for p in goal_params:
            if p in self.instance_benefits:
                benefit += self.instance_benefits[p] 
        benefit /= n_params
        if goal_name in self.goal_benefits and params_str in self.goal_benefits[goal_name]:
            benefit += self.goal_benefits[goal_name][params_str]

        # Windows
        if goal_name in self.timing_windows:
            twindows = self.timing_windows[goal_name]
            if params_str in twindows:
                window = twindows[params_str]
                if self.current_time < window[0]:
                    # (< (current_time) (goal__window_start ?r0 ?w1))) (decrease (total_benefit) (* 50 (- (goal__window_start ?r0 ?w1) (current_time)))))
                    benefit -= self.outwindow_penalty * (window[0] - self.current_time)
                elif self.current_time > window[1]:
                    # (> (current_time) (goal__window_end ?r0 ?w1))) (decrease (total_benefit) (* 50 (- (current_time) (goal__window_end ?r0 ?w1)))))
                    benefit -= self.outwindow_penalty * (self.current_time - window[1])
                else: #if self.current_time >= window[0] and self.current_time <= window[1]:
                    benefit += self.inwindow_reward
            
        # Cyclics
        if goal_last_time is not None and goal_name in self.timing_cyclics:
            cycles = self.timing_cyclics[goal_name]
            if params_str in cycles:
                interval = cycles[params_str]
                time_since_goal = self.current_time - goal_last_time
                if (time_since_goal - interval) < -0.001:
                #if time_since_goal < interval:
                    # (< (- (current_time) (goal__last_time ?o0 ?l1)) (goal__interval ?o0 ?l1))) (decrease (total_benefit) (* 50 (- (goal__interval ?o0 ?l1) (- (current_time) (goal__last_time ?o0 ?l1))))))
                    benefit -= self.outwindow_penalty * (interval - time_since_goal)
                if (time_since_goal - interval) > 0.001:
                #elif time_since_goal > interval:
                    # (> (- (current_time) (goal__last_time ?o0 ?l1)) (goal__interval ?o0 ?l1))) (increase (total_benefit) (* 500 (- (- (current_time) (goal__last_time ?o0 ?l1)) (goal__interval ?o0 ?l1)))))
                    benefit += self.inwindow_reward * (time_since_goal - interval)
                if (time_since_goal - interval) < 0.001 and (time_since_goal - interval) >= -0.001:
                #else: # if time_since_goal == interval:
                    benefit += self.inwindow_reward
                
        return benefit
    

    def get_grounded_predicates(self, predicate_list: list[AtomicFormula]):
        grounding = self.problem.grounding
        grounding.ground_problem(self.pddl_extractor.domain, self.problem)
        grounded_goals = []
        for g in predicate_list:
            id = grounding.predicate_heads[g.name]
            for i in range(id[0], id[1]):
                grounded_goals.append(grounding.get_proposition_from_id(i))
        return grounded_goals
    
    def find_unachievable_goals(self):
        unreachable_goals = []
        grounded_goals = self.get_grounded_predicates(self.potential_goals)
        problem = deepcopy(self.problem)
        problem.goal = GoalConjunction([GoalSimple(g) for g in grounded_goals])
        problem.metric = None
        
        #### Get unreachable goals from rewrite. This must be done in two steps, first to find the trivially false goals, and then to get the RPG ones
        # Write problem to file
        def run_reachability_checker():
            if not os.path.exists('/tmp/goal_generation'):
                os.makedirs('/tmp/goal_generation')
            with open(f'/tmp/goal_generation/{self.uuid}_problem_all_grap_goals.pddl', 'w') as f:
                f.write(str(problem))
            reachability_exe = os.path.dirname(__file__) + '/../bin/reachability_rewrite-no-lp'
            cmd = reachability_exe + ' --do-not-plan ' + self.pddl_extractor.domain_file + f' /tmp/goal_generation/{self.uuid}_problem_all_grap_goals.pddl'
            output = subprocess.check_output(cmd, shell=True, stderr=subprocess.DEVNULL).decode('utf-8')
            return output
        output = run_reachability_checker()
        rw_unreachable_goals = re.findall(r'Unreachable goal in preprocessing:\s+\((.*)\)', output)
        if rw_unreachable_goals: # Remove unreachable goals from problem and call the reachability checker again to get the RPG ones
            # This is needed as the reachability checker does not check RPG goals if trivially false are found. If it is not the case,
            # we use that output normally
            to_rm = []
            for i, g in enumerate(problem.goal.goals):
                g_str = g.atomic_formula.name + ' ' + ' '.join([tp.value for tp in g.atomic_formula.typed_parameters])
                if g_str in rw_unreachable_goals:
                    unreachable_goals.append(g.atomic_formula)
                    to_rm.append(i)
            for i in sorted(to_rm, reverse=True):
                #print(problem.goal.goals[i])
                del problem.goal.goals[i]
            output = run_reachability_checker()
            assert 'Unreachable goal in preprocessing' not in output
        rw_unreachable_goals = re.findall(r'Start of unreachable goals when.*\s(?:[\t ]*;.*$\n)*((?:.*\(.*\).*$\n?)+)End of unreachable goals',
                                          output, re.MULTILINE)
        if rw_unreachable_goals:
            def str_2_goal(g):
                g = g.strip()[1:-1].split(' ') # Strip whitespaces and remove parenthesis
                formula = deepcopy(self.pddl_extractor.domain.predicates[g[0]])
                for i, p in enumerate(g[1:]):
                    formula.typed_parameters[i].value = p
                    if formula.typed_parameters[i].type == 'object': formula.typed_parameters[i].type = 'obj'
                return formula
            unreachable_goals += [str_2_goal(g) for g in rw_unreachable_goals[0].strip().split('\n')]
        return unreachable_goals
        ###

        # RPG imlpementation of otpl seems to be quite resource intensive and slow
        rpg = RelaxedPlanGraph(self.pddl_extractor.domain, problem)
        layer_count = rpg.build_graph(stop_at_goal=True)
        if not rpg.goal_reached:
            unreachable_goals = np.nonzero(rpg.fact_membership[rpg.problem_goal_spike] < 0) # Get bool vector of unreachable goals
            unreachable_goals = np.nonzero(rpg.problem_goal_spike)[0][unreachable_goals] # Get ids of unreachable goals
            unreachable_goals = [rpg.grounding.get_proposition_from_id(x) for x in unreachable_goals] # Get propositions of unreachable goals
            for goal in unreachable_goals:
                goal.typed_parameters = [tp if tp.type != 'object' else TypedParameter('obj', tp.label, tp.value) for tp in goal.typed_parameters]
        return unreachable_goals
