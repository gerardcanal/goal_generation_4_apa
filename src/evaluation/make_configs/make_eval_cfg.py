#!/usr/bin/python3.9
import sys; import os; sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))))  # Hack to add src to path

import yaml
import argparse
from datetime import datetime
import random
from abstract_goal_generator import AbstractGoalGenerator

class MakeCFG:
    def __init__(self, args):
        self.args = args
        self.cfg = {}
        self.args.current_time = self.round(args.current_time)
        self.cfg['current_time'] = self.args.current_time

        # Init
        self.adg = AbstractGoalGenerator(args.domain, args.problem, current_time=self.args.current_time, check_unreachable=False, check_mutexes=False)
        self.all_goals = self.adg.get_grounded_predicates(self.adg.potential_goals)
        random.shuffle(self.all_goals)  # Make it more random...
        types = set()
        for g in self.all_goals:
            for p in g.typed_parameters: 
                types.add(p.type)
        self.instances = [instance for instance, type in self.adg.problem.objects_type_map.items() if type in types]
        self.cfg['n_goals'] = min(args.n_goals, len(self.all_goals))
        if args.n_goals > len(self.all_goals):
            print('\033[93m' + f'Warning: n_goals > len(all_goals). Using all {len(self.all_goals)} goals instead of {args.n_goals}.' + '\033[0m')

        # Instances
        ci = self.generate_instance_benefits()

        # Goal benefit
        cg = self.generate_goal_benefits()

        print(f'Generated - Instance benefits: {ci["pos"]} positive, {ci["neg"]} negative, {ci["neg"]+ci["pos"]}={self.args.n_instance_benefit}.' +
              f' Goal benefits: {cg["pos"]} positive, {cg["neg"]} negative,  {cg["neg"]+cg["pos"]}={self.args.n_goal_benefit}')

        # Windows
        selected_window_goals = self.generate_window_benefits()

        # Cycles
        self.generate_cycles(selected_window_goals)
        
        # Max benefit
        self.cfg['max_benefit'] = self.compute_benefit_goal_list(self.all_goals, self.cfg['last_goal_times'])
    
    def generate_goal_benefits(self):
        self.cfg['goal_benefit'] = {}
        selected_goals = random.sample(self.all_goals, min(self.args.n_goal_benefit, len(self.all_goals)))
        if self.args.n_goal_benefit > len(self.all_goals):
            print('\033[93m' + f'Warning: n_goal_benefit > len(all_goals). Using all {len(self.all_goals)} goals instead of {self.args.n_goal_benefit}.' + '\033[0m')
        counts = {'total': len(selected_goals)/2, 'pos': 0, 'neg': 0}
        for goal in selected_goals:
            params_str = ', '.join([t.value for t in goal.typed_parameters])
            if not goal.name in self.cfg['goal_benefit']:
                self.cfg['goal_benefit'][goal.name] = {}
            self.cfg['goal_benefit'][goal.name][params_str] = self.get_random_benefit(counts)
        assert counts['pos'] + counts['neg'] == len(selected_goals) and (self.args.lower_bound_benefit >= 0 or abs(counts['pos'] - counts['neg']) < 3), str(counts)
        return counts

    def generate_cycles(self, goals_to_skip=[]):
        total = self.args.n_in_cycle + self.args.n_before_cycle + self.args.n_after_cycle
        goals_to_skip = [g for g in self.all_goals if g not in goals_to_skip]
        selected_goals = random.sample(goals_to_skip, min(total, len(goals_to_skip)))
        
        self.cfg['timing_cycles'] = {}
        self.cfg['last_goal_times'] = {}
        in_counter = before_counter = after_counter = 0
        for goal in selected_goals:
            params_str = ', '.join([t.value for t in goal.typed_parameters])
            if not goal.name in self.cfg['timing_cycles']:
                self.cfg['timing_cycles'][goal.name] = {}
                self.cfg['last_goal_times'][goal.name] = {}
            # Set random cycle
            interval = self.round(random.uniform(0.1, self.args.cycle_bounds))
            self.cfg['timing_cycles'][goal.name][params_str] = interval
            # Set last time to fulfill the cycle constraint
            if in_counter < self.args.n_in_cycle:
                in_counter += 1
                self.cfg['last_goal_times'][goal.name][params_str] = self.round(self.args.current_time-interval)
                assert self.round(self.args.current_time-self.cfg['last_goal_times'][goal.name][params_str]) == interval
            elif before_counter < self.args.n_before_cycle:  # Before the interval hits. So, between current_time and current_time + interval
                before_counter += 1
                val = self.round(random.uniform(self.args.current_time-interval+0.05, self.args.current_time))
                self.cfg['last_goal_times'][goal.name][params_str] = val
                assert (self.args.current_time-self.cfg['last_goal_times'][goal.name][params_str]) < interval
            else: # after_counter < self.args.n_after_cycle
                after_counter += 1
                val = self.round(random.uniform(self.args.current_time-interval-0.1, self.args.current_time-interval-self.args.cycle_bounds))
                self.cfg['last_goal_times'][goal.name][params_str] = val
                assert (self.args.current_time-self.cfg['last_goal_times'][goal.name][params_str]) > interval

        assert before_counter + after_counter + in_counter == total
        return selected_goals

    def generate_window_benefits(self, goals_to_skip=[]):
        total = self.args.n_in_time_windows + self.args.n_out_time_windows
        goals_to_skip = [g for g in self.all_goals if g not in goals_to_skip]
        selected_goals = random.sample(goals_to_skip, min(total, len(goals_to_skip)))

        self.cfg['timing_windows'] = {}
        in_counter = out_counter = 0
        for goal in selected_goals:
            params_str = ', '.join([t.value for t in goal.typed_parameters])
            if not goal.name in self.cfg['timing_windows']:
                self.cfg['timing_windows'][goal.name] = {}
            if in_counter < self.args.n_in_time_windows:
                in_counter += 1
                self.cfg['timing_windows'][goal.name][params_str] = self.get_in_bounds() # Make windows of random +- 2hours
                assert self.args.current_time >= self.cfg['timing_windows'][goal.name][params_str][0] and\
                       self.args.current_time <= self.cfg['timing_windows'][goal.name][params_str][1], f'{self.args.current_time} >= {self.cfg["timing_windows"][goal.name][params_str][0]} and ' +\
                       f'{self.args.current_time} <= {self.cfg["timing_windows"][goal.name][params_str][1]}'
            else:
                out_counter += 1
                self.cfg['timing_windows'][goal.name][params_str] = self.get_out_bounds() # Make windows of random +- 2hours
                assert self.args.current_time < self.cfg['timing_windows'][goal.name][params_str][0] or\
                       self.args.current_time > self.cfg['timing_windows'][goal.name][params_str][1], f'{self.args.current_time} < {self.cfg["timing_windows"][goal.name][params_str][0]} or ' +\
                       f'{self.args.current_time} > {self.cfg["timing_windows"][goal.name][params_str][1]}'

        assert in_counter + out_counter == total
        return selected_goals

    def get_in_bounds(self):
        # Should be %24, purposedly leaving it like this to avoid messing up with the windows in the domain definition (which consider something
        # as in if time > start and < end). If %24 is used, the windows will be in the next day, and the domain will not consider them as in.
        bounds = [self.round(self.args.current_time-random.uniform(0.1, self.args.window_bounds)),
                  self.round(self.args.current_time+random.uniform(0.1, self.args.window_bounds))]
        if bounds[0] < 0:
            bounds[0] = self.round(random.uniform(0, max(self.args.current_time-0.01, 0)))
        assert bounds[0] < bounds[1]
        return bounds
    
    def get_out_bounds(self):
        start = self.round(random.uniform(0, 23)) # Any random start time from 0 to 23 to give time to end the window
        if start == self.args.current_time: # Should be unlikely, but to play safe
            start += random.random()
            start = self.round(min(23, start))
            print('\033[93m' + 'Warning: start == current_time. New start:', start, 'current_time:', self.args.current_time, '\033[0m')
        if start < self.args.current_time:
            if self.args.current_time - start <= 0.5:
                start -= 1
            end = self.round(random.uniform(start+0.1, self.args.current_time-0.1))
        else:
            end = self.round(random.uniform(start+0.1, 23.999))
        assert start < end
        return [start, end]
    
    def generate_instance_benefits(self):
        self.cfg['instance_benefit'] = {}
        instances = random.sample(self.instances, min(self.args.n_instance_benefit, len(self.instances)))
        if self.args.n_instance_benefit > len(self.instances):
            print('\033[93m' + f'Warning: n_instance_benefit > len(instances). Using all {len(self.instances)} instances instead of {self.args.n_instance_benefit}.' + '\033[0m')
        counts = {'total': round(len(instances)/2), 'pos': 0, 'neg': 0}
        for instance in instances:
            self.cfg['instance_benefit'][instance] = self.get_random_benefit(counts)
        assert counts['pos'] + counts['neg'] == len(instances) and (self.args.lower_bound_benefit >= 0 or abs(counts['pos'] - counts['neg']) < 3), str(counts)
        return counts

    def get_random_benefit(self, counts):
        if self.args.lower_bound_benefit < 0:
            if counts['neg'] == counts['total'] and counts['pos'] < counts['total']:
                counts['pos'] += 1
                return self.round(random.uniform(0, self.args.upper_bound_benefit))
            if counts['pos'] == counts['total'] and counts['neg'] < counts['total']:
                counts['neg'] += 1
                return self.round(random.uniform(self.args.lower_bound_benefit, 0))
        r = self.round(random.uniform(self.args.lower_bound_benefit, self.args.upper_bound_benefit))
        if r > 0: counts['pos'] += 1
        elif r <= 0: counts['neg'] += 1  # Consider 0 as negative (won't make it be selected in general)
        return r

    def compute_benefit_goal_list(self, goals, last_times_map, print_goals=False):
        self.adg.current_time = float(self.cfg['current_time'])
        self.adg.last_goal_times = self.cfg['last_goal_times']
        self.adg.instance_benefits = self.cfg['instance_benefit']
        self.adg.goal_benefits = self.cfg['goal_benefit']
        self.adg.timing_windows = self.cfg['timing_windows']
        self.adg.timing_cyclics = self.cfg['timing_cycles']

        total_benefit = 0
        total_pos_benefit = 0
        for g in goals:
            params = [t.value for t in g.typed_parameters]
            params_str = ', '.join(params)
            lt = self.args.current_time
            if g.name in last_times_map and params_str in last_times_map[g.name]:
                lt = last_times_map[g.name][params_str]
            
            b = self.adg.get_grounded_goal_benefit([g.name] + params, lt)
            if print_goals: print('\t\t', g.name + '(' + params_str + ')', b)
            total_benefit += b
            if b > 0:
                total_pos_benefit += b
        return [self.round(total_benefit), self.round(total_pos_benefit)]

    def write_cfg_file(self, path):
        with open(path, 'w') as yaml_file:
            yaml.dump(self.cfg, yaml_file, default_flow_style=False, sort_keys=False)
        print(f'Done writing config to {path} with current time {self.args.current_time}')
    
    @staticmethod
    def round(x, d=2):
        return round(x, d)

def check_args(args=None):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-d", '--domain', type=str,  help="Domain to be evaluated", default='/home/gerard/code/ROSPlan_ws/src/goalgeneration_as_planning/testdomains/domain.pddl')
    parser.add_argument("-p", '--problem', type=str,  help="Problem to be evaluated", default='/home/gerard/code/ROSPlan_ws/src/goalgeneration_as_planning/testdomains/problem.pddl')

    parser.add_argument("-o", '--output', type=str,  help="Output YAML file", default='/tmp/cfg.yaml')

    parser.add_argument("-n", '--n_goals', type=int,  help="Number of goals to generate", default=10)
    parser.add_argument("-c", '--current_time', type=str,  help="Time used as present for the time window evaluation. For a random value [0, 23.99] use 'r'", default=str(datetime.now().hour + datetime.now().minute/60.0))
    
    parser.add_argument("-lb", '--lower_bound_benefit', type=float,  help="Lower bound of benefit to generate", default=-500)
    parser.add_argument("-ub", '--upper_bound_benefit', type=float,  help="Upper bound of benefit to generate", default=500)
    
    parser.add_argument("-i", '--n_instance_benefit', type=int,  help="Number of random instance benefits to generate", default=10)
    parser.add_argument("-g", '--n_goal_benefit', type=int,  help="Number of random grounded goal benefits to generate", default=10)
    parser.add_argument("-ti", '--n_in_time_windows', type=int,  help="Number of random goals in a time window", default=10)
    parser.add_argument("-to", '--n_out_time_windows', type=int,  help="Number of random goals in a time window", default=10)

    parser.add_argument("-ci", '--n_in_cycle', type=int,  help="Number of cyclic random goals in time", default=10)
    parser.add_argument("-cb", '--n_before_cycle', type=int,  help="Number of cyclic random goals before time", default=10)
    parser.add_argument("-ca", '--n_after_cycle', type=int,  help="Number of cyclic random goals after time", default=10)
    
    parser.add_argument("-wb", '--window_bounds', type=float,  help="Time in hours to create the windows inside the goals", default=3)
    parser.add_argument("-cbd", '--cycle_bounds', type=float,  help="Time in hours to create the cycles inside the goals", default=12)

    args = parser.parse_args(args)
    if args.current_time.lower() == 'r':
        args.current_time = random.uniform(0, 23) # until 23 to give some time to get windows out of bounds later
    else:
        args.current_time = float(args.current_time)
    return args

if __name__ == '__main__':
    args = check_args()
    makecfg = MakeCFG(args)
    makecfg.write_cfg_file(args.output)