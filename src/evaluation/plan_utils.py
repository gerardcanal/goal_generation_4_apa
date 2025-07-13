#!/usr/bin/python3.9
import re
import os
import time
import random
from pddl.atomic_formula import AtomicFormula
from pddl.domain import Domain
from pddl.problem import Problem
import numpy as np
from plans.temporal_plan import PlanTemporalNetwork

class PlanUtils:
    def __init__(self):
        pass
    
    @staticmethod
    def compute_plan_makespan(plan):
        # plan format: list of (start_time, action, duration)
        makespan = plan[-1][0] + plan[-1][2]  # end of last action
        for i in range(len(plan)-2, 0, -1):
            ms = plan[i][0] + plan[i][2]
            if ms > makespan:
                makespan = ms
        return makespan
    
    def compute_plan_makespan_v2(plan):
        makespan = 0
        for a in plan:
            init = a[0]; duration = a[2]
            if makespan > init:
                # We are interested by the remainder of the action in the time that has not yet passed if the action started before the last action finished
                init = makespan  
                duration -= (makespan - a[0])  # Duration is lowered by the start time difference
            
            ams = init + duration
            if ams > makespan:
                makespan = ams
        return makespan

    @staticmethod
    def get_action_from_plan_line(line):
        line = re.findall(r'(\d+(?:.\d+)?).*\((.*)\).*\[(\d+(?:.\d+)?)\]', line)[0]
        return (float(line[0]), line[1].split(' '), float(line[2]))
    
    @staticmethod
    def get_plan_as_list(plan):
        return [PlanUtils.get_action_from_plan_line(a) for a in plan.rstrip().split('\n')]

    @staticmethod
    def simulate_plan_execution(domain : Domain, problem : Problem, plan):
        ptn = PlanTemporalNetwork(domain, problem)
        # Workaround to not modify  the PlanTemporalNetwork class... # FIXME do it at some point?!
        path = f'/tmp/plan_sim_{time.time()}_{random.randint(0,1000)}.txt'
        with open(path, 'w') as f:
            for a in plan:
                f.write(f'{a[0]}: ({" ".join(a[1])}) [{a[2]}]\n')
        ptn.read_from_file(path)
        #os.remove(path)
        return ptn.simulate_execution()

