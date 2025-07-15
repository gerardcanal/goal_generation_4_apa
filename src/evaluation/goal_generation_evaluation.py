#!/usr/bin/env python

# Gerard Canal <gerard.canal@kcl.ac.uk>
# King's College London, 2025

import sys; import os; sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))  # Hack to add src to path

from ILP_goal_generation_pipeline import GoalGenerationPipeline, parse_args, eprint
import time
from eval_utils import EvalUtils
from plan_utils import PlanUtils

class GGEval(GoalGenerationPipeline):
    def __init__(self, *args, **kwargs):
        csv_file_path = args[0].csv_path
        should_write_header = not os.path.exists(csv_file_path)
        self.csv_file = open(csv_file_path, 'a')
        self.csv_header(should_write_header)
        self.csv_line = {'init_total_time': 0} # Initialise csv_line

        start_time = time.time()
        super().__init__(*args, **kwargs)
        self.csv_line['init_total_time'] = time.time() - start_time
        #self.csv_line['init_total_time'] = sum([t for _, t in self.goal_generator.agg.init_times.items()])

        self.csv_line['comment'] = ''
        self.csv_line['planner_timeout'] = self.planner_timeout
        self.csv_line.update(self.goal_generator.agg.init_times)

    def csv_header(self, write_header):
        self.csv_fields = ['init_total_time', 'potential_goals_time', 'unreachable_goals_time', 'mutexes_time', 
                           'gg_solver_time', 'solver_w_init_time', 'total_time', 'remove_percent', 'removal_strategy',
                           'n_replans', # Number of times the planner had to replan after removing goals
                           'n_unsolvable_plans',
                           'requested_goals', 'num_generated_goals', 'final_solved_goals', 
                           'initial_n_goals_in_window', 'final_n_goals_in_window',
                           'initial_n_goals_after_window', 'final_n_goals_after_window',
                           'initial_n_goals_before_window', 'final_n_goals_before_window',
                           'initial_n_goals_in_cycle', 'final_n_goals_in_cycle',
                           'initial_n_goals_over_cycle', 'final_n_goals_over_cycle',
                           'initial_n_goals_before_cycle', 'final_n_goals_before_cycle', 
                           'initial_n_goals_benefit', 'final_n_goals_benefit',
                           'planner_timeout',
                           'plan_length',
                           'plan_makespan',
                           'initial_gen_benefit', 'max_benefit', 'max_positive_benefit', 'final_benefit', 'benefit_difference',
                           'measured_planning_time', 'computed_planning_time', 'solved_reported_planning_time',
                                                     'first_computed_planning_time', 'first_solved_reported_planning_time',
                           'comment'
                          ]
        if write_header:
            self.csv_file.write(','.join(self.csv_fields) + '\n') # Write header

    def evaluate_pipeline(self):
        # TODO2: move things out of the pddl3_evaluation and reuse them when needed!

        self.csv_line['max_benefit'] = self.goal_generator.agg.max_gainable_benefit[0]
        self.csv_line['max_positive_benefit'] = self.goal_generator.agg.max_gainable_benefit[1]
        self.csv_line['requested_goals'] = self.goal_generator.agg.n_goals
        self.csv_line['remove_percent'] = self.remove_percent
        self.csv_line['removal_strategy'] = self.removal_strategy

        start_time = time.time()
        goals, solver_benefit, solver_time, computed_solver_time = self.generate_goals()
        
        self.csv_line['gg_solver_time'] = computed_solver_time
        self.csv_line['solver_w_init_time'] = computed_solver_time + self.csv_line['init_total_time']
        self.csv_line['n_replans'] = 0
        self.csv_line['n_unsolvable_plans'] = 0

        if not goals:
            eprint('No goals found!')
            self.csv_line['comment'] += '[WARN] No goals found! '
            self.csv_line['num_generated_goals'] = self.csv_line['initial_gen_benefit'] = 0
            self.csv_line['gg_solver_time'] = computed_solver_time
            self.csv_line['final_solved_goals'] =  self.csv_line['plan_makespan'] = self.csv_line['final_benefit'] = self.csv_line['num_generated_goals'] = 0
            self.csv_line['total_time'] = time.time() - start_time
            self.csv_line['plan_length'] = self.csv_line['benefit_difference'] = -1
            self.csv_line['measured_planning_time'] = self.csv_line['computed_planning_time'] = self.csv_line['solved_reported_planning_time'] = -1
            self.csv_line['first_computed_planning_time'] = self.csv_line['first_solved_reported_planning_time'] = -1
            EvalUtils.count_goals_in_windows(self.csv_line, self.goals_wo_benefit(goals), {}, self.goal_generator.agg, 'initial')
            EvalUtils.count_goals_in_windows(self.csv_line, self.goals_wo_benefit(goals), {}, self.goal_generator.agg, 'final') 
            self.write_csv_line(self.csv_line)
            return
        
        last_times_map = self.get_last_time_map(goals)
        print('Sorted goals:')
        total_benefit = self.compute_benefit_goal_list(goals, last_times_map, True)
        print('\tTotal benefit:', total_benefit, '(Solver benefit:', str(solver_benefit) +'). Solver time: ' + str(solver_time) + ' milliseconds, total time: ' + str(computed_solver_time) + ' seconds)')
        assert abs(total_benefit-solver_benefit) < 0.1, 'Solver benefit is different to the planner\'s benefit!'

        # Goals metrics
        self.csv_line['num_generated_goals'] = len(goals)
        self.csv_line['initial_gen_benefit'] = solver_benefit
        EvalUtils.count_goals_in_windows(self.csv_line, self.goals_wo_benefit(goals), last_times_map, self.goal_generator.agg, 'initial')

        planning_start_time = time.time()
        success, plan, goals, self.csv_line['n_replans'], self.csv_line['n_unsolvable_plans'], first_plan = self.solve_goals_pipeline(goals, last_times_map)
        if not success:
            eprint(f'[WARN] Failed to solve for any set of goals! ({plan})')
            self.csv_line['comment'] += f'[WARN] Failed to solve for any set of goals! ({plan}) '
            self.csv_line['final_solved_goals'] =  self.csv_line['plan_makespan'] = self.csv_line['final_benefit'] = 0
            self.csv_line['total_time'] = time.time() - start_time
            self.csv_line['plan_length'] = self.csv_line['benefit_difference'] = -1
            self.csv_line['measured_planning_time'] = self.csv_line['computed_planning_time'] = self.csv_line['solved_reported_planning_time'] = -1
            self.csv_line['first_computed_planning_time'] = self.csv_line['first_solved_reported_planning_time'] = -1
            EvalUtils.count_goals_in_windows(self.csv_line, self.goals_wo_benefit(goals), last_times_map, self.goal_generator.agg, 'final') 
            self.write_csv_line(self.csv_line)
            return
        
        end_time = time.time()
        print('Final solved goals:')
        final_benefit = self.compute_benefit_goal_list(goals, last_times_map, True)
        print('\tFinal benefit:', final_benefit)
        if end_time - start_time > 60:
            print('\tTotal time:', round((end_time - start_time) / 60.0, 2), 'minutes')
        else:
            print('\tTotal time:', end_time - start_time, 'seconds')
    
        self.csv_line['measured_planning_time'] = end_time - planning_start_time
        self.csv_line['computed_planning_time'] = self.csv_line['n_replans'] * self.planner_timeout + float(plan[0])
        self.csv_line['first_computed_planning_time'] = self.csv_line['n_replans'] * self.planner_timeout + float(first_plan[0])
        self.csv_line['solved_reported_planning_time'] = float(plan[0])
        self.csv_line['first_solved_reported_planning_time'] = float(first_plan[0])
        self.csv_line['final_solved_goals'] = len(goals)
        self.csv_line['final_benefit'] = final_benefit
        self.csv_line['benefit_difference'] = final_benefit - self.csv_line['initial_gen_benefit']
        self.csv_line['total_time'] = end_time - start_time
        self.csv_line['plan_length'] = len(plan)
        self.csv_line['plan_makespan'] = PlanUtils.compute_plan_makespan(plan)
        self.csv_line['comment'] += f'Completed successfully. '
        EvalUtils.count_goals_in_windows(self.csv_line, self.goals_wo_benefit(goals), last_times_map, self.goal_generator.agg, 'final')
        self.write_csv_line(self.csv_line)
    
    @staticmethod
    def goals_wo_benefit(goals):
        return [g[0] for g in goals]

    def write_csv_line(self, line):
        for field in self.csv_fields:
            assert field in line, f'Field {field} not in line {line}'
        for field in line:
            assert field in self.csv_fields, f'Field {field} not in csv fields'
        self.csv_file.write(','.join([str(line[field]) for field in self.csv_fields]) + '\n')
        self.csv_file.flush()

    def __del__(self):
        self.csv_file.close()
        print(f'Closed CSV file {self.csv_file.name}')

if __name__ == '__main__':
   args = parse_args(eval=True)
   #print(args)
   gpp = GGEval(args)
   gpp.evaluate_pipeline()