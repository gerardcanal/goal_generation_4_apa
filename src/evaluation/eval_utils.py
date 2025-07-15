#!/usr/bin/env python

# Gerard Canal <gerard.canal@kcl.ac.uk>
# King's College London, 2025

class EvalUtils:
    @staticmethod
    def count_goals_in_windows(csv_line, goals, last_times_map, adg, prefix='final'):
        if prefix and prefix[-1] != '_': prefix += '_'
        csv_line[prefix + 'n_goals_in_window'] = 0
        csv_line[prefix + 'n_goals_after_window'] = 0
        csv_line[prefix + 'n_goals_before_window'] = 0
        csv_line[prefix + 'n_goals_in_cycle'] = 0
        csv_line[prefix + 'n_goals_over_cycle'] = 0
        csv_line[prefix + 'n_goals_before_cycle'] = 0
        csv_line[prefix + 'n_goals_benefit'] = 0  # Goals which have explicit benefit
        for grounded_goal in goals:
            goal_name = grounded_goal.name
            if '__' in goal_name:
                goal_name = goal_name[:goal_name.find('__')]
            goal_params = [t.value for t in grounded_goal.typed_parameters]
            params_str = ', '.join(goal_params)

            if goal_name in adg.goal_benefits and params_str in adg.goal_benefits[goal_name]:
                csv_line[prefix + 'n_goals_benefit'] += 1

            # Windows
            if goal_name in adg.timing_windows:
                twindows = adg.timing_windows[goal_name]
                if params_str in twindows:
                    window = twindows[params_str]
                    if adg.current_time < window[0]:
                        # (< (current_time) (goal__window_start ?r0 ?w1))) (decrease (total_benefit) (* 50 (- (goal__window_start ?r0 ?w1) (current_time)))))
                        csv_line[prefix + 'n_goals_before_window'] += 1 # Goals before the window
                    elif adg.current_time > window[1]:
                        # (> (current_time) (goal__window_end ?r0 ?w1))) (decrease (total_benefit) (* 50 (- (current_time) (goal__window_end ?r0 ?w1)))))
                        csv_line[prefix + 'n_goals_after_window'] += 1
                    else: #if self.current_time >= window[0] and self.current_time <= window[1]:
                        csv_line[prefix + 'n_goals_in_window'] += 1
                
            # Cyclics
            goal_last_time = last_times_map[str(grounded_goal)]
            if goal_last_time and goal_name in adg.timing_cyclics:
                cycles = adg.timing_cyclics[goal_name]
                if params_str in cycles:
                    interval = cycles[params_str]
                    time_since_goal = adg.current_time - goal_last_time
                    if time_since_goal < interval:
                        # (< (- (current_time) (goal__last_time ?o0 ?l1)) (goal__interval ?o0 ?l1))) (decrease (total_benefit) (* 50 (- (goal__interval ?o0 ?l1) (- (current_time) (goal__last_time ?o0 ?l1))))))
                        #benefit -= self.outwindow_penalty * (interval - time_since_goal)
                        csv_line[prefix + 'n_goals_before_cycle'] += 1
                    elif time_since_goal > interval:
                        # (> (- (current_time) (goal__last_time ?o0 ?l1)) (goal__interval ?o0 ?l1))) (increase (total_benefit) (* 500 (- (- (current_time) (goal__last_time ?o0 ?l1)) (goal__interval ?o0 ?l1)))))
                        #benefit += self.inwindow_reward * (time_since_goal - interval)
                        csv_line[prefix + 'n_goals_over_cycle'] += 1
                    else: # if time_since_goal == interval:
                        #benefit += self.inwindow_reward
                        csv_line[prefix + 'n_goals_in_cycle'] += 1
