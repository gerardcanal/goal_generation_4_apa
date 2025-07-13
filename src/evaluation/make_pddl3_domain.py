#!/usr/bin/env python
import sys; import os; sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))  # Hack to add src to path

from pddl_goal_domain_generator import PDDLDomainGenerator
from pddl_goal_domain_generator import *
from pddl.goal_descriptor import GoalDescriptor
from pddl.effect import EffectType
from pddl.parser import Parser
import copy
import re

USE_BENEFIT = True
PRECOMPUTED_BENEFIT = True
USE_CONDITIONAL_TIMED_EFFECTS = False
BENEFIT_FACTOR = -0.01
TIME_FACTOR = 0.03
VIOLATED_PREFS_FACTOR = 0.5

class MakePDDL3GRAP:
    def __init__(self, domain, problem, cfg_path, use_benefit=USE_BENEFIT, use_conditionals=USE_CONDITIONAL_TIMED_EFFECTS,
                 benefit_factor=BENEFIT_FACTOR, time_factor=TIME_FACTOR, violated_prefs_factor=VIOLATED_PREFS_FACTOR, out_folder=None, postfix=''):
        self.adg = PDDLDomainGenerator(domain, problem, check_unreachable=False)
        self.all_goals = self.adg.get_grounded_predicates(self.adg.potential_goals)
        self.adg.load_cfg(cfg_path) 
        self.use_benefit = use_benefit
        self.precomputed_benefit = PRECOMPUTED_BENEFIT
        self.benefit_factor = benefit_factor
        self.time_factor = time_factor
        self.violated_prefs_factor = violated_prefs_factor
        if not use_conditionals:
            self.adg.timing_windows = {}
            self.adg.timing_cyclics = {}
        self.out_folder = out_folder
        self.postfix = postfix
        self.is_domain_durative = any([op.durative for op in self.adg.pddl_extractor.domain.operators.values()])
        if self.precomputed_benefit:
            self.grounded_potential_goals = self.adg.get_grounded_predicates(self.adg.potential_goals)

    def make_pddl3_problem(self):
        pddl_goal_metric = ('(:goal (and\n')
        for i, g in enumerate(self.all_goals):
            pddl_goal_metric += (f'    (preference p{i} {g})\n')
        pddl_goal_metric += ('))\n')
        pddl_goal_metric += '(:metric minimize '
        if self.time_factor > 0 or self.violated_prefs_factor > 0:
            pddl_goal_metric += '(+ '
        if self.time_factor > 0:
            func = 'total-time' if self.is_domain_durative else 'total-cost'
            pddl_goal_metric += f'(* {self.time_factor} ({func})) '
        pddl_goal_metric += f'(* {self.benefit_factor} (total_benefit))'
        if self.add_violated_prefs_factor():
            pddl_goal_metric += f' (* {self.violated_prefs_factor} (+'
        if self.violated_prefs_factor > 0:
            pddl_goal_metric += '\n'
            for i in range(len(self.all_goals)):
                pddl_goal_metric += (f'    (is-violated p{i})\n')
                #pddl_goal_metric += (f' (is-violated p{i})')
        if self.add_violated_prefs_factor():
            pddl_goal_metric += '  ))\n'
        if self.time_factor > 0 or self.violated_prefs_factor > 0:
            pddl_goal_metric += ')'
        pddl_goal_metric += (')')
        #print(pddl_goal_metric)

        problem = copy.deepcopy(self.adg.problem)
        self.adg._sf_mutex = []
        if self.use_benefit:
            if self.precomputed_benefit:
                self.add_init_state_precomputed_benefit(problem)
            else:
                self.adg.add_init_state(problem, add_supertype_benefit=True)

        for inst, _type in problem.objects_type_map.items():
            if _type == 'object':
                problem.objects_type_map[inst] = 'obj'
        # Add initial state metrics
        problem.goal = GoalDescriptor()
        problem.metric = None
        prob_str = str(problem)
        prob_str = prob_str.replace('(:goal ())', pddl_goal_metric)
        self.problem_file = self.adg.pddl_extractor.problem_file.replace('.pddl', '_with-prefs.pddl')
        if self.out_folder:
            self.problem_file = self.out_folder + self.problem_file.split('/')[-1]
        if self.postfix:
            self.problem_file = self.problem_file.replace('.pddl', '_' + self.postfix + '.pddl')
        with open(self.problem_file, 'w') as f:
            f.write(prob_str)
        
    def add_init_state_precomputed_benefit(self, problem):
        for goal in self.grounded_potential_goals:
            gstr = str(goal)[1:-1].split(' ')
            gbenefit = self.adg.get_grounded_goal_benefit(gstr, self.adg.get_last_goal_time(goal))
            problem.add_assignment_from_str(gbenefit, f'benefit_{goal.name}', [tp.value for tp in goal.typed_parameters])
        problem.add_assignment_from_str(0.0, "total_benefit")

    def add_violated_prefs_factor(self):
        return self.violated_prefs_factor > 0 and self.violated_prefs_factor < 1

    def prepare_domain(self):
        # Fixes issues by changing types like object to obj to prevent otpl errors
        with open(self.adg.pddl_extractor.domain_file, 'r') as f:
            domain = f.read()
        
        types = re.findall(r'\(:types(?:\s*.*\s*)*?\)', domain)[0]
        domain.replace(types, re.sub(r'-\s+object', '- tmp_object', types))
        domain = re.sub(r' object([\)\s])', r' obj\1', domain)
        domain.replace('tmp_object', 'object')
        with open('/tmp/tmp_domain.pddl', 'w') as f:
            f.write(domain)
        parser = Parser()
        parser.parse_file('/tmp/tmp_domain.pddl')
        domain = parser.domain
        return domain
    
    def add_functions(self, d):
        if self.precomputed_benefit:
            for goal in self.grounded_potential_goals:
                params_map = {f'?{tp.type[0]}': tp.type if tp.type != 'object' else 'obj' for tp in goal.typed_parameters}
                d.add_function_from_str('benefit_' + goal.name, params_map)
        else:
            # Adapted from pddl_domain_generator.py
            processed_types = []
            for goal in self.potential_goals:
                for tp in goal.typed_parameters:
                    if tp.type in processed_types:
                        continue
                    d.add_function_from_str(f'benefit_{tp.type if tp.type != "object" else "obj"}', {'?' + tp.type[0]: tp.type if tp.type != 'object' else 'obj'})
                    processed_types.append(tp.type)
            if self.adg.timing_windows or self.adg.timing_cyclics:
                d.add_function_from_str('current_time')
            for pred_name, _ in self.adg.timing_windows.items():
                    formula = self.adg.pddl_extractor.domain.predicates[pred_name]
                    params_map = {'?' + tp.type[0] + str(i): tp.type if tp.type != 'object' else 'obj' for i, tp in enumerate(formula.typed_parameters)}
                    d.add_function_from_str(pred_name + '__window_start', params_map)
                    d.add_function_from_str(pred_name + '__window_end', params_map)
            for pred_name, _ in self.adg.timing_cyclics.items():
                    formula = self.adg.pddl_extractor.domain.predicates[pred_name]
                    params_map = {'?' + tp.type[0] + str(i): tp.type if tp.type != 'object' else 'obj' for i, tp in enumerate(formula.typed_parameters)}
                    d.add_function_from_str(pred_name + '__interval', params_map)
                    d.add_function_from_str(pred_name + '__last_time', params_map)
            for goal, _ in self.adg.goal_benefits.items():
                formula = self.adg.pddl_extractor.domain.predicates[goal]
                params_map = {'?' + tp.type[0] + str(i): tp.type if tp.type != 'object' else 'obj' for i, tp in enumerate(formula.typed_parameters)}
                d.add_function_from_str('benefit_' + goal, params_map)
        d.add_function_from_str('total_benefit')
        d.add_function_from_str('n_actions')

    def add_window_effects(self, op, pred_name, params):
        # Adapted from pddl_domain_generator.py
        total_benefit_function = AtomicFormula("total_benefit")
        current_time = ExprBase(expr_type=ExprBase.ExprType.FUNCTION, function=AtomicFormula("current_time"))
        
        # If before start window...
        start_function = AtomicFormula(f"{pred_name}__window_start", params)
        start_time = ExprBase(expr_type=ExprBase.ExprType.FUNCTION, function=start_function)
        #condition = Inequality(Inequality.ComparisonType.LESS, ExprComposite([current_time]), ExprComposite([start_time]))
        condition = GoalConjunction([Inequality(Inequality.ComparisonType.GREATEREQ, ExprComposite([start_time]), ExprComposite([ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=0)])),
                                     Inequality(Inequality.ComparisonType.LESS, ExprComposite([current_time]), ExprComposite([start_time]))])
        value_expr = ExprComposite([ExprBase(ExprBase.ExprType.BINARY_OPERATOR, op=ExprBase.BinaryOperator.MUL),
                                    ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=self.adg.outwindow_penalty),
                                    ExprBase(ExprBase.ExprType.BINARY_OPERATOR, op=ExprBase.BinaryOperator.SUB), start_time, current_time])
        self.adg.add_complex_effect(EffectConditional(condition, Assignment(AssignmentType.DECREASE, total_benefit_function, value_expr)), op, time_spec=TimeSpec.AT_END)

        # If after end window...
        end_function = AtomicFormula(f"{pred_name}__window_end", params)
        end_time = ExprBase(expr_type=ExprBase.ExprType.FUNCTION, function=end_function)
        #condition = Inequality(Inequality.ComparisonType.GREATER, ExprComposite([current_time]), ExprComposite([end_time]))
        condition = GoalConjunction([Inequality(Inequality.ComparisonType.GREATEREQ, ExprComposite([end_time]), ExprComposite([ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=0)])),
                                     Inequality(Inequality.ComparisonType.GREATER, ExprComposite([current_time]), ExprComposite([end_time]))])
        value_expr = ExprComposite([ExprBase(ExprBase.ExprType.BINARY_OPERATOR, op=ExprBase.BinaryOperator.MUL),
                                    ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=self.adg.outwindow_penalty),
                                    ExprBase(ExprBase.ExprType.BINARY_OPERATOR, op=ExprBase.BinaryOperator.SUB), current_time, end_time])
        self.adg.add_complex_effect(EffectConditional(condition, Assignment(AssignmentType.DECREASE, total_benefit_function, value_expr)), op, time_spec=TimeSpec.AT_END)

        # Add reward for not being in the window
        condition = GoalConjunction([Inequality(Inequality.ComparisonType.GREATEREQ, ExprComposite([start_time]), ExprComposite([ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=0)])),
                                     Inequality(Inequality.ComparisonType.GREATEREQ, ExprComposite([current_time]), ExprComposite([start_time])),
                                     Inequality(Inequality.ComparisonType.LESSEQ, ExprComposite([current_time]), ExprComposite([end_time]))])
        value_expr = ExprComposite([ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=self.adg.inwindow_reward)])
        self.adg.add_complex_effect(EffectConditional(condition, Assignment(AssignmentType.INCREASE, total_benefit_function, value_expr)), op, time_spec=TimeSpec.AT_END)

    def add_cyclic_effect(self, op, pred_name, params):
        # Adapted from pddl_domain_generator.py
        total_benefit_function = AtomicFormula("total_benefit")
        current_time = ExprBase(expr_type=ExprBase.ExprType.FUNCTION, function=AtomicFormula("current_time"))
        
        cycle_interval_fn = AtomicFormula(f"{pred_name}__interval", params)
        cycle_interval = ExprBase(expr_type=ExprBase.ExprType.FUNCTION, function=cycle_interval_fn)

        last_time_fn = AtomicFormula(f"{pred_name}__last_time", params)
        last_time = ExprBase(expr_type=ExprBase.ExprType.FUNCTION, function=last_time_fn)

        time_since_last = ExprComposite([ExprBase(ExprBase.ExprType.BINARY_OPERATOR, op=ExprBase.BinaryOperator.SUB), current_time, last_time])

        #condition = Inequality(Inequality.ComparisonType.LESS, time_since_last, ExprComposite([cycle_interval]))
        condition = GoalConjunction([Inequality(Inequality.ComparisonType.GREATEREQ, ExprComposite([cycle_interval]), ExprComposite([ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=0)])),
                                     Inequality(Inequality.ComparisonType.LESS, time_since_last, ExprComposite([cycle_interval]))])
        value_expr = ExprComposite([ExprBase(ExprBase.ExprType.BINARY_OPERATOR, op=ExprBase.BinaryOperator.MUL),
                                    ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=self.adg.outwindow_penalty),
                                    ExprBase(ExprBase.ExprType.BINARY_OPERATOR, op=ExprBase.BinaryOperator.SUB), cycle_interval, *time_since_last.tokens])
        self.adg.add_complex_effect(EffectConditional(condition, Assignment(AssignmentType.DECREASE, total_benefit_function, value_expr)), op, time_spec=TimeSpec.AT_END)

        # If after the time, we increase benefit, goal is now urgent!
        #condition = Inequality(Inequality.ComparisonType.GREATER, time_since_last, ExprComposite([cycle_interval]))
        condition = GoalConjunction([Inequality(Inequality.ComparisonType.GREATEREQ, ExprComposite([cycle_interval]), ExprComposite([ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=0)])),
                                     Inequality(Inequality.ComparisonType.GREATER, time_since_last, ExprComposite([cycle_interval]))])
        value_expr = ExprComposite([ExprBase(ExprBase.ExprType.BINARY_OPERATOR, op=ExprBase.BinaryOperator.MUL),
                                    ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=self.adg.inwindow_reward),
                                    ExprBase(ExprBase.ExprType.BINARY_OPERATOR, op=ExprBase.BinaryOperator.SUB), *time_since_last.tokens, cycle_interval])
        self.adg.add_complex_effect(EffectConditional(condition, Assignment(AssignmentType.INCREASE, total_benefit_function, value_expr)), op, time_spec=TimeSpec.AT_END)

        condition = Inequality(Inequality.ComparisonType.EQUALS, time_since_last, ExprComposite([cycle_interval]))
        value_expr = ExprComposite([ExprBase(ExprBase.ExprType.CONSTANT, constant=self.adg.inwindow_reward)])
        self.adg.add_complex_effect(EffectConditional(condition, Assignment(AssignmentType.INCREASE, total_benefit_function, value_expr)), op, time_spec=TimeSpec.AT_END)


    def get_action_benefit(self, op: Operator, goal, params):
        # Increase total_benefit
        benefit_val = []
        n = len(goal.typed_parameters)
        add = ExprBase(ExprBase.ExprType.BINARY_OPERATOR, op=ExprBase.BinaryOperator.ADD)
        for i, tp in enumerate(params.items()):
            if i > 0 and (n-i)%2 == 0:
                benefit_val.append(add)
            e = ExprBase(expr_type=ExprBase.ExprType.FUNCTION, 
                         # Replacing "__st" in case if was a supertype, as benefit do not use the artificial __st subtypes
                         function=AtomicFormula(f"benefit_{tp[1].replace('__st', '')}", [TypedParameter(tp[1], tp[0])]))
            benefit_val.append(e)
        if n > 1:  # Add / in the front and len in the back for the prefix notation
            div = ExprBase(ExprBase.ExprType.BINARY_OPERATOR, op=ExprBase.BinaryOperator.DIV)
            benefit_val = [div, add] + benefit_val
            benefit_val.append(ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=n))
        if goal.name in self.adg.goal_benefits:
            benefit_val.insert(0, add)
            benefit_val.append(ExprBase(expr_type=ExprBase.ExprType.FUNCTION, function=AtomicFormula(f"benefit_{goal.name}", [TypedParameter(l, t) for t, l in params.items()])))
        return ExprComposite(benefit_val)

    def make_pddl3_domain(self):
        domain = self.prepare_domain()
        #domain = copy.deepcopy(self.adg.pddl_extractor.domain)
        domain.requirements.append(':preferences') 
        domain.requirements.append(':fluents') 

        if self.use_benefit:
            if not self.precomputed_benefit:
                domain.requirements.append(':conditional-effects') 
            
            self.potential_goals = self.adg.potential_goals
            self.add_functions(domain)

            benefit_effects = {}
            add = ExprBase(ExprBase.ExprType.BINARY_OPERATOR, op=ExprBase.BinaryOperator.ADD)
            for op_name, op in domain.operators.items():
                for g in self.adg.potential_goals:
                    effects = op.effect.effects if op.effect.effect_type != EffectType.TIMED else [op.effect]
                    for e in effects:
                        if e.effect_type == EffectType.TIMED:
                            e = e.effect
                        if e.effect_type == EffectType.SIMPLE and g.name == e.formula.name:
                            print(f'Found {g.name} in {op_name} effect')
                            if self.precomputed_benefit:
                                exp = ExprBase(expr_type=ExprBase.ExprType.FUNCTION, function=AtomicFormula(f"benefit_{g.name}", e.formula.typed_parameters))
                                if op_name in benefit_effects:
                                    benefit_effects[op_name] = ExprComposite([add, *benefit_effects[op_name].tokens, exp])
                                else:
                                    benefit_effects[op_name] = ExprComposite([exp])
                            else:
                                if op_name in benefit_effects:
                                    _benefit = self.get_action_benefit(op, e.formula, {t.label: t.type for t in e.formula.typed_parameters})
                                    benefit_effects[op_name] = ExprComposite([add, *benefit_effects[op_name].tokens, *_benefit.tokens])
                                else:
                                    benefit_effects[op_name] = self.get_action_benefit(op, e.formula, {t.label: t.type for t in e.formula.typed_parameters})
                                if e.formula.name in self.adg.timing_windows:
                                    self.add_window_effects(op, e.formula.name, e.formula.typed_parameters)
                                if e.formula.name in self.adg.timing_cyclics:
                                    self.add_cyclic_effect(op, e.formula.name, e.formula.typed_parameters)

            total_benefit = AtomicFormula.from_string('total_benefit')
            for op_name, ben in benefit_effects.items():
                op = domain.operators[op_name]
                #self.adg.add_complex_assign_effect(total_benefit, ben, op, assign_type=AssignmentType.INCREASE, time_spec=TimeSpec.AT_END)
                self.adg.add_complex_assign_effect(total_benefit, ben, op, assign_type=AssignmentType.INCREASE, time_spec=TimeSpec.AT_START)

        self.domain_file = self.adg.pddl_extractor.domain_file.replace('.pddl', '_with-prefs.pddl')
        if self.out_folder:
            self.domain_file = self.out_folder + self.domain_file.split('/')[-1]
        if self.postfix:
            self.domain_file = self.domain_file.replace('.pddl', '_' + self.postfix + '.pddl')
        with open(self.domain_file, 'w') as f:
            f.write(str(domain))
        self.adg.problem.domain = domain
        print(f"Domain with preferences saved to {self.domain_file}")
        return domain
        with open(self.adg.pddl_extractor.domain_file, 'r') as f:
            domain = f.read()
        domain = domain.replace('(:requirements', '(:requirements :preferences')
        self.domain_file = self.adg.pddl_extractor.domain_file.replace('.pddl', '_with-prefs.pddl')
        with open(self.domain_file, 'w') as f:
            f.write(str(domain))

if __name__ == '__main__':
    #domain = '/tmp/domai.pddl'
    #domain  = '/home/gerard/code/ROSPlan_ws/src/goalgeneration_as_planning/testdomains/domain.pddl'
    #problem = '/tmp/transport_instance-1.pddl'
    #problem = sys.argv[2]
    #problem = '/home/gerard/code/ROSPlan_ws/src/goalgeneration_as_planning/testdomains/problem.pddl'
    import os
    domain = os.path.dirname(__file__) + '/../../domains/rovers-time-simple-automatic/domain.pddl'
    problem = os.path.dirname(__file__) + '/../../domains/rovers-time-simple-automatic/instance-20.pddl'
    
    if len(sys.argv) > 1:
        domain = sys.argv[1]
        problem = sys.argv[2]
    if len(sys.argv) > 2:
        cfg = sys.argv[3]
    else:
        cfg = '/home/gerard/code/ROSPlan_ws/src/goalgeneration_as_planning/cfg/blank_benefit.yaml'
        cfg = '/mnt/Data/grap_results/rovers_results/L2_n50_lb5_ub5/cfg.yaml'
        cfg = '/media/gerard/Elements/backups/arbok/Data/grap_results/rovers_results/L2_n50_lb5_ub5/cfg.yaml'

    domain_maker = MakePDDL3GRAP(domain, problem, cfg_path=cfg)
    domain_maker.make_pddl3_domain()
    domain_maker.make_pddl3_problem()