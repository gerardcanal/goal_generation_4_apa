#!/usr/bin/env python

# Gerard Canal <gerard.canal@kcl.ac.uk>
# King's College London, 2025

from pddl.domain import Domain
from pddl.problem import Problem
from pddl.operator import Operator
from pddl.goal_descriptor_inequality import Inequality
from pddl.metric import Metric, MetricSpec
from pddl.expression import ExprBase, ExprComposite
from pddl.atomic_formula import AtomicFormula, TypedParameter
from pddl.effect_assignment import AssignmentType, Assignment, AssignmentType
from pddl.effect import TimedEffect, EffectConjunction, EffectType, EffectForall, EffectSimple, EffectNegative, EffectConditional
from pddl.time_spec import TimeSpec
from pddl.goal_descriptor import GoalConjunction, GoalSimple, GoalQuantified, GoalNegative, GoalType, TimedGoal
from datetime import datetime, timezone
from abstract_goal_generator import AbstractGoalGenerator
import copy
import itertools

DEFAULT_BENEFIT = 0.0

class PDDLDomainGenerator(AbstractGoalGenerator):
    def __init__(self, domain_file, problem_file, n_goals=15, current_time=(datetime.now().hour + datetime.now().minute/60.0), check_unreachable=True):
        super().__init__(domain_file, problem_file, n_goals, current_time, check_unreachable)

        self.op_types_map = {}

    def generate_domain(self, pddl_path):
        # Using OTPL to write the domain
        d = Domain(f'{self.domain_name}_goal_generator')
        
        # Requeriments
        d.requirements = [":strips", ":typing", ":equality", ":fluents", ':negative-preconditions', ':conditional-effects']
        
        self.add_types(d)
        self.add_predicates(d)
        self.add_functions(d)
        self.add_actions(d)  # TODO add conditions and effects to actions

        # Write to file
        with open(pddl_path, 'w') as f:
            self.write_header(f)
            f.write(str(d))
        return d

    def generate_instance(self, pddl_path, generated_domain):
        problem = Problem(f"{self.domain_name}_goal_generator_problem", generated_domain)
        # Lowercase all instances
        self.problem.type_objects_map = {k.lower(): v for k, v in self.problem.type_objects_map.items()}
        self.add_objects(problem)
        self.add_init_state(problem)
        self.add_goal(problem)
        self.add_metric(problem)
            
        # Write to file
        with open(pddl_path, 'w') as f:
            self.write_header(f, 'PROBLEM')
            f.write(str(problem))
        return problem

    def add_types(self, d: Domain):
        goal_types = set()
        for goal in self.potential_goals:
            for i, tp in enumerate(goal.typed_parameters):
                if tp.type == 'object':  # In case the PDDL has a type object, we rename it to obj in RDDL as object is a reserved type
                    goal_types.add('obj')
                    goal.typed_parameters[i].type = 'obj'
                    self.problem.type_objects_map['obj'] = self.problem.type_objects_map['object']
                    self.problem.type_objects_map.pop('object')
                else:
                    goal_types.add(tp.type)
                    if self.pddl_extractor.is_supertype(tp.type):
                        subtypes = self.pddl_extractor.get_subtypes(tp.type)
                        for subtype in subtypes:
                            goal_types.add(subtype)
        
        tree = self.pddl_extractor.get_type_tree()
        for _, ptype in tree.items():  # To find supertypes
            if ptype.name == 'object':
                d.add_type('obj')
            elif ptype.name in goal_types:
                d.add_type(ptype.name, ptype.parent)
            if ptype.parent in goal_types:  
                # If we have a supertype, we add a new subtype. This is done becaue then we duplicate all goal actions that have a supertype parameter
                # in case there are mutexes that affect only a supertype. Given that some instances may have the supertype as the type,
                # we need an action for the supertype too. In that case, the planner can use that action to bypass the mutex precondition on the subtype action.
                # To avoid this, we create a new subtype that represents instances of the supertype (with the same supertype), and the corresponding action
                # will have parameters of that subtype. This way, the planner will not be able to bypass the mutex precondition.
                d.add_type(ptype.parent + '__st', ptype.parent)

    def add_predicates(self, d):
        for goal in self.potential_goals:
            params = {'?' + tp.type[0] + str(i): tp.type for i, tp in enumerate(goal.typed_parameters)}
            d.add_predicate_from_str(f'goal_sel__{goal.name}', params)

        for sf, _, _ in self._sf_mutex:
            params = {'?' + tp.type[0] + str(i): tp.type for i, tp in enumerate(sf.typed_parameters)}
            d.add_predicate_from_str(f'sf__{sf.name}', params)

        self.unreachable_preds = []
        if self.unreachable_goals:
            for goal in self.unreachable_goals:
                if goal.name in self.unreachable_preds: continue
                params = {'?' + tp.type[0] + str(i): tp.type if tp.type != 'object' else 'obj' for i, tp in enumerate(goal.typed_parameters)}
                d.add_predicate_from_str(f'is_unreachable_goal__{goal.name}', params)
                self.unreachable_preds.append(goal.name)

    def add_functions(self, d):
        processed_types = []
        for goal in self.potential_goals:
            for tp in goal.typed_parameters:
                if tp.type in processed_types:
                    continue
                d.add_function_from_str(f'benefit_{tp.type}', {'?' + tp.type[0]: tp.type})
                processed_types.append(tp.type)

                if self.pddl_extractor.is_supertype(tp.type):
                    subtypes = self.pddl_extractor.get_subtypes(tp.type)
                    for subtype in subtypes:
                        if subtype in processed_types:
                            continue
                        d.add_function_from_str(f'benefit_{subtype}', {'?' + subtype[0]: subtype})
                        processed_types.append(subtype)

        if self.timing_windows or self.timing_cyclics:
            d.add_function_from_str('current_time')
        for pred_name, _ in self.timing_windows.items():
                formula = self.pddl_extractor.domain.predicates[pred_name]
                params_map = {'?' + tp.type[0] + str(i): tp.type for i, tp in enumerate(formula.typed_parameters)}
                d.add_function_from_str(pred_name + '__window_start', params_map)
                d.add_function_from_str(pred_name + '__window_end', params_map)
        for pred_name, _ in self.timing_cyclics.items():
                formula = self.pddl_extractor.domain.predicates[pred_name]
                params_map = {'?' + tp.type[0] + str(i): tp.type for i, tp in enumerate(formula.typed_parameters)}
                d.add_function_from_str(pred_name + '__interval', params_map)
                d.add_function_from_str(pred_name + '__last_time', params_map)
        for goal, _ in self.goal_benefits.items():
            formula = self.pddl_extractor.domain.predicates[goal]
            params_map = {'?' + tp.type[0] + str(i): tp.type for i, tp in enumerate(formula.typed_parameters)}
            d.add_function_from_str('benefit_' + goal, params_map)
        d.add_function_from_str('total_benefit')
        d.add_function_from_str('n_actions')

    def add_actions(self, d: Domain):
        def add_action(op_name, params, goal):
            d.add_operator_from_str(op_name, params, durative=False)
            op = d.operators[op_name]
            op.add_simple_condition_from_str(f"goal_sel__{goal.name}", params, is_negative=True)

            if goal.name in self.unreachable_preds:
                op.add_simple_condition_from_str(f"is_unreachable_goal__{goal.name}", params, is_negative=True)

            op.add_simple_effect_from_str(f"goal_sel__{goal.name}", params, is_delete=False)
            # Actions + 1
            op.add_assign_effect_from_str('n_actions', assign_type=AssignmentType.INCREASE, value=1)
            self.add_action_benefit(op, goal, params)
            if goal.name in self.timing_windows:
                self.add_window_effects(op, goal.name)
            if goal.name in self.timing_cyclics:
                self.add_cyclic_effect(op, goal.name)
            
            # Effect
            op.add_simple_effect_from_str(f"goal_sel__{goal.name}", params, is_delete=False)  # FIXME this is not needed when a forall mutex on itself is added

        for goal in self.potential_goals:
            supertype_params = self._get_supertype_params(goal.typed_parameters)
            op_name = f'goal_{goal.name}'
            if supertype_params: 
                # If there are parameters which are supertype, we generate an action for all possible combinations of subtypes
                # so that we can get an action for each subtype (i.e., goal_<action>__subtype1_subtype2)
                self.op_types_map[op_name] = []
                pns = self._permut_param_names(supertype_params, [None]*len(supertype_params))
                for pn in pns:  # For all permutations of subtypes
                    # New action will be op_name__subtype1_subtype2. Removing the "__st" in case it was a supertype for improved readability
                    st_op_name = op_name + '__' + '_'.join(pn).replace('__st', '')
                    params = self._remap_subtypes(goal.typed_parameters, supertype_params, pn)  # Rename params to match the subtypes
                    add_action(st_op_name, params, goal)
                    self.op_types_map[op_name].append(st_op_name)
            else:
                params = {'?' + tp.type[0] + str(i): tp.type for i, tp in enumerate(goal.typed_parameters)}
                add_action(op_name, params, goal)
                self.op_types_map[op_name] = [op_name]
        
        #self.mutex_existential_preconditions(d)
        self.mutex_effects(d)
        self.mutex_sf_effects(d)
        
    def add_window_effects(self, op, pred_name):
        total_benefit_function = AtomicFormula("total_benefit")
        current_time = ExprBase(expr_type=ExprBase.ExprType.FUNCTION, function=AtomicFormula("current_time"))
        
        # If before start window...
        start_function = AtomicFormula(f"{pred_name}__window_start", op.formula.typed_parameters)
        start_time = ExprBase(expr_type=ExprBase.ExprType.FUNCTION, function=start_function)
        #condition = Inequality(Inequality.ComparisonType.LESS, ExprComposite([current_time]), ExprComposite([start_time]))
        condition = GoalConjunction([Inequality(Inequality.ComparisonType.GREATEREQ, ExprComposite([start_time]), ExprComposite([ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=0)])),
                                     Inequality(Inequality.ComparisonType.LESS, ExprComposite([current_time]), ExprComposite([start_time]))])
        value_expr = ExprComposite([ExprBase(ExprBase.ExprType.BINARY_OPERATOR, op=ExprBase.BinaryOperator.MUL),
                                    ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=self.outwindow_penalty),
                                    ExprBase(ExprBase.ExprType.BINARY_OPERATOR, op=ExprBase.BinaryOperator.SUB), start_time, current_time])
        self.add_complex_effect(EffectConditional(condition, Assignment(AssignmentType.DECREASE, total_benefit_function, value_expr)), op)

        # If after end window...
        end_function = AtomicFormula(f"{pred_name}__window_end", op.formula.typed_parameters)
        end_time = ExprBase(expr_type=ExprBase.ExprType.FUNCTION, function=end_function)
        #condition = Inequality(Inequality.ComparisonType.GREATER, ExprComposite([current_time]), ExprComposite([end_time]))
        condition = GoalConjunction([Inequality(Inequality.ComparisonType.GREATEREQ, ExprComposite([end_time]), ExprComposite([ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=0)])),
                                     Inequality(Inequality.ComparisonType.GREATER, ExprComposite([current_time]), ExprComposite([end_time]))])
        value_expr = ExprComposite([ExprBase(ExprBase.ExprType.BINARY_OPERATOR, op=ExprBase.BinaryOperator.MUL),
                                    ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=self.outwindow_penalty),
                                    ExprBase(ExprBase.ExprType.BINARY_OPERATOR, op=ExprBase.BinaryOperator.SUB), current_time, end_time])
        self.add_complex_effect(EffectConditional(condition, Assignment(AssignmentType.DECREASE, total_benefit_function, value_expr)), op)

        # Add reward for not being in the window
        condition = GoalConjunction([Inequality(Inequality.ComparisonType.GREATEREQ, ExprComposite([start_time]), ExprComposite([ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=0)])),
                                     Inequality(Inequality.ComparisonType.GREATEREQ, ExprComposite([current_time]), ExprComposite([start_time])),
                                     Inequality(Inequality.ComparisonType.LESSEQ, ExprComposite([current_time]), ExprComposite([end_time]))])
        value_expr = ExprComposite([ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=self.inwindow_reward)])
        self.add_complex_effect(EffectConditional(condition, Assignment(AssignmentType.INCREASE, total_benefit_function, value_expr)), op)

    def add_cyclic_effect(self, op, pred_name):
        threshold = 0.0001
        total_benefit_function = AtomicFormula("total_benefit")
        current_time = ExprBase(expr_type=ExprBase.ExprType.FUNCTION, function=AtomicFormula("current_time"))
        
        cycle_interval_fn = AtomicFormula(f"{pred_name}__interval", op.formula.typed_parameters)
        cycle_interval = ExprBase(expr_type=ExprBase.ExprType.FUNCTION, function=cycle_interval_fn)

        last_time_fn = AtomicFormula(f"{pred_name}__last_time", op.formula.typed_parameters)
        last_time = ExprBase(expr_type=ExprBase.ExprType.FUNCTION, function=last_time_fn)

        sub = ExprBase(ExprBase.ExprType.BINARY_OPERATOR, op=ExprBase.BinaryOperator.SUB)
        time_since_last = ExprComposite([sub, current_time, last_time])
        ineq_lhs = ExprComposite([sub, *time_since_last.tokens, cycle_interval])

        #condition = Inequality(Inequality.ComparisonType.LESS, time_since_last, ExprComposite([cycle_interval]))
        if threshold == 0:  # Precise floating point operations
            condition = GoalConjunction([Inequality(Inequality.ComparisonType.GREATEREQ, ExprComposite([cycle_interval]), ExprComposite([ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=0)])), # interval > 0
                                        Inequality(Inequality.ComparisonType.LESS, time_since_last, ExprComposite([cycle_interval]))]) # time < interval
        else:
            condition = GoalConjunction([Inequality(Inequality.ComparisonType.GREATEREQ, ExprComposite([cycle_interval]), ExprComposite([ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=0)])), # interval > 0
                                        Inequality(Inequality.ComparisonType.LESS, ineq_lhs, ExprComposite([ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=-threshold)]))]) # time - interval < -threshold
        value_expr = ExprComposite([ExprBase(ExprBase.ExprType.BINARY_OPERATOR, op=ExprBase.BinaryOperator.MUL),
                                    ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=self.outwindow_penalty),
                                    sub, cycle_interval, *time_since_last.tokens])

        self.add_complex_effect(EffectConditional(condition, Assignment(AssignmentType.DECREASE, total_benefit_function, value_expr)), op)

        # If after the time, we increase benefit, goal is now urgent!
        #condition = Inequality(Inequality.ComparisonType.GREATER, time_since_last, ExprComposite([cycle_interval]))
        if threshold == 0:  # Precise floating point operations
            condition = GoalConjunction([Inequality(Inequality.ComparisonType.GREATEREQ, ExprComposite([cycle_interval]), ExprComposite([ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=0)])),
                                        Inequality(Inequality.ComparisonType.GREATER, time_since_last, ExprComposite([cycle_interval]))])
        else:
            condition = GoalConjunction([Inequality(Inequality.ComparisonType.GREATEREQ, ExprComposite([cycle_interval]), ExprComposite([ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=0)])),
                                        Inequality(Inequality.ComparisonType.GREATER, ineq_lhs, ExprComposite([ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=threshold)]))])
        value_expr = ExprComposite([ExprBase(ExprBase.ExprType.BINARY_OPERATOR, op=ExprBase.BinaryOperator.MUL),
                                    ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=self.inwindow_reward),
                                    sub, *time_since_last.tokens, cycle_interval])
        self.add_complex_effect(EffectConditional(condition, Assignment(AssignmentType.INCREASE, total_benefit_function, value_expr)), op)

        if threshold == 0:  # Precise floating point operations
            condition = Inequality(Inequality.ComparisonType.EQUALS, time_since_last, ExprComposite([cycle_interval]))
        else:
            condition = GoalConjunction([Inequality(Inequality.ComparisonType.GREATEREQ, ExprComposite([cycle_interval]), ExprComposite([ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=0)])),
                                         Inequality(Inequality.ComparisonType.LESSEQ, ineq_lhs, ExprComposite([ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=threshold)])),
                                         Inequality(Inequality.ComparisonType.GREATEREQ, ineq_lhs, ExprComposite([ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=-threshold)]))])
        value_expr = ExprComposite([ExprBase(ExprBase.ExprType.CONSTANT, constant=self.inwindow_reward)])
        self.add_complex_effect(EffectConditional(condition, Assignment(AssignmentType.INCREASE, total_benefit_function, value_expr)), op)

    def add_action_benefit(self, op: Operator, goal, params):
        total_benefit = AtomicFormula.from_string('total_benefit')
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
        if goal.name in self.goal_benefits:
            benefit_val.insert(0, add)
            benefit_val.append(ExprBase(expr_type=ExprBase.ExprType.FUNCTION, function=AtomicFormula(f"benefit_{goal.name}", op.formula.typed_parameters)))
        self.add_complex_assign_effect(total_benefit, ExprComposite(benefit_val), op, assign_type=AssignmentType.INCREASE)

    def mutex_existential_preconditions(self, d: Domain):
        # MUTEX Handling for preconditions - with existential preconditions
        for mutex_group in self.ungrounded_mutex_groups:
            is_negative = True
            for pred in mutex_group[1]:
                pvar = self.pred_to_pvariable(pred[0], True)
                if pvar.startswith('sf__'):
                    continue
                for pvar in self.op_types_map[pvar]:
                    op = d.operators[pvar]
                    params_map = {p: op.formula.typed_parameters[i] for i, p in enumerate(pred[1:])} # Map mutex params to op params
                    quant_params = []
                    exists_body = []
                    for rhspred in mutex_group[1]:  # Find params for the quanitfier
                        if ((rhspred == pred and len(mutex_group[1]) > 1)): # If only predicate, the predicate is mutex with itself
                            continue
                        tp = []
                        for i, p in enumerate(rhspred[1:]):
                            typed_params = self.pddl_extractor.domain.predicates[rhspred[0]].typed_parameters
                            if p == mutex_group[0][0]:
                                tp.append(TypedParameter(typed_params[i].type, params_map[p].label))
                                continue
                            qp = TypedParameter(typed_params[i].type, p + str(i))
                            tp.append(qp)
                            quant_params.append(qp)
                        cond_pvar = self.pred_to_pvariable(rhspred[0], False)
                        if cond_pvar.startswith('sf__'):
                            # Add precondition to the action - only if parameters match!
                            for i, p in enumerate(rhspred[1:]):
                                if typed_params[i].type != params_map[p].type:
                                    break
                            else:  # Loop did not break
                                exists_body.append(GoalSimple(AtomicFormula(cond_pvar, tp)))                                
                                is_negative = False
                        else:
                            exists_body.append(GoalSimple(AtomicFormula(cond_pvar, tp)))

                    # Add quantifier
                    if not exists_body: continue
                    if len(exists_body) > 1:
                        exists_body = GoalConjunction(exists_body)
                    else:
                        exists_body = exists_body[0]
                    mutex_cond = GoalQuantified(quant_params, exists_body, GoalType.EXISTENTIAL) if len(quant_params) > 0 else exists_body
                    self.add_complex_condition(mutex_cond, op, is_negative=is_negative)

    def mutex_effects(self, d: Domain):
        # In case existential preconditions are not supported, this will add the precondition without existential, enforcing the mutex by
        # adding the effect of having applied the goal on the mutex actions
        for mutex_group in self.ungrounded_mutex_groups:
            for pred in mutex_group[1]:
                pvar = self.pred_to_pvariable(pred[0], True)
                if pvar.startswith('sf__'):
                    continue
                op_names = self.op_types_map[pvar]
                for pvar in op_names:
                    op = d.operators[pvar]
                    params_map = {p: op.formula.typed_parameters[i] for i, p in enumerate(pred[1:])} # Map mutex params to op params
                    for rhspred in mutex_group[1]:
                        if rhspred == pred and len(mutex_group[1]) > 1: # If only predicate, the predicate is mutex with itself
                            continue
                        # Add effects of the mutex actions
                        typed_params = self.pddl_extractor.domain.predicates[rhspred[0]].typed_parameters
                        free_params = []
                        tp = []
                        for i, p in enumerate(rhspred[1:]):
                            if p not in params_map or (rhspred == pred and p != mutex_group[0][0]):
                                free_params.append(TypedParameter(typed_params[i].type, p + str(i+1)))
                                tp.append(free_params[-1])
                            else:
                                tp.append(TypedParameter(typed_params[i].type, params_map[p].label))

                        rhs_pvar = self.pred_to_pvariable(rhspred[0], False)
                        if rhs_pvar.startswith('sf__'):
                            # Add precondition to the action - only if parameters match!
                            for i, p in enumerate(rhspred[1:]):
                                if p in params_map and typed_params[i].type != params_map[p].type:
                                    break
                            else:
                                cond = GoalSimple(AtomicFormula(rhs_pvar, tp))
                                if free_params:  # Add a forall quantifier
                                    print('INFO: Adding forall quantifier to mutex condition ######')
                                    cond = GoalQuantified(free_params, cond, GoalType.UNIVERSAL)
                                self.add_complex_condition(cond, op, is_negative=False)
                            continue

                        effect = EffectSimple(AtomicFormula(rhs_pvar, tp))
                        if free_params:  # Add a forall quantifier
                            effect = EffectForall(free_params, effect)
                        self.add_complex_effect(effect, op)


    def mutex_sf_effects(self, d: Domain):
        for sf, mg, sf_i in self._sf_mutex:
            for pred in mg[1]:
                _pvar = self.pred_to_pvariable(pred[0], True)
                if pred[0] == sf.name or _pvar not in self.op_types_map:
                    continue
                for pvar in self.op_types_map[_pvar]:
                    op = d.operators[pvar]
                    params_map = {p: op.formula.typed_parameters[i] for i, p in enumerate(pred[1:])} # Map mutex params to op params
                    # Only add effect if parameters types match
                    typed_parameters = []
                    free_params = []
                    for i, tp in enumerate(sf.typed_parameters):
                        if mg[1][sf_i][i+1] in params_map:
                            op_tp = params_map[mg[1][sf_i][i+1]]
                            if tp.type != op_tp.type:
                                break
                        else:
                            op_tp = TypedParameter(tp.type, tp.label + str(i) + str(i+1))
                            free_params.append(op_tp)
                        typed_parameters.append(TypedParameter(tp.type, op_tp.label))
                    else: # if we didn't break..
                        formula = AtomicFormula(self.pred_to_pvariable(sf.name, False), typed_parameters)
                        if free_params:
                            print('INFO: Adding forall quantifier to mutex effect ######')
                            effect = EffectNegative(formula)
                            effect = EffectForall(free_params, effect)
                            self.add_complex_effect(effect, op)
                        else:
                            op.add_simple_effect(formula, is_delete=True)

    def add_objects(self, problem):
        processed_types = []
        for goal in self.potential_goals:
            for tp in goal.typed_parameters:
                # Add subtypes if this is a supertype
                if self.pddl_extractor.is_supertype(tp.type):
                    subtypes = self.pddl_extractor.get_subtypes(tp.type)
                    for subtype in subtypes:
                        if subtype in processed_types: continue
                        obj_instances = self.problem.type_objects_map[subtype]
                        for o in obj_instances:
                            problem.add_object(o, subtype)
                        processed_types.append(subtype)

                # Add normal instances. If a supertype has an instance, it will be added here, otherwise supertypes won't have instances
                if tp.type in processed_types or tp.type not in self.problem.type_objects_map:
                    continue
                tp_type = tp.type
                if self.pddl_extractor.is_supertype(tp_type):
                    # If we have a supertype, we append the __st to make it an instance of the artificial supertype
                    tp_type += '__st'
                obj_instances = self.problem.type_objects_map[tp.type]
                for o in obj_instances:
                    problem.add_object(o, tp_type)
                processed_types.append(tp.type)

    def add_init_state(self, problem: Problem, add_supertype_benefit=False):
        # Propositions
        for pred in self.grounded_state_fluents:
            pred = pred.split(' ')
            if any(map(lambda x: x[0].name == pred[0], self._sf_mutex)):
                problem.add_proposition_from_str(f'sf__{pred[0]}', pred[1:])
            elif any(map(lambda x: x.name == pred[0], self.potential_goals)):
                problem.add_proposition_from_str(f'goal_sel__{pred[0]}', pred[1:])

        # Functions
        for instance, benefit in self.instance_benefits.items():
            # Find type of instance (as benefit must have a type for RDDL syntax)
            if instance in self.problem.objects_type_map:
                tp = self.problem.objects_type_map[instance]
                if tp == 'object': tp = 'obj' # In case the PDDL has a type called object, which is not allowed in RDDL
                problem.add_assignment_from_str(benefit, f'benefit_{tp}', [instance])
                # Benefit_supertype won't be needed in the normla PDDL grapgiven that actions will be filtered by subtype, but adding for PDDL3 evaluation
                if add_supertype_benefit:
                    supertype = self.pddl_extractor.get_inst_type(instance, True)
                    if supertype != type:
                        if supertype == 'object': continue #supertype = 'obj' # In case the PDDL has a type called object, which is not allowed in RDDL
                        problem.add_assignment_from_str(benefit, f'benefit_{supertype}', [instance])

        # Now set default benefit
        for instance, tp in self.problem.objects_type_map.items():
            if tp == 'object': tp = 'obj' # In case the PDDL has a type called object, which is not allowed in RDDL
            if f'benefit_{tp}' in problem.domain.functions and instance not in self.instance_benefits:
                problem.add_assignment_from_str(DEFAULT_BENEFIT, f'benefit_{tp}', [instance])
                # Benefit_supertype won't be needed given that actions will be filtered by subtype
                if add_supertype_benefit:
                    supertype = self.pddl_extractor.get_inst_type(instance, True)
                    if supertype != type:
                        if supertype == 'object': continue #supertype = 'obj' # In case the PDDL has a type called object, which is not allowed in RDDL
                        problem.add_assignment_from_str(DEFAULT_BENEFIT, f'benefit_{supertype}', [instance])
        problem.add_assignment_from_str(0.0, "n_actions")
        problem.add_assignment_from_str(0.0, "total_benefit")

        # Time windows
        if self.timing_windows or self.timing_cyclics:
            problem.add_assignment_from_str(self.current_time, 'current_time')
        for pred_name, tw in self.timing_windows.items():
            processed_params = []  # List of lists of params for which we have a time window
            for params, windows in tw.items():
                params = params.replace(' ', '').split(',')
                processed_params.append(params)
                problem.add_assignment_from_str(windows[0], f'{pred_name}__window_start', params)
                problem.add_assignment_from_str(windows[1], f'{pred_name}__window_end', params)
            # Now add a default time window for the rest of the params
            all_instance_params = self.get_all_instances_predicate_params(pred_name)
            for params in itertools.product(*all_instance_params):
                if list(params) in processed_params: continue
                problem.add_assignment_from_str(-1, f'{pred_name}__window_start', params)  # 0
                problem.add_assignment_from_str(-1, f'{pred_name}__window_end', params)  # 24
        
        # Cyclic goals
        for pred_name, tc in self.timing_cyclics.items():
            processed_params = []  # List of lists of params for which we have a time window
            for params, cyc in tc.items():
                params = params.replace(' ', '').split(',')
                processed_params.append(params)
                last_time = self.current_time # Default is current time. This will make it not trigger a cycle condition
                # It used to be  (self.current_time-cyc)%24  to force trigger the goal
                if pred_name in self.last_goal_times:
                    _aux = ', '.join(params)
                    if _aux in self.last_goal_times[pred_name]:
                        last_time = self.last_goal_times[pred_name][_aux]
                problem.add_assignment_from_str(last_time, f'{pred_name}__last_time', params)  # TODO where does this come from?!?!
                problem.add_assignment_from_str(cyc, f'{pred_name}__interval', params)
            
            # Now add a default time window for the rest of the params
            all_instance_params = self.get_all_instances_predicate_params(pred_name)
            for params in itertools.product(*all_instance_params):
                if list(params) in processed_params: continue
                problem.add_assignment_from_str(self.current_time, f'{pred_name}__last_time', params)  # FIXME check this makes sense
                problem.add_assignment_from_str(-1, f'{pred_name}__interval', params)  # 0
        
        # Goal benefit
        for goal, benefit in self.goal_benefits.items():
            processed_params = []
            for params, gbenefit in benefit.items():
                params = params.replace(' ', '').split(',')
                problem.add_assignment_from_str(gbenefit, f'benefit_{goal}', params)
                processed_params.append(params)
            # Now add a default time window for the rest of the params
            all_instance_params = self.get_all_instances_predicate_params(goal)
            for params in itertools.product(*all_instance_params):
                if list(params) in processed_params: continue
                problem.add_assignment_from_str(DEFAULT_BENEFIT, f'benefit_{goal}', params)  # 0

        # Unreachable goals
        for goal in self.unreachable_goals:
            problem.add_proposition_from_str(f'is_unreachable_goal__{goal.name}', [p.value for p in goal.typed_parameters])


    def get_all_instances_predicate_params(self, pred_name):
            all_instance_params = []
            for tp in self.pddl_extractor.domain.predicates[pred_name].typed_parameters:
                subtypes = self.pddl_extractor.get_subtypes(tp.type)
                if subtypes:
                    # Get all instances of subtypes and flatten the list
                    all_instance_params.append([item for sl in [self.problem.type_objects_map[subt] for subt in subtypes] for item in sl])
                else:
                    all_instance_params.append(self.problem.type_objects_map[tp.type])
            return all_instance_params


    def add_goal(self, problem: Problem):
        exp_function = ExprBase(expr_type=ExprBase.ExprType.FUNCTION, function=AtomicFormula('n_actions'))
        number_actions = ExprBase(expr_type=ExprBase.ExprType.CONSTANT, constant=self.n_goals)
        goal = Inequality(Inequality.ComparisonType.EQUALS, ExprComposite([number_actions]), ExprComposite([exp_function]))
        # <= N goal = Inequality(Inequality.ComparisonType.LESSEQ, ExprComposite([exp_function]), ExprComposite([number_actions]))
        problem.goal = goal  # No function to set goal
    
    @staticmethod
    def add_metric(problem: Problem):
        metric_spec = MetricSpec.MAX
        expression = ExprComposite([ExprBase(expr_type=ExprBase.ExprType.FUNCTION, function=AtomicFormula('total_benefit'))])
        problem.metric = Metric(metric_spec, expression) # No function to set metric

    def write_header(self, f, t='DOMAIN'):
        n = (76-47-len(self.domain_name))/2
        sp = ' '*4
        #if t=='PROBLEM': n+=1
        title = f'{sp}THIS {t} WAS AUTOMATICALLY GENERATED ON {datetime.now(timezone.utc).strftime("%d/%m/%Y %H:%M:%S")} (UTC){sp}'
        subtitle = ' '*int(n) + f'A domain to generate goals for the {self.domain_name} PDDL domain' + ' '*int(n+0.5)
        header = ';;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;'
        if len(subtitle) < len(title):
            subtitle += ' '*(len(title)-len(subtitle))
            header += ';'
        f.write(header + '\n'+
                f';;{" "*len(title)};;\n' +
               f';;{title};;\n' +
                f';;{" "*len(title)};;\n' +
                ';;' + subtitle + ';;\n' +
                f';;{" "*len(title)};;\n' +
                header + '\n\n')
                # Above, int(n+0.5) rounds to closest upper integer

    @staticmethod
    def add_complex_assign_effect(function : AtomicFormula,
                            value_expr : ExprComposite,
                            op: Operator,
                            time_spec : TimeSpec = TimeSpec.AT_START,
                            assign_type : AssignmentType = AssignmentType.ASSIGN):
        """
        Adds a numeric effect.
        If the operator already has a non-conjunctive effect then the new and existing
        effects will be wrapped together within a new conjunctive effect.
        """
        effect = Assignment(assign_type, function, value_expr)
        if op.durative: effect = TimedEffect(time_spec, effect)
        if op.effect.effect_type == EffectType.CONJUNCTION:
            op.effect : EffectConjunction
            if effect.effect_type == EffectType.CONJUNCTION:
                op.effect.effects.extend(effect.effects)
            else:    
                op.effect.effects.append(effect)
        elif op.effect.effect_type == EffectType.EMPTY:
            op.effect = effect
        else:
            op.effect = EffectConjunction(effects=[op.effect, effect])

    @staticmethod
    def add_complex_effect(effect, op : Operator, time_spec : TimeSpec = TimeSpec.AT_START, is_delete : bool = False):
        if is_delete: effect = EffectNegative(op.formula)
        if op.durative: 
            if effect.effect_type == EffectType.CONDITIONAL:
                effect.condition = TimedGoal(time_spec, effect.condition)
                effect.effect = TimedEffect(time_spec, effect.effect)
            else: effect = TimedEffect(time_spec, effect)
        if op.effect.effect_type == EffectType.CONJUNCTION:
            op.effect : EffectConjunction
            if effect.effect_type == EffectType.CONJUNCTION:
                op.effect.effects.extend(effect.effects)
            else:    
                op.effect.effects.append(effect)
        elif op.effect.effect_type == EffectType.EMPTY:
            op.effect = effect
        else:
            op.effect = EffectConjunction(effects=[op.effect, effect])

    @staticmethod
    def add_complex_condition(condition, op: Operator, 
                              time_spec : TimeSpec = TimeSpec.AT_START, is_negative : bool = False):
        if is_negative: condition = GoalNegative(condition)
        if op.durative: condition = TimedGoal(time_spec, condition)
        if isinstance(op.condition, GoalConjunction):
            op.condition.goals.append(condition)
        elif op.condition.goal_type == GoalType.EMPTY:
            op.condition = condition
        else:
            op.condition = GoalConjunction(goals=[op.condition, condition])

    @staticmethod
    def _permut_param_names(supertypes, curr, i=0):
        # Given a list of lists, generates another list of lists with all the permutations
        if i==len(supertypes):
            return [curr]
        ret = []
        for j in supertypes[i][0]:
            curr[i] = j
            r = PDDLDomainGenerator._permut_param_names(supertypes, curr, i+1)
            ret.extend(copy.deepcopy(r))
        return ret
    
    def _get_supertype_params(self, param_list: list[TypedParameter]):
        # The subtype parameters will include the artificial supertype __st
        return [(self.pddl_extractor.get_subtypes(p.type) + [p.type + '__st'], i) for i, p in enumerate(param_list) if self.pddl_extractor.is_supertype(p.type)]

    def _remap_subtypes(self, param_list, supertype_params, pn):
        # Remaps the supertypes in param_list with the corresponding subtypes in the type permutation pn
        params = {}
        j = 0
        for i, tp in enumerate(param_list):
            if j >= len(supertype_params) or i < supertype_params[j][1]:
                params['?' + tp.type[0] + str(i)] = tp.type
            else:
                params['?' + pn[j][0] + str(i)] = pn[j]
                j += 1
        return params

if __name__ == '__main__':
    pddl_path = '/tmp/GoalGeneratorDomain.pddl'
    domain_file = 'domain.pddl'
    problem_file = 'problem.pddl'
    generator = PDDLDomainGenerator(domain_file, problem_file)
    generator.load_cfg('cfg/office_robot_benefits.yaml')
    #generator.set_state(['not_holding_object robot_assistant', 'robot_at robot_assistant wp0'])
    d = generator.generate_domain(pddl_path)
    pddl_path = '/tmp/GoalGeneratorProblem.pddl'
    generator.generate_instance(pddl_path, d)
