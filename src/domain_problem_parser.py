#!/usr/bin/env python

# Gerard Canal <gerard.canal@kcl.ac.uk>
# King's College London, 2025

from pddl.parser import Parser
from pddl.time_spec import TimeSpec
from pddl.effect import EffectSimple, EffectNegative
from pddl.goal_descriptor import GoalSimple
import os
import sys

class PDDLExtractor:
    def __init__(self, domain_file, problem_file):
        self.domain_file = domain_file
        self.problem_file = problem_file
        parser = Parser()
        parser.parse_file(domain_file)
        self.domain = parser.domain
        if not self.domain:
            print(f"[ERROR] Could not parse domain file {domain_file}.")
            sys.exit(1)
        parser.parse_file(problem_file)
        self.problem = parser.problem

    def get_domain_name(self):
        return self.domain.domain_name

    def get_type_tree(self):
        return self.domain.type_tree

    def get_inst_type(self, inst, super=False):  # if super, it returns the supertype when available
        typename = self.problem.objects_type_map[inst]
        if super and typename in self.domain.type_tree:
            parent = self.domain.type_tree[typename].parent
            while parent != typename:
                typename = parent
                if typename in self.domain.type_tree:
                    parent = self.domain.type_tree[typename].parent
        return typename
    
    def is_supertype(self, type):
        for t in self.domain.type_tree.values():
            if t.parent == type:
                return True
        return False
    
    def get_subtypes(self, type):
        subtypes = []
        for t in self.domain.type_tree.values():
            if t.parent == type:
                subtypes.append(t.name)
        return subtypes
    
    # Finds potential goals in the domain. Potential goals are those that change (delete and add) in the same action,
    # and those which are added by an action but never used as precondition
    def find_potential_goals(self):
        potential_goals = set()
        for _, pred in self.domain.predicates.items():
            is_unique_effect = True
            found = False
            effs, preconds = [], []  # To store the predicates where this predicate appears as effect or precondition
            for _, op in self.domain.operators.items():
                eff = op.effect
                atstart = eff.filter_effects_to_time_spec(TimeSpec.AT_START)
                atend = eff.filter_effects_to_time_spec(TimeSpec.AT_END)
                as_found = _PredicateInEffect(pred, False)
                ae_found = _PredicateInEffect(pred, False)
                if atstart: atstart.visit(PDDLExtractor.check_in_effect, valid_types=(EffectSimple), args=[as_found])
                if atend: atend.visit(PDDLExtractor.check_in_effect, valid_types=(EffectSimple), args=[ae_found])
                # if it was found at start and at end 
                if ((as_found.found and ae_found.found) and ((as_found.delete and ae_found.add) or (as_found.add and ae_found.delete))) or \
                   (as_found.found and (as_found.add and as_found.delete)) or (ae_found.found and (ae_found.add and ae_found.delete)):
                    potential_goals.add(pred)
                if as_found.found: effs.append(as_found)
                if ae_found.found: effs.append(ae_found)
                
                precondition = op.condition
                precond_found = _PredicateInCondition(pred, False)
                precondition.visit(PDDLExtractor.check_in_cond, valid_types=(GoalSimple), args=[precond_found])

                if precond_found.found: preconds.append(precond_found)
            # If not in any precondition but in some effect as add, add it as goal
            if not preconds:
                for eff in effs:
                    if eff.add:  # FIXME, should we consider negative effects?
                        potential_goals.add(pred)
                        break
        return potential_goals

    @staticmethod
    def check_in_effect(effect, pred):
        if effect.formula.name == pred.predicate.name:
            #pred.predicate = effect.formula  # To get the parameters as in the effect 
            pred.found = True
            pred.add = pred.add or type(effect) != EffectNegative
            pred.delete = pred.delete or type(effect) == EffectNegative

    @staticmethod
    def check_in_cond(cond, pred):
        if not pred.found and cond.atomic_formula.name == pred.predicate.name:
            pred.predicate = cond.atomic_formula  # To get the parameters as in the effect 
            pred.found = True

# Helper class to act as a KeyValue to foster the visit pattern to store a predicate from an effect, whether it was found, and if it was add or delete
class _PredicateInEffect:
    def __init__(self, pred, found, add=False, delete=False):
        self.predicate = pred
        self.found = found
        self.add = add
        self.delete = delete

    def __repr__(self):
        return f"{self.predicate}={self.found} (" + (f"add)" if self.add else f"del)")
        
# Helper class to act as a KeyValue to foster the visit pattern to store a predicate from a condition and whether it was found
class _PredicateInCondition:
    def __init__(self, pred, found):
        self.predicate = pred
        self.found = found

    def __repr__(self):
        return f"{self.predicate}={self.found}"
        



if __name__ == "__main__":
    domain_file = os.path.dirname(__file__) + '/../domains/rovers-time-simple-automatic/domain.pddl'
    problem_file = os.path.dirname(__file__) + '/../domains/rovers-time-simple-automatic/instance-20.pddl'
    domain_extractor = PDDLExtractor(domain_file, problem_file)
    print(domain_extractor.get_domain_name())
    print(domain_extractor.find_potential_goals())


