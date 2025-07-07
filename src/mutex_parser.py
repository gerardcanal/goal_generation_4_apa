#!/usr/bin/env python3
import subprocess
import re
import os
from domain_problem_parser import PDDLExtractor

MUTEX_EXECUTABLE = 'mutex_rewrite-no-lp'

class PDDLMutexParser:
    def __init__(self, pddl_extractor):
        self.pddl_extractor = pddl_extractor
        self._mutex_finder_path = os.path.dirname(__file__) + '/../bin/' + MUTEX_EXECUTABLE

        # Compile regexes
        self.final_mutex_re = re.compile(r'^Final mutex groups:[\s\S]*?(M[\S\s]*)$', re.IGNORECASE | re.MULTILINE)
        self.mutex_filter_re = re.compile(r'^Mutex group ([\w-]+).*{((?:\s*\([\w-]+(?:\s[\w-]+)*\))+)}$', re.IGNORECASE | re.MULTILINE) # group 1 is the instance name, group 2 is the list of predicates
        
    def _run_mutex_finder(self):
        cmd = [self._mutex_finder_path, self.pddl_extractor.domain_file, self.pddl_extractor.problem_file]
        output = subprocess.check_output(cmd)
        return output.decode('utf-8')

    ''' Returns grounded mutex groups by object instance with grounded predicates'''
    def _parse_mutex_groups(self, cmd_output):
        mutex_groups = re.findall(self.final_mutex_re, cmd_output) # Get the final mutex groups        
        if mutex_groups:
            mutex_groups = mutex_groups[0]
            raw_mutex_groups = re.findall(self.mutex_filter_re, mutex_groups)
            mutex_groups = []
            predicate_split_re = re.compile(r'\(([\w-]+(?:\s*[\w-]*)*)\)', re.MULTILINE | re.IGNORECASE)
            for object, predicates in raw_mutex_groups:
                mutex_groups.append((object, re.findall(predicate_split_re, predicates)))
                # TODO -> Mutex groups can be repeated by object, so we can't override
            return mutex_groups
        else:
            print('No mutex groups found')

    def _unground_mutex_groups(self, mutex_groups):  # FIXME Assumes that if a grounded predicate is in mutex group, the ungrounded one is too
        ungrounded_mutex_groups = set()  # To remove duplicates

        for object, predicates in mutex_groups:
            i = 0
            inst_type = self.pddl_extractor.problem.objects_type_map[object] # This is just to read the output better by type name
            ungrounding = {object: '?' + inst_type[0] + str(i)}
            i += 1
            ungrounded_predicates = {}
            pred_id = {} # To avoid reusing an ungrounded predicate name
            pred_count = 1
            for predicate in sorted(predicates):
                p = predicate.split(' ')
                ungrounded_predicate = [p[0]]
                if p[0] not in pred_id: 
                    pred_id[p[0]] = pred_count
                    pred_count += 1
                formula = self.pddl_extractor.domain.predicates[p[0]]
                for j, param in enumerate(p[1:]):
                    if param not in ungrounding:
                         ungrounding[param] = '?' + formula.typed_parameters[j].type[0] + str(pred_id[p[0]]) + str(j+1)  # param[0] for instance initial, formula.typed_parameters[j].type[0] for type initial
                         i += 1
                    ungrounded_predicate.append(ungrounding[param])
                ungrounded_predicate = tuple(ungrounded_predicate)
                if ungrounded_predicate in ungrounded_predicates:
                    ungrounded_predicates[ungrounded_predicate] += 1
                else: 
                    ungrounded_predicates[ungrounded_predicate] = 1
            for mtx, count in ungrounded_predicates.items():
                if count > 1: # Add mutex with itself when the grounded predicate appears with other parameters (assumes all parameters are present if > 1)
                    ungrounded_mutex_groups.add(((ungrounding[object], self.pddl_extractor.get_inst_type(object)), (mtx,))) # Tuple of size 1 for consistency with the rest
            all_mutexes = list(ungrounded_predicates.keys())
            mutex_pairs = [(a, b) for i, a in enumerate(all_mutexes) for b in all_mutexes[i+1:]]
            for p in mutex_pairs:
                ungrounded_mutex_groups.add(((ungrounding[object], self.pddl_extractor.get_inst_type(object)), p))
        return ungrounded_mutex_groups

    def _get_instances_by_type(self, typename):
        if typename not in self.pddl_extractor.domain.type_tree:
            # It's a supertype, find subtypes
            instances = []
            for t, p in self.pddl_extractor.domain.type_tree.items():
                if typename == p.parent:
                    instances.extend(self._get_instances_by_type(t))
            return instances
        return self.pddl_extractor.problem.type_objects_map[typename]
    


if __name__ == '__main__':
    domain_file = os.path.dirname(__file__) + '/../testdomains/domain.pddl'
    problem_file = os.path.dirname(__file__) + '/../testdomains/problem.pddl'

    pddl_mutex_manager = PDDLMutexParser(PDDLExtractor(domain_file, problem_file))
    output = pddl_mutex_manager._run_mutex_finder()
    mutex_groups = pddl_mutex_manager._parse_mutex_groups(output)
    ungrounded_mutex = pddl_mutex_manager._unground_mutex_groups(mutex_groups)
    print("Found {} mutex groups".format(len(ungrounded_mutex)))
    for i, ((p, t), mutex) in enumerate(ungrounded_mutex):
        print(str(i) + " | " + p + ' - ' + t + ':', mutex, '\n')
        #print('\n'.join(pddl_mutex_manager.RDDL_preconds(((p,t), mutex))))
    #print(pddl_mutex_manager._run_mutex_finder())
