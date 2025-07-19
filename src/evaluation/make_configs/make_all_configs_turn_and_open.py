#!/usr/bin/python3.9

import sys
from make_eval_cfg import *
import os
import re

if __name__ == '__main__':
    DOMAIN = sys.argv[1]
    PROBLEM = sys.argv[2]

    OUTPUT_FOLDER = sys.argv[3]
    if OUTPUT_FOLDER[-1] != '/':
        OUTPUT_FOLDER += '/'

# PARAMS
NUMBER_OF_GOALS=[5, 10, 25, 50, 60, -1]
#NUMBER_OF_GOALS=[5, 10, 20, 30, 40, 50, 60]
LB=[-500, 0]
UB=[5, 500]
# LB=[-500, -250, 0]
# UB=[5, 250, 500]
N_LEVELS=5
N_LP_LEVELS=2
LP1 = {
    '-lb': 5,
    '-ub': 500,
    '-i': 20,
    '-g': 20,
    '-ti': 0,
    '-to': 0,
    '-ci': 0,
    '-cb': 0,
    '-ca': 0,
}
LP2 = {
    '-i': 68,  # Max instances is 68
    '-g': 150,
    '-ti': 0,
    '-to': 0,
    '-ci': 0,
    '-cb': 0,
    '-ca': 0,
}
L1 = {
     '-lb': 5,
    # '-ub': UB,
    '-i': 50,
    '-g': 50,
    '-ti': 50,
    '-to': 0,
    '-ci': 50,
    '-cb': 0,
    '-ca': 0,
}
L2 = {
     '-lb': 5,
    # '-ub': UB,
    '-i': 15,
    '-g': 15,
    '-ti': 15,
    '-to': 15,
    '-ci': 15,
    '-cb': 15,
    '-ca': 15,
}
L3 = {
    # '-lb': 5,
    # '-ub': UB,
    '-i': 50,
    '-g': 30,
    '-ti': 30,
    '-to': 30,
    '-ci': 30,
    '-cb': 60,
    '-ca': 30,
}
L4 = {
    # '-lb': 5,
    # '-ub': UB,
    '-i': 50,
    '-g': 30,
    '-ti': 50,
    '-to': 40,
    '-ci': 20,
    '-cb': 70,
    '-ca': 10,
}
L5 = {
    # '-lb': LB,
    # '-ub': UB,
    '-i': 0,
    '-g': 0,
    '-ti': 5,
    '-to': 150,
    '-ci': 5,
    '-cb': 150,
    '-ca': 5,
}
#-c 'r'

def dict_to_string_list(d):
    r = []
    for k, v in d.items():
        r.append(str(k))
        r.append(str(v))
    return r

def make_cfg(l, level):
    if not os.path.exists(OUTPUT_FOLDER):
        os.makedirs(OUTPUT_FOLDER) 
    fname = level + ''.join(l[0:6])
    fname = re.sub(r'-([a-zA-Z])', r'_\1', fname)
    path = OUTPUT_FOLDER + fname + '/'
    os.makedirs(path)
    #print(path)
    subdirs = ['pddl3_out', 'pddl3_csv', 'pddl3_pddl', 'goalgen_out', 'goalgen_csv']
    for subd in subdirs:
        os.makedirs(path + subd)

    l = ['-d', DOMAIN, '-p', PROBLEM] + l + ['-c', 'r', '-o', path + 'cfg.yaml']
    print('Making Level', level, ' '.join(params))
    args = check_args(l)
    makecfg = MakeCFG(args)
    makecfg.write_cfg_file(args.output)
    with open(path + 'config_parameters.txt', 'a') as f:
        a = '    ' + str(args).replace('Namespace(', '').replace(', ', '\n    ').replace('=', ' = ')[:-1]
        f.write('Configuration file generated with the following parameters (Level ' + level + '):\n------\n\n' + a + '\n')
    global count
    count += 1

if __name__ == '__main__':
    count = 0
    for ng in NUMBER_OF_GOALS:
        levels = ['LP' + str(l) for l in range(1, N_LP_LEVELS+1)] + ['L' + str(l) for l in range(1, N_LEVELS+1)]
        for l in levels:
            lev = globals()[l]
            for ub in UB:
                if '-ub' in lev:
                    ub = lev['-ub']
                for lb in LB:
                    # DO
                    if '-lb' in lev:
                        lb = lev['-lb']
                    params = ['-n', str(ng), '-lb', str(lb), '-ub', str(ub)] + dict_to_string_list(lev)
                    make_cfg(params, l)
                    
                    if '-lb' in lev:
                        break # skip this all the LB ones as it has been done
                if '-ub' in lev:
                    break # skip this all the UB ones as it has been done
    print('Done. Made', count, 'configs.')