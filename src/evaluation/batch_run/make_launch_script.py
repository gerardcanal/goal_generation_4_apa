#!/usr/bin/python3
import argparse
SCRIPTS_FOLDER = '/scratch/gerard/code/goal_generation_4_apa/src/evaluation/batch_run'
BASE_CMD = f'time bash {SCRIPTS_FOLDER}/run_container.sh '

def make_goalgen_sh(args):
    PLANNER_TIMEOUTS = [30]#, 150]# , 300] # Do 300 if there's time...!
    REMOVAL_STRATEGY = ['worst_n', 'random']
    REMOVE_PERCENT = [10, 25, 50]
    ret = []
    for planner_timeout in PLANNER_TIMEOUTS:
        for removal_strategy in REMOVAL_STRATEGY:
            for remove_percent in REMOVE_PERCENT:
                    #cmd = 'time bash ' + SCRIPTS_FOLDER + '/run_goal_gen_cfg.sh '
                    cmd = BASE_CMD + SCRIPTS_FOLDER + '/run_goal_gen_cfg.sh '
                    cmd += args.domain + ' ' # DOMAIN=$1
                    cmd += args.problem + ' ' # PROBLEM=$2
                    cmd += str(planner_timeout) + ' ' # PLANNER_TIMEOUT=$3
                    cmd += removal_strategy + ' ' # REMOVAL_STRATEGY=$4
                    cmd += str(remove_percent) + ' ' # REMOVE_PERCENT=$5
                    cmd += args.results_folder + ' ' # RESULTS_FOLDER=$6
                    t = float(planner_timeout)*100 / remove_percent
                    ret.append((t, cmd))
    return ret

def make_pddl3_sh(args):
    base_cmd = BASE_CMD + f'{SCRIPTS_FOLDER}/run_pddl3.sh '
    #timeouts = [30, 150, 300, 600]
    timeouts = [600]
    # factors = [
    #     (-1, 0.005, 0),
    #     (-1, 0.005, 1),
    #     (-0.25, 0.005, 0.75),
    #     (-0.75, 0.005, 0.25),  # EMPTY PLAN)
    #     (-1, 0.05, 1), # EMPTY PLAN)
    #     (-1, 0.0, 0),
    #     (-1, 0.0, 1),
    #     (-0.25, 0.0, 0.75),
    #     (-0.75, 0.0, 0.25),  # EMPTY PLAN)
    # ]
    factors = [ # b t p
        (-1, 0.1, 1),
        (-1, 0.05, 1),
        (-10, 0.05, 1),
        (-1, 0.05, 10),
    ]
    ret = []
    for b, t, p in factors:
        for to in timeouts:
            cmd = base_cmd + args.domain + ' ' + args.problem + ' ' + str(to) + ' ' + str(b) + ' ' + str(t) + ' ' + str(p) + ' ' + args.results_folder
            ret.append((to, cmd))
    return ret

def make_scripts(cmds, output_folder, num_splits, type):
    if output_folder[-1] != '/':
        output_folder += '/'
    cmds = sorted(cmds, reverse=True)
    files = []
    counters = []
    if (len(cmds) < num_splits):
        num_splits = len(cmds)
        print("Warning: Number of splits is greater than number of commands. Reducing number of splits to", str(num_splits) + "!")
    for i in range(num_splits):
        files.append(open(output_folder + f'run_{type}_cfg_batch_' + str(i) + '.sh', 'w'))
        files[i].write('#!/bin/bash\n')
        #files[i].write('if [[ ! -d /tmp/prost ]]; then\n  bash ~/code/grap_ws/src/goalgeneration_as_planning/src/evaluation/batch_run/reboot_prost.sh;\n  echo "Copied prost to /tmp";\nfi\n\n')
        files[i].write('echo "[$(date +"%d/%m/%y %R:%S")] Starting batch"\n')
        counters.append(1)
    
    i = 0
    for cmd in cmds:
        files[i].write('echo "Starting command ' + str(counters[i]) + '"\n')
        files[i].write(cmd[1] + '\n')
        counters[i] += 1
        i = (i+1) % num_splits

    for i in range(num_splits):
        files[i].close()

def parse_args():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-d", '--domain', type=str,  help="Domain to be evaluated")
    parser.add_argument("-p", '--problem', type=str,  help="Problem to be evaluated")

    parser.add_argument("-t", '--type', type=str,  help="Type: goalgen or pddl3")
    parser.add_argument("-rf", '--results_folder', type=str,  help="Folder to use to store results (only name of folder)")
    parser.add_argument("-s", '--num_splits', type=int,  help="Number of scripts to create")
    parser.add_argument("-f", '--out_folder', type=str,  help="Folder where scripts will be generated")
    parser.add_argument('--print-only', action='store_true',  default=False,  help="Only print the scripts")

    args = parser.parse_args()
    assert args.type == 'goalgen' or args.type == 'pddl3', "Type must be goalgen or pddl3"
    return args

if __name__ == '__main__':
    args = parse_args()
    if args.type == 'goalgen':
        cmds = make_goalgen_sh(args)
    else:
        cmds = make_pddl3_sh(args)
    if args.print_only:
        for cmd in cmds:
            print(cmd[1])
    else:
        print("Creating scripts for", len(cmds), "commands.")
        make_scripts(cmds, args.out_folder, args.num_splits, args.type)
    
