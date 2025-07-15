# Goal Generation for Autonomous Planning Agents

## Abstract
To become more autonomous, intelligent agents must be able to determine which objectives to pursue from all the achievable ones. Once a set of goals has been chosen, a symbolic planner can find an executable plan to fulfil them. However, this is a complex problem that requires the optimisation of different conflicting metrics, such as the goal utility, the plan cost, and its makespan.
In this paper, we first propose a method to identify a set of potential goals in a PDDL domain. Next, we find the achievable subset of goals that yield the most benefit by posing the problem as an integer linear program. Finally, a planner is used to find a plan for the resulting goal subset. 
The proposed method is generic, planner-independent, and generates sets of goals that are reachable and not mutually exclusive, increasing the chances of finding a solvable plan.
We demonstrate the effectiveness of the proposed approach in comparison with a baseline oversubscription planner.

## Usage
The main run script is `ILP_goal_generation_pipeline.py`, which will run the goal generator and call the planner. Options to run it are as follows.
```
usage: ILP_goal_generation_pipeline.py [-h] [-d DOMAIN] [-p PROBLEM] [-g CFG_PATH] [-s SOLVER_ID] [-u UUID] [-t PLANNER_TIMEOUT] [-r REMOVE_PERCENT]
                                       [-m {worst_n,random}]

options:
  -d DOMAIN, --domain DOMAIN
                        Domain to be evaluated
  -p PROBLEM, --problem PROBLEM
                        Problem to be evaluated
  -g CFG_PATH, --cfg_path CFG_PATH
                        Config YAML file with benefits
  -s SOLVER_ID, --solver_id SOLVER_ID
                        Solver ID (default: GLOP)
  -u UUID, --uuid UUID  UUID for this run (optional)
  -t PLANNER_TIMEOUT, --planner_timeout PLANNER_TIMEOUT
                        Planner timeout (default: 30)
  -r REMOVE_PERCENT, --remove_percent REMOVE_PERCENT
                        Percentage of goals to remove if no plan is found (default: 15)
  -m {worst_n,random}, --removal_strategy {worst_n,random}
                        Removal strategy for goals if no plan is found (default: worst_n)
```
