#!/bin/bash
export PYTHONPATH=$PYTHONPATH:/scratch/gerard/code/otpl

RESULTS_FOLDER=$7 # DOMAIN NAME
GOAL_GENERATOR_ROOT="/scratch/gerard/code/goal_generation_4_apa"
RESULTS_BASE_PATH="/data/data/gerard/results_goal_generation_4_apa"

mkdir -p $RESULTS_BASE_PATH/run_status_${RESULTS_FOLDER}/completed

echo "STARTING PDDL3 BATCH RUN:"
echo ${BASH_SOURCE[0]} $@

i=1
TOTAL=$(ls $RESULTS_BASE_PATH/$RESULTS_FOLDER/ | wc -l)
for OUT_FOLDER in $RESULTS_BASE_PATH/$RESULTS_FOLDER/*; do
    OUTPUT_FILE=$OUT_FOLDER/pddl3_csv/pddl3_t${3}_b${4}_f${5}_v${6}_results.csv
    PLANNER_EXEC=$GOAL_GENERATOR_ROOT/bin/optic-clp

    python3 $GOAL_GENERATOR_ROOT/src/evaluation/pddl3_evaluation.py -d $1 \
                                                                    -p $2 \
                                                                    --cfg_path $OUT_FOLDER/cfg.yaml \
                                                                    --planner_timeout $3 \
                                                                    --benefit_factor $4 \
                                                                    --time_factor $5 \
                                                                    --violated_prefs_factor $6 \
                                                                    -e $PLANNER_EXEC \
                                                                    -o $OUTPUT_FILE \
                                                                    -od $OUT_FOLDER/pddl3_pddl/ \
                                                                    -post "t${3}_b${4}_f${5}_v${6}" &> $OUT_FOLDER/pddl3_out/pddl3_t${3}_b${4}_f${5}_v${6}_output.txt
    echo "[$(date +"%d/%m/%y %R:%S")] Completed $i/$TOTAL with params -d $1 -p $2 --cfg_path $OUT_FOLDER/cfg.yaml -t $3 -b $4 -f $5 -v $6 -e $PLANNER_EXEC > pddl3_t${3}_b${4}_f${5}_v${6}_output.txt"
    echo "[$(date +"%d/%m/%y %R:%S")] Completed $i/$TOTAL with params -d $1 -p $2 --cfg_path $OUT_FOLDER/cfg.yaml -t $3 -b $4 -f $5 -v $6 -e $PLANNER_EXEC > pddl3_t${3}_b${4}_f${5}_v${6}_output.txt" >> $RESULTS_BASE_PATH/run_status_${RESULTS_FOLDER}/pddl3_t${3}_b${4}_f${5}_v${6}_results.txt
    i=$((i+1))
done

mv $RESULTS_BASE_PATH/run_status_${RESULTS_FOLDER}/pddl3_t${3}_b${4}_f${5}_v${6}_results.txt $RESULTS_BASE_PATH/run_status_${RESULTS_FOLDER}/completed/pddl3_t${3}_b${4}_f${5}_v${6}_results.txt
