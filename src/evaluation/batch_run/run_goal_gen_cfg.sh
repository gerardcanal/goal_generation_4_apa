#!/bin/bash
export PYTHONPATH=$PYTHONPATH:/scratch/gerard/code/otpl

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

echo "STARTING GOAL GENERATION BATCH RUN:"
echo ${BASH_SOURCE[0]} $@

DOMAIN=$1
PROBLEM=$2
PLANNER_TIMEOUT=$3

REMOVAL_STRATEGY=$4
REMOVE_PERCENT=$5
RESULTS_FOLDER=$6 # DOMAIN NAME
RESULTS_BASE_PATH="/data/data/gerard/results_goal_generation_4_apa"
GOAL_GENERATOR_ROOT="/scratch/gerard/code/goal_generation_4_apa"
SOLVER_ID="GLOP"

mkdir -p $RESULTS_BASE_PATH/run_status_${RESULTS_FOLDER}/completed

i=1
TOTAL=$(ls $RESULTS_BASE_PATH/$RESULTS_FOLDER/ | wc -l)
for OUT_FOLDER in $RESULTS_BASE_PATH/$RESULTS_FOLDER/*; do
    EXP=${REMOVAL_STRATEGY}_${REMOVE_PERCENT}_${SOLVER_ID}_t${PLANNER_TIMEOUT}
    OUT_RESULTS_CSV=$OUT_FOLDER/goalgen_csv/goalgen_${EXP}_results.csv
    
    BENEFIT_FILE=$OUT_FOLDER/cfg.yaml
    GOALGEN_OUT=$OUT_FOLDER/goalgen_out/goalgen_${EXP}_output.txt

    echo "                    Started $i with params $EXP $OUT_FOLDER $GOALGEN_OUT"
    echo "                    Started $i with params $EXP $OUT_FOLDER $GOALGEN_OUT" >> $RESULTS_BASE_PATH/run_status_${RESULTS_FOLDER}/goalgen_${EXP}_results.txt

    _UUID=$(uuidgen | sed s/-//g)
    python3 $GOAL_GENERATOR_ROOT/src/evaluation/goal_generation_evaluation.py -d $DOMAIN \
                                                                              -p $PROBLEM \
                                                                              --cfg_path $BENEFIT_FILE \
                                                                              --solver_id $SOLVER_ID \
                                                                              -uuid $_UUID \
                                                                              --planner_timeout $PLANNER_TIMEOUT \
                                                                              --remove_percent $REMOVE_PERCENT \
                                                                              --removal_strategy $REMOVAL_STRATEGY \
                                                                              --csv_path $OUT_RESULTS_CSV &> $GOALGEN_OUT

    echo "[$(date +"%d/%m/%y %R:%S")] Completed $i/$TOTAL with params $EXP $OUT_FOLDER"
    echo "[$(date +"%d/%m/%y %R:%S")] Completed $i/$TOTAL with params $EXP $OUT_FOLDER $GOALGEN_OUT" >> $RESULTS_BASE_PATH/run_status_${RESULTS_FOLDER}/goalgen_${EXP}_results.txt
    echo ""
    echo ""  >> $RESULTS_BASE_PATH/run_status_${RESULTS_FOLDER}/goalgen_${EXP}_results.txt
    i=$((i+1))
done

mv $RESULTS_BASE_PATH/run_status_${RESULTS_FOLDER}/goalgen_${EXP}_results.txt $RESULTS_BASE_PATH/run_status_${RESULTS_FOLDER}/completed/goalgen_${EXP}_results.txt
