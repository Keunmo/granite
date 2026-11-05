#!/bin/bash
# Usage: [TAG=bin tag] run_one.sh NAME CONFIG CAM(0-3) SET(1000|full) [extra runner args]
# Runs the offline Granite mono VIO runner on ~/data/granite_euroc/cam<CAM>[_1000] and writes
# eval/results/NAME (data: eval/data/cam<CAM>[_1000], from the raw capture)
# eval/results/NAME/{granite_traj_tum.txt,granite_states.csv,run.log,metrics.json}.
set -u
E=$(cd $(dirname $0)/.. && pwd)
NAME=$1; CFG=$2; CAM=$3; SET=$4; shift 4
D=${DATA:-$E/data}/cam$CAM; [ "$SET" = 1000 ] && D=${D}_1000
R=$E/results/$NAME; mkdir -p $R
GT=$HOME/multipleye_docker/data/unreal_data_city_sky/gt_poses.txt
cp $CFG $R/config.json
echo "cmd: $E/bin/${TAG:-base}/granite_mono_vio_dataset cam=$CAM set=$SET $*" > $R/cmd.txt
/usr/bin/time -v -o $R/time_v.txt $E/bin/${TAG:-base}/granite_mono_vio_dataset --dataset-path $D \
  --cam-calib ${CALIB:-$E/config/raw_cam${CAM}_kb4_calib.json} --config-path $CFG --out $R/granite "$@" > $R/run.log 2>&1
echo "exit=$?" >> $R/run.log
python3 $E/scripts/metrics.py $R/granite_traj_tum.txt $GT --cam $CAM --frames $D/mav0/cam0/data.csv --json $R/metrics.json > $R/metrics.txt 2>&1
tail -1 $R/metrics.txt
