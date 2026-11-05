#!/bin/bash
# Usage: batch.sh JOBFILE PARALLEL ; each JOBFILE line: TAG NAME CONFIG CAM SET [runner args]
E=$(cd $(dirname $0)/.. && pwd)
grep -v '^#' $1 | grep -v '^\s*$' | xargs -P ${2:-4} -L 1 bash -c 'r=$(TAG=$0 '$E'/scripts/run_one.sh "$@"); echo "$0 $1 $r"'
