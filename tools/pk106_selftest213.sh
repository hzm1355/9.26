#!/bin/bash
# 对账器自测的重跑器：把退出码写进文件再 echo，省得怀疑是 Git Bash 把 $? 自己吃了。
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
LOG=/mnt/d/desktop1/9.26.1/_diag155/selftest213.log
timeout -k 20 420 python3 /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/pk_record_audit_selftest.py > $LOG 2>&1
RC=$?
echo "SELFTEST_RC=$RC" | tee -a $LOG
tail -14 $LOG | tr -d '\r'
