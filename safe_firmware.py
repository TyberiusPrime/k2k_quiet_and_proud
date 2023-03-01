#!/usr/bin/env python3
import time
import subprocess
import datetime

dt = datetime.datetime.now()
st = dt.strftime("%y-%m-%d_%H-%M")
cmds = [
    #'dfu-util -v -d 1eaf:003 --list",
    "dfu-util -v -d 1eaf:003 --devnum=26 -U k2k_advantage_20230301_pre_rebuild.bin",
]

for c in cmds:
    while True:
        try:
            subprocess.check_call(c, shell=True)
            break
        except subprocess.CalledProcessError:
            if "dfu-util" in c:
                time.sleep(1)
            else:
                raise
