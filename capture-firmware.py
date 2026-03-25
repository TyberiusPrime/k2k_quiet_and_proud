#!/usr/bin/env python3
import time
import subprocess
import datetime

dt = datetime.datetime.now()
st = dt.strftime("%y-%m-%d_%H-%M")
cmds = [
    "sudo dfu-util -v -d 1eaf:003 -U firmware-backup-2026-03-25.bin -a2",
]


timeout = 30

for c in cmds:
    while True:
        try:
            timeout = timeout - 1
            if timeout <= 0:
                raise TimeoutError("Command timed out: " + c)
            subprocess.check_call(c, shell=True)
            break
        except subprocess.CalledProcessError:
            if "dfu-util" in c:
                time.sleep(1)
            else:
                raise
