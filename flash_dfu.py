#!/usr/bin/env python3
import time
import subprocess
import datetime
dt = datetime.datetime.now()
st = dt.strftime("%y-%m-%d_%H-%M")
cmds = [
    "cargo build --release",
    "arm-none-eabi-objcopy -O binary target/thumbv7m-none-eabi/release/k2k_advantage k2k_advantage.bin",
	"cp k2k_advantage.bin old/k2k_advantage_%s.bin" %  st,
	'sudo dfu-util -v -d 1eaf:003 -D k2k_advantage.bin -a2 -R',
]

for c in cmds:
	while True:
		try:
			subprocess.check_call(c, shell=True)
			break
		except subprocess.CalledProcessError:
			if 'dfu-util' in c:
				time.sleep(1)
			else:
				raise
