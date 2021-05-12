#!/usr/bin/env python3
import sys
import psutil
import time
from statistics import mean
from datetime import datetime

# datetime object containing current date and time
# now = datetime.now()
#
# print("now =", now)

measure_pid = int(sys.argv[1])
nLocations = int(sys.argv[2])
p = psutil.Process(pid = measure_pid)
cpu_list = [];
mem_list = [];
# print(p)

while (psutil.pid_exists(measure_pid)):
    cpu_list.append(p.cpu_percent())
    mem_list.append(p.memory_percent())
    time.sleep(0.1)

# print(cpu_list)
# print(mem_list)

cpu_mean = mean(cpu_list)
cpu_max = max(cpu_list)
cpu_min = min(cpu_list)

mem_mean = mean(mem_list)
mem_max = max(mem_list)
mem_min = min(mem_list)

save_string = "{},{},{},{},{},{},{}"

print(save_string.format(nLocations,cpu_mean,cpu_max,cpu_min,mem_mean,mem_max,mem_min))

# datetime object containing current date and time
# now = datetime.now()
#
# print("now =", now)
