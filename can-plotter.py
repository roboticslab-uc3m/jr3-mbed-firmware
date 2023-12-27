import argparse
import collections
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import re
import signal
import sys
import threading

TIME_RANGE = 10 # [s]
TIME_INTERVAL = 0.01 # [s]
MAX_SCALE = 16384 # 2^14
FULL_SCALES = [v / MAX_SCALE for v in [115, 111, 185, 5.6, 5.2, 6.1]]
OP_FORCES = 0x600
OP_MOMENTS = 0x680

parser = argparse.ArgumentParser()
parser.add_argument('--id', type=int, required=True, help='node ID')
parser.add_argument('--channel', type=str, help='JR3 channel', choices=['fx', 'fy', 'fz', 'mx', 'my', 'mz'])
args = parser.parse_args()

t = np.arange(0, TIME_RANGE, TIME_INTERVAL)
vals = collections.deque(np.zeros(t.shape))
v_min, v_max = 0, 0
value = 0
should_stop = False

def handler(signum, frame):
    global should_stop
    should_stop = True

signal.signal(signal.SIGINT, handler)
signal.signal(signal.SIGTERM, handler)

def hex_to_signed_int(data):
    hex = int(data[1] + data[0], 16)
    return hex - 2**16 if hex > 2**15 - 1 else hex

def do_read():
    global value, v_min, v_max

    for line in sys.stdin:
        if should_stop:
            break

        if (m := re.match('^(\w+)\s+(\w+)\s+\[(\d)\]((?:\s+\w{2}){0,8})$', line.strip())) is not None:
            if not int(m.group(3), 10) == 8: # dlc
                continue

            op = int(m.group(2), 16) - args.id
            data = m.group(4).strip().split(' ')

            if args.channel == 'fx' and op == OP_FORCES:
                value = hex_to_signed_int(data[0:2]) * FULL_SCALES[0]
            elif args.channel == 'fy' and op == OP_FORCES:
                value = hex_to_signed_int(data[2:4]) * FULL_SCALES[1]
            elif args.channel == 'fz' and op == OP_FORCES:
                value = hex_to_signed_int(data[4:6]) * FULL_SCALES[2]
            elif args.channel == 'mx' and op == OP_MOMENTS:
                value = hex_to_signed_int(data[0:2]) * FULL_SCALES[3]
            elif args.channel == 'my' and op == OP_MOMENTS:
                value = hex_to_signed_int(data[2:4]) * FULL_SCALES[4]
            elif args.channel == 'mz' and op == OP_MOMENTS:
                value = hex_to_signed_int(data[4:6]) * FULL_SCALES[5]
            else:
                continue

            v_min = min(v_min, value)
            v_max = max(v_max, value)

fig, ax = plt.subplots()

def do_draw(i):
    # see also (might boost performance): https://stackoverflow.com/a/15724978
    vals.popleft()
    vals.append(value)
    ax.cla()
    ax.plot(vals)
    ax.scatter(len(vals) - 1, vals[-1])
    ax.set_ylim(v_min, v_max)

ani = FuncAnimation(fig, do_draw, interval=TIME_INTERVAL * 1000)
thread = threading.Thread(target=do_read)

thread.start()
plt.show()
