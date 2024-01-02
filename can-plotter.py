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
FULL_SCALES = [v / MAX_SCALE for v in [115, 111, 185, 5.6, 5.2, 6.1]] # TEO's right arm
OP_FORCES = 0x600
OP_MOMENTS = 0x680

parser = argparse.ArgumentParser()
parser.add_argument('--id', type=int, required=True, help='node ID')
args = parser.parse_args()

t = np.arange(0, TIME_RANGE, TIME_INTERVAL)
values = [collections.deque(np.zeros(t.shape)) for i in range(6)]
last_value = [0 for i in range(6)]
limits = [(0, 0) for i in range(2)]

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
    for line in sys.stdin:
        if should_stop:
            break

        if (m := re.match('^(\w+)\s+(\w+)\s+\[(\d)\]((?:\s+\w{2}){0,8})$', line.strip())) is not None:
            if not int(m.group(3), 10) == 8: # dlc
                continue

            op = int(m.group(2), 16) - args.id
            data = m.group(4).strip().split(' ')

            if op == OP_FORCES:
                last_value[0] = hex_to_signed_int(data[0:2]) * FULL_SCALES[0]
                last_value[1] = hex_to_signed_int(data[2:4]) * FULL_SCALES[1]
                last_value[2] = hex_to_signed_int(data[4:6]) * FULL_SCALES[2]
            elif op == OP_MOMENTS:
                last_value[3] = hex_to_signed_int(data[0:2]) * FULL_SCALES[3]
                last_value[4] = hex_to_signed_int(data[2:4]) * FULL_SCALES[4]
                last_value[5] = hex_to_signed_int(data[4:6]) * FULL_SCALES[5]
            else:
                continue

            limits[0] = (min([limits[0][0]] + last_value[0:3]), max([limits[0][1]] + last_value[0:3]))
            limits[1] = (min([limits[1][0]] + last_value[3:6]), max([limits[1][1]] + last_value[3:6]))

fig, axes = plt.subplots(1, 2)

# TODO: https://matplotlib.org/stable/users/explain/animations/blitting.html

def do_draw(i):
    # see also (might boost performance): https://stackoverflow.com/a/15724978
    for i in range(len(values)):
        values[i].popleft()
        values[i].append(last_value[i])

    for i, ax in enumerate(axes):
        ax.cla()

        for j, v in enumerate([('x', 'red'), ('y', 'green'), ('z', 'blue')]):
            ax.plot(values[j + (3 * i)], label=v[0], color=v[1])

        for j in range(3):
            ax.scatter(len(values[j + (3 * i)]) - 1, values[j + (3 * i)][-1])

        ax.set_ylim(limits[i][0], limits[i][1])

    axes[0].set_title('forces')
    axes[1].set_title('moments')

ani = FuncAnimation(fig, do_draw, interval=TIME_INTERVAL * 1000)
thread = threading.Thread(target=do_read)

thread.start()
plt.show()
