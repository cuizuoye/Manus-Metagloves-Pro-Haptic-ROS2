#!/usr/bin/env python3
"""Synthetic MANUS-like finger input, existing Shadow Hand controller, simulation only."""
import argparse
import math
import time
import mujoco
import mujoco.viewer
import numpy as np
from shadow_teleop import SCENE, FINGERS, Retarget


def input_for(t):
    # 2 seconds per finger, smooth full flex then open. Includes a final all-finger cycle.
    segment = int(t // 2.0) % 6
    flex = (1.0 - math.cos(math.pi * (t % 2.0))) / 2.0
    v = np.zeros(5)
    if segment == 5:
        v[:] = 40.0 * flex
    else:
        v[segment] = 40.0 * flex
    return v, segment


def main():
    p = argparse.ArgumentParser(description='Synthetic input; no glove or robot commands.')
    p.add_argument('--seconds', type=float, default=0, help='Exit after N seconds (0: until window closes)')
    args = p.parse_args()
    if args.seconds < 0: p.error('--seconds must be >= 0')
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    ctrl = Retarget(model, data)
    ctrl.open = np.zeros(5); ctrl.closed = np.full(5, 40.0); ctrl.enabled = True
    paused = False
    def key(k):
        nonlocal paused
        if k == 32:
            paused = not paused
            print('已暂停' if paused else '已继续', flush=True)
    print('离线模拟输入：自动轮流活动五指，再同时握拳；空格暂停或继续，关闭窗口退出。', flush=True)
    print('不连接手套或真实机器人；手腕固定，不代表真实抓取能力。', flush=True)
    with mujoco.viewer.launch_passive(model, data, key_callback=key) as viewer:
        start = last = time.monotonic(); simulated = 0.0; previous = -1
        while viewer.is_running():
            now = time.monotonic()
            if args.seconds and now-start >= args.seconds: break
            elapsed = min(now-last, 0.05); last = now
            if not paused:
                simulated += elapsed
                inp, seg = input_for(simulated)
                if seg != previous:
                    print('当前动作：' + (FINGERS[seg] if seg < 5 else '五指同时'), flush=True)
                    previous = seg
                # Fixed physics step; bound catch-up to keep UI responsive.
                for _ in range(max(1, min(10, int(elapsed/model.opt.timestep)))):
                    ctrl.apply(inp, model.opt.timestep)
                    mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(1/120)

if __name__ == '__main__': main()
