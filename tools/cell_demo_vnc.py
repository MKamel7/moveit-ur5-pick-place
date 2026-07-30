"""Capture the UR teach pendant from URSim over VNC (demo video stage 1b).

Why VNC and not a screen recorder: this box runs GNOME on Wayland, where
wmctrl/xdotool cannot see windows and GNOME blocks programmatic screenshots, so
there is no way to record the URSim window from the desktop side. URSim exports
the pendant on its own VNC server, so the reliable route is to read the
framebuffer straight from the container and never involve the compositor.

Runs in its own process because vncdotool is not part of the ROS Python
environment; install it in a venv and point this script at that interpreter.
Frames are named after their wall-clock capture time in milliseconds, matching
cell_demo_capture.py so the renderer can line every stream up on one timeline.

    <venv>/bin/python cell_demo_vnc.py --out <session_dir> --host 172.17.0.2::5900
"""
import argparse
import os
import time

from vncdotool import api


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--host", default="172.17.0.2::5900",
                    help="vncdotool server spec (host::port)")
    ap.add_argument("--fps", type=float, default=5.0)
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="stop after this long; 0 runs until interrupted")
    args = ap.parse_args()

    out = os.path.join(args.out, "pendant")
    os.makedirs(out, exist_ok=True)
    period = 1.0 / args.fps

    client = api.connect(args.host)
    # A short timeout keeps one slow framebuffer update from stalling the whole
    # capture: a dropped frame just leaves a gap the renderer bridges.
    client.timeout = 5.0
    n, t0 = 0, time.time()
    try:
        while True:
            start = time.time()
            if args.seconds and start - t0 >= args.seconds:
                break
            try:
                client.captureScreen(os.path.join(out, f"{int(start * 1000)}.png"))
                n += 1
            except Exception as exc:            # noqa: BLE001 - keep capturing
                print(f"pendant frame dropped: {exc}")
            if n % 25 == 0:
                print(f"pendant frames: {n}")
            slack = period - (time.time() - start)
            if slack > 0:
                time.sleep(slack)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"pendant frames captured: {n}")
        api.shutdown()


if __name__ == "__main__":
    main()
