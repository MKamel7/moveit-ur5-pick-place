"""Render the captured session into the demo video (stage 2).

Reads a session directory written by cell_demo_capture.py (+ cell_demo_vnc.py),
walks one common timeline at a fixed frame rate, and for each frame composes a
four-panel view of the cell:

    PERCEPTION  |  GAZEBO DIGITAL TWIN  |  UR CONTROLLER  |  COLOUR SORTING CELL
    RGB-D + the    the sim mirror of       the real teach     live process +
    detector       the running cell        pendant (URSim)    telemetry

Every stream is sampled by nearest-earlier timestamp, so streams running at
different rates (15 Hz iso, 5 Hz RGB-D and pendant, 1 Hz telemetry) all hold
their last value between updates instead of flickering. A stream with no data yet
renders as an explicit placeholder rather than a blank hole.

    python3 cell_demo_render.py --session <dir> --out demo.mp4
"""
import argparse
import bisect
import glob
import json
import os
import shutil
import subprocess
import tempfile

from PIL import Image, ImageDraw, ImageFont

# --- canvas ---------------------------------------------------------------
H = 820
PAD = 24

# The layout follows what was actually captured. A panel whose stream produced no
# frames is dropped rather than left as an empty column, and the canvas narrows to
# match: a run without the Gazebo twin, or without URSim, should not carry a
# placeholder for a fifth of the frame. Panel order is fixed; only presence and
# width vary. The dashboard is always present because it comes from telemetry.
PANEL_WIDTHS = {          # width when the panel is shown
    "perception": 440,
    "gazebo": 720,
    "pendant": 900,
    "dashboard": 660,
}
# Extra width handed to these panels when fewer are shown, so a two or three
# column video fills the frame instead of leaving a gap.
PANEL_STRETCH = {"gazebo": 80, "pendant": 60, "dashboard": 80}
PANEL_ORDER = ["perception", "gazebo", "pendant", "dashboard"]

W, COLS = 2720, []


def set_layout(present):
    """Build the column list from the panels that actually have data.

    `present` maps panel name -> bool. Returns nothing; sets the module globals
    W and COLS that compose() and the panel functions read.
    """
    global W, COLS
    shown = [p for p in PANEL_ORDER if present.get(p, False)]
    missing = len(PANEL_ORDER) - len(shown)
    COLS, x = [], 0
    for name in shown:
        w = PANEL_WIDTHS[name] + (PANEL_STRETCH.get(name, 0) if missing else 0)
        COLS.append((x, w, name))
        x += w
    W = x


# Labels that depend on which hardware backend the captured run used.
HARDWARE = {
    "ursim": {
        "cell_sub": "UR5e on URSim  |  ros2_control + MoveIt  |  RTDE",
        "pendant_title": "UR CONTROLLER",
        "pendant_sub": "URSim  ·  RTDE  ·  external_control.urscript",
        "pendant_notes": (
            "the real UR control stack: ros2_control drives it over RTDE",
            "the arm in the twin is THIS robot's state, not a simulated one",
        ),
        "title_sub": "Gazebo digital twin  ·  UR5e on URSim (RTDE)"
                     "  ·  ROS 2 Jazzy + MoveIt 2",
        "title_tags": "machine vision  |  real robot control"
                      "  |  functional safety (ISO/TS 15066)  |  live process telemetry",
    },
    "mock": {
        "cell_sub": "UR5e  |  ros2_control + MoveIt  |  mock hardware",
        "pendant_title": "UR CONTROLLER",
        "pendant_sub": "not in this run (mock hardware)",
        "pendant_notes": (
            "this run used ros2_control's mock hardware, so there is no pendant",
            "the RTDE path is the same; only the hardware plugin changes",
        ),
        "title_sub": "Gazebo digital twin  ·  UR5e on mock hardware"
                     "  ·  ROS 2 Jazzy + MoveIt 2",
        "title_tags": "machine vision  |  motion planning"
                      "  |  functional safety (ISO/TS 15066)  |  live process telemetry",
    },
}
HW = HARDWARE["ursim"]

BG = (8, 8, 9)
PANEL = (18, 18, 20)
LINE = (34, 34, 38)
FG = (238, 238, 240)
DIM = (140, 140, 148)
FAINT = (96, 96, 104)
ACCENT = (61, 220, 132)
WARN = (240, 176, 64)
ALARM = (232, 84, 76)
SIGNATURE = (255, 148, 42)      # byline on the title card
COLOUR_RGB = {
    "red": (216, 62, 58),
    "green": (58, 190, 84),
    "blue": (66, 116, 224),
    "yellow": (224, 190, 60),
}

FONT_DIR = "/usr/share/fonts/truetype/dejavu"

# Fraction of the RGB-D frame kept for the perception panel (see crop_centre).
RGBD_CROP = 0.6


def font(size, bold=False, mono=False):
    name = ("DejaVuSansMono-Bold.ttf" if bold else "DejaVuSansMono.ttf") if mono \
        else ("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf")
    return ImageFont.truetype(os.path.join(FONT_DIR, name), size)


F = {}


def init_fonts():
    F.update({
        "h1": font(30, bold=True), "h2": font(26, bold=True),
        "sub": font(15), "body": font(17), "body_b": font(17, bold=True),
        "small": font(14), "tiny": font(12),
        "kpi": font(34, bold=True), "kpi_lbl": font(12),
        "log": font(14, mono=True), "badge": font(17, bold=True),
        "title": font(64, bold=True), "title_sub": font(24),
        "title_small": font(18), "sig": font(20, bold=True),
    })


# --- session loading -------------------------------------------------------
class Stream:
    """Timestamped files on disk, sampled by nearest-earlier time."""

    def __init__(self, directory, exts=("jpg", "png")):
        self.items = []
        for ext in exts:
            for p in glob.glob(os.path.join(directory, f"*.{ext}")):
                stem = os.path.splitext(os.path.basename(p))[0]
                if stem.isdigit():
                    self.items.append((int(stem) / 1000.0, p))
        self.items.sort()
        self.times = [t for t, _ in self.items]
        self._cache = (None, None)

    def __len__(self):
        return len(self.items)

    def at(self, t):
        if not self.items:
            return None
        i = bisect.bisect_right(self.times, t) - 1
        if i < 0:
            i = 0
        path = self.items[i][1]
        if self._cache[0] != path:
            try:
                self._cache = (path, Image.open(path).convert("RGB"))
            except OSError:
                return self._cache[1]
        return self._cache[1]


class Events:
    """Telemetry / safety / detection samples plus a derived event log."""

    def __init__(self, path, t_start):
        self.samples = {"telemetry": [], "safety": [], "detections": [],
                        "detections_raw": []}
        rows = []
        if os.path.exists(path):
            with open(path) as fh:
                for line in fh:
                    try:
                        rows.append(json.loads(line))
                    except json.JSONDecodeError:
                        continue
        rows.sort(key=lambda r: r["t"])
        for r in rows:
            if r["kind"] in self.samples:
                self.samples[r["kind"]].append((r["t"], r["data"]))
        self.times = {k: [t for t, _ in v] for k, v in self.samples.items()}
        self.log = self._build_log(rows, t_start)
        self.log_times = [t for t, _ in self.log]

    def at(self, kind, t):
        times = self.times[kind]
        if not times:
            return None
        i = bisect.bisect_right(times, t) - 1
        return self.samples[kind][max(i, 0)][1]

    def _build_log(self, rows, t_start):
        """Turn state transitions into the operator-log lines the panel shows."""
        log = [(t_start, "demo started")]
        prev_state = prev_colour = None
        prev_safety = None
        prev_alarm = ""
        for r in rows:
            t, d = r["t"], r["data"]
            if r["kind"] == "telemetry":
                state, colour = d.get("state"), d.get("current_color") or ""
                if state == "sorting" and (state, colour) != (prev_state, prev_colour):
                    log.append((t, f"SORT {colour.upper()} commanded"))
                if state == "idle" and prev_state == "sorting" and prev_colour:
                    log.append((t, f"{prev_colour.upper()} placed on conveyor"))
                prev_state, prev_colour = state, colour
                alarm = d.get("alarm_msg") or ""
                if alarm and alarm != prev_alarm:
                    log.append((t, f"ALARM: {alarm}"))
                prev_alarm = alarm
            elif r["kind"] == "safety":
                st = d.get("state")
                if st != prev_safety:
                    scale = float(d.get("speed_scale", 1.0))
                    if st == "RUN" and prev_safety is not None:
                        log.append((t, "zone clear -> full speed"))
                    elif st == "REDUCED":
                        log.append((t, f"HUMAN in zone -> {scale * 100:.0f}% speed"))
                    elif st in ("STOP", "ESTOP"):
                        log.append((t, f"{st}: {d.get('reason', '')}"))
                    prev_safety = st
        log.sort(key=lambda e: e[0])
        return log

    def log_upto(self, t, n):
        i = bisect.bisect_right(self.log_times, t)
        return list(reversed(self.log[max(i - n, 0):i]))


# --- drawing helpers -------------------------------------------------------
def rrect(d, box, radius, fill=None, outline=None, width=1):
    d.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=width)


def crop_centre(img, frac):
    """Keep the central `frac` of the image.

    The RGB-D sensor is framed for the detector, not for a viewer: at 1 m over
    the bin most of its 70 deg field is table and pedestal. Cropping to the
    working area makes the parts legible in a 400 px panel without touching the
    camera, which must keep its calibrated pose and FOV for detector.py.
    """
    w, h = int(img.width * frac), int(img.height * frac)
    left, top = (img.width - w) // 2, (img.height - h) // 2
    return img.crop((left, top, left + w, top + h))


def fit(img, w, h):
    """Scale to fit inside w x h, preserving aspect."""
    scale = min(w / img.width, h / img.height)
    return img.resize((max(int(img.width * scale), 1), max(int(img.height * scale), 1)),
                      Image.LANCZOS)


def place(canvas, img, x, y, w, h, bg=(14, 14, 16)):
    """Letterbox img into the w x h slot at (x, y) and return the slot box."""
    canvas.paste(Image.new("RGB", (w, h), bg), (x, y))
    if img is not None:
        s = fit(img, w, h)
        canvas.paste(s, (x + (w - s.width) // 2, y + (h - s.height) // 2))
    return (x, y, x + w, y + h)


def placeholder(d, box, text):
    d.rectangle(box, outline=LINE, width=1)
    tw = d.textlength(text, font=F["small"])
    d.text(((box[0] + box[2] - tw) / 2, (box[1] + box[3]) / 2 - 8), text,
           font=F["small"], fill=FAINT)


def header(d, x, y, title, subtitle, w):
    d.text((x, y), title, font=F["h1"], fill=FG)
    if subtitle:
        d.text((x, y + 38), subtitle, font=F["sub"], fill=DIM)
    return y + 68


# --- panels ----------------------------------------------------------------
def panel_perception(canvas, d, x0, w, t, streams, events):
    x = x0 + PAD
    inner = w - 2 * PAD
    y = header(d, x, PAD, "PERCEPTION", "Gazebo RGB-D camera  ·  headless sim", inner)

    img = streams["rgbd"].at(t)
    if img is not None:
        img = crop_centre(img, RGBD_CROP)
    box = place(canvas, img, x, y, inner, int(inner * 0.75))
    if img is None:
        placeholder(d, box, "waiting for /rgbd_camera/image")
    y = box[3] + 10
    d.text((x, y), "top-down over the supply bin", font=F["tiny"], fill=FAINT)
    y += 32

    # Labelled detections if the detector publishes them, otherwise fall back to
    # the PoseArray it always publishes. PoseArray has no labels, so that path
    # reports how many parts were found and where, and does NOT guess a colour
    # per pose: inventing the mapping would make the panel say more than the
    # perception output actually knows.
    det = events.at("detections", t) or {}
    detected = det.get("detected", {}) or {}
    raw = events.at("detections_raw", t) or {}
    n_found = len(detected) if detected else int(raw.get("n", 0) or 0)

    d.text((x, y), "DETECTION", font=F["small"], fill=FG)
    label = f"{n_found} found"
    d.text((x + inner - d.textlength(label, font=F["small"]), y), label,
           font=F["small"], fill=ACCENT if n_found else FAINT)
    y += 22
    d.text((x, y), "colour segmentation + depth -> 3D pose",
           font=F["tiny"], fill=FAINT)
    y += 24
    d.line((x, y, x + inner, y), fill=LINE, width=1)
    y += 14

    # Use the labelled layout whenever labelled data exists at all, even when it
    # currently reports nothing: the arm regularly occludes the bin mid-cycle,
    # and "Red / not found" reads as the detector working, whereas dropping to
    # the unlabelled fallback for those seconds looks like a different panel.
    if det.get("colours"):
        colours = ["red", "green", "blue"]
        if "yellow" in detected:
            colours.append("yellow")
        for c in colours:
            cy = y + 9
            d.ellipse((x, cy - 7, x + 14, cy + 7), fill=COLOUR_RGB[c])
            d.text((x + 26, y), c.capitalize(), font=F["body"], fill=FG)
            hit = detected.get(c)
            if hit:
                d.text((x + 128, y + 1), "detected", font=F["small"], fill=ACCENT)
                d.text((x + 128, y + 20), f"x {hit['x']:+.3f}   y {hit['y']:+.3f}",
                       font=F["tiny"], fill=FAINT)
            else:
                d.text((x + 128, y + 1), "not found", font=F["small"], fill=FAINT)
            y += 46
    else:
        d.text((x, y), "/detected_parts  (world frame)", font=F["tiny"], fill=FAINT)
        y += 24
        for i, (px, py) in enumerate(raw.get("xy", [])[:4]):
            d.text((x, y), f"part {i + 1}", font=F["body"], fill=FG)
            d.text((x + 110, y + 2), f"x {px:+.3f}   y {py:+.3f}",
                   font=F["small"], fill=ACCENT)
            y += 34
        if not raw.get("xy"):
            d.text((x, y), "no detections yet", font=F["small"], fill=FAINT)
            y += 34
        y += 12

    y += 10
    for line in ("RGB-D 640x480 @ 10 Hz  ·  70 deg FOV",
                 "pose deprojected against the known camera pose",
                 "no learned model: thresholds + blob centroid + PCA yaw"):
        d.text((x, y), line, font=F["tiny"], fill=FAINT)
        y += 20

    d.text((x, H - PAD - 16), "Gazebo Sim 8.11  ·  ros_gz_bridge",
           font=F["tiny"], fill=FAINT)


def panel_gazebo(canvas, d, x0, w, t, streams, events):
    x = x0 + PAD
    inner = w - 2 * PAD
    y = header(d, x, PAD, "GAZEBO DIGITAL TWIN",
               "sim mirror of the running cell", inner)

    img = streams["iso"].at(t)
    box = place(canvas, img, x, y, inner, H - y - 100 - PAD)
    if img is None:
        placeholder(d, box, "waiting for /iso_camera")
    y = box[3] + 14

    tele = events.at("telemetry", t) or {}
    live = img is not None
    d.ellipse((x, y + 4, x + 12, y + 16), fill=ACCENT if live else FAINT)
    d.text((x + 22, y), "MIRRORING" if live else "NO SIGNAL",
           font=F["body_b"], fill=ACCENT if live else FAINT)
    state = (tele.get("state") or "").upper()
    if state:
        label = f"cell: {state}"
        d.text((x + inner - d.textlength(label, font=F["small"]), y + 3), label,
               font=F["small"], fill=DIM)
    y += 32

    for line in ("UR5e joints driven from /joint_states, parts from the planning scene",
                 "one-way mirror: the twin never writes back to the cell"):
        d.text((x, y), line, font=F["tiny"], fill=FAINT)
        y += 20


def panel_pendant(canvas, d, x0, w, t, streams, events):
    x = x0 + PAD
    inner = w - 2 * PAD
    y = header(d, x, PAD, HW["pendant_title"], HW["pendant_sub"], inner)

    img = streams["pendant"].at(t)
    box = place(canvas, img, x, y, inner, H - y - 70 - PAD, bg=(20, 20, 22))
    if img is None:
        placeholder(d, box, "no pendant capture in this session")
    y = box[3] + 14

    for line in HW["pendant_notes"]:
        d.text((x, y), line, font=F["tiny"], fill=FAINT)
        y += 20


def panel_dashboard(canvas, d, x0, w, t, streams, events):
    x = x0 + PAD
    inner = w - 2 * PAD
    d.text((x, PAD), "COLOUR SORTING CELL", font=F["h2"], fill=FG)
    d.text((x, PAD + 34), HW["cell_sub"], font=F["small"], fill=DIM)
    y = PAD + 66

    tele = events.at("telemetry", t) or {}
    safety = events.at("safety", t) or {}

    # state badge
    state = (tele.get("state") or "waiting").upper()
    colour = (tele.get("current_color") or "").upper()
    text = f"SORTING / {colour}" if state == "SORTING" and colour else state
    hot = state == "SORTING"
    tw = d.textlength(text, font=F["badge"])
    rrect(d, (x, y, x + tw + 40, y + 40), 20,
          outline=ACCENT if hot else FAINT, width=2)
    d.text((x + 20, y + 10), text, font=F["badge"], fill=ACCENT if hot else DIM)
    y += 56

    # safety bar
    sstate = safety.get("state") or tele.get("safety_state") or "WAITING"
    scale = float(safety.get("speed_scale", tele.get("speed_scale", 1.0)) or 1.0)
    tone = {"RUN": ACCENT, "REDUCED": WARN,
            "WAITING": FAINT}.get(sstate, ALARM)
    rrect(d, (x, y, x + inner, y + 54), 8, fill=(20, 24, 21), outline=tone, width=2)
    d.ellipse((x + 16, y + 21, x + 30, y + 35), fill=tone)
    d.text((x + 42, y + 15), f"SAFETY: {sstate}", font=F["body_b"], fill=tone)
    sp = f"Speed {scale * 100:.0f}%"
    d.text((x + inner - 16 - d.textlength(sp, font=F["body"]), y + 16), sp,
           font=F["body"], fill=DIM)
    y += 74

    # KPI tiles
    tiles = [
        (str(tele.get("parts_sorted", 0)), "Parts sorted"),
        (f"{float(tele.get('throughput_ppm', 0) or 0):.1f}", "Throughput /min"),
        (f"{float(tele.get('last_cycle_s', 0) or 0):.1f}", "Cycle (s)"),
        (str(len(tele.get("available", []) or [])), "On board"),
    ]
    gap, n = 8, len(tiles)
    tw_ = (inner - gap * (n - 1)) // n
    for i, (value, label) in enumerate(tiles):
        tx = x + i * (tw_ + gap)
        rrect(d, (tx, y, tx + tw_, y + 100), 8, fill=PANEL)
        d.text((tx + 14, y + 16), value, font=F["kpi"], fill=FG)
        d.text((tx + 14, y + 70), label, font=F["kpi_lbl"], fill=DIM)
    y += 122

    # per-colour counts
    counts = tele.get("counts") or {}
    rrect(d, (x, y, x + inner, y + 186), 8, fill=PANEL)
    d.text((x + 16, y + 14), "SORTED BY COLOUR", font=F["small"], fill=DIM)
    by = y + 46
    peak = max([1] + [int(v) for v in counts.values()])
    for c in ("red", "green", "blue"):
        v = int(counts.get(c, 0))
        d.text((x + 16, by), c.capitalize(), font=F["body"], fill=FG)
        track_x0, track_x1 = x + 108, x + inner - 46
        d.rounded_rectangle((track_x0, by + 6, track_x1, by + 22), 8, fill=(30, 30, 34))
        if v:
            fill_w = int((track_x1 - track_x0) * v / peak)
            d.rounded_rectangle((track_x0, by + 6, track_x0 + max(fill_w, 16), by + 22),
                                8, fill=COLOUR_RGB[c])
        d.text((x + inner - 32, by), str(v), font=F["body"], fill=FG)
        by += 46
    y += 206

    # event log
    log_h = H - PAD - y
    rrect(d, (x, y, x + inner, y + log_h), 8, fill=PANEL)
    d.text((x + 16, y + 14), "EVENT LOG", font=F["small"], fill=DIM)
    ly = y + 46
    rows = max(int((log_h - 56) / 26), 1)
    for ts, text in events.log_upto(t, rows):
        stamp = _clock(ts)
        d.text((x + 16, ly), stamp, font=F["log"], fill=FAINT)
        d.text((x + 104, ly), text, font=F["log"], fill=FG)
        ly += 26


def _clock(ts):
    import time as _t
    return _t.strftime("%H:%M:%S", _t.localtime(ts))


PANELS = {
    "perception": panel_perception,
    "gazebo": panel_gazebo,
    "pendant": panel_pendant,
    "dashboard": panel_dashboard,
}


def compose(t, streams, events):
    canvas = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(canvas)
    for x0, w, key in COLS:
        PANELS[key](canvas, d, x0, w, t, streams, events)
    for x0, w, _ in COLS[1:]:
        d.line((x0, PAD, x0, H - PAD), fill=LINE, width=1)
    d.line((0, 0, W, 0), fill=(20, 60, 36), width=3)
    return canvas


# --- title card ------------------------------------------------------------
def title_card():
    canvas = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(canvas)
    d.line((0, 0, W, 0), fill=(20, 60, 36), width=3)
    lines = [
        (F["title"], "Colour Sorting Cell Digital Twin", FG, 250),
        (F["title_sub"], HW["title_sub"], DIM, 356),
        (F["title_small"], HW["title_tags"], FAINT, 408),
        (F["sig"], "Mo Kamel", SIGNATURE, 560),
    ]
    for fnt, text, fill, y in lines:
        d.text(((W - d.textlength(text, font=fnt)) / 2, y), text, font=fnt, fill=fill)
    return canvas


def faded(img, alpha):
    return Image.blend(Image.new("RGB", img.size, BG), img, max(min(alpha, 1.0), 0.0))


# --- main ------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=int, default=24)
    ap.add_argument("--title-seconds", type=float, default=4.0)
    ap.add_argument("--start", type=float, default=0.0,
                    help="skip this many seconds from the start of the capture")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="seconds of capture to render; 0 renders all of it")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="play the capture faster than real time. A real RTDE "
                         "cycle takes ~37 s, so showing three colours plus a "
                         "safety event honestly takes minutes; 2 to 3 keeps every "
                         "event and still gives a video worth watching")
    ap.add_argument("--hardware", choices=sorted(HARDWARE), default="ursim",
                    help="which hardware backend the captured run used; sets the "
                         "labels so the video does not claim RTDE for a mock run")
    ap.add_argument("--keep-frames", action="store_true")
    args = ap.parse_args()
    init_fonts()

    global HW
    HW = HARDWARE[args.hardware]

    s = args.session
    streams = {
        "iso": Stream(os.path.join(s, "iso")),
        "rgbd": Stream(os.path.join(s, "rgbd")),
        "pendant": Stream(os.path.join(s, "pendant")),
    }
    for k, v in streams.items():
        print(f"  {k:9} {len(v)} frames")
    set_layout({
        "perception": len(streams["rgbd"]) > 0,
        "gazebo": len(streams["iso"]) > 0,
        "pendant": len(streams["pendant"]) > 0,
        "dashboard": True,          # always: it is drawn from telemetry
    })
    print(f"  layout    {W}x{H}  panels: {', '.join(c[2] for c in COLS)}")

    spans = [v.times[0] for v in streams.values() if len(v)]
    ends = [v.times[-1] for v in streams.values() if len(v)]
    if not spans:
        raise SystemExit("no captured frames in the session")
    # Start where the LAST stream started, not the first. A stream that comes up
    # late (the VNC client needs a moment for its first framebuffer) would
    # otherwise leave its panel showing a placeholder, or frozen on one early
    # frame, for the opening seconds of the video.
    t0, t1 = max(spans), max(ends)
    events = Events(os.path.join(s, "events.jsonl"), t0)
    print(f"  events    {sum(len(v) for v in events.samples.values())} samples,"
          f" {len(events.log)} log lines")

    t_from = t0 + args.start
    t_to = min(t_from + args.duration, t1) if args.duration else t1
    speed = max(args.speed, 0.01)
    span = t_to - t_from
    n_demo = max(int(span / speed * args.fps), 1)
    n_title = int(args.title_seconds * args.fps)
    print(f"  timeline  {span:.1f}s of capture at {speed:g}x "
          f"-> {span / speed:.1f}s, {n_demo} frames @ {args.fps}fps")

    work = tempfile.mkdtemp(prefix="cell_demo_frames_")
    try:
        card = title_card()
        for i in range(n_title):
            p = i / max(n_title - 1, 1)
            # ease in over the first third, hold, ease out over the last sixth
            alpha = min(p / 0.33, 1.0) if p < 0.33 else (
                1.0 if p < 0.84 else max((1.0 - p) / 0.16, 0.0))
            faded(card, alpha).save(os.path.join(work, f"{i:06d}.jpg"), quality=94)

        for i in range(n_demo):
            t = t_from + (i / args.fps) * speed
            frame = compose(t, streams, events)
            if i < args.fps // 3:          # fade up from the title card
                frame = faded(frame, (i + 1) / (args.fps // 3))
            frame.save(os.path.join(work, f"{n_title + i:06d}.jpg"), quality=94)
            if i % 100 == 0:
                print(f"    frame {i}/{n_demo}")

        cmd = [
            "ffmpeg", "-y", "-framerate", str(args.fps),
            "-i", os.path.join(work, "%06d.jpg"),
            "-c:v", "libx264", "-preset", "medium", "-crf", "20",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", args.out,
        ]
        subprocess.run(cmd, check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        print(f"wrote {args.out}")
    finally:
        if args.keep_frames:
            print(f"frames kept in {work}")
        else:
            shutil.rmtree(work, ignore_errors=True)


if __name__ == "__main__":
    main()
