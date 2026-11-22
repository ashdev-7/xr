"""Build the deck used to RECORD the walkthrough video with PowerPoint's own recorder (no other software needed).

From the final submission deck: slides in narration order, the demo slide shows the real output of the demo
commands (captured in results/demo_terminal.txt), and every slide carries its narration in the speaker notes
(PowerPoint shows them in the recording teleprompter). Also rewrites VIDEO_SCRIPT.md from the same narration.
usage: python tools/make_video_deck.py   (after tools/make_deck.py)
"""
import glob, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pptx import Presentation
from pptx.util import Inches

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
src = glob.glob(os.path.join(ROOT, "dist", "*_ashdev7.pptx"))[0]
OUT = os.path.join(ROOT, "dist", "PathGuard_video_deck.pptx")

# (original slide number, time, narration)
PLAN = [
    (1, "0:00-0:15", "An aircraft is flying with GPS jammed. Its only sensor is an IMU, and the jamming keeps getting "
                     "worse, until the IMU dies and the aircraft must dead-reckon five more seconds. This is PathGuard, "
                     "from team ashdev7."),
    (2, "0:15-0:40", "We get a raw EuRoC-format IMU log, degraded with noise, drifting biases, spikes and lost frames, and "
                     "must output a six-degree-of-freedom pose for every row, in Python, within 8 megabytes and 20 "
                     "milliseconds per second of data. Integrating the accelerometer twice puts you thousands of metres off."),
    (7, "0:40-1:10", "Here is a real run on a sample flight. It is one standard-library Python file. The log shows what it "
                     "found: a rotorcraft, from its rotor vibration; the jamming level, read from packet loss; frozen and "
                     "dead rows; rest periods. Eighty-five seconds of IMU take a third of a second and under three "
                     "megabytes. One pose per input row, checked by eighteen malformed-input tests."),
    (4, "1:10-1:45", "First, one streaming pass cleans the data and measures the jamming. An error-state Kalman smoother "
                     "estimates attitude from gyro, gravity and rest periods. Position comes from a second smoother with a "
                     "motion prior per platform. For drones we use rotor drag: the sideways accelerometer reading is "
                     "proportional to velocity, so the accelerometer becomes a velocity sensor. Every input row gets one pose."),
    (5, "1:45-2:15", "We validated on real EuRoC and TUM-VI data, degraded with the organizers' own script, on flights held "
                     "out while we built it. Drones: 2.35 metres ATE against 3.20 for standing still and 18,645 for naive "
                     "integration. Handheld: 0.52 against 1.10. PathGuard beats standing still on every held-out flight."),
    (8, "2:15-2:35", "This is the drag model at work: body velocity from the accelerometer alone, against ground truth. "
                     "Every component is backed by an ablation, and we attacked the system with 52 adversarial inputs and "
                     "checked our scores with the organizers' evo tool."),
    (6, "2:35-2:50", "Physics first, measured at every step, and inside every budget, with a 4x margin on memory and runtime."),
    (9, "2:50-2:55", "PathGuard. Thank you."),
]


def terminal_png(path):
    lines = open(os.path.join(ROOT, "results", "demo_terminal.txt"), encoding="utf-8").read().splitlines()
    lines = [l if len(l) <= 112 else l[:109] + "..." for l in lines]
    lines += ["", "# 85 s of IMU in 0.36 s (4.2 ms per s of IMU), peak traced heap 2.8 MB"]
    fig = plt.figure(figsize=(9.2, 3.9), dpi=250)
    fig.patch.set_facecolor("#0B0F1A")
    y = 0.97
    for l in lines:
        col = "#29B6F6" if l.startswith("$") else ("#FFB74D" if l.startswith("#") and "IMU in" in l else "#E6EDF3")
        fig.text(0.01, y, l, family="DejaVu Sans Mono", fontsize=7.6, color=col, va="top")
        y -= 0.052
    fig.savefig(path, facecolor=fig.get_facecolor()); plt.close(fig)


def main():
    prs = Presentation(src)
    slides = list(prs.slides)
    png = os.path.join(ROOT, "figures", "dark", "demo_terminal.png")
    terminal_png(png)
    demo = slides[6]
    for sh in list(demo.shapes):  # replace the command list by the real run
        if sh.has_text_frame and sh.text_frame.text.lstrip().startswith("# run"):
            sh._element.getparent().remove(sh._element)
    demo.shapes.add_picture(png, Inches(0.4), Inches(1.45), Inches(9.2), Inches(3.9))
    for n, _, text in PLAN:
        slides[n - 1].notes_slide.notes_text_frame.text = text
    # reorder / drop slides to the narration order
    lst = prs.slides._sldIdLst
    ids = list(lst)
    keep = [ids[n - 1] for n, _, _ in PLAN]
    for el in ids:
        lst.remove(el)
        if el not in keep:
            prs.part.drop_rel(el.rId)
    for el in keep:
        lst.append(el)
    prs.save(OUT)
    words = sum(len(t.split()) for _, _, t in PLAN)
    print("video deck:", OUT, f"({len(PLAN)} slides, {words} words, ~{words / 2.5:.0f} s at 150 words/min)")
    with open(os.path.join(ROOT, "VIDEO_SCRIPT.md"), "w", encoding="utf-8", newline="\n") as f:
        f.write("# PathGuard: walkthrough video (hard limit 3:00)\n\n"
                "Recorded with PowerPoint's built-in recorder from `dist/PathGuard_video_deck.pptx` (built by\n"
                "`tools/make_video_deck.py`); the narration below is also in each slide's speaker notes.\n\n")
        for n, tm, text in PLAN:
            f.write(f"**{tm}** (submission deck slide {n})\n> {text}\n\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
