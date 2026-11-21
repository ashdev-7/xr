"""Build the submission deck (16:9 .pptx). Usage: python tools/make_deck.py <TEAM_NAME> "<member 1, member 2, ...>" """
import sys
from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE, MSO_CONNECTOR
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.oxml.ns import qn

TEAM = sys.argv[1] if len(sys.argv) > 1 else "CollegeName_TeamName"
MEMBERS = sys.argv[2] if len(sys.argv) > 2 else "Member 1 · Member 2 · Member 3 · Member 4"
INK, MUTED, ACCENT, RED, LIGHT = RGBColor(0x1F, 0x23, 0x28), RGBColor(0x57, 0x60, 0x6A), RGBColor(0x1F, 0x6F, 0xEB), RGBColor(0xD1, 0x24, 0x2F), RGBColor(0xF3, 0xF5, 0xF8)
FONT = "Arial"
prs = Presentation(); prs.slide_width, prs.slide_height = Inches(13.333), Inches(7.5)
BLANK = prs.slide_layouts[6]
W, H = prs.slide_width, prs.slide_height


def text(slide, x, y, w, h, runs, size=18, color=INK, bold=False, align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP, spacing=1.1):
    tb = slide.shapes.add_textbox(x, y, w, h); tf = tb.text_frame; tf.word_wrap = True
    tf.vertical_anchor = anchor; tf.margin_left = tf.margin_right = Inches(0.05)
    paras = runs if isinstance(runs, list) else [runs]
    for i, p in enumerate(paras):
        para = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        para.alignment = align; para.line_spacing = spacing
        segs = p if isinstance(p, tuple) and isinstance(p[0], tuple) else (p if isinstance(p, tuple) else ((p, {}),))
        if isinstance(p, str): segs = ((p, {}),)
        for s, st in segs:
            r = para.add_run(); r.text = s; f = r.font
            f.name = FONT; f.size = Pt(st.get("size", size)); f.bold = st.get("bold", bold)
            f.color.rgb = st.get("color", color)
        para.space_after = Pt(st.get("after", 8) if segs else 8)
    return tb


def header(slide, title, kicker=None):
    if kicker: text(slide, Inches(0.7), Inches(0.45), Inches(12), Inches(0.4), kicker, size=13, color=ACCENT, bold=True)
    text(slide, Inches(0.7), Inches(0.75), Inches(12), Inches(1.0), title, size=32, bold=True)


def footer(slide, n):
    text(slide, Inches(0.7), Inches(7.0), Inches(9), Inches(0.3), f"PathGuard · Track 03 Path Finder · {TEAM}", size=10, color=MUTED)
    text(slide, Inches(11.6), Inches(7.0), Inches(1.0), Inches(0.3), str(n), size=10, color=MUTED, align=PP_ALIGN.RIGHT)


def box(slide, x, y, w, h, fill=LIGHT, line=None):
    s = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, x, y, w, h)
    s.adjustments[0] = 0.08; s.fill.solid(); s.fill.fore_color.rgb = fill
    if line: s.line.color.rgb = line; s.line.width = Pt(1.25)
    else: s.line.fill.background()
    s.shadow.inherit = False
    return s


def bullets(slide, x, y, w, h, items, size=18):
    return text(slide, x, y, w, h, [(("•  ", {"color": ACCENT, "bold": True, "size": size}), (it, {"size": size})) for it in items], size=size)


def stat(slide, x, y, w, big, small, color=ACCENT):
    text(slide, x, y, w, Inches(0.9), big, size=40, bold=True, color=color)
    text(slide, x, y + Inches(0.85), w, Inches(0.8), small, size=14, color=MUTED)


n = 0
def new():
    global n; n += 1; s = prs.slides.add_slide(BLANK)
    if n > 1: footer(s, n)
    return s

# 1 title
s = new()
bg = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, W, H); bg.fill.solid(); bg.fill.fore_color.rgb = INK; bg.line.fill.background()
text(s, Inches(0.9), Inches(1.0), Inches(11), Inches(0.5), "SAMSUNG PRISM XR HACKATHON 2026  ·  THEME: TRACK 03, PATH FINDER", size=14, color=RGBColor(0x8A, 0xB4, 0xF8), bold=True)
text(s, Inches(0.9), Inches(2.0), Inches(11.5), Inches(1.3), "PathGuard", size=66, bold=True, color=RGBColor(255, 255, 255))
text(s, Inches(0.9), Inches(3.3), Inches(11.5), Inches(1.2), "Trust-aware inertial navigation under progressive sensor degradation", size=26, color=RGBColor(0xD0, 0xD7, 0xDE))
text(s, Inches(0.9), Inches(5.3), Inches(11.5), Inches(0.5), TEAM, size=20, bold=True, color=RGBColor(255, 255, 255))
text(s, Inches(0.9), Inches(5.8), Inches(11.5), Inches(0.5), MEMBERS, size=16, color=RGBColor(0xD0, 0xD7, 0xDE))

# 2 problem
s = new(); header(s, "Navigating with a sensor that is being jammed", "THE PROBLEM, IN OUR WORDS")
bullets(s, Inches(0.7), Inches(1.9), Inches(7.2), Inches(4.8), [
    "GPS is denied. The only sensor is an IMU: gyroscope (rotation) and accelerometer (push), 200 samples per second.",
    "Jamming escalates over the mission: noise grows, sensor bias drifts, 2-second dropouts appear, and the IMU finally dies.",
    "We must output the full 6-DoF path through every checkpoint, then dead-reckon 5 seconds with no sensor at all.",
    "All of it in ≤ 8 MB of memory and ≤ 20 ms of compute per second of data, on one CPU thread, offline.",
], size=19)
box(s, Inches(8.4), Inches(1.95), Inches(4.25), Inches(4.4))
text(s, Inches(8.7), Inches(2.2), Inches(3.7), Inches(0.4), "Why it is hard", size=16, bold=True, color=MUTED)
stat(s, Inches(8.7), Inches(2.7), Inches(3.7), "0.01 m/s²", "of accelerometer bias becomes")
stat(s, Inches(8.7), Inches(4.2), Inches(3.7), "≈ 100 m", "of position error within 2.5 minutes. Position comes from integrating acceleration twice.", RED)

# 3 the trap
s = new(); header(s, "The obvious solution ends up 15 km away", "THE TRAP")
s.shapes.add_picture("figures/error_over_time.png", Inches(0.7), Inches(1.8), width=Inches(8.4))
bullets(s, Inches(9.4), Inches(2.0), Inches(3.4), Inches(4.5), [
    "Naive double integration of the degraded IMU: 15,345 m average error on unseen flights.",
    "PathGuard stays bounded through all three dropouts and the blind window.",
    "Same input, same flight: the difference is how much each step trusts the sensor.",
], size=16)

# 4 architecture
s = new(); header(s, "PathGuard: five stages, one C++ file", "SOLUTION AND ARCHITECTURE")
stages = [("1  Integrity layer", "Reject NaN, zero and frozen rows. Repair spikes. Find dropouts and the final failure."),
          ("2  Self-calibration", "Gravity direction and gyro bias from the calmest early second. Gravity alignment is unknown."),
          ("3  Attitude ESKF", "Forward and backward error-state Kalman passes, fused. Gravity corrects tilt; trust follows measured noise."),
          ("4  Translation smoother", "Per-axis Kalman + RTS smoother on position, velocity, bias, workspace centre, with a learned motion prior."),
          ("5  Blind forecast", "After failure: hold the last smoothed position, decay the last rotation rate.")]
bw, gap, y0 = Inches(2.3), Inches(0.18), Inches(2.3)
for i, (t, d) in enumerate(stages):
    x = Inches(0.7) + i * (bw + gap)
    box(s, x, y0, bw, Inches(2.9), fill=LIGHT, line=ACCENT if i in (3,) else None)
    text(s, x + Inches(0.15), y0 + Inches(0.2), bw - Inches(0.3), Inches(0.8), t, size=16, bold=True, color=ACCENT if i == 3 else INK)
    text(s, x + Inches(0.15), y0 + Inches(0.95), bw - Inches(0.3), Inches(1.9), d, size=13, color=MUTED)
    if i < 4:
        c = s.shapes.add_connector(MSO_CONNECTOR.STRAIGHT, x + bw, y0 + Inches(1.45), x + bw + gap, y0 + Inches(1.45))
        c.line.color.rgb = ACCENT; c.line.width = Pt(2)
        c.line._get_or_add_ln().append(c.line._get_or_add_ln().makeelement(qn("a:tailEnd"), {"type": "triangle"}))
text(s, Inches(0.7), Inches(1.75), Inches(12), Inches(0.4), "imu0/data.csv  →", size=15, color=MUTED)
text(s, Inches(0.7), Inches(5.45), Inches(12), Inches(1.2), [
    (("Offline is a feature: ", {"bold": True}), ("we see the whole log, so both passes run forward and backward in time. A 2 s dropout is bridged from both sides instead of extrapolated blindly.", {})),
    (("Output: ", {"bold": True}), ("estimated_trajectory.txt, one pose per IMU row (timestamp tx ty tz qx qy qz qw) plus 5 s of forecast.", {}))], size=15)

# 5 insight
s = new(); header(s, "We measured what an IMU can see, then reached it", "KEY INSIGHT")
s.shapes.add_picture("figures/floor.png", Inches(0.7), Inches(1.8), width=Inches(8.4))
bullets(s, Inches(9.4), Inches(1.9), Inches(3.5), Inches(4.8), [
    "The drone's room-scale loops take 20 to 40 s. In that band, bias and tilt errors swamp any IMU.",
    "Oracle test: know every movement faster than 10 s perfectly, nothing slower. Best possible error: 3.39 m.",
    "PathGuard: 3.39 m. The rest of the error is motion the sensor cannot observe, and we do not invent it.",
], size=16)

# 6 results
s = new(); header(s, "Results on flights never used for tuning", "RESULTS")
rows = [("Method", "ATE", "Waypoints", "Dropout", "Blind 5 s", "RPE 1 s"),
        ("Naive double integration", "15,345", "14,766", "14,885", "37,792", "591"),
        ("Stand still (no motion)", "3.51", "3.66", "3.56", "3.29", "0.79"),
        ("PathGuard", "3.39", "3.53", "3.50", "3.28", "0.77")]
tbl = s.shapes.add_table(len(rows), 6, Inches(0.7), Inches(1.9), Inches(11.9), Inches(2.4)).table
tbl.columns[0].width = Inches(4.4)
for j in range(1, 6): tbl.columns[j].width = Inches(1.5)
for i, r in enumerate(rows):
    for j, v in enumerate(r):
        c = tbl.cell(i, j); c.text = v; p = c.text_frame.paragraphs[0]
        p.alignment = PP_ALIGN.LEFT if j == 0 else PP_ALIGN.RIGHT
        f = p.runs[0].font; f.name = FONT; f.size = Pt(17); f.bold = i in (0, 3)
        f.color.rgb = RGBColor(255, 255, 255) if i == 0 else (ACCENT if i == 3 else INK)
        c.fill.solid(); c.fill.fore_color.rgb = INK if i == 0 else (RGBColor(0xEA, 0xF2, 0xFE) if i == 3 else RGBColor(255, 255, 255))
text(s, Inches(0.7), Inches(4.5), Inches(11.9), Inches(2.2), [
    "Errors in metres. ATE: SE(3)-aligned absolute trajectory error (evo-style). RPE: translation drift over 1 s.",
    "Test set: 5 EuRoC flights × 2 degradation styles (missing rows, dead rows at 1.5× severity). Tuning used 6 other flights only.",
    "Degradation follows the brief: noise rising to 6×, bias drift, spikes, three 2 s dropouts, IMU failure 5 s before the end."], size=15, color=MUTED)

# 7 ablations
s = new(); header(s, "The motion prior and the forecast choice, measured", "WHAT EACH PIECE BUYS")
s.shapes.add_picture("figures/ablation_prior.png", Inches(0.7), Inches(1.9), width=Inches(6.0))
s.shapes.add_picture("figures/blind.png", Inches(6.9), Inches(1.9), width=Inches(6.0))
text(s, Inches(0.7), Inches(5.3), Inches(5.9), Inches(1.5), "The motion prior is the core: without it the same filter drifts to 155 m.", size=15, color=MUTED)
text(s, Inches(6.9), Inches(5.3), Inches(5.9), Inches(1.5), "Blind 5 s: holding the last position beats constant-velocity extrapolation by 43%. A guessed velocity after jamming does more harm than good.", size=15, color=MUTED)

# 8 edge
s = new(); header(s, "Built for the edge from day one", "EXECUTION AND EFFICIENCY")
stat(s, Inches(0.7), Inches(2.0), Inches(3.8), "3.1 MB", "peak memory on a 3-minute flight (budget 8 MB)")
stat(s, Inches(4.7), Inches(2.0), Inches(3.8), "0.6 ms", "compute per second of IMU (budget 20 ms)")
stat(s, Inches(8.7), Inches(2.0), Inches(3.8), "0 crashes", "across every malformed-input test")
bullets(s, Inches(0.7), Inches(4.0), Inches(12), Inches(2.8), [
    "Single-threaded, offline, one dependency-free C++17 file; static build. main.py launches the same binary.",
    "Lean storage: quaternions packed in int16, filter covariances in float, translation smoothed at 10 Hz, prediction recomputed in the RTS pass.",
    "Survives: NaN rows, all-zero rows, frozen rows, spikes, missing rows, duplicated rows, no '#' header, CRLF, spaces, a 300-row file.",
    "Exactly one pose per input row, exact nanosecond-to-second timestamps, unit quaternions.",
], size=16)

# 9 honest view
s = new(); header(s, "What PathGuard will and will not claim", "HONEST BY DESIGN")
s.shapes.add_picture("figures/top_view.png", Inches(0.7), Inches(1.6), height=Inches(5.3))
bullets(s, Inches(6.7), Inches(2.0), Inches(6.0), Inches(4.8), [
    "Top view of an unseen flight. The true loop spans about 6 m; PathGuard's estimate stays compact.",
    "That is deliberate. The IMU cannot observe the slow loop, so an estimator that draws a big confident loop is guessing, and guesses score worse.",
    "PathGuard follows the fast motion the sensor really measures, and shrinks toward the workspace where it cannot know.",
    "In a real aircraft this is the difference between an honest uncertainty and a confident wrong position.",
], size=16)

# 10 stack + validation
s = new(); header(s, "Tools, tech stack and how we validated", "TECH STACK")
box(s, Inches(0.7), Inches(1.9), Inches(5.8), Inches(4.7))
text(s, Inches(1.0), Inches(2.1), Inches(5.3), Inches(0.5), "Stack", size=18, bold=True)
bullets(s, Inches(1.0), Inches(2.7), Inches(5.3), Inches(3.8), [
    "C++17, no libraries (estimator)", "Python + NumPy/SciPy (data, scoring, tuning)", "evo-style ATE / RPE scorer, reimplemented",
    "EuRoC MAV ground truth (11 flights)", "Make, Docker, Matplotlib", "Claude (AI assistant) for research and scaffolding"], size=16)
box(s, Inches(6.8), Inches(1.9), Inches(5.8), Inches(4.7))
text(s, Inches(7.1), Inches(2.1), Inches(5.3), Inches(0.5), "Validation", size=18, bold=True)
bullets(s, Inches(7.1), Inches(2.7), Inches(5.3), Inches(3.8), [
    "200 Hz IMU synthesised from ground truth with the dataset's own biases, sensor noise and propeller vibration.",
    "Degradation generator that follows the brief, two dropout styles.",
    "Split by flight: tuned on 6, reported on 5 unseen.",
    "Baselines, oracle floor, ablations, memory/runtime and malformed-input tests, all scripted and reproducible."], size=16)

# 11 innovation
s = new(); header(s, "Innovation highlights", "WHAT IS NEW")
items = [("Trust as a state", "Each stage estimates how much to believe the sensor now: integrity checks, noise-scaled filter trust, motion prior where the IMU is blind."),
         ("Offline both ways", "Forward-backward attitude fusion and RTS smoothing: dropouts are bridged with data from both sides."),
         ("A measured ceiling", "We quantified the IMU-only floor with an oracle and showed PathGuard reaches it, instead of claiming a vague improvement."),
         ("Edge-native", "3 MB and 0.6 ms per second of data: the compliance budget is met with large margins, not squeezed.")]
for i, (t, d) in enumerate(items):
    x = Inches(0.7) + (i % 2) * Inches(6.1); y = Inches(1.9) + (i // 2) * Inches(2.45)
    box(s, x, y, Inches(5.8), Inches(2.2))
    text(s, x + Inches(0.3), y + Inches(0.2), Inches(5.2), Inches(0.5), t, size=19, bold=True, color=ACCENT)
    text(s, x + Inches(0.3), y + Inches(0.8), Inches(5.2), Inches(1.4), d, size=15, color=MUTED)

# 12 limitations
s = new(); header(s, "Limitations and next steps", "LIMITATIONS")
bullets(s, Inches(0.7), Inches(1.9), Inches(12), Inches(4.8), [
    "IMU-only cannot observe slow room-scale motion; absolute error stays near 3.4 m on EuRoC for that reason.",
    "Development IMU was synthesised from ground truth; parameters may need re-tuning for the organizers' degradation script.",
    "Memory grows about 48 bytes per IMU row: inputs up to about 12 minutes at 200 Hz fit in 8 MB.",
    "Next: a small learned motion prior (TLIO-style, a few hundred KB) to push below the 10 s oracle; any absolute aiding (barometer, magnetometer, a camera checkpoint) would cut error sharply, and the smoother already accepts such updates.",
], size=18)

prs.save(f"{TEAM}.pptx"); print(f"saved {TEAM}.pptx")
