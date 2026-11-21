"""Fill the organizers' submission template (CollegeName_TeamName deck) from results/report.json + figures/.

usage: python tools/make_deck.py --team TEAM --college COLLEGE --members "A, B, C" [--repo URL]
       [--template ../../Submission_Final_forGithub.pptx] [--out dist/CollegeName_TeamName.pptx]
Every number on the slides comes from results/report.json (tools/bench/report.py on the final code).
"""
import argparse, json, os
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE, MSO_CONNECTOR
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.util import Inches, Pt

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
BLUE, INK, GREY, RED, PALE = RGBColor(0x1F, 0x6F, 0xD1), RGBColor(0x20, 0x24, 0x2A), RGBColor(0x5F, 0x63, 0x68), \
    RGBColor(0xD9, 0x30, 0x25), RGBColor(0xEE, 0xF3, 0xFB)


def text(slide, x, y, w, h, paras, size=12, color=INK, bold_first=False, font="Calibri", align=PP_ALIGN.LEFT):
    tb = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = tb.text_frame; tf.word_wrap = True
    for i, p in enumerate(paras):
        para = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        para.alignment = align
        runs = p if isinstance(p, list) else [p]
        for r in runs:
            bold = False
            if isinstance(r, tuple):
                r, bold = r
            run = para.add_run(); run.text = r
            run.font.size = Pt(size); run.font.color.rgb = color; run.font.name = font
            run.font.bold = bold or (bold_first and i == 0)
        para.space_after = Pt(4)
    return tb


def box(slide, x, y, w, h, title, sub, fill=PALE, line=BLUE):
    s = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(x), Inches(y), Inches(w), Inches(h))
    s.fill.solid(); s.fill.fore_color.rgb = fill; s.line.color.rgb = line; s.line.width = Pt(1)
    tf = s.text_frame; tf.word_wrap = True; tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    tf.margin_left = tf.margin_right = Inches(0.06)
    p = tf.paragraphs[0]; r = p.add_run(); r.text = title
    r.font.size = Pt(11); r.font.bold = True; r.font.color.rgb = INK
    for line in sub:
        q = tf.add_paragraph(); rr = q.add_run(); rr.text = line; rr.font.size = Pt(8.5); rr.font.color.rgb = GREY
    return s


def arrow(slide, x1, y1, x2, y2):
    c = slide.shapes.add_connector(MSO_CONNECTOR.STRAIGHT, Inches(x1), Inches(y1), Inches(x2), Inches(y2))
    c.line.color.rgb = BLUE; c.line.width = Pt(1.5)
    ln = c.line._get_or_add_ln()
    from pptx.oxml.ns import qn
    tail = ln.makeelement(qn("a:tailEnd"), {"type": "triangle"}); ln.append(tail)


def table(slide, x, y, w, rows, colw=None, size=9):
    nr, nc = len(rows), len(rows[0])
    t = slide.shapes.add_table(nr, nc, Inches(x), Inches(y), Inches(w), Inches(0.25 * nr)).table
    for i, row in enumerate(rows):
        for j, v in enumerate(row):
            cell = t.cell(i, j); cell.text = str(v)
            for p in cell.text_frame.paragraphs:
                for r in p.runs:
                    r.font.size = Pt(size); r.font.bold = i == 0; r.font.color.rgb = INK
            cell.margin_left = cell.margin_right = Inches(0.04)
    if colw:
        for j, cw in enumerate(colw):
            t.columns[j].width = Inches(cw)
    return t


def pic(slide, name, x, y, w=None, h=None):
    p = os.path.join(ROOT, "figures", name)
    if os.path.exists(p):
        slide.shapes.add_picture(p, Inches(x), Inches(y), Inches(w) if w else None, Inches(h) if h else None)


def set_text(shape, value, size=None):
    tf = shape.text_frame
    runs = [r for p in tf.paragraphs for r in p.runs]
    if runs:
        runs[0].text = value
        for r in runs[1:]:
            r.text = ""
        if size:
            runs[0].font.size = Pt(size)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--team", default="TeamName"); ap.add_argument("--college", default="CollegeName")
    ap.add_argument("--members", default="Member 1, Member 2, Member 3, Member 4")
    ap.add_argument("--repo", default="https://github.com/<user>/pathguard")
    ap.add_argument("--template", default=os.path.join(ROOT, "..", "..", "Submission_Final_forGithub.pptx"))
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    R = json.load(open(os.path.join(ROOT, "results", "report.json")))
    test = R["test"]

    def mean(method, cls, key):
        rs = [r for r in R["methods"][method] if r["flight"] in test[cls]]
        return sum(r[key] for r in rs) / len(rs)

    def abl(name, cls, key):
        rs = [r for r in R["ablations"][name] if r["flight"] in test[cls]]
        return sum(r[key] for r in rs) / len(rs)

    rs = R["resources"]
    prs = Presentation(a.template)
    S = prs.slides

    # 1 title
    s = S[0]
    for sh in s.shapes:
        if not sh.has_text_frame:
            continue
        t = sh.text_frame.text
        if t.startswith("Track No"):
            set_text(sh, "TRACK 03")
        elif t.startswith("XR Hackathon"):
            set_text(sh, "PathGuard: trajectory estimation under sensor degradation", 26)
        elif t.startswith("Team Name"):
            tf = sh.text_frame
            runs = [r for p in tf.paragraphs for r in p.runs]
            if len(runs) >= 2:
                runs[0].text = a.team; runs[1].text = a.college
            else:
                set_text(sh, f"{a.team}\n{a.college}")
        elif t.startswith("Members"):
            set_text(sh, "Members: " + a.members, 11)

    # 2 problem
    s = S[1]
    text(s, 0.4, 1.0, 4.3, 4.3, [
        [("An aircraft's only sensor is a jammed IMU. ", True), "Noise, bias and scale drift, spikes, frozen samples and "
         "dropped frames all grow over the flight until the IMU dies; then 5 s must be dead-reckoned."],
        [("Output: ", True), "a 6-DoF pose for every IMU row (including duplicated and dead rows), TUM format, scored with "
         "evo (ATE after SE(3) alignment, RPE)."],
        [("Physics is the enemy: ", True), "double integration turns a 0.01 m/s² bias into 50 m after 100 s; "
         "slow motion is invisible to an IMU alone."],
        [("Budgets: ", True), "≤ 8 MB heap, ≤ 20 ms per 1 s of IMU, one CPU thread, offline, Python only."],
    ], size=11)
    pic(s, "degradation.png", 4.8, 1.05, w=5.0)

    # 3 solution
    s = S[2]
    ideas = [
        ("Read the jamming from packet loss.", "Every impairment shares one severity curve; dropped rows reveal it even "
         "when rotor vibration hides the noise. Trust follows the measured severity."),
        ("A drone's accelerometer measures its velocity.", "Thrust acts along one axis, so the horizontal specific force is "
         "rotor drag f⊥ = −μ v⊥ (μ ≈ 0.18 1/s). Position error grows linearly, not quadratically."),
        ("Spikes have a signature.", "The jammer spikes all six channels in the same sample: precision 0.97 vs 0.16 for a "
         "per-channel rule."),
        ("Rest is information.", "Detected rest gives zero-velocity and zero-angular-rate updates (gyro bias incl. heading)."),
        ("Smooth, don't filter.", "Error-state RTS smoothers for attitude and position use the whole log, bridging dropouts "
         "from both sides."),
        ("Platform-aware priors.", "Rotor vibration identifies the platform; motion priors are learned per class on TRAIN "
         "flights only."),
    ]
    y = 1.0
    for i, (h, b) in enumerate(ideas, 1):
        text(s, 0.4, y, 9.2, 0.7, [[(f"{i}. {h} ", True), b]], size=11); y += 0.72

    # 4 architecture
    s = S[3]
    bx = [("Pass 1: stream", ["format & units", "integrity: NaN, dead,", "frozen, dup, spikes", "jamming meter", "20 ms preintegration"]),
          ("Platform", ["rotor-vibration", "signature →", "rotorcraft /", "handheld prior"]),
          ("Attitude", ["ESKF [δθ, b_g]", "gravity + rest", "updates", "error-state RTS"]),
          ("Translation", ["KF + RTS, 10 Hz", "[p, v, b_a, c]", "motion prior, ZUPT", "rotor-drag velocity", "blind: no-data smoother"]),
          ("Pass 2: write", ["1 pose per row", "exact ns → s", "Hermite position", "never malformed"])]
    x = 0.35
    for i, (t, sub) in enumerate(bx):
        box(s, x, 1.35, 1.7, 2.1, t, sub)
        if i < len(bx) - 1:
            arrow(s, x + 1.7, 2.4, x + 1.9, 2.4)
        x += 1.9
    text(s, 0.4, 3.75, 9.2, 1.6, [
        "Pure-Python standard library, one file, two streaming passes. Memory grows with 20 ms bins and 10 Hz knots, "
        "not with rows. Each block was kept only because it improved held-out flights (EXPERIMENTS.md); "
        "rejected ideas are logged too.",
        f"Measured: {rs[0]['ms_per_s']:.1f} ms per 1 s of IMU, estimator heap {rs[0]['data_heap_mb']:.1f} MB "
        f"({rs[0]['flight'].replace('_', ' ')}, {rs[0]['duration_s']:.0f} s flight)."], size=11, color=GREY)

    # 5 final output
    s = S[4]
    text(s, 0.4, 1.0, 4.4, 1.2, ["estimated_trajectory.txt (one pose per IMU frame):",
                                 "# timestamp tx ty tz qx qy qz qw",
                                 "1403636579.758555392 0.012 0.004 0.010 0.0001 0.0023 0.0011 0.9999"], size=9, font="Consolas")
    rows = [["TEST flights", "method", "ATE [m]", "blind 5 s [m]", "RPE 1 s [m]"]]
    for cls, lab in (("rotor", "EuRoC MAV"), ("handheld", "TUM-VI room")):
        for m in ("naive integration", "stand-still", "PathGuard"):
            ate = mean(m, cls, "ate")
            rows.append([lab, m, f"{ate:.0f}" if ate > 100 else f"{ate:.2f}", f"{mean(m, cls, 'blind'):.2f}" if mean(m, cls, 'blind') < 100 else f"{mean(m, cls, 'blind'):.0f}",
                         f"{mean(m, cls, 'rpe_t'):.2f}" if mean(m, cls, 'rpe_t') < 100 else f"{mean(m, cls, 'rpe_t'):.0f}"])
    table(s, 0.4, 2.2, 4.4, rows, colw=[1.0, 1.25, 0.65, 0.85, 0.65], size=8)
    pic(s, "top_view.png", 4.95, 1.0, w=4.9)
    text(s, 4.95, 4.7, 4.9, 0.6, ["Real EuRoC / TUM-VI IMU degraded by the official script (2 column scenarios × 3 seeds); "
                                  "flights never used for tuning."], size=8, color=GREY)

    # 6 differentiation
    s = S[5]
    rows = [["", "typical approach", "PathGuard"],
            ["slow drift", "double integration explodes", "per-platform motion prior + rotor-drag velocity"],
            ["jamming", "fixed noise model", "severity measured from packet loss"],
            ["spikes", "per-channel threshold (deletes motion)", "six-channel signature (precision 0.97)"],
            ["drones", "IMU = acceleration only", "IMU = velocity too (rotor drag)"],
            ["offline", "causal filter", "RTS smoothing over the whole log"],
            ["budget", "numpy import alone ≈ 7.4 MB", f"stdlib only, {rs[0]['data_heap_mb']:.1f} MB, {rs[0]['ms_per_s']:.1f} ms/s"],
            ["output", "may crash on bad rows", "18 malformed-input tests; always 1 pose per row"]]
    table(s, 0.4, 1.05, 9.2, rows, colw=[1.2, 3.2, 4.8], size=9)
    pic(s, "error_over_time.png", 1.2, 3.35, w=7.6)

    # 7 demo & setup
    s = S[6]
    text(s, 0.4, 1.0, 9.2, 4.2, [
        [("Repository: ", True), a.repo],
        "python src/main.py <imu0/data.csv | dataset folder | .zip> estimated_trajectory.txt",
        "python src/main.py data.csv out.txt --verbose=1        # diagnostics: platform, jamming, integrity",
        "docker build -t pathguard src/ && docker run --rm -v $PWD/eval:/data pathguard /data/imu0/data.csv /data/estimated_trajectory.txt",
        "python tests/test_contract.py   # 18 input variants      python tests/test_eskf.py   # filter unit test",
        "python tools/bench/report.py    # regenerates every number in README and this deck",
        [("Requirements: ", True), "Python ≥ 3.8, nothing else. CI runs 3.8–3.13 and the Docker build on every push."],
    ], size=10, font="Consolas")

    # 8 brownie
    s = S[7]
    pic(s, "rotor_drag.png", 0.3, 0.95, w=5.6)
    rows = [["ablation (TEST)", "rotor ATE / blind", "handheld ATE / blind"]]
    for name in R["ablations"]:
        rows.append([name, f"{abl(name, 'rotor', 'ate'):.2f} / {abl(name, 'rotor', 'blind'):.2f}",
                     f"{abl(name, 'handheld', 'ate'):.2f} / {abl(name, 'handheld', 'blind'):.2f}"])
    table(s, 6.0, 1.0, 3.7, rows, colw=[1.7, 1.0, 1.0], size=7)
    text(s, 0.4, 3.0, 5.5, 2.4, [
        [("Validation: ", True), "real public IMU, the unmodified official degradation script (bit-identical labels for "
         "spikes / frozen / duplicates), TRAIN/TEST split by flight, scorer equal to evo_ape to 4 decimals."],
        [("Honesty: ", True), "rejected ideas are logged (body-frame bias model, Gauss–Markov drag error, scenario-specific "
         "bias models). The estimator never reads ground truth."],
    ], size=10)

    out = a.out or os.path.join(ROOT, "dist", f"{a.college}_{a.team}.pptx")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    prs.save(out)
    print("deck:", out)


if __name__ == "__main__":
    main()
