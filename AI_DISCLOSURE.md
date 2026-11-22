# AI disclosure

Team ashdev7 (Yash Tyagi), Thapar Institute of Engineering and Technology, Patiala · Track 03 (Path Finder)

## Tools used
| Tool | Role |
|---|---|
| Claude Code (Anthropic, Claude Opus 5.5) | Engineering and research assistant: proposed design options, implemented the estimator and tools, ran the experiments, wrote the documentation, figures, deck and bundle generators |
| Windows text-to-speech (Microsoft Hazel voice) | Narration of the walkthrough video, read from the script in the slide notes (`tools/make_video.ps1`) |
| ChatGPT (OpenAI) and Gemini (Google) | Independent reviewers of the finished submission (scoring against the rubric, code review). Their findings were checked against the code and measurements before any change; see the last entries of `EXPERIMENTS.md` |

## How the work was divided
**The team (Yash Tyagi) led and directed the project:**
- Framed the problem and set the bar: win on architecture, design, accuracy and precision within the official
  budgets (8 MB, 20 ms per second of IMU, Python, offline).
- Set the rules of the work: the estimator never reads ground truth, no fingerprinting of evaluation sequences,
  held-out validation by flight, measure before claiming, every result reproducible.
- Steered the direction at each stage: questioned whether trajectory scale was handled correctly, required an
  exhaustive full-text literature review (2017 onwards) before further changes, rejected deliverables that were not
  of submission quality, required an adversarial QA audit, and commissioned independent reviews.
- Reviewed results and deliverables and decided what was submitted.

**Claude worked under that direction:** it proposed candidate designs (for example the rotor-drag velocity
measurement, the packet-loss jamming meter and the error-state smoothers), implemented them, and ran the
experiments. Design decisions were taken on measured evidence: a component was kept only if it improved flights
held out from tuning, and every decision, including rejected ideas and corrections to earlier claims, is logged in
`EXPERIMENTS.md`.

## Integrity commitments
- The estimator reads only the degraded IMU file. Ground truth is used only by the offline scorer and oracle
  experiments, never by `main.py`.
- No sequence fingerprinting and no use of the degradation script's fixed seed to regenerate or subtract noise.
- All reported numbers are reproducible with `tools/bench/report.py`, `tools/bench/stats.py` and
  `tools/qa/break_it.py`.
