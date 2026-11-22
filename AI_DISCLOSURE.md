# AI disclosure

Team ashdev7 (Yash Tyagi), Thapar Institute of Engineering and Technology, Patiala · Track 03 (Path Finder)

## Tools used
| Tool | Role |
|---|---|
| Claude Code (Anthropic, Claude Opus 5.5) | Coding and research assistant: implementation, experiments, literature review, documentation, figures, deck and bundle generation |
| ChatGPT (OpenAI) and Gemini (Google) | Independent reviewers of the finished submission (scoring against the rubric, code review). Their findings were checked against the code and measurements before any change; see the last entries of `EXPERIMENTS.md` |

## How AI was used
- **Implementation.** The estimator (`src/stages/*.py`, embedded in `src/main.py`), the benchmark and QA tools, and the
  deck/bundle generators were written with Claude Code under the team's direction.
- **Research.** Claude carried out the literature review (`docs/literature_review_full_text.md`); every source is
  labelled full-text or abstract-only. Ideas from the literature were kept only if they improved held-out flights.
- **Experiments.** Every design decision was made by measurement on real public EuRoC and TUM-VI data degraded with
  the organizers' unmodified script, split by flight into TRAIN and TEST. Each change has a hypothesis, a metric and
  a before/after in `EXPERIMENTS.md`, including the ideas that were rejected and the corrections to earlier claims.

## What the team did
- Set the objectives, constraints and acceptance standards (the official rubric and FAQ; no ground truth in the
  estimator, no fingerprinting of evaluation sequences, held-out validation, measure before claiming).
- Directed the work, reviewed results and deliverables, requested the independent reviews, and decided what was
  submitted.

## Integrity commitments
- The estimator reads only the degraded IMU file. Ground truth is used only by the offline scorer and oracle
  experiments, never by `main.py`.
- No sequence fingerprinting and no use of the degradation script's fixed seed to regenerate or subtract noise.
- All reported numbers are reproducible with `tools/bench/report.py`, `tools/bench/stats.py` and
  `tools/qa/break_it.py`.
