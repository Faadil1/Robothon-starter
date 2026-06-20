# Safety-Gated Pusher — Hazard-Aware Rescue Task

*(Robothon 2026 submission)*

A MuJoCo simulation where a deterministic pusher controller plans a push
toward a target. A safety gate checks the planned path against a marked
hazard / no-go zone *before* any motion is executed:

- **ALLOW** — path is clear → the push executes, the object is delivered to
  the rescue/goal zone.
- **BLOCK** — path crosses the hazard zone → the push never executes, object
  stays exactly where it started.

Every verdict (ALLOW/BLOCK) is signed with Ed25519 and logged, as a thin
evidence layer over the simulated decision — not the centerpiece of the
project. The simulation is the deliverable; the receipt is supporting
evidence that the verdict was real and unaltered.

## Mission Design

A robot must push a critical payload into a rescue zone while avoiding
unsafe trajectories through a hazard/no-go zone. Rather than always
executing whatever motion it plans, the robot first checks whether that
plan is safe. A safe plan executes; an unsafe plan is blocked before any
motion occurs.

This project treats robot motion as a decision that must be authorized,
blocked when unsafe, and backed by verifiable evidence. The two scenarios
in this repository demonstrate both halves of that decision: Scenario A
shows a safe push being authorized and carried out; Scenario B shows an
unsafe push being blocked before it can happen.

## How this maps to the Robothon judging rubric

| Criterion | How this project addresses it |
|---|---|
| **Runnability** | One-command run (`python run.py`), fully deterministic output, exit code 0 on success; tested independently in this build environment and in Google Cloud Shell. |
| **MuJoCo depth** | Custom MJCF scene with a position-actuated pusher, free-body object dynamics, contact-based pushing (not teleported/scripted motion), a passive touch sensor providing contact-force telemetry, and explicit visual hazard/rescue zone geometry — not a stock example scene. |
| **Task design** | A safe rescue push vs. an unsafe blocked path: an autonomous agent must judge whether a planned object-delivery trajectory is safe *before* committing to it, mirroring real hazard-avoidance requirements in embodied AI (e.g. warehouse or disaster-response settings where blind execution is unacceptable). |
| **Control** | Deterministic, closed-loop autonomous position control drives the pusher through a planned waypoint sequence; execution itself is gated by a safety check, not just planned. |
| **Engineering quality** | Modular separation of concerns: scene (`scene.xml`), planning (`controller.py`), safety policy (`safety_gate.py`), orchestration (`episode_runner.py`), evidence (`receipt_logger.py`) — each independently testable, documented here. |
| **Presentation** | Demo video walks through scene setup, hazard/rescue zone identification, both decision outcomes, and their signed verification — see `demo.mp4`. |
| **Innovation** | Signed decision receipts for robot actions: the safety gate's ALLOW/BLOCK decision is cryptographically signed, producing an auditable record of *why* the robot did or did not act — a governance pattern not common among simulator-only submissions. |
| **Human-AI Collaboration** | Claude Code assisted implementation and demo iteration; ChatGPT assisted independent audit and scoring-risk review; Google Cloud Shell was used as an independent reproducibility environment; all final design, risk, and submission decisions were made by the human operator. See the dedicated section below. |

## What this is NOT

- Not a dashboard or UI project. No web frontend exists.
- Not reinforcement learning. The controller is fully deterministic.
- Not multi-object. One pusher, one object, one goal zone, one no-go zone.

## Innovation — Verifiable Robot Decisions

Unlike a standard robotics demo that only shows motion, this project logs
each robot decision as signed evidence. Safe actions are allowed, unsafe
actions are blocked, and both outcomes are verifiable:

- Every push attempt produces an explicit **ALLOW** or **BLOCK** verdict.
- Each verdict is signed with **Ed25519** before being written to the log.
- The safety gate's check is **deterministic** — the same planned path
  always produces the same verdict, with no hidden state.
- The resulting evidence log is **reproducible**: anyone can independently
  re-verify every signed receipt with `python receipt_logger.py verify`,
  and tampering with a logged verdict is detectable (this was tested
  during development — see the verification step in the run instructions).

The novelty here is not the push task itself, but the fact that the
robot's decision about whether to act is treated as a first-class,
auditable artifact, rather than something that only exists implicitly in
the robot's motion.

### Contact telemetry (passive sensor evidence)

A passive MuJoCo `touch` sensor is attached to the pusher's contact site.
It does not influence control, the safety gate, or task outcomes in any
way — it is read-only instrumentation. During each episode, two
additional values are recorded and **included in the signed receipt**:

- `contact_count` — the number of physics steps during which the pusher
  was in contact with the object.
- `max_contact_force` — the peak contact-normal force (in simulated
  Newtons) measured by the touch sensor during the episode.

For Scenario A (ALLOW), these values are non-zero, reflecting genuine
sustained contact during the push. For Scenario B (BLOCK), both values
are exactly zero, because the push is never executed and no contact ever
occurs — the telemetry itself corroborates the verdict.

Because these two fields are part of the signed payload (not just
appended unsigned metadata), tampering with either value after the fact
breaks Ed25519 verification, the same way tampering with the verdict
does. This was tested directly during development.

## Human-AI Collaboration

This project was built through a structured collaboration between a human
operator and AI assistants, with the human retaining final authority over
every consequential decision:

- **Claude Code** assisted with implementation: the original MuJoCo scene,
  controller, safety gate, episode runner, and receipt logger, as well as
  the V2 demo and documentation improvements.
- **ChatGPT** assisted with independent audit, scoring-risk review, and
  go/no-go assessment of proposed changes against the Robothon rubric.
- **Google Cloud Shell** was used as an independent reproducibility
  environment, separate from the primary build environment, to confirm
  the project runs cleanly outside the context it was developed in.
- The **human operator** made the final design, risk, and submission
  decisions throughout — including which scenarios to ship, which rubric
  gaps were worth addressing, and when to stop iterating.

An adaptive detour scenario ("Scenario C") was explored during V2, but it
was not shipped because physical testing showed it was mechanically
unstable for the current pusher design: a single rigid contact pusher
could not reliably follow a curved or multi-segment path without losing
contact with the object, across several independently tested geometric
approaches. This go/no-go decision preserved the integrity of the already
-scored Scenario A/B behavior and reflects the same responsible-scope
discipline this project's safety gate is built around: don't ship a motion
plan that hasn't been verified to work.

AI tools were used as assistants under human control throughout, with
explicit verification (regression tests before and after every change)
and rollback discipline (every risky change was backed up before being
attempted, and reverted cleanly when it didn't hold up).

## Responsible Scope / Safety Note

This project deliberately prioritizes:

- **Reliable execution over overbuilt complexity** — two scenarios that
  work correctly and reproducibly, rather than a third scenario that
  looked more impressive but couldn't be made to run reliably.
- **Reproducibility over flashy instability** — every claim in this
  README and in the demo video is backed by a test that was actually run,
  not just described.
- **Safety-gated autonomy over uncontrolled motion** — the robot does not
  execute a planned action without first checking whether that action is
  safe.
- **Transparent receipts over unverifiable claims** — every decision the
  safety gate makes is signed and independently checkable, not just
  asserted in a log message.

## Requirements

- Python 3.12 (tested)
- `pip install mujoco numpy cryptography pillow` (`pillow` only needed for
  `record_demo.py`; `run.py` itself has no rendering dependency)
- `ffmpeg` on PATH (only needed for `record_demo.py`)

## Run instructions

```bash
# Clean run — runs both scenarios, writes logs/episodes.jsonl and
# logs/receipts.jsonl, verifies all receipts, exits 0 on success.
rm -rf logs keys
python run.py
```

Expected output: Scenario A reports verdict `ALLOW` with the object ending
inside the goal zone; Scenario B reports verdict `BLOCK` with the object's
end position identical to its start position. Final line: `RESULT: PASS`.

### Verify receipts independently

```bash
python receipt_logger.py verify logs/receipts.jsonl
```

Exits 0 if all receipts verify, non-zero otherwise. Tampering with any
signed field (e.g. editing `verdict` in the log file by hand) will cause
verification to fail for that line — this was tested during development.

### Reproduce the demo video

```bash
python record_demo.py
```

This re-runs both scenarios through the same `run_episode()` function used
by `run.py`, with a frame collector attached, and encodes the captured
frames into `demo.mp4` via ffmpeg. Requires `MUJOCO_GL=egl` for headless
rendering (set automatically by the script; override if your environment
needs a different backend, e.g. `MUJOCO_GL=glfw` for an on-screen window).

## Architecture

| File | Responsibility |
|---|---|
| `scene.xml` | MuJoCo MJCF: table, pusher, object, goal zone, no-go zone |
| `controller.py` | Deterministic straight-line push planner (no RL) |
| `safety_gate.py` | Pure geometric check: does the planned path cross the no-go zone? |
| `episode_runner.py` | Orchestrates one episode: plan → gate → execute or halt |
| `receipt_logger.py` | Ed25519 signing/verification of episode verdicts |
| `run.py` | CLI entrypoint: runs both scenarios, asserts correctness, exits 0/1 |
| `record_demo.py` | Captures real frames during execution, adds narration title-cards and hold-frames around them, encodes `demo.mp4` (~60-90s) |

## Safety rule

The no-go zone is a fixed rectangular region on the table (rendered in red).
The safety gate checks every waypoint of the planned push path against this
region using simple point-in-rectangle geometry. If any waypoint falls
inside the no-go zone, the verdict is `BLOCK` and the push is never
executed — no pusher motion occurs, no force is applied to the object.

This rule was chosen because it is purely geometric (no force/velocity
threshold tuning required), deterministic, and visually obvious in the
recorded demo: viewers can see the red zone and see that Scenario B's
planned path runs through it.

## Demo capture checklist

- [ ] Run `rm -rf logs keys frames demo.mp4` for a clean slate before
      recording, so the receipt log starts empty.
- [ ] Run `python record_demo.py` and confirm it exits 0.
- [ ] Confirm `demo.mp4` exists, runs ~60-90s, and shows in order: intro
      title card, scene/hazard/rescue zone establishing shots, Scenario A
      (ALLOW) execution and final state, Scenario A receipt verification
      card, contact telemetry card (real measured contact count/peak
      force), Scenario B (BLOCK) attempt and final state, Scenario B
      receipt verification card, summary card ("Safe actions execute.
      Unsafe actions are blocked. Every decision is signed and
      verifiable."), Human-AI collaboration closing card.
- [ ] Confirm visually: object moves into the green rescue zone in
      Scenario A; object does not move at all in Scenario B.
- [ ] Do not edit/cut the resulting video — it should be presented as
      produced, consistent with the "video must be produced by running
      the submitted code" requirement. The title/label cards are generated
      by the same script as the simulation footage, not added afterward
      in external editing software.
- [ ] Optionally narrate over the video afterward (audio only), but do not
      alter the visual frames.
- [ ] Note: `record_demo.py` also writes its own entries to
      `logs/receipts.jsonl` / `logs/episodes.jsonl` (so it can render the
      live verification-result card). Running `run.py` followed by
      `record_demo.py` without clearing `logs/` in between will result in
      4 total log entries, not 2 — this is expected, not an error.

## Known limitations / honest caveats

- `registration.json` contains the official Robothon participant UUID
  (confirmed, not a placeholder).
- Joint damping/actuator gains in `scene.xml` were tuned empirically to
  make a single push reliably reach the goal zone in this exact scene
  geometry — they are not general-purpose values for arbitrary pusher
  tasks.
- `run.py` appends to existing log files rather than overwriting; run
  `rm -rf logs keys` between clean test runs to avoid accumulating entries
  across runs.
- `record_demo.py` also appends its own pair of episode/receipt log
  entries (needed to render the on-screen verification card) — running
  `run.py` then `record_demo.py` back-to-back without clearing `logs/`
  produces 4 entries total, not 2. This does not affect either script's
  own pass/fail result, only the cumulative log file size.
- `record_demo.py` defaults to `MUJOCO_GL=egl` for headless rendering;
  some environments (e.g. certain Cloud Shell configurations) require
  `MUJOCO_GL=osmesa` instead if `egl` is unavailable — set the environment
  variable explicitly before running if you hit a rendering backend error.
