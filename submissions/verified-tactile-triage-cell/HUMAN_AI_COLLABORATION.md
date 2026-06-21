# Human–AI Collaboration

## Role separation

### Human operator

The human selected the project direction, approved scope and checkpoints,
made every go/no-go decision, controlled all external accounts and
submission actions, and retained final authority over every claim.

### Claude Code

Claude Code implemented and debugged the MuJoCo scene, tactile control,
transport, disturbance recovery, safety gate, signed receipts,
reliability harness, and demo capture. It executed tests and produced
measured outputs that were then independently reviewed.

### ChatGPT

ChatGPT acted as an independent strategy and claim-audit layer. Its role
was to challenge unsupported claims, verify arithmetic and terminology,
check consistency between reports and demo evidence, and stop packaging
when evidence or security controls were incomplete.

Concrete audit interventions included:

- correcting the slip ratio from an incorrect `3.4×` claim to `1.20×`;
- separating force-asymmetry detection from later displacement-threshold
  detection, avoiding a false zero-latency displacement claim;
- identifying unreachable placement coordinates before acceptance;
- tracing negative `min_force_transport` values to a command/force naming
  collision;
- rejecting a stale or incorrect demo artifact;
- catching a fixed-position camera bug that hid the gripper during
  recovery and requiring a live wrist-following camera;
- detecting that the final archive still contained a private signing key
  and requiring its removal before packaging.

### Google Cloud Shell

Google Cloud Shell provided an independent execution environment for
the final reproducibility check:

- Phases 1–7: PASS
- consolidated regression exit code: 0
- OSMesa demo render: PASS
- final demo: H.264, 640×480, 30 fps, 2370 frames, 79.0 seconds

## Examples of rejected or corrected output

- An unstable or unsupported result was not accepted merely because it
  looked plausible.
- A solver configuration with lower slip but harmful force asymmetry was
  rejected in favor of the better-balanced elliptic/Newton configuration.
- Demo claims were required to match live report fields rather than
  hand-entered numbers.
- The final evidence card was required to state the actual rendering
  backend and was visually checked for clipping.
- The signing private key was moved outside the project before the final
  archive was created.

## What this is not

This is not an autonomous-agent submission and it does not claim that AI
made final decisions independently. AI tools assisted implementation,
testing, and review; the human retained authorship, judgment, approval,
and submission control.

Every published metric is tied to executed code, a generated report,
a recorded log, or a visually inspected artifact.
