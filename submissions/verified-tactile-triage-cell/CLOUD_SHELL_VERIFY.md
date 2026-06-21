# CLOUD_SHELL_VERIFY.md

## 1. Upload and unzip
```bash
unzip phase8-handoff.zip -d triage_cell
cd triage_cell
```

## 2. Virtualenv + dependencies
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## 3. System OSMesa library (if missing)
```bash
sudo apt-get update && sudo apt-get install -y libosmesa6 libosmesa6-dev
```

## 4. Exact verification command
```bash
MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa python run.py
```

## 5. Expected output (tail)
```
  [PASS] Phase 1: scene_load_and_stability
  [PASS] Phase 2: actuator_validation
  [PASS] Phase 3: grasp_5_of_5
  [PASS] Phase 4: lift_transport_5_of_5
  [PASS] Phase 5: slip_detection_and_recovery
  [PASS] Phase 6: triage_decision_and_placement
  [PASS] Phase 7: signed_receipts

OVERALL: PASS
```
Exit code must be `0`.

## 6. Confirm report exists
```bash
test -f reliability_report.json && echo "report present" && cat reliability_report.json | python3 -m json.tool | head -20
```

## 7. Re-package after verification
```bash
cd ..
zip -r phase8-verified-cloudshell.zip triage_cell -x "triage_cell/venv/*" "triage_cell/__pycache__/*" "triage_cell/keys/triage_receipt_private.pem"
```
