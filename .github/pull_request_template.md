## Change

Describe the behavior or evidence changed.

## Verification

- [ ] `scripts/verify_release.py`
- [ ] `python -m unittest discover -s raspberry_pi/tests -q`
- [ ] `git diff --check`
- [ ] Hardware-facing changes have an explicit site validation record.

## Evidence boundary

State whether the result is source review, offline replay, simulation, or a
current hardware record. Do not mix those labels in the same metric.
