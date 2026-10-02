# Mono VIO robustness implementation

Spec: docs/superpowers/specs/2026-10-02-mono-vio-robustness-design.md
Baseline: 1a86588. Isolated feature branch / local workspace already exists.
Execution: Native autonomous work, as explicitly requested earlier; independent final code review. No push or merge.

1. Add metric backend fixtures producing rotation-only bearings, finite/infinite track replacement and resumed translation. Watch infinity insertion and recovery expectations fail against baseline.
2. Implement single-camera landmark birth/promotion in a separate backend source. Preserve metric state, inherited robust loss, configured bounds and pyramid weights; reject small parallax and invalid reprojections. Disable no-motion regularization only for mono inertial backend. Run focused tests.
3. Add connected-landmark support diagnostics, visual-information keyframe selection and inertial-only tracking status. Fix outlier connection bookkeeping without changing original VO behavior. Add loss/empty/outlier/velocity/rotation recovery assertions; run focused tests and full suite.
4. Review, resolve actionable findings, verify builds and available device access, document behavior/limits, commit and capture memory.

Review focus: Infinity landmarks must never fake zero translation; pure rotation must not generate spurious finite depth; recovery must triangulate existing tracks; filtering must work with no pose-only states; inactive old finite landmarks must not claim current visual support; no metric gauge/bias reset across feature loss.

Completed: steps1–4, plus default OpenCV T265 GUI, bootstrap visualization and USB-denial preflight. Final154 native tests pass; current RPi4 source transfer blocked by execution environment, so remote build/window/live validation remains open. Review added majority-validated finite PnP and infinity rotation reacquisition, including a positive-depth parallax veto below the promotion baseline.
