# Monocular VIO through visual degeneracy

User intent: Carry Granite mono VO's practical strengths into mono VIO, especially pure rotation, loss of finite-depth landmarks and recovery at the existing metric scale. Earlier instructions authorize autonomous implementation, testing, debugging and commits.

Keep the original mono VO algorithm unchanged. Extend the single-camera inertial backend, which retains velocity, gravity, bias and marginalization throughout visual degeneracy. Do not switch back to an arbitrary-scale VO map after metric initialization.

Use Granite's existing inverse-depth landmark representation and robust reprojection optimizer for both finite and infinity landmarks. Insert new bearings without requiring a translation baseline. Infinity bearings constrain orientation only: disable the VO-specific zero-translation regularizer in the single-camera inertial backend. Promote them to finite depth when rotation-compensated parallax, metric baseline, positive depth and reprojection validation permit. Use the best supported baseline rather than the first candidate, and preserve pyramid-level weights and configured inverse-distance bounds. Revisit existing infinity landmarks after translation resumes.

Keep current observations and host bookkeeping consistent when robust filtering removes landmarks. Report currently connected finite/infinity counts and whether metric translation has visual support. No visual support means inertial propagation; bearing-only support is a degraded condition, not a metric reset or a claim of fully constrained translation.

Add a visual-information keyframe signal alongside coverage loss, respecting frame spacing and bounded windows. During bearing-only tracking consider rotation information, excluding unobservable visual translation. Preserve existing metric priors when selecting/marginalizing keyframes.

Verify known-metric backend behavior with real synthetic observations/IMU: new bearings during rotation, nonzero velocity with bearings only (no fake stationarity), complete feature replacement, same-track promotion after translation resumes, finite-point loss/recovery, empty observations, reprojection outliers and maintained metric pose/velocity/bias. Retain automatic bootstrap tests and the complete existing suite. Physical moving-scene validation requires device access and motion; do not replace it with a synthetic accuracy claim.

Review refinement: Gate observations at point birth as well as promotion. Preserve metric hosts through empty frames and use bounded, majority-validated metric PnP reacquisition before rejecting matches under moderate inertial drift. Evaluate visual information on the first optimizer iteration so early convergence cannot bypass it; reset its reference at each keyframe.

Review refinement: Recover infinity-only orientation with robust bearing alignment after actual image gaps, preserving translation/velocity/bias. Require strong majority reprojection support and bounded innovation. Never run this recovery merely because translation parallax lowered bearing support; retain infinity depth while its model still explains the current image.
