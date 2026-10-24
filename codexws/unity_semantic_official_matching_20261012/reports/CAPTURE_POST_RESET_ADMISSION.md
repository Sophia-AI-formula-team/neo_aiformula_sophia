# Exposure validity and current epoch are distinct

Readback issue identified before changing the readback audit. Installed source,
tests, Unity state and packets are not changed by this diagnostic correction.

Both v2 cases retained one inference completing just after the existing follower
shutdown reset: outer sequence 503 and inner sequence 202 carry capture epoch 1,
while the sampled current detector state is epoch 2. Both exact v2 capsules have
internally bound old capture-time/image/sequence/hash/calibration. This does not
make either frame admissible to a new epoch-2 observer or controller.

The first readback audit correctly found this difference but combined current
state admission with capsule validity. Its failed receipt is preserved. A second
readback reports these packets as explicitly rejected for current-context use,
while independently checking unchanged capture binding. It must not rewrite the
capsules, alter their epochs, refresh their age, duplicate their support, or call
them current. New observer/association consumers must enforce epoch/freshness
at admission; a top-level diagnostic `valid` flag alone is insufficient.

The existing original lane/semantic guards are preserved; no new observer or
control consumer is enabled. Both actual runs stopped workers normally with no
cleanup error. This is a diagnostic correction, not a driving pass.
