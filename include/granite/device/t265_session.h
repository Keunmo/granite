// SPDX-License-Identifier: MIT
#pragma once
#include <exception>

namespace granite {
enum class T265SessionOutcome { FINISHED, ERROR, TRANSPORT_ERROR };

// A USB failure may make SDK stop throw. Worker cancellation must still run.
template <class Stop, class Report>
bool stopT265Capture(Stop stop, Report report) {
  try {
    stop();
    return true;
  } catch (const std::exception& error) {
    report(error);
    return false;
  }
}


// A session owns its SDK and estimator resources. It must finish cleanup before
// returning; a new session represents a new trajectory/metric initialization.
template <class Run, class Wait, class Notify>
int runT265Sessions(int reconnect_attempts, Run run, Wait wait, Notify notify) {
  for (int segment = 0; ; ++segment) {
    const auto outcome = run(segment, segment != 0);
    if (outcome == T265SessionOutcome::FINISHED) return 0;
    if (outcome == T265SessionOutcome::ERROR || segment >= reconnect_attempts) return 1;
    notify(segment + 1);
    if (!wait()) return 0;
  }
}
}  // namespace granite
