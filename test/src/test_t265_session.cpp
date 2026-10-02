// SPDX-License-Identifier: MIT
#include <granite/device/t265_session.h>
#include <gtest/gtest.h>
#include <vector>
#include <stdexcept>

using granite::T265SessionOutcome;

TEST(T265Session, DisconnectStartsFreshSessionAfterCleanupAndWait) {
  std::vector<int> events;
  int result=granite::runT265Sessions(3,[&](int segment,bool recovering) {
    events.push_back(10+segment);
    EXPECT_EQ(recovering,segment!=0);
    // Returning from this call represents the resource cleanup boundary.
    events.push_back(20+segment);
    return segment==0 ? T265SessionOutcome::TRANSPORT_ERROR : T265SessionOutcome::FINISHED;
  },[&]{events.push_back(30);return true;},[&](int segment){events.push_back(40+segment);});
  EXPECT_EQ(result,0);
  EXPECT_EQ(events,(std::vector<int>{10,20,41,30,11,21}));
}

TEST(T265Session, RepeatedFailureStopsAtConfiguredLimit) {
  int sessions=0,waits=0;
  EXPECT_EQ(granite::runT265Sessions(2,[&](int,bool){++sessions;return T265SessionOutcome::TRANSPORT_ERROR;},
      [&]{++waits;return true;},[](int){}),1);
  EXPECT_EQ(sessions,3);
  EXPECT_EQ(waits,2);
}

TEST(T265Session, EstimatorErrorDoesNotRetry) {
  int sessions=0,waits=0;
  EXPECT_EQ(granite::runT265Sessions(3,[&](int,bool){++sessions;return T265SessionOutcome::ERROR;},
      [&]{++waits;return true;},[](int){}),1);
  EXPECT_EQ(sessions,1);EXPECT_EQ(waits,0);
}

TEST(T265Session, InterruptDuringReconnectDoesNotStartAnotherSession) {
  int sessions=0;
  EXPECT_EQ(granite::runT265Sessions(3,[&](int,bool){++sessions;return T265SessionOutcome::TRANSPORT_ERROR;},
      []{return false;},[](int){}),0);
  EXPECT_EQ(sessions,1);
}

TEST(T265Session, ReconnectDisabledKeepsFailureExitStatus) {
  int sessions=0,waits=0;
  EXPECT_EQ(granite::runT265Sessions(0,[&](int,bool){++sessions;return T265SessionOutcome::TRANSPORT_ERROR;},
      [&]{++waits;return true;},[](int){}),1);
  EXPECT_EQ(sessions,1);EXPECT_EQ(waits,0);
}

TEST(T265Session, ThrowingSdkStopDoesNotSkipWorkerCancellation) {
  std::vector<int> events;
  EXPECT_NO_THROW({
    const bool stopped=granite::stopT265Capture([&] {
      events.push_back(1);
      throw std::runtime_error("USB disconnected during startup");
    },[&](const std::exception&){events.push_back(2);});
    EXPECT_FALSE(stopped);
    events.push_back(3);  // The caller can now cancel and join its workers.
  });
  EXPECT_EQ(events,(std::vector<int>{1,2,3}));
}
