#include <gtest/gtest.h>
#include "srm_gimbal_follower/manual_step.hpp"
using namespace srm_gimbal_follower;
StepParameters config(){StepParameters p;p.allow_hardware_commands=true;p.absolute_target_protocol_verified=true;p.mechanical_limits_verified=true;p.yaw_min=-4;p.yaw_max=4;p.pitch_min=-3;p.pitch_max=3;return p;}
void prepare(ManualStep & c){for(int i=0;i<=20;++i){c.status(true,true,i*.02);c.feedback(0,0,i*.02);}}
TEST(Step, DefaultAndUnverifiedNeverArm){ManualStep c;prepare(c);EXPECT_FALSE(c.arm(.4));auto p=config();p.mechanical_limits_verified=false;ManualStep d(p);prepare(d);EXPECT_FALSE(d.arm(.4));p=config();p.absolute_target_protocol_verified=false;ManualStep e(p);prepare(e);EXPECT_FALSE(e.arm(.4));}
TEST(Step, ArmAloneDoesNotSend){ManualStep c(config());prepare(c);ASSERT_TRUE(c.arm(.4));EXPECT_FALSE(c.tick(.42));}
TEST(Step, SmoothBoundedAndNoFireInterface){ManualStep c(config());prepare(c);ASSERT_TRUE(c.arm(.4));ASSERT_TRUE(c.step(.2,-.2,.4));double previous=0;
 for(int i=1;i<6;++i){double t=.4+i*.02;c.status(true,true,t);c.feedback(previous,-previous,t);auto v=c.tick(t);ASSERT_TRUE(v);EXPECT_LE(std::hypot((*v)[0]-previous,(*v)[1]+previous),2*.02+1e-9);previous=(*v)[0];}}
TEST(Step, RangeIntersectionAndImmutableAnchor){auto p=config();p.yaw_max=.1;ManualStep c(p);prepare(c);ASSERT_TRUE(c.arm(.4));EXPECT_FALSE(c.step(.2,0,.4));EXPECT_DOUBLE_EQ((*c.anchor)[0],0);}
TEST(Step, RejectLargeStepAndNonfinite){ManualStep c(config());prepare(c);c.arm(.4);EXPECT_FALSE(c.step(1.1,0,.4));c.arm(.4);EXPECT_FALSE(c.step(NAN,0,.4));}
TEST(Step, FeedbackLossDisarmsAndNoAutomaticResume){ManualStep c(config());prepare(c);c.arm(.4);c.step(.2,0,.4);EXPECT_FALSE(c.tick(.6));EXPECT_FALSE(c.armed);c.status(true,true,.62);c.feedback(0,0,.62);EXPECT_FALSE(c.tick(.62));}
TEST(Step, StopDisconnectAndLease){ManualStep c(config());prepare(c);c.arm(.4);c.step(.2,0,.4);c.stop();EXPECT_FALSE(c.tick(.42));c.arm(.4);c.status(false,false,.41);EXPECT_FALSE(c.armed);}
TEST(Step, FeedbackJumpAndRateConfiguration){ManualStep c(config());prepare(c);c.arm(.4);EXPECT_FALSE(c.feedback(1,0,.42));EXPECT_FALSE(c.armed);auto p=config();p.rate=5.1;EXPECT_THROW(ManualStep d(p),std::invalid_argument);}
TEST(Step, HardStartupWindowNeverReanchors){ManualStep c(config());prepare(c);
 for(int i=0;i<=20;++i){double t=1+i*.02;c.status(true,true,t);c.feedback(2.99,1.99,t);}
 ASSERT_TRUE(c.arm(1.4));EXPECT_DOUBLE_EQ((*c.anchor)[0],0);EXPECT_FALSE(c.step(.02,0,1.4));
 ASSERT_TRUE(c.arm(1.4));EXPECT_FALSE(c.step(0,.02,1.4));}
TEST(Step, FiveDegreeRateIsUpperLimit){auto p=config();p.rate=5;ManualStep c(p);prepare(c);c.arm(.4);c.step(.3,0,.4);c.status(true,true,.42);c.feedback(0,0,.42);
 auto v=c.tick(.42);ASSERT_TRUE(v);EXPECT_LE((*v)[0],.1+1e-9);}
