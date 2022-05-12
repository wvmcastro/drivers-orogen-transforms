/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PoseAndTwistFrameChangeRBSTask.hpp"

using namespace base;
using namespace transforms;

PoseAndTwistFrameChangeRBSTask::PoseAndTwistFrameChangeRBSTask(std::string const& name)
    : PoseAndTwistFrameChangeRBSTaskBase(name)
{
}

PoseAndTwistFrameChangeRBSTask::~PoseAndTwistFrameChangeRBSTask()
{
}

void PoseAndTwistFrameChangeRBSTask::source2ref_samplesTransformerCallback(
    const base::Time &ts, const ::base::samples::RigidBodyState &source2ref_rbs
)
{
    Eigen::Affine3d source2target_pose;
    if (!_source2target.get(ts, source2target_pose, false)) {
        return;
    }

    Eigen::Affine3d source2ref = source2ref_rbs;
    auto source2ref_rot = source2ref.rotation();
    Eigen::Affine3d target2ref = source2ref * source2target_pose.inverse();

    Eigen::Vector3d induced_velocity_ref =
        (source2ref_rbs.orientation * source2ref_rbs.angular_velocity)
        .cross(source2ref_rot * source2target_pose.inverse().translation());

    base::samples::RigidBodyState target2ref_rbs;
    target2ref_rbs.time = source2ref_rbs.time;
    target2ref_rbs.sourceFrame = _target_frame.get();
    target2ref_rbs.targetFrame = source2ref_rbs.targetFrame;

    target2ref_rbs.position = target2ref.translation();
    target2ref_rbs.orientation = target2ref.rotation();
    target2ref_rbs.velocity = source2ref_rbs.velocity + induced_velocity_ref;
    target2ref_rbs.angular_velocity =
        source2target_pose.rotation() * source2ref_rbs.angular_velocity;
    _target2ref_samples.write(target2ref_rbs);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PoseAndTwistFrameChangeRBSTask.hpp for more detailed
// documentation about them.

bool PoseAndTwistFrameChangeRBSTask::configureHook()
{
    if (! PoseAndTwistFrameChangeRBSTaskBase::configureHook())
        return false;
    return true;
}
bool PoseAndTwistFrameChangeRBSTask::startHook()
{
    if (! PoseAndTwistFrameChangeRBSTaskBase::startHook())
        return false;
    return true;
}
void PoseAndTwistFrameChangeRBSTask::updateHook()
{
    PoseAndTwistFrameChangeRBSTaskBase::updateHook();
}
void PoseAndTwistFrameChangeRBSTask::errorHook()
{
    PoseAndTwistFrameChangeRBSTaskBase::errorHook();
}
void PoseAndTwistFrameChangeRBSTask::stopHook()
{
    PoseAndTwistFrameChangeRBSTaskBase::stopHook();
}
void PoseAndTwistFrameChangeRBSTask::cleanupHook()
{
    PoseAndTwistFrameChangeRBSTaskBase::cleanupHook();
}
