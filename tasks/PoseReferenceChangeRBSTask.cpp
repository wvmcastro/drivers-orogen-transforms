/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PoseAndTwistReferenceChangeRBSTask.hpp"

using namespace transforms;

PoseAndTwistReferenceChangeRBSTask::PoseAndTwistReferenceChangeRBSTask(std::string const& name)
    : PoseAndTwistReferenceChangeRBSTaskBase(name)
{
}

PoseAndTwistReferenceChangeRBSTask::~PoseAndTwistReferenceChangeRBSTask()
{
}

void PoseAndTwistReferenceChangeRBSTask::source2ref_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &source2ref_rbs)
{
    Eigen::Affine3d new_ref2ref_pose;
    if (!_new_ref2ref.get(ts, new_ref2ref_pose, false)) {
        return;
    }

    Eigen::Affine3d source2ref = source2ref_rbs;
    auto source2ref_rot = source2ref.rotation();
    Eigen::Affine3d source2new_ref = new_ref2ref_pose.inverse() * source2ref;

    // TODO: Add velocity and angular_velocity
    // Eigen::Vector3d induced_velocity_ref =
    //     (source2ref_rbs.orientation * source2ref_rbs.angular_velocity)
    //     .cross( new_ref2ref_pose.inverse().translation() * source2ref_rot);

    base::samples::RigidBodyState source2new_ref_rbs;
    source2new_ref_rbs.time = source2ref_rbs.time;
    source2new_ref_rbs.sourceFrame = source2ref_rbs.sourceFrame; 
    source2new_ref_rbs.targetFrame = _new_ref_frame.get();

    source2new_ref_rbs.position = source2new_ref.translation();
    source2new_ref_rbs.orientation = source2new_ref.rotation();
    // source2new_ref_rbs.velocity = source2ref_rbs.velocity + induced_velocity_ref;
    // source2new_ref_rbs.angular_velocity =
    //     source2target_pose.rotation() * source2ref_rbs.angular_velocity;
    _source2new_ref_samples.write(source2new_ref_rbs);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PoseAndTwistReferenceChangeRBSTask.hpp for more detailed
// documentation about them.

bool PoseAndTwistReferenceChangeRBSTask::configureHook()
{
    if (! PoseAndTwistReferenceChangeRBSTaskBase::configureHook())
        return false;
    return true;
}
bool PoseAndTwistReferenceChangeRBSTask::startHook()
{
    if (! PoseAndTwistReferenceChangeRBSTaskBase::startHook())
        return false;
    return true;
}
void PoseAndTwistReferenceChangeRBSTask::updateHook()
{
    PoseAndTwistReferenceChangeRBSTaskBase::updateHook();
}
void PoseAndTwistReferenceChangeRBSTask::errorHook()
{
    PoseAndTwistReferenceChangeRBSTaskBase::errorHook();
}
void PoseAndTwistReferenceChangeRBSTask::stopHook()
{
    PoseAndTwistReferenceChangeRBSTaskBase::stopHook();
}
void PoseAndTwistReferenceChangeRBSTask::cleanupHook()
{
    PoseAndTwistReferenceChangeRBSTaskBase::cleanupHook();
}
