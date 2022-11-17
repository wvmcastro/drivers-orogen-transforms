/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PoseReferenceChangeRBSTask.hpp"

using namespace transforms;

PoseReferenceChangeRBSTask::PoseReferenceChangeRBSTask(std::string const& name)
    : PoseReferenceChangeRBSTaskBase(name)
{
}

PoseReferenceChangeRBSTask::~PoseReferenceChangeRBSTask()
{
}

void PoseReferenceChangeRBSTask::source2ref_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &source2ref_rbs)
{
    Eigen::Affine3d new_ref2ref_pose;
    if (!_new_ref2ref.get(ts, new_ref2ref_pose, false)) {
        return;
    }

    Eigen::Affine3d source2new_ref = new_ref2ref_pose.inverse() * source2ref_rbs;

    base::samples::RigidBodyState source2new_ref_rbs;
    source2new_ref_rbs.time = source2ref_rbs.time;
    source2new_ref_rbs.sourceFrame = source2ref_rbs.sourceFrame; 
    source2new_ref_rbs.targetFrame = _new_ref_frame.get();

    source2new_ref_rbs.position = source2new_ref.translation();
    source2new_ref_rbs.orientation = source2new_ref.rotation();
    _source2new_ref_samples.write(source2new_ref_rbs);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PoseReferenceChangeRBSTask.hpp for more detailed
// documentation about them.

bool PoseReferenceChangeRBSTask::configureHook()
{
    if (! PoseReferenceChangeRBSTaskBase::configureHook())
        return false;
    return true;
}
bool PoseReferenceChangeRBSTask::startHook()
{
    if (! PoseReferenceChangeRBSTaskBase::startHook())
        return false;
    return true;
}
void PoseReferenceChangeRBSTask::updateHook()
{
    PoseReferenceChangeRBSTaskBase::updateHook();
}
void PoseReferenceChangeRBSTask::errorHook()
{
    PoseReferenceChangeRBSTaskBase::errorHook();
}
void PoseReferenceChangeRBSTask::stopHook()
{
    PoseReferenceChangeRBSTaskBase::stopHook();
}
void PoseReferenceChangeRBSTask::cleanupHook()
{
    PoseReferenceChangeRBSTaskBase::cleanupHook();
}
