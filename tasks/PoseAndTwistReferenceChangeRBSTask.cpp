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

void PoseAndTwistReferenceChangeRBSTask::source2ref_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &source2ref_samples_sample)
{
    throw std::runtime_error("Transformer callback for source2ref_samples not implemented");
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
