/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PoseCombinationRBSTask.hpp"

using namespace transforms;

PoseCombinationRBSTask::PoseCombinationRBSTask(std::string const& name)
    : PoseCombinationRBSTaskBase(name)
{
}

PoseCombinationRBSTask::~PoseCombinationRBSTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PoseCombinationRBSTask.hpp for more detailed
// documentation about them.

bool PoseCombinationRBSTask::configureHook()
{
    if (! PoseCombinationRBSTaskBase::configureHook())
        return false;
    return true;
}
bool PoseCombinationRBSTask::startHook()
{
    if (! PoseCombinationRBSTaskBase::startHook())
        return false;
    return true;
}
void PoseCombinationRBSTask::updateHook()
{
    base::samples::RigidBodyState object2ref_rbs;
    if (_object2ref_pose.read(object2ref_rbs) == RTT::NoData) {
        return;
    }

    base::samples::RigidBodyState newref2ref_rbs;
    if (_newref2ref_pose.read(newref2ref_rbs) != RTT::NewData) {
        return;
    }

    Eigen::Affine3d object2ref = object2ref_rbs;
    Eigen::Affine3d newref2ref = newref2ref_rbs;
    Eigen::Affine3d ref2newref = newref2ref.inverse();
    Eigen::Affine3d object2newref = ref2newref * object2ref;
    base::samples::RigidBodyState object2newref_rbs;
    object2newref_rbs.time = std::min(object2ref_rbs.time, newref2ref_rbs.time);
    object2newref_rbs.position = object2newref.translation();
    object2newref_rbs.orientation = object2newref.rotation();
    object2newref_rbs.velocity = ref2newref.rotation() * object2ref_rbs.velocity;
    object2newref_rbs.angular_velocity = object2ref_rbs.angular_velocity;
    _object2newref_pose.write(object2newref_rbs);

    PoseCombinationRBSTaskBase::updateHook();
}
void PoseCombinationRBSTask::errorHook()
{
    PoseCombinationRBSTaskBase::errorHook();
}
void PoseCombinationRBSTask::stopHook()
{
    PoseCombinationRBSTaskBase::stopHook();
}
void PoseCombinationRBSTask::cleanupHook()
{
    PoseCombinationRBSTaskBase::cleanupHook();
}
