# frozen_string_literal: true

using_task_library "transforms"

describe OroGen.transforms.PoseCombinationRBSTask do
    run_live

    attr_reader :task

    before do
        @task = syskit_deploy(
            OroGen.transforms.PoseCombinationRBSTask
                  .deployed_as("task_under_test")
        )
        # Set up properties with @task.properties....=
        syskit_configure(@task)
    end

    after do
    end

    it "performs the transformation" do
        syskit_start(@task)

        t0 = make_rock_time
        a2ref = Types.base.samples.RigidBodyState.Invalid
        a2ref.time = t0 + 1
        a2ref.position = Eigen::Vector3.new(1, 2, 3)
        a2ref.orientation =
            Eigen::Quaternion.from_angle_axis(-Math::PI/2, Eigen::Vector3.UnitZ)
        a2ref.velocity = Eigen::Vector3.new(5, 1, 2)
        a2ref.angular_velocity = Eigen::Vector3.new(rand, rand, rand)

        b2ref = Types.base.samples.RigidBodyState.Invalid
        b2ref.time = t0
        b2ref.position = Eigen::Vector3.new(4, 3, 2)
        b2ref.orientation =
            Eigen::Quaternion.from_angle_axis(Math::PI/2, Eigen::Vector3.UnitZ)
        b2ref.velocity = Eigen::Vector3.new(rand, rand, rand)
        b2ref.angular_velocity = Eigen::Vector3.new(rand, rand, rand)

        a2b = expect_execution do
            syskit_write task.object2ref_pose_port, a2ref
            syskit_write task.newref2ref_pose_port, b2ref
        end.to { have_one_new_sample task.object2newref_pose_port }

        assert_equal t0, a2b.time
        assert_approx Eigen::Vector3.new(-1, 3, 1), a2b.position
        assert_approx Eigen::Quaternion.from_angle_axis(Math::PI, Eigen::Vector3.UnitZ),
                      a2b.orientation
        assert_approx Eigen::Vector3.new(1, -5, 2), a2b.velocity
        assert_approx a2ref.angular_velocity, a2b.angular_velocity
    end

    def assert_approx(a, b)
        assert a.approx?(b), "#{b} was expected to be approximately equal to #{a}"
    end

    def make_rock_time
        now = Time.now
        Time.at(now.tv_sec, now.tv_usec)
    end
end