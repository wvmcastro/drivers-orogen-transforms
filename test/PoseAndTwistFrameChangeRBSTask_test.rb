# frozen_string_literal: true

using_task_library "transforms"

describe OroGen.transforms.PoseAndTwistFrameChangeRBSTask do
    run_live

    attr_reader :task

    before do
        @task = syskit_deploy(
            OroGen.transforms.PoseAndTwistFrameChangeRBSTask
                  .deployed_as("task_under_test")
        )

        @task.properties.source_frame = "a"
        @task.properties.target_frame = "b"

        now = Time.now
        @rock_now = Time.at(now.tv_sec, now.tv_usec)
        @source2ref = Types.base.samples.RigidBodyState.Invalid
        @source2ref.time = @rock_now
        @source2ref.sourceFrame = "a"
        @source2ref.targetFrame = "ref"

        @target2source = Types.base.samples.RigidBodyState.Invalid
        @target2source.sourceFrame = "b"
        @target2source.targetFrame = "a"
    end

    it "handles a simple translation" do
        configure_target2source translation: Eigen::Vector3.UnitY
        syskit_configure_and_start @task

        @source2ref.position = Eigen::Vector3.new(rand, rand, rand)
        @source2ref.orientation = Eigen::Quaternion.Identity
        @source2ref.velocity = Eigen::Vector3.new(rand, rand, rand)
        @source2ref.angular_velocity = Eigen::Vector3.UnitX

        target2ref =
            expect_execution { syskit_write @task.source2ref_samples_port, @source2ref }
            .to { have_one_new_sample task.target2ref_samples_port }

        assert_eigen_approx @source2ref.position + Eigen::Vector3.UnitY, target2ref.position
        assert_eigen_approx @source2ref.orientation, target2ref.orientation
        assert_eigen_approx @source2ref.velocity + Eigen::Vector3.UnitZ,
                            target2ref.velocity
        assert_eigen_approx @source2ref.angular_velocity, target2ref.angular_velocity
    end

    it "interprets the angular velocity vector in the source frame" do
        source2ref_q = Eigen::Quaternion.from_angle_axis(
            Math::PI / 2, Eigen::Vector3.UnitY
        )
        configure_target2source translation: Eigen::Vector3.UnitY
        syskit_configure_and_start @task

        @source2ref.position = Eigen::Vector3.new(rand, rand, rand)
        @source2ref.orientation = source2ref_q
        @source2ref.velocity = Eigen::Vector3.new(rand, rand, rand)
        @source2ref.angular_velocity = Eigen::Vector3.UnitZ

        target2ref =
            expect_execution { syskit_write @task.source2ref_samples_port, @source2ref }
            .to { have_one_new_sample task.target2ref_samples_port }

        assert_eigen_approx @source2ref.position + Eigen::Vector3.UnitY, target2ref.position
        assert_eigen_approx @source2ref.orientation, target2ref.orientation
        assert_eigen_approx @source2ref.velocity + Eigen::Vector3.UnitZ,
                            target2ref.velocity
        assert_eigen_approx @source2ref.angular_velocity, target2ref.angular_velocity
    end

    it "handles the source having a different orientation than the reference" do
        source2ref_q = Eigen::Quaternion.from_angle_axis(
            Math::PI / 2, Eigen::Vector3.UnitX
        )
        configure_target2source translation: Eigen::Vector3.UnitY
        syskit_configure_and_start @task

        @source2ref.position = Eigen::Vector3.new(rand, rand, rand)
        @source2ref.orientation = source2ref_q
        @source2ref.velocity = Eigen::Vector3.new(rand, rand, rand)
        @source2ref.angular_velocity = Eigen::Vector3.UnitX

        target2ref =
            expect_execution { syskit_write @task.source2ref_samples_port, @source2ref }
            .to { have_one_new_sample task.target2ref_samples_port }

        assert_eigen_approx @source2ref.position + Eigen::Vector3.UnitZ,
                            target2ref.position
        assert_eigen_approx source2ref_q, target2ref.orientation
        assert_eigen_approx @source2ref.velocity - Eigen::Vector3.UnitY, target2ref.velocity
        assert_eigen_approx @source2ref.angular_velocity, target2ref.angular_velocity
    end

    it "handles the target and reference having different orientations" do
        target_q = Eigen::Quaternion.from_angle_axis(Math::PI / 2, Eigen::Vector3.UnitZ)
        configure_target2source translation: Eigen::Vector3.UnitY, rotation: target_q
        syskit_configure_and_start @task

        @source2ref.position = Eigen::Vector3.new(rand, rand, rand)
        @source2ref.orientation = Eigen::Quaternion.Identity
        @source2ref.velocity = Eigen::Vector3.new(rand, rand, rand)
        @source2ref.angular_velocity = Eigen::Vector3.UnitX

        target2ref =
            expect_execution { syskit_write @task.source2ref_samples_port, @source2ref }
            .to { have_one_new_sample task.target2ref_samples_port }

        assert_eigen_approx @source2ref.position + Eigen::Vector3.UnitY, target2ref.position
        assert_eigen_approx target_q, target2ref.orientation
        assert_eigen_approx @source2ref.velocity + Eigen::Vector3.UnitZ, target2ref.velocity
        assert_eigen_approx -Eigen::Vector3.UnitY, target2ref.angular_velocity
    end

    # The target orientation does not affect the result, but it does affect the
    # computation
    it "handles both source and target having a different orientation than the reference" do
        source2ref_q = Eigen::Quaternion.from_angle_axis(
            Math::PI / 2, Eigen::Vector3.UnitX
        )
        target2source_q = Eigen::Quaternion.from_angle_axis(
            Math::PI / 2, Eigen::Vector3.UnitZ
        )
        configure_target2source translation: Eigen::Vector3.UnitY,
                                rotation: target2source_q
        syskit_configure_and_start @task

        @source2ref.position = Eigen::Vector3.new(rand, rand, rand)
        @source2ref.orientation = source2ref_q
        @source2ref.velocity = Eigen::Vector3.new(rand, rand, rand)
        @source2ref.angular_velocity = Eigen::Vector3.UnitX

        target2ref =
            expect_execution { syskit_write @task.source2ref_samples_port, @source2ref }
            .to { have_one_new_sample task.target2ref_samples_port }

        assert_eigen_approx @source2ref.position + Eigen::Vector3.UnitZ,
                            target2ref.position
        assert_eigen_approx source2ref_q * target2source_q, target2ref.orientation
        assert_eigen_approx @source2ref.velocity - Eigen::Vector3.UnitY,
                            target2ref.velocity
        assert_eigen_approx -Eigen::Vector3.UnitY, target2ref.angular_velocity
    end


    def configure_target2source(
        translation: Eigen::Vector3.Zero,
        rotation: Eigen::Quaternion.Identity
    )
        @target2source.position = translation
        @target2source.orientation = rotation
        @task.properties.static_transformations = [@target2source]
    end

    def assert_eigen_approx(expected, actual, delta = 1e-6)
        assert expected.approx?(actual, delta),
               "expected #{actual} to be approximately #{expected}"
    end
end