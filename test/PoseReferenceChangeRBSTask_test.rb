# frozen_string_literal: true

using_task_library "transforms"

describe OroGen.transforms.PoseReferenceChangeRBSTask do
    run_live

    attr_reader :task

    before do
        @task = syskit_deploy(
            OroGen.transforms.PoseReferenceChangeRBSTask
                  .deployed_as("task_under_test")
        )

        @task.properties.ref_frame = "source"
        @task.properties.new_ref_frame = "new_ref"

        now = Time.now
        @rock_now = Time.at(now.tv_sec, now.tv_usec)
        @source2ref = Types.base.samples.RigidBodyState.Invalid
        @source2ref.time = @rock_now
        @source2ref.sourceFrame = "source"
        @source2ref.targetFrame = "ref"

        @new_ref2ref = Types.base.samples.RigidBodyState.Invalid
        @new_ref2ref.sourceFrame = "new_ref"
        @new_ref2ref.targetFrame = "ref"
    end

    it "handles a simple translation" do
        configure_new_ref2ref translation: Eigen::Vector3.UnitY
        syskit_configure_and_start @task

        @source2ref.position = Eigen::Vector3.new(rand, rand, rand)
        @source2ref.orientation = Eigen::Quaternion.Identity

        source2new_ref =
            expect_execution { syskit_write @task.source2ref_samples_port, @source2ref }
            .to { have_one_new_sample task.source2new_ref_samples_port }

        assert_eigen_approx @source2ref.position - Eigen::Vector3.UnitY, \
                            source2new_ref.position
        assert_eigen_approx @source2ref.orientation, source2new_ref.orientation
    end

    it "handles the new ref having a different orientation than the original reference" do
        source2new_ref_q = Eigen::Quaternion.from_angle_axis(
            - Math::PI / 2, Eigen::Vector3.UnitZ
        )
        configure_new_ref2ref translation: Eigen::Vector3.UnitY, 
                              rotation: Eigen::Quaternion.from_angle_axis(
                                        Math::PI / 2, Eigen::Vector3.UnitZ)
        syskit_configure_and_start @task

        @source2ref.position = Eigen::Vector3.new(rand, rand, rand)
        @source2ref.orientation = Eigen::Quaternion.new(1, 0, 0, 0)

        source2new_ref =
            expect_execution { syskit_write @task.source2ref_samples_port, @source2ref }
            .to { have_one_new_sample task.source2new_ref_samples_port }

        source2newref_p = \
            Eigen::Vector3.new(@source2ref.position[0] * Math.cos(-Math::PI / 2) \
                               - @source2ref.position[1] * Math.sin(-Math::PI / 2), \
                               @source2ref.position[0] * Math.sin(-Math::PI / 2) \
                               + @source2ref.position[1] * Math.cos(-Math::PI / 2), \
                               @source2ref.position[2])
        new_ref2ref_p = Eigen::Vector3.UnitX
        assert_eigen_approx (source2newref_p - new_ref2ref_p),
                            source2new_ref.position
        assert_eigen_approx source2new_ref_q, source2new_ref.orientation
    end

    def configure_new_ref2ref(
        translation: Eigen::Vector3.Zero,
        rotation: Eigen::Quaternion.Identity
    )
        @new_ref2ref.position = translation
        @new_ref2ref.orientation = rotation
        @task.properties.static_transformations = [@new_ref2ref]
    end

    def assert_eigen_approx(expected, actual, delta = 1e-6)
        assert expected.approx?(actual, delta),
               "expected #{actual} to be approximately #{expected}"
    end
end