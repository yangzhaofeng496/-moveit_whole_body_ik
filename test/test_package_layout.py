#!/usr/bin/env python3
"""Static contracts for the extracted MoveIt whole-body IK package."""

import pathlib
import json
import unittest

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]
DESCRIPTION_ROOT = ROOT.parent / "robot_description"


class CorePackageLayoutTest(unittest.TestCase):
    def setUp(self):
        self.package_xml = (ROOT / "package.xml").read_text()
        self.pcd_source = (ROOT / "src/pcd_octomap_planning_scene_node.cpp").read_text()

    def test_core_package_has_no_remani_planner_dependency(self):
        self.assertNotIn("remani_planner", self.package_xml)

    def test_pcd_node_keeps_shared_display_and_collision_crop(self):
        self.assertIn("workspace_max_z_", self.pcd_source)
        start = self.pcd_source.index("const double z = source_z + translation_z_;")
        end = self.pcd_source.index("transformed.emplace_back", start)
        transform = self.pcd_source[start:end]
        self.assertLess(transform.index("if (crop_to_workspace_"),
                        transform.index("addDisplayPoint(x, y, z);"))

    def test_launch_uses_only_bundled_packages_and_map(self):
        launch = (ROOT / "launch/whole_body_ik.launch").read_text()
        self.assertIn("$(find moveit_whole_body_ik)/launch/moveit/move_group.launch", launch)
        self.assertNotIn("ir100_moveit", launch)
        self.assertIn(
            "$(find moveit_whole_body_ik)/../../maps/scans_voxel_5cm_xyz.pcd",
            launch,
        )
        self.assertNotIn("remani_planner", launch)

    def test_robot_description_package_owns_model_assets(self):
        for relative_path in (
            "urdf/ir100_robot.xacro",
            "urdf/ir100.xacro",
            "urdf/cr10_robot_urdf.xacro",
            "meshes/ir100/base_link.STL",
            "meshes/cr10/Link6.STL",
        ):
            self.assertTrue((DESCRIPTION_ROOT / relative_path).is_file(), relative_path)

        for relative_path in (
            "config/ir100.srdf",
            "config/ir100_gazebo.srdf",
            "launch/moveit/planning_context.launch",
        ):
            self.assertTrue((ROOT / relative_path).is_file(), relative_path)

    def test_robot_description_package_uses_its_own_model_references(self):
        model = (DESCRIPTION_ROOT / "urdf/ir100.xacro").read_text()
        arm = (DESCRIPTION_ROOT / "urdf/cr10_robot_urdf.xacro").read_text()
        entrypoint = (DESCRIPTION_ROOT / "urdf/ir100_robot.xacro").read_text()

        self.assertIn("$(find robot_description)", model + entrypoint)
        self.assertIn("package://robot_description/meshes/", model + arm)
        self.assertNotIn("$(find moveit_whole_body_ik)", model + entrypoint)
        self.assertNotIn("package://moveit_whole_body_ik/meshes/", model + arm)

    def test_robot_description_entrypoint_includes_ir100_model(self):
        entrypoint = (DESCRIPTION_ROOT / "urdf/ir100_robot.xacro").read_text()
        self.assertIn('filename="$(find robot_description)/urdf/ir100.xacro"', entrypoint)
        self.assertIn("<xacro:ir100_robot />", entrypoint)
        self.assertNotIn("$(find moveit_whole_body_ik)", entrypoint)

    def test_vscode_urdf_visualizer_maps_robot_description_package(self):
        settings_path = ROOT.parents[1] / ".vscode/settings.json"
        if not settings_path.is_file():
            self.skipTest("local VS Code settings are intentionally git-ignored")
        settings = json.loads(settings_path.read_text())
        packages = settings["urdf-visualizer.packages"]
        self.assertEqual(packages["robot_description"], "${workspaceFolder}/src/robot_description")

    def test_packaged_model_and_moveit_launch_have_no_old_package_references(self):
        model = (DESCRIPTION_ROOT / "urdf/ir100.xacro").read_text()
        arm = (DESCRIPTION_ROOT / "urdf/cr10_robot_urdf.xacro").read_text()
        context = (ROOT / "launch/moveit/planning_context.launch").read_text()
        self.assertNotIn("ir100_description", model)
        self.assertNotIn("dobot_description", model + arm)
        self.assertNotIn("ir100_moveit", context)
        self.assertNotIn("/tmp/ir100.urdf", context)
        self.assertIn('command="xacro ', context)
        for launch_path in (ROOT / "launch/moveit").glob("*.launch*"):
            text = launch_path.read_text()
            self.assertNotIn("ir100_moveit", text, launch_path.name)
            self.assertNotIn("/tmp/ir100.urdf", text, launch_path.name)

    def test_rviz_uses_the_new_panel_plugin_class(self):
        rviz = (ROOT / "launch/moveit_whole_body_ik.rviz").read_text()
        self.assertIn("moveit_whole_body_ik/CsvWholeBodyIkPanel", rviz)

    def test_mid360_simulation_defines_live_point_cloud_sectors(self):
        model = (DESCRIPTION_ROOT / "urdf/ir100.xacro").read_text()
        gazebo_launch = (ROOT / "launch/moveit/gazebo.launch").read_text()
        mid360_launch = (ROOT / "launch/moveit/demo_gazebo_mid360.launch").read_text()
        self.assertIn('<xacro:arg name="mid360_sim_enabled" default="false"/>', model)
        self.assertIn('name="mid360_sim_enabled" default="false"', gazebo_launch)
        self.assertIn('mid360_sim_enabled" value="$(arg enable_mid360_point_cloud)"', mid360_launch)
        self.assertIn('name="mid360_sim_body_link"', model)
        self.assertEqual(model.count("libgazebo_ros_depth_camera.so"), 5)
        # Keep the five-sector simulation within Gazebo/MoveIt real-time
        # capacity.  A dense 160x60 image at 10 Hz per sector caused old
        # point clouds to arrive after the TF cache had advanced.
        self.assertEqual(model.count("<update_rate>5</update_rate>"), 4)
        self.assertEqual(model.count("<width>80</width><height>30</height>"), 5)
        self.assertEqual(model.count("<updateRate>5.0</updateRate>"), 4)
        self.assertEqual(model.count("<pointCloudCutoffMax>20.0</pointCloudCutoffMax>"), 5)

        for sector in ("front", "left", "rear", "right", "up"):
            self.assertIn(f'name="mid360_sim_{sector}_link"', model)
            self.assertIn(f'name="mid360_sim_{sector}_optical_link"', model)
            self.assertIn(f"/mid360_sim/{sector}/points", model)
            self.assertIn(
                f"<frameName>mid360_sim_{sector}_optical_link</frameName>", model
            )

    def test_mid360_simulation_point_clouds_feed_moveit_octomap(self):
        config_path = ROOT / "config/sensors_mid360_sim.yaml"
        config = yaml.safe_load(config_path.read_text())
        sensors = config["sensors"]
        self.assertEqual(len(sensors), 5)
        self.assertEqual(
            {sensor["point_cloud_topic"] for sensor in sensors},
            {
                "/mid360_sim/front/points",
                "/mid360_sim/left/points",
                "/mid360_sim/rear/points",
                "/mid360_sim/right/points",
                "/mid360_sim/up/points",
            },
        )
        self.assertEqual(
            {sensor["sensor_plugin"] for sensor in sensors},
            {"occupancy_map_monitor/PointCloudOctomapUpdater"},
        )
        self.assertEqual({sensor["max_range"] for sensor in sensors}, {20.0})
        self.assertEqual({sensor["point_subsample"] for sensor in sensors}, {2})
        self.assertEqual({sensor["max_update_rate"] for sensor in sensors}, {2.0, 5.0})
        self.assertEqual(len({sensor["filtered_cloud_topic"] for sensor in sensors}), 5)

    def test_mid360_gazebo_demo_uses_only_live_sensor_octomap_input(self):
        demo = (ROOT / "launch/moveit/demo_gazebo_mid360.launch").read_text()
        sensor_manager = (
            ROOT / "launch/moveit/sensor_manager_mid360_sim.launch.xml"
        ).read_text()
        move_group = (ROOT / "launch/moveit/move_group.launch").read_text()
        generic_demo = (ROOT / "launch/moveit/demo.launch").read_text()

        self.assertIn('file="$(dirname)/gazebo.launch"', demo)
        self.assertIn('file="$(dirname)/demo.launch"', demo)
        self.assertIn("sensor_manager_mid360_sim.launch.xml", demo)
        self.assertNotIn("pcd_octomap_planning_scene_node", demo)
        self.assertIn("sensors_mid360_sim.yaml", sensor_manager)
        self.assertIn('<param name="max_range" type="double" value="20.0" />', sensor_manager)
        self.assertIn('name="sensor_manager_launch"', generic_demo)
        self.assertIn('name="sensor_manager_launch"', move_group)

    def test_mid360_demo_uses_moveit_as_the_gazebo_joint_state_authority(self):
        sync_source = (ROOT / "src/moveit_to_gazebo_sync_node.cpp").read_text()
        package_xml = (ROOT / "package.xml").read_text()
        cmake = (ROOT / "CMakeLists.txt").read_text()
        gazebo_launch = (ROOT / "launch/moveit/gazebo.launch").read_text()
        demo = (ROOT / "launch/moveit/demo_gazebo_mid360.launch").read_text()
        model = (DESCRIPTION_ROOT / "urdf/ir100.xacro").read_text()

        self.assertIn("/move_group/fake_controller_joint_states", sync_source)
        self.assertIn("gazebo_msgs::SetModelConfiguration", sync_source)
        self.assertIn("service_name_, false", sync_source)
        for joint in ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6"):
            self.assertIn(f'"{joint}"', sync_source)
        self.assertIn("gazebo_msgs", package_xml)
        self.assertIn("moveit_to_gazebo_sync_node", cmake)
        self.assertIn('name="gazebo_static" default="false"', gazebo_launch)
        self.assertIn('gazebo_static" value="false"', demo)
        self.assertIn('world_pose" default="-x 0 -y 0 -z 0.2', demo)
        self.assertIn('moveit_controller_manager" value="simple"', demo)
        self.assertIn('type="moveit_to_gazebo_sync_node"', demo)
        self.assertIn('<xacro:arg name="gazebo_static" default="false"/>', model)
        self.assertIn('<static>$(arg gazebo_static)</static>', model)

    def test_sphere_collision_model_is_selectable(self):
        model = (DESCRIPTION_ROOT / "urdf/ir100.xacro").read_text()
        sphere_model = (DESCRIPTION_ROOT / "urdf/cr10_robot_sphere_collision.xacro").read_text()
        gazebo_launch = (ROOT / "launch/moveit/gazebo.launch").read_text()

        self.assertIn('use_mesh_collision" default="false"', model)
        self.assertIn('cr10_robot_sphere_collision.xacro', model)
        self.assertIn('<sphere radius=', sphere_model)
        self.assertIn('collision_sphere_radius_scale" value="0.90"', sphere_model)
        self.assertNotIn('<collision><geometry><mesh', sphere_model)
        self.assertIn('use_mesh_collision:=', gazebo_launch)

    def test_mobile_base_uses_the_remani_box_collision(self):
        model = (DESCRIPTION_ROOT / "urdf/ir100.xacro").read_text()

        self.assertIn('<box size="0.85 0.65 0.77" />', model)
        self.assertIn('<origin xyz="0 0 0.385" rpy="0 0 0" />', model)

    def test_srdf_only_disables_directly_adjacent_collision_pairs(self):
        srdf = (ROOT / "config/ir100.srdf").read_text()

        self.assertNotIn('reason="Never"', srdf)
        self.assertNotIn('link1="arm_base_link" link2="base_link"', srdf)
        for pair in (("arm_base_link", "Link1"), ("Link1", "Link2"),
                     ("Link2", "Link3"), ("Link3", "Link4"),
                     ("Link4", "Link5"), ("Link5", "Link6"),
                     ("Link6", "arm_gripper_link")):
            self.assertTrue(
                f'link1="{pair[0]}" link2="{pair[1]}"' in srdf or
                f'link1="{pair[1]}" link2="{pair[0]}"' in srdf
            )

    def test_gazebo_trajectory_bridge_uses_follow_joint_trajectory(self):
        bridge = (ROOT / "src/gazebo_follow_joint_trajectory_node.cpp").read_text()
        controllers = (ROOT / "config/simple_moveit_controllers.yaml").read_text()
        demo = (ROOT / "launch/moveit/demo_gazebo_mid360.launch").read_text()
        model = (DESCRIPTION_ROOT / "urdf/ir100.xacro").read_text()

        self.assertIn("control_msgs/FollowJointTrajectoryAction.h", bridge)
        self.assertIn("/arm_controller/follow_joint_trajectory", bridge)
        self.assertIn("SetModelConfiguration", bridge)
        self.assertIn("arm_controller", controllers)
        self.assertIn("FollowJointTrajectory", controllers)
        self.assertIn('moveit_controller_manager" value="simple"', demo)
        self.assertIn('gazebo_follow_joint_trajectory_node', demo)
        self.assertIn('arm_gripper_collision_radius', model)

    def test_gazebo_demo_can_select_real_robot_controller(self):
        demo = (ROOT / "launch/moveit/demo_gazebo_mid360.launch").read_text()
        generic_demo = (ROOT / "launch/moveit/demo.launch").read_text()
        move_group = (ROOT / "launch/moveit/move_group.launch").read_text()
        trajectory_execution = (
            ROOT / "launch/moveit/trajectory_execution.launch.xml"
        ).read_text()
        real_manager = (
            ROOT / "launch/moveit/real_moveit_controller_manager.launch.xml"
        )
        real_controllers = ROOT / "config/real_moveit_controllers.yaml"

        self.assertIn('name="use_real_robot" default="false"', demo)
        self.assertIn('name="use_real_robot"', generic_demo)
        self.assertIn('name="use_real_robot"', move_group)
        self.assertIn('name="use_real_robot"', trajectory_execution)
        self.assertIn('if="$(arg use_real_robot)"', trajectory_execution)
        self.assertIn('real_moveit_controller_manager.launch.xml', trajectory_execution)
        self.assertTrue(real_manager.is_file())
        self.assertTrue(real_controllers.is_file())

        controller_config = yaml.safe_load(real_controllers.read_text())
        controller = controller_config["controller_list"][0]
        self.assertEqual(controller["name"], "cr10_robot/joint_controller")
        self.assertEqual(controller["action_ns"], "follow_joint_trajectory")
        self.assertEqual(controller["type"], "FollowJointTrajectory")
        self.assertIn('unless="$(arg use_real_robot)"', demo)
        self.assertIn('unless="$(arg use_real_robot)"', demo)
        self.assertIn('gazebo_follow_joint_trajectory_node', demo)

    def test_real_mode_publishes_world_to_base_tf(self):
        demo = (ROOT / "launch/moveit/demo_gazebo_mid360.launch").read_text()
        self.assertIn(
            '<arg name="publish_virtual_joint_tf" value="$(arg use_real_robot)" />',
            demo,
        )

    def test_mid360_demo_unpauses_gazebo_only_after_move_group_is_ready(self):
        demo = (ROOT / "launch/moveit/demo_gazebo_mid360.launch").read_text()
        source = (ROOT / "src/move_group_ready_unpause_gazebo_node.cpp").read_text()
        cmake = (ROOT / "CMakeLists.txt").read_text()

        self.assertIn('name="paused" value="$(arg paused)"', demo)
        self.assertIn('if="$(arg enable_mid360_point_cloud)"', demo)
        self.assertIn('type="move_group_ready_unpause_gazebo_node"', demo)
        self.assertIn('get_model_state', source)
        self.assertIn('model_name', source)
        self.assertIn('/gazebo/unpause_physics', source)
        self.assertIn('move_group_ready_unpause_gazebo_node', cmake)

    def test_mid360_demo_disables_sensor_simulation_and_octomap_by_default(self):
        demo = (ROOT / "launch/moveit/demo_gazebo_mid360.launch").read_text()

        self.assertIn('name="enable_mid360_point_cloud" default="false"', demo)
        self.assertIn('mid360_sim_enabled" value="$(arg enable_mid360_point_cloud)"', demo)
        self.assertEqual(
            demo.count('sensor_manager_launch" value="$(dirname)/sensor_manager_mid360_sim.launch.xml"'),
            1,
        )
        self.assertEqual(
            demo.count('sensor_manager_launch" value="$(dirname)/sensor_manager_livox.launch.xml"'),
            1,
        )
        self.assertNotIn('sensor_manager_disabled.launch.xml', demo)

    def test_gazebo_demo_uses_a_fixed_semantic_root(self):
        """The arm-only Gazebo demo must not load the planar whole-body joint."""
        demo = (ROOT / "launch/moveit/demo_gazebo_mid360.launch").read_text()
        context = (ROOT / "launch/moveit/planning_context.launch").read_text()
        move_group = (ROOT / "launch/moveit/move_group.launch").read_text()
        semantic = (ROOT / "config/ir100_gazebo.srdf")

        self.assertTrue(semantic.is_file())
        self.assertIn('name="semantic_description"', context)
        self.assertIn('semantic_description" value="$(arg semantic_description)"', move_group)
        self.assertIn('name="semantic_description"', demo)
        self.assertIn('default="$(find moveit_whole_body_ik)/config/ir100_gazebo.srdf"', demo)
        self.assertIn(
            'publish_virtual_joint_tf" value="$(arg use_real_robot)"', demo
        )
        semantic_text = semantic.read_text()
        self.assertIn('type="fixed"', semantic_text)
        self.assertNotIn('type="planar"', semantic_text)
        self.assertNotIn('name="whole_body"', semantic_text)

    def test_gazebo_demo_uses_the_requested_initial_arm_pose(self):
        demo = (ROOT / "launch/moveit/demo_gazebo_mid360.launch").read_text()
        compact_demo = " ".join(demo.split())
        bridge = (ROOT / "src/gazebo_follow_joint_trajectory_node.cpp").read_text()
        initial = (
            "-J joint1 -0.1298212446016354 "
            "-J joint2 -1.1983068889034356 "
            "-J joint3 2.4643989495602585 "
            "-J joint4 0.33790999598645527 "
            "-J joint5 -1.4614805964938609 "
            "-J joint6 -0.1545975306168726"
        )
        self.assertIn(f'<arg name="initial_joint_positions" default="{initial}"', compact_demo)
        self.assertIn(
            '<rosparam param="initial_positions"> '
            '[-0.1298212446016354, -1.1983068889034356, 2.4643989495602585, '
            '0.33790999598645527, -1.4614805964938609, -0.1545975306168726] </rosparam>',
            compact_demo,
        )
        self.assertIn('getParam("initial_positions"', bridge)

    def test_base_link_has_no_root_inertia_for_kdl(self):
        model = (DESCRIPTION_ROOT / "urdf/ir100.xacro").read_text()
        start = model.index('<link name="base_link"')
        end = model.index('</link>', start)
        self.assertNotIn('<inertial>', model[start:end])

    def test_gazebo_world_uses_moveit_world_frame(self):
        world = (ROOT / "worlds/mid360_obstacle.world").read_text()
        self.assertIn('<world name="world">', world)
        self.assertNotIn('mid360_obstacle_world', world)

    def test_gazebo_world_keeps_sim_time_near_wall_time(self):
        """Headless Gazebo must not outrun the TF/joint-state publishers."""
        world = (ROOT / "worlds/mid360_obstacle.world").read_text()
        self.assertIn('<real_time_update_rate>1000</real_time_update_rate>', world)
        self.assertIn('<real_time_factor>1</real_time_factor>', world)

    def test_move_group_waits_for_historical_sensor_transforms(self):
        move_group = (ROOT / "launch/moveit/move_group.launch").read_text()
        demo = (ROOT / "launch/moveit/demo_gazebo_mid360.launch").read_text()
        self.assertIn(
            '<param name="shape_transform_cache_lookup_wait_time" value="5.0" />',
            move_group,
        )
        self.assertNotIn("robot_description_planning/shape_transform_cache_lookup_wait_time", demo)


if __name__ == "__main__":
    unittest.main()
