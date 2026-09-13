"""Lazy Isaac Lab runtime for the shared provisional G1/FTP debug task."""
from .g1_inspire import build_articulation_cfg, evaluate_snapshot, transition_record


def create_env(spec, usd_path, *, num_envs=1, seed=17):
    """Create one or a small batch of actual Lab environments after AppLauncher.

    This verifies environment/control/export plumbing. It is not a trained
    standing, grasping, or writing task. The same source robot and public
    SceneConfig are cloned; only collisions between separate environments are
    filtered. Robot self-collisions and source mimic constraints stay active.
    """
    if type(num_envs) is not int or not 1 <= num_envs <= 8:
        raise ValueError("Debug batch must contain 1..8 environments")
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation, ArticulationCfg
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
    from isaaclab.scene import InteractiveSceneCfg
    from isaaclab.sensors import ContactSensor, ContactSensorCfg
    from isaac.twin.inspire.whiteboard_scene import SceneConfig, build_scene

    cfg = DirectRLEnvCfg(seed=seed, decimation=spec.decimation,
        episode_length_s=spec.episode_steps * spec.physics_dt * spec.decimation,
        action_space=41, observation_space=spec.observation_width, state_space=0,
        ui_window_class_type=None, wait_for_textures=False,
        sim=sim_utils.SimulationCfg(dt=spec.physics_dt, render_interval=spec.decimation,
            device="cpu", physx=sim_utils.PhysxCfg(solver_type=1, enable_ccd=True,
                enable_stabilization=False, enable_external_forces_every_iteration=True,
                bounce_threshold_velocity=0.)),
        scene=InteractiveSceneCfg(num_envs=num_envs, env_spacing=4.,
            replicate_physics=False, lazy_sensor_update=False))

    class InspireDebugEnv(DirectRLEnv):
        def __init__(self):
            self.spec = spec
            self.last_transition_records = []
            self._last_evaluation = None
            super().__init__(cfg=cfg)
            mapping = spec.bind(self.robot.joint_names)
            self.measurement_ids = list(mapping["measurement_indices"])
            self.action_ids = list(mapping["action_indices"])
            self.actions = torch.tensor(spec.reset_action(), device=self.device).repeat(self.num_envs, 1)
            self.marker_spring_target = torch.full((self.num_envs, 1),
                SceneConfig.from_dict(spec.scene_config).holder.spring_target_m, device=self.device)
            self.episode_ids = torch.full((self.num_envs,), -1, dtype=torch.long, device=self.device)
            self.sequence = 0

        def _setup_scene(self):
            robot_cfg = build_articulation_cfg(spec, usd_path)
            robot_cfg.prim_path = "/World/envs/env_.*/Robot"
            self.robot = Articulation(robot_cfg)
            self.scene.articulations["robot"] = self.robot
            scene_config = SceneConfig.from_dict(spec.scene_config)
            self.shared_scene_manifest = build_scene(self.sim.stage, scene_config,
                board_path="/World/envs/env_0/Whiteboard", marker_path="/World/envs/env_0/Marker")
            x, y = scene_config.initial_xy_board_m
            position = scene_config.frame.to_world((x, y, scene_config.holder.tip_offset_m + scene_config.initial_gap_m))
            self.marker = Articulation(ArticulationCfg(prim_path="/World/envs/env_.*/Marker", spawn=None,
                init_state=ArticulationCfg.InitialStateCfg(pos=position,
                    rot=scene_config.frame.orientation_world_qwxyz,
                    joint_pos={"NibCompression": 0.}, joint_vel={".*": 0.}),
                soft_joint_pos_limit_factor=1., actuators={"source_nib_spring": ImplicitActuatorCfg(
                    joint_names_expr=["NibCompression"], stiffness=None, damping=None,
                    effort_limit_sim=None, armature=None, friction=None)}))
            self.scene.articulations["marker"] = self.marker
            self.contact = ContactSensor(ContactSensorCfg(prim_path="/World/envs/env_.*/Robot/.*",
                update_period=0., history_length=1, debug_vis=False))
            self.scene.sensors["robot_contacts"] = self.contact
            self.scene.clone_environments(copy_from_source=True)
            self.scene.filter_collisions(global_prim_paths=[])
            sim_utils.DomeLightCfg(intensity=350.).func("/World/DebugLight", sim_utils.DomeLightCfg(intensity=350.))

        def _pre_physics_step(self, actions):
            if tuple(actions.shape) != (self.num_envs, 41):
                raise ValueError("Lab action must be an N x41 named position matrix")
            for row in actions.detach().cpu().tolist():
                spec.validate_action(row)
            self.actions = actions.clone()
            self._last_evaluation = None

        def _apply_action(self):
            # Only 41 independent targets. Passive mimic coordinates receive no
            # independent target or drive effort; constraints move them.
            self.robot.set_joint_position_target(self.actions, joint_ids=self.action_ids)
            # The shared source's passive preload target is negative. Lab's
            # default zero target would silently remove that preload.
            self.marker.set_joint_position_target(self.marker_spring_target)

        def _get_observations(self):
            root = self.robot.data.root_state_w.clone()
            root[:, :3] -= self.scene.env_origins
            return {"policy": torch.cat((self.robot.data.joint_pos[:, self.measurement_ids],
                self.robot.data.joint_vel[:, self.measurement_ids], root), dim=-1)}

        def _capture_transition(self):
            q = self.robot.data.joint_pos[:, self.measurement_ids].detach().cpu().tolist()
            dq = self.robot.data.joint_vel[:, self.measurement_ids].detach().cpu().tolist()
            roots = self.robot.data.root_state_w.detach().cpu().tolist()
            commands = self.actions.detach().cpu().tolist()
            try:
                forces = self.contact.data.net_forces_w.detach().cpu().tolist()
                contact_names = list(self.contact.body_names)
                contact_error = None
            except Exception as exc:
                forces, contact_names, contact_error = None, [], repr(exc)
            evaluations, records = [], []
            for env in range(self.num_envs):
                evaluation = evaluate_snapshot(spec, q[env], dq[env], roots[env], commands[env],
                                               int(self.episode_length_buf[env]))
                evaluations.append(evaluation)
                record = transition_record(spec, env_id=env, episode_id=int(self.episode_ids[env]),
                    sequence=self.sequence, physics_time_s=float(self.sim.current_time), q=q[env], dq=dq[env],
                    root_state=roots[env], action=commands[env], evaluation=evaluation,
                    contact_forces=forces[env] if forces is not None else None, contact_names=contact_names)
                record["contacts"]["unavailable_reason"] = contact_error
                record["environment_origin_world_m"] = self.scene.env_origins[env].detach().cpu().tolist()
                record["episode_step"] = int(self.episode_length_buf[env])
                records.append(record)
            self.last_transition_records = records
            self._last_evaluation = evaluations
            self.sequence += 1

        def _get_dones(self):
            self._capture_transition()
            return (torch.tensor([e["terminated"] for e in self._last_evaluation], device=self.device),
                    torch.tensor([e["truncated"] for e in self._last_evaluation], device=self.device))

        def _get_rewards(self):
            if self._last_evaluation is None:
                self._capture_transition()
            return torch.tensor([e["reward"] for e in self._last_evaluation], device=self.device)

        def _reset_idx(self, env_ids):
            if env_ids is None:
                env_ids = self.robot._ALL_INDICES
            super()._reset_idx(env_ids)
            for articulation in (self.robot, self.marker):
                root = articulation.data.default_root_state[env_ids].clone()
                root[:, :3] += self.scene.env_origins[env_ids]
                articulation.write_root_pose_to_sim(root[:, :7], env_ids)
                articulation.write_root_velocity_to_sim(root[:, 7:], env_ids)
                articulation.write_joint_state_to_sim(articulation.data.default_joint_pos[env_ids].clone(),
                    articulation.data.default_joint_vel[env_ids].clone(), env_ids=env_ids)
            self.actions[env_ids] = torch.tensor(spec.reset_action(), device=self.device)
            self.robot.set_joint_position_target(self.actions[env_ids], joint_ids=self.action_ids, env_ids=env_ids)
            self.marker.set_joint_position_target(self.marker_spring_target[env_ids], env_ids=env_ids)
            self.episode_ids[env_ids] += 1

    return InspireDebugEnv()
