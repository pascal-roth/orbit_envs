# Copyright (c) 2024 ETH Zurich (Robotic Systems Lab)
# Author: Pascal Roth
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# python
import os
from dataclasses import dataclass

DATA_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../data"))


@dataclass
class SimCfg:
    """Simulation physics."""

    dt = 0.005  # physics-dt:(s)
    substeps = 8  # rendering-dt = physics-dt * substeps (s)
    gravity = [0.0, 0.0, -9.81]  # (m/s^2)

    enable_scene_query_support = False  # disable scene query for more speed-up
    use_flatcache = True  # output from simulation to flat cache
    use_gpu_pipeline = True  # direct GPU access functionality
    device = "cpu"  # device on which to run simulation/environment

    @dataclass
    class PhysxCfg:
        """PhysX solver parameters."""

        worker_thread_count = 10  # note: unused
        solver_position_iteration_count = 4  # note: unused
        solver_velocity_iteration_count = 1  # note: unused
        enable_sleeping = True  # note: unused
        max_depenetration_velocity = 1.0  # note: unused
        contact_offset = 0.002  # note: unused
        rest_offset = 0.0  # note: unused

        use_gpu = True  # GPU dynamics pipeline and broad-phase type
        solver_type = 1  # 0: PGS, 1: TGS
        enable_stabilization = True  # additional stabilization pass in solver

        # (m/s): contact with relative velocity below this will not bounce
        bounce_threshold_velocity = 0.5
        # (m): threshold for contact point to experience friction force
        friction_offset_threshold = 0.04
        # (m): used to decide if contacts are close enough to merge into a single friction anchor point
        friction_correlation_distance = 0.025

        # GPU buffers parameters
        gpu_max_rigid_contact_count = 512 * 1024
        gpu_max_rigid_patch_count = 80 * 1024 * 2
        gpu_found_lost_pairs_capacity = 1024 * 1024 * 2
        gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 32
        gpu_total_aggregate_pairs_capacity = 1024 * 1024 * 2
        gpu_max_soft_body_contacts = 1024 * 1024
        gpu_max_particle_contacts = 1024 * 1024
        gpu_heap_capacity = 128 * 1024 * 1024
        gpu_temp_buffer_capacity = 32 * 1024 * 1024
        gpu_max_num_partitions = 8

    physx: PhysxCfg = PhysxCfg()


@dataclass
class CarlaLoaderConfig:
    # carla map
    root_path: str = "path_to_unreal_mesh"
    usd_name: str = "Town01_Opt.usd"
    suffix: str = "/Town01_Opt"
    # prim path for the carla map
    prim_path: str = "/World/Carla"
    # SimCfg
    sim_cfg: SimCfg = SimCfg()
    # scale
    scale: float = 0.01  # scale the scene to be in meters
    # up axis
    axis_up: str = "Y"
    # multiply crosswalks
    cw_config_file: str | None = os.path.join(
        DATA_DIR, "town01", "cw_multiply_cfg.yml"
    )  # if None, no crosswalks are added
    # mesh to semantic class mapping --> only if set, semantic classes will be added to the scene
    sem_mesh_to_class_map: str | None = os.path.join(
        DATA_DIR, "town01", "keyword_mapping.yml"
    )  # os.path.join(DATA_DIR, "park", "keyword_mapping.yml")  os.path.join(DATA_DIR, "town01", "keyword_mapping.yml")
    # add Groundplane to the scene
    groundplane: bool = True
    # add people to the scene
    people_config_file: str | None = os.path.join(DATA_DIR, "town01", "people_cfg.yml")  # if None, no people are added
    # multiply vehicles
    vehicle_config_file: str | None = os.path.join(
        DATA_DIR, "town01", "vehicle_cfg.yml"
    )  # if None, no vehicles are added

    @property
    def usd_path(self) -> str:
        return os.path.join(self.root_path, self.usd_name)
