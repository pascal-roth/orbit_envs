# Copyright (c) 2024 ETH Zurich (Robotic Systems Lab)
# Author: Pascal Roth
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# python
import os
from typing import List, Tuple

# omniverse
import carb
import omni
import omni.isaac.core.utils.prims as prim_utils
import omni.isaac.debug_draw._debug_draw as omni_debug_draw
import scipy.spatial.transform as tf
import yaml

# isaac-carla
from omni.isaac.carla.configs import CarlaLoaderConfig

# isaac-core
from omni.isaac.core.materials import PhysicsMaterial
from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.core.prims import GeometryPrim
from omni.isaac.core.simulation_context import SimulationContext
from omni.isaac.core.utils.semantics import add_update_semantics, remove_all_semantics
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.orbit.utils.assets import ISAAC_NUCLEUS_DIR

# isaac-orbit
from omni.isaac.orbit.utils.configclass import class_to_dict
from pxr import Gf, PhysxSchema, Usd


class CarlaLoader:
    debug: bool = False

    def __init__(self, cfg: CarlaLoaderConfig) -> None:
        self._cfg = cfg

        # Load kit helper
        self.sim = SimulationContext(
            stage_units_in_meters=1.0,
            physics_dt=self._cfg.sim_cfg.dt,
            rendering_dt=self._cfg.sim_cfg.dt * self._cfg.sim_cfg.substeps,
            backend="torch",
            sim_params=class_to_dict(self._cfg.sim_cfg.physx),
            device=self._cfg.sim_cfg.device,
        )

        # Set main camera
        set_camera_view([20 / self._cfg.scale, 20 / self._cfg.scale, 20 / self._cfg.scale], [0.0, 0.0, 0.0])

        # Acquire draw interface
        self.draw_interface = omni_debug_draw.acquire_debug_draw_interface()

        self.material: PhysicsMaterial = None
        return

    def load(self) -> None:
        """Load the scene."""
        # design scene
        assert os.path.isfile(self._cfg.usd_path), f"USD File not found: {self._cfg.usd_path}"
        self._design_scene()
        self.sim.reset()

        # modify mesh
        if self._cfg.cw_config_file:
            self._multiply_crosswalks()

        if self._cfg.people_config_file:
            self._insert_people()

        if self._cfg.vehicle_config_file:
            self._insert_vehicles()

        # assign semantic labels
        if self._cfg.sem_mesh_to_class_map:
            self._add_semantics()

        return

    """ Scene Helper Functions """

    def _design_scene(self):
        """Add prims to the scene."""

        self._xform_prim = prim_utils.create_prim(
            prim_path=self._cfg.prim_path,
            translation=(0.0, 0.0, 0.0),
            usd_path=self._cfg.usd_path,
            scale=(self._cfg.scale, self._cfg.scale, self._cfg.scale),
        )

        # physics material
        self.material = PhysicsMaterial(
            "/World/PhysicsMaterial", static_friction=0.7, dynamic_friction=0.7, restitution=0
        )
        # enable patch-friction: yields better results!
        physx_material_api = PhysxSchema.PhysxMaterialAPI.Apply(self.material._prim)
        physx_material_api.CreateImprovePatchFrictionAttr().Set(True)
        physx_material_api.CreateFrictionCombineModeAttr().Set("max")
        physx_material_api.CreateRestitutionCombineModeAttr().Set("max")

        # assign each submesh it's own geometry prim --> important for raytracing to be able to identify the submesh
        submeshes = prim_utils.get_prim_children(self._xform_prim)[1].GetAllChildren()

        for submesh in submeshes:
            submesh_path = submesh.GetPath().pathString
            # create geometry prim
            GeometryPrim(
                prim_path=submesh_path,
                name="collision",
                position=None,
                orientation=None,
                collision=True,
            ).apply_physics_material(self.material)
            # physx_utils.setCollider(submesh, approximationShape="None")
            # "None" will use the base triangle mesh if available

        # Lights-1
        prim_utils.create_prim(
            "/World/Light/GreySphere",
            "SphereLight",
            translation=(45 / self._cfg.scale, 100 / self._cfg.scale, 100 / self._cfg.scale),
            attributes={"radius": 10, "intensity": 30000.0, "color": (0.75, 0.75, 0.75)},
        )
        # Lights-2
        prim_utils.create_prim(
            "/World/Light/WhiteSphere",
            "SphereLight",
            translation=(100 / self._cfg.scale, 100 / self._cfg.scale, 100 / self._cfg.scale),
            attributes={"radius": 10, "intensity": 30000.0, "color": (1.0, 1.0, 1.0)},
        )

        if self._cfg.axis_up == "Y" or self._cfg.axis_up == "y":
            world_prim = prim_utils.get_prim_at_path(self._cfg.prim_path)
            rot_quat = tf.Rotation.from_euler("XYZ", [90, 90, 0], degrees=True).as_quat()
            gf_quat = Gf.Quatf()
            gf_quat.real = rot_quat[3]
            gf_quat.imaginary = Gf.Vec3f(list(rot_quat[:3]))
            world_prim.GetAttribute("xformOp:orient").Set(gf_quat)

        if self._cfg.groundplane:
            _ = GroundPlane("/World/GroundPlane", z_position=0.0, physics_material=self.material, visible=False)

        return

    """ Assign Semantic Labels """

    def _add_semantics(self):
        # remove all previous semantic labels
        remove_all_semantics(prim_utils.get_prim_at_path(self._cfg.prim_path + self._cfg.suffix), recursive=True)

        # get mesh prims
        mesh_prims, mesh_prims_name = self.get_mesh_prims(self._cfg.prim_path + self._cfg.suffix)

        carb.log_info(f"Total of {len(mesh_prims)} meshes in the scene, start assigning semantic class ...")

        # mapping from prim name to class
        with open(self._cfg.sem_mesh_to_class_map) as file:
            class_keywords = yaml.safe_load(file)

        # make all the string lower case
        mesh_prims_name = [mesh_prim_single.lower() for mesh_prim_single in mesh_prims_name]
        keywords_class_mapping_lower = {
            key: [value_single.lower() for value_single in value] for key, value in class_keywords.items()
        }

        # assign class to mesh in ISAAC
        def recursive_semUpdate(prim, sem_class_name: str, update_submesh: bool) -> bool:
            # Necessary for Park Mesh
            # FIXME: include all meshes leads to OgnSdStageInstanceMapping does not support more than 65535 semantic entities (2718824 requested) error since entities are restricted to int16
            if (
                prim.GetName() == "HierarchicalInstancedStaticMesh"
            ):  # or "FoliageInstancedStaticMeshComponent" in prim.GetName():
                add_update_semantics(prim, sem_class_name)
                update_submesh = True
            children = prim.GetChildren()
            if len(children) > 0:
                for child in children:
                    update_submesh = recursive_semUpdate(child, sem_class_name, update_submesh)
            return update_submesh

        def recursive_meshInvestigator(mesh_idx, mesh_name, mesh_prim_list) -> bool:
            success = False
            for class_name, keywords in keywords_class_mapping_lower.items():
                if any([keyword in mesh_name for keyword in keywords]):
                    update_submesh = recursive_semUpdate(mesh_prim_list[mesh_idx], class_name, False)
                    if not update_submesh:
                        add_update_semantics(mesh_prim_list[mesh_idx], class_name)
                    success = True
                    break

            if not success:
                success_child = []
                mesh_prims_children, mesh_prims_name_children = self.get_mesh_prims(
                    mesh_prim_list[mesh_idx].GetPrimPath().pathString
                )
                mesh_prims_name_children = [mesh_prim_single.lower() for mesh_prim_single in mesh_prims_name_children]
                for mesh_idx_child, mesh_name_child in enumerate(mesh_prims_name_children):
                    success_child.append(
                        recursive_meshInvestigator(mesh_idx_child, mesh_name_child, mesh_prims_children)
                    )
                success = any(success_child)

            return success

        mesh_list = []
        for mesh_idx, mesh_name in enumerate(mesh_prims_name):
            success = recursive_meshInvestigator(mesh_idx=mesh_idx, mesh_name=mesh_name, mesh_prim_list=mesh_prims)
            if success:
                mesh_list.append(mesh_idx)

        missing = [i for x, y in zip(mesh_list, mesh_list[1:]) for i in range(x + 1, y) if y - x > 1]
        assert len(mesh_list) > 0, "No mesh is assigned a semantic class!"
        assert len(mesh_list) == len(
            mesh_prims_name
        ), f"Not all meshes are assigned a semantic class! Following mesh names are included yet: {[mesh_prims_name[miss_idx] for miss_idx in missing]}"
        carb.log_info("Semantic mapping done.")

        return

    """ Modify Mesh """

    def _multiply_crosswalks(self) -> None:
        """Increase number of crosswalks in the scene."""

        with open(self._cfg.cw_config_file) as stream:
            multipy_cfg: dict = yaml.safe_load(stream)

        # get the stage
        stage = omni.usd.get_context().get_stage()

        # get town prim
        town_prim = multipy_cfg.pop("town_prim")

        # init counter
        crosswalk_add_counter = 0

        for key, value in multipy_cfg.items():
            print(f"Execute crosswalk multiplication '{key}'")

            # iterate over the number of crosswalks to be created
            for copy_idx in range(value["factor"]):
                success = omni.usd.duplicate_prim(
                    stage=stage,
                    prim_path=os.path.join(self._cfg.prim_path, town_prim, value["cw_prim"]),
                    path_to=os.path.join(
                        self._cfg.prim_path, town_prim, value["cw_prim"] + f"_cp{copy_idx}" + value.get("suffix", "")
                    ),
                    duplicate_layers=True,
                )
                assert success, f"Failed to duplicate crosswalk '{key}'"

                # get crosswalk prim
                prim_utils.get_prim_at_path(
                    os.path.join(
                        self._cfg.prim_path, town_prim, value["cw_prim"] + f"_cp{copy_idx}" + value.get("suffix", "")
                    )
                ).GetAttribute("xformOp:translate").Set(
                    Gf.Vec3d(value["translation"][0], value["translation"][1], value["translation"][2]) * (copy_idx + 1)
                )

                # update counter
                crosswalk_add_counter += 1

        carb.log_info(f"Number of crosswalks added: {crosswalk_add_counter}")
        print(f"Number of crosswalks added: {crosswalk_add_counter}")

        return

    def _insert_vehicles(self):
        # load vehicle config file
        with open(self._cfg.vehicle_config_file) as file:
            vehicle_cfg: dict = yaml.safe_load(file)

        # get the stage
        stage = omni.usd.get_context().get_stage()

        # get town prim and all its meshes
        town_prim = vehicle_cfg.pop("town_prim")
        mesh_prims: dict = prim_utils.get_prim_at_path(f"{self._cfg.prim_path}/{town_prim}").GetChildren()
        mesh_prims_name = [mesh_prim_single.GetName() for mesh_prim_single in mesh_prims]

        # car counter
        car_add_counter = 0

        for key, vehicle in vehicle_cfg.items():
            print(f"Execute vehicle multiplication '{key}'")

            # get all meshs that include the keystring
            meshs = [
                mesh_prim_single for mesh_prim_single in mesh_prims_name if vehicle["prim_part"] in mesh_prim_single
            ]

            # iterate over the number of vehicles to be created
            for idx, translation in enumerate(vehicle["translation"]):
                for single_mesh in meshs:
                    success = omni.usd.duplicate_prim(
                        stage=stage,
                        prim_path=os.path.join(self._cfg.prim_path, town_prim, single_mesh),
                        path_to=os.path.join(self._cfg.prim_path, town_prim, single_mesh + key + f"_cp{idx}"),
                        duplicate_layers=True,
                    )
                    assert success, f"Failed to duplicate vehicle '{key}'"

                    # get vehicle prim
                    prim_utils.get_prim_at_path(
                        os.path.join(self._cfg.prim_path, town_prim, single_mesh + key + f"_cp{idx}")
                    ).GetAttribute("xformOp:translate").Set(Gf.Vec3d(translation[0], translation[1], translation[2]))

                car_add_counter += 1

        carb.log_info(f"Number of vehicles added: {car_add_counter}")
        print(f"Number of vehicles added: {car_add_counter}")

        return

    def _insert_people(self):
        # load people config file
        with open(self._cfg.people_config_file) as file:
            people_cfg: dict = yaml.safe_load(file)

        for key, person_cfg in people_cfg.items():
            carb.log_verbose(f"Insert person '{key}'")

            self.insert_single_person(
                person_cfg["prim_name"],
                person_cfg["translation"],
                scale_people=1,  # scale_people,
                usd_path=person_cfg.get("usd_path", "People/Characters/F_Business_02/F_Business_02.usd"),
            )
            # TODO: movement of the people

        carb.log_info(f"Number of people added: {len(people_cfg)}")
        print(f"Number of people added: {len(people_cfg)}")

        return

    @staticmethod
    def insert_single_person(
        prim_name: str,
        translation: list,
        scale_people: float = 1.0,
        usd_path: str = "People/Characters/F_Business_02/F_Business_02.usd",
    ) -> None:
        person_prim = prim_utils.create_prim(
            prim_path=os.path.join("/World/People", prim_name),
            translation=tuple(translation),
            usd_path=os.path.join(ISAAC_NUCLEUS_DIR, usd_path),
            scale=(scale_people, scale_people, scale_people),
        )

        if isinstance(person_prim.GetAttribute("xformOp:orient").Get(), Gf.Quatd):
            person_prim.GetAttribute("xformOp:orient").Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
        else:
            person_prim.GetAttribute("xformOp:orient").Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

        add_update_semantics(person_prim, "person")

        return

    @staticmethod
    def get_mesh_prims(env_prim: str) -> Tuple[List[Usd.Prim], List[str]]:
        def recursive_search(start_prim: str, mesh_prims: list):
            for curr_prim in prim_utils.get_prim_at_path(start_prim).GetChildren():
                if curr_prim.GetTypeName() == "Xform" or curr_prim.GetTypeName() == "Mesh":
                    mesh_prims.append(curr_prim)
                elif curr_prim.GetTypeName() == "Scope":
                    mesh_prims = recursive_search(start_prim=curr_prim.GetPath().pathString, mesh_prims=mesh_prims)

            return mesh_prims

        assert prim_utils.is_prim_path_valid(env_prim), f"Prim path '{env_prim}' is not valid"

        mesh_prims = []
        mesh_prims = recursive_search(env_prim, mesh_prims)

        # mesh_prims: dict = prim_utils.get_prim_at_path(self._cfg.prim_path + "/" + self._cfg.usd_name.split(".")[0]).GetChildren()
        mesh_prims_name = [mesh_prim_single.GetName() for mesh_prim_single in mesh_prims]

        return mesh_prims, mesh_prims_name


# EoF
