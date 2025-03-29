# Note: This file patches the ContactSensor class to support `filter_prim_paths_expr` at any level of the hierarchy

import isaaclab.sim as sim_utils
import omni.physics.tensors.impl.api as physx
import torch
from isaaclab.sensors import ContactSensor, ContactSensorCfg  # noqa: F401
from pxr import UsdPhysics

from srb.core.sensor import SensorBase

from pxr import PhysxSchema  # isort: skip


def __initialize_impl(self):
    SensorBase._initialize_impl(self)
    # create simulation view
    self._physics_sim_view = physx.create_simulation_view(self._backend)
    self._physics_sim_view.set_subspace_roots("/")
    # check that only rigid bodies are selected
    leaf_pattern = self.cfg.prim_path.rsplit("/", 1)[-1]
    template_prim_path = self._parent_prims[0].GetPath().pathString
    body_names = []
    for prim in sim_utils.find_matching_prims(template_prim_path + "/" + leaf_pattern):
        # check if prim has contact reporter API
        if prim.HasAPI(PhysxSchema.PhysxContactReportAPI):
            prim_path = prim.GetPath().pathString
            body_names.append(prim_path.rsplit("/", 1)[-1])
    # check that there is at least one body with contact reporter API
    if not body_names:
        raise RuntimeError(
            f"Sensor at path '{self.cfg.prim_path}' could not find any bodies with contact reporter API."
            "\nHINT: Make sure to enable 'activate_contact_sensors' in the corresponding asset spawn configuration."
        )

    # construct regex expression for the body names
    body_names_regex = r"(" + "|".join(body_names) + r")"
    body_names_regex = f"{self.cfg.prim_path.rsplit('/', 1)[0]}/{body_names_regex}"
    # convert regex expressions to glob expressions for PhysX
    body_names_glob = body_names_regex.replace(".*", "*")

    ### PATCH BEGINS HERE ###
    # filter_prim_paths_glob = [
    #     expr.replace(".*", "*") for expr in self.cfg.filter_prim_paths_expr
    # ]
    filter_prim_paths_glob = []
    for expr in self.cfg.filter_prim_paths_expr:
        queue = sim_utils.find_matching_prims(
            expr.replace(".*", "*").replace("/World/envs/env_*", "/World/envs/env_0")
        )
        while queue:
            child_prim = queue.pop(0)
            if child_prim.HasAPI(
                UsdPhysics.CollisionAPI  # type: ignore
            ) or child_prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
                filter_prim_paths_glob.append(child_prim.GetPath().pathString)
            else:
                queue.extend(child_prim.GetChildren())
    if not filter_prim_paths_glob:
        filter_prim_paths_glob = [
            expr.replace(".*", "*") for expr in self.cfg.filter_prim_paths_expr
        ]
    ### PATCH ENDS HERE ###

    # create a rigid prim view for the sensor
    self._body_physx_view = self._physics_sim_view.create_rigid_body_view(
        body_names_glob
    )
    self._contact_physx_view = self._physics_sim_view.create_rigid_contact_view(
        body_names_glob, filter_patterns=filter_prim_paths_glob
    )
    # resolve the true count of bodies
    self._num_bodies = self.body_physx_view.count // self._num_envs
    # check that contact reporter succeeded
    if self._num_bodies != len(body_names):
        raise RuntimeError(
            "Failed to initialize contact reporter for specified bodies."
            f"\n\tInput prim path    : {self.cfg.prim_path}"
            f"\n\tResolved prim paths: {body_names_regex}"
        )

    # prepare data buffers
    self._data.net_forces_w = torch.zeros(
        self._num_envs, self._num_bodies, 3, device=self._device
    )
    # optional buffers
    # -- history of net forces
    if self.cfg.history_length > 0:
        self._data.net_forces_w_history = torch.zeros(
            self._num_envs,
            self.cfg.history_length,
            self._num_bodies,
            3,
            device=self._device,
        )
    else:
        self._data.net_forces_w_history = self._data.net_forces_w.unsqueeze(1)
    # -- pose of sensor origins
    if self.cfg.track_pose:
        self._data.pos_w = torch.zeros(
            self._num_envs, self._num_bodies, 3, device=self._device
        )
        self._data.quat_w = torch.zeros(
            self._num_envs, self._num_bodies, 4, device=self._device
        )
    # -- air/contact time between contacts
    if self.cfg.track_air_time:
        self._data.last_air_time = torch.zeros(
            self._num_envs, self._num_bodies, device=self._device
        )
        self._data.current_air_time = torch.zeros(
            self._num_envs, self._num_bodies, device=self._device
        )
        self._data.last_contact_time = torch.zeros(
            self._num_envs, self._num_bodies, device=self._device
        )
        self._data.current_contact_time = torch.zeros(
            self._num_envs, self._num_bodies, device=self._device
        )
    # force matrix: (num_envs, num_bodies, num_filter_shapes, 3)
    if len(self.cfg.filter_prim_paths_expr) != 0:
        num_filters = self.contact_physx_view.filter_count
        self._data.force_matrix_w = torch.zeros(
            self._num_envs, self._num_bodies, num_filters, 3, device=self._device
        )


ContactSensor._initialize_impl = __initialize_impl
