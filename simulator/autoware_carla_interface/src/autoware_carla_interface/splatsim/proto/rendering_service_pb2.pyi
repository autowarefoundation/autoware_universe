from typing import ClassVar as _ClassVar
from typing import Iterable as _Iterable
from typing import Mapping as _Mapping
from typing import Optional as _Optional
from typing import Union as _Union

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf.internal import containers as _containers

DESCRIPTOR: _descriptor.FileDescriptor

class Timestamp(_message.Message):
    __slots__ = ("sec", "nanosec")
    SEC_FIELD_NUMBER: _ClassVar[int]
    NANOSEC_FIELD_NUMBER: _ClassVar[int]
    sec: int
    nanosec: int
    def __init__(self, sec: _Optional[int] = ..., nanosec: _Optional[int] = ...) -> None: ...

class CameraIntrinsics(_message.Message):
    __slots__ = ("fx", "fy", "cx", "cy", "width", "height")
    FX_FIELD_NUMBER: _ClassVar[int]
    FY_FIELD_NUMBER: _ClassVar[int]
    CX_FIELD_NUMBER: _ClassVar[int]
    CY_FIELD_NUMBER: _ClassVar[int]
    WIDTH_FIELD_NUMBER: _ClassVar[int]
    HEIGHT_FIELD_NUMBER: _ClassVar[int]
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    def __init__(
        self,
        fx: _Optional[float] = ...,
        fy: _Optional[float] = ...,
        cx: _Optional[float] = ...,
        cy: _Optional[float] = ...,
        width: _Optional[int] = ...,
        height: _Optional[int] = ...,
    ) -> None: ...

class Vector3(_message.Message):
    __slots__ = ("x", "y", "z")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    Z_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    z: float
    def __init__(
        self, x: _Optional[float] = ..., y: _Optional[float] = ..., z: _Optional[float] = ...
    ) -> None: ...

class Quaternion(_message.Message):
    __slots__ = ("w", "x", "y", "z")
    W_FIELD_NUMBER: _ClassVar[int]
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    Z_FIELD_NUMBER: _ClassVar[int]
    w: float
    x: float
    y: float
    z: float
    def __init__(
        self,
        w: _Optional[float] = ...,
        x: _Optional[float] = ...,
        y: _Optional[float] = ...,
        z: _Optional[float] = ...,
    ) -> None: ...

class Pose(_message.Message):
    __slots__ = ("position", "rotation")
    POSITION_FIELD_NUMBER: _ClassVar[int]
    ROTATION_FIELD_NUMBER: _ClassVar[int]
    position: Vector3
    rotation: Quaternion
    def __init__(
        self,
        position: _Optional[_Union[Vector3, _Mapping]] = ...,
        rotation: _Optional[_Union[Quaternion, _Mapping]] = ...,
    ) -> None: ...

class InitializeRequest(_message.Message):
    __slots__ = (
        "tileset_path",
        "use_sh",
        "intrinsics",
        "initial_pose",
        "frame_rate",
        "clock_initial",
        "image_topic",
        "camera_info_topic",
        "frame_id",
        "near_plane",
        "far_plane",
        "device",
        "background_color",
        "compress_format",
    )
    TILESET_PATH_FIELD_NUMBER: _ClassVar[int]
    USE_SH_FIELD_NUMBER: _ClassVar[int]
    INTRINSICS_FIELD_NUMBER: _ClassVar[int]
    INITIAL_POSE_FIELD_NUMBER: _ClassVar[int]
    FRAME_RATE_FIELD_NUMBER: _ClassVar[int]
    CLOCK_INITIAL_FIELD_NUMBER: _ClassVar[int]
    IMAGE_TOPIC_FIELD_NUMBER: _ClassVar[int]
    CAMERA_INFO_TOPIC_FIELD_NUMBER: _ClassVar[int]
    FRAME_ID_FIELD_NUMBER: _ClassVar[int]
    NEAR_PLANE_FIELD_NUMBER: _ClassVar[int]
    FAR_PLANE_FIELD_NUMBER: _ClassVar[int]
    DEVICE_FIELD_NUMBER: _ClassVar[int]
    BACKGROUND_COLOR_FIELD_NUMBER: _ClassVar[int]
    COMPRESS_FORMAT_FIELD_NUMBER: _ClassVar[int]
    tileset_path: str
    use_sh: bool
    intrinsics: CameraIntrinsics
    initial_pose: Pose
    frame_rate: float
    clock_initial: Timestamp
    image_topic: str
    camera_info_topic: str
    frame_id: str
    near_plane: float
    far_plane: float
    device: str
    background_color: Vector3
    compress_format: str
    def __init__(
        self,
        tileset_path: _Optional[str] = ...,
        use_sh: bool = ...,
        intrinsics: _Optional[_Union[CameraIntrinsics, _Mapping]] = ...,
        initial_pose: _Optional[_Union[Pose, _Mapping]] = ...,
        frame_rate: _Optional[float] = ...,
        clock_initial: _Optional[_Union[Timestamp, _Mapping]] = ...,
        image_topic: _Optional[str] = ...,
        camera_info_topic: _Optional[str] = ...,
        frame_id: _Optional[str] = ...,
        near_plane: _Optional[float] = ...,
        far_plane: _Optional[float] = ...,
        device: _Optional[str] = ...,
        background_color: _Optional[_Union[Vector3, _Mapping]] = ...,
        compress_format: _Optional[str] = ...,
    ) -> None: ...

class InitializeResponse(_message.Message):
    __slots__ = ("success", "message", "scene_origin", "ecef_translation", "ecef_rotation")
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    SCENE_ORIGIN_FIELD_NUMBER: _ClassVar[int]
    ECEF_TRANSLATION_FIELD_NUMBER: _ClassVar[int]
    ECEF_ROTATION_FIELD_NUMBER: _ClassVar[int]
    success: bool
    message: str
    scene_origin: Vector3
    ecef_translation: Vector3
    ecef_rotation: _containers.RepeatedScalarFieldContainer[float]
    def __init__(
        self,
        success: bool = ...,
        message: _Optional[str] = ...,
        scene_origin: _Optional[_Union[Vector3, _Mapping]] = ...,
        ecef_translation: _Optional[_Union[Vector3, _Mapping]] = ...,
        ecef_rotation: _Optional[_Iterable[float]] = ...,
    ) -> None: ...

class CameraData(_message.Message):
    __slots__ = ("stamp", "pose")
    STAMP_FIELD_NUMBER: _ClassVar[int]
    POSE_FIELD_NUMBER: _ClassVar[int]
    stamp: Timestamp
    pose: Pose
    def __init__(
        self,
        stamp: _Optional[_Union[Timestamp, _Mapping]] = ...,
        pose: _Optional[_Union[Pose, _Mapping]] = ...,
    ) -> None: ...

class StreamSummary(_message.Message):
    __slots__ = ("frames_rendered", "poses_received")
    FRAMES_RENDERED_FIELD_NUMBER: _ClassVar[int]
    POSES_RECEIVED_FIELD_NUMBER: _ClassVar[int]
    frames_rendered: int
    poses_received: int
    def __init__(
        self, frames_rendered: _Optional[int] = ..., poses_received: _Optional[int] = ...
    ) -> None: ...

class LidarSensorConfig(_message.Message):
    __slots__ = (
        "name",
        "sensor_type",
        "n_rows",
        "n_columns",
        "fps",
        "min_range_m",
        "max_range_m",
        "extrinsic",
        "elevation_deg",
        "pointcloud_topic",
        "frame_id",
        "drop_threshold",
        "alpha_threshold",
    )
    NAME_FIELD_NUMBER: _ClassVar[int]
    SENSOR_TYPE_FIELD_NUMBER: _ClassVar[int]
    N_ROWS_FIELD_NUMBER: _ClassVar[int]
    N_COLUMNS_FIELD_NUMBER: _ClassVar[int]
    FPS_FIELD_NUMBER: _ClassVar[int]
    MIN_RANGE_M_FIELD_NUMBER: _ClassVar[int]
    MAX_RANGE_M_FIELD_NUMBER: _ClassVar[int]
    EXTRINSIC_FIELD_NUMBER: _ClassVar[int]
    ELEVATION_DEG_FIELD_NUMBER: _ClassVar[int]
    POINTCLOUD_TOPIC_FIELD_NUMBER: _ClassVar[int]
    FRAME_ID_FIELD_NUMBER: _ClassVar[int]
    DROP_THRESHOLD_FIELD_NUMBER: _ClassVar[int]
    ALPHA_THRESHOLD_FIELD_NUMBER: _ClassVar[int]
    name: str
    sensor_type: str
    n_rows: int
    n_columns: int
    fps: float
    min_range_m: float
    max_range_m: float
    extrinsic: Pose
    elevation_deg: _containers.RepeatedScalarFieldContainer[float]
    pointcloud_topic: str
    frame_id: str
    drop_threshold: float
    alpha_threshold: float
    def __init__(
        self,
        name: _Optional[str] = ...,
        sensor_type: _Optional[str] = ...,
        n_rows: _Optional[int] = ...,
        n_columns: _Optional[int] = ...,
        fps: _Optional[float] = ...,
        min_range_m: _Optional[float] = ...,
        max_range_m: _Optional[float] = ...,
        extrinsic: _Optional[_Union[Pose, _Mapping]] = ...,
        elevation_deg: _Optional[_Iterable[float]] = ...,
        pointcloud_topic: _Optional[str] = ...,
        frame_id: _Optional[str] = ...,
        drop_threshold: _Optional[float] = ...,
        alpha_threshold: _Optional[float] = ...,
    ) -> None: ...

class InitializeLidarRequest(_message.Message):
    __slots__ = ("sensor",)
    SENSOR_FIELD_NUMBER: _ClassVar[int]
    sensor: LidarSensorConfig
    def __init__(self, sensor: _Optional[_Union[LidarSensorConfig, _Mapping]] = ...) -> None: ...

class LidarData(_message.Message):
    __slots__ = ("stamp", "pose")
    STAMP_FIELD_NUMBER: _ClassVar[int]
    POSE_FIELD_NUMBER: _ClassVar[int]
    stamp: Timestamp
    pose: Pose
    def __init__(
        self,
        stamp: _Optional[_Union[Timestamp, _Mapping]] = ...,
        pose: _Optional[_Union[Pose, _Mapping]] = ...,
    ) -> None: ...
