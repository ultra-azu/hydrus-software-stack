
from dataclasses import dataclass
from typing import List
@dataclass
class OutputBBox:
    x1: float
    y1: float
    x2: float
    y2: float
    cls: str
    conf: float
    depth: float = 0


@dataclass 
class point_3d:
    x: float
    y: float
    z: float


@dataclass
class mission_data:
    detections: List[OutputBBox]
    current_point: point_3d
    history: list


@dataclass
class rotation_3d:
    a: float
    b: float
    c: float
    d:float
