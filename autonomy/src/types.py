
from dataclasses import dataclass
from typing import List
@dataclass
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    cls: str
    conf: float
    depth: float = 0

@dataclass
class Detections:
    detections: List[Detection]
    detector_name: str

@dataclass 
class Point3D:
    x: float
    y: float
    z: float



@dataclass
class Rotation3D:
    a: float
    b: float
    c: float
    d:float
