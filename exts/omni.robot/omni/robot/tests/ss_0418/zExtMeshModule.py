import ctypes
from .DllModeule import DLLConfig

DLLFile = DLLConfig.zExternalDLLFile

# Define the zExtMesh struct in Python
class zExtMesh(ctypes.Structure):
    _fields_ = [
        ("mesh", ctypes.c_void_p),
        ("vCount", ctypes.c_int),
        ("fCount", ctypes.c_int)
        ]

# Define the zExtMesh struct in Python
class zExtMeshArray(ctypes.Structure):
    _fields_ = [
        ("arrayPointer", ctypes.c_void_p),
        ("arrayCount", ctypes.c_int),
        ]

# Define the function prototypes with ctypes function annotations
ext_meshUtil_createMeshOBJFromArray = DLLFile.ext_meshUtil_createMeshOBJFromArray
ext_meshUtil_createMeshOBJFromArray.restype = None
ext_meshUtil_createMeshOBJFromArray.argtypes = [
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.c_int,
    ctypes.c_int,
    ctypes.POINTER(zExtMesh),
    ]

ext_meshUtil_createMeshOBJFromFile = DLLFile.ext_meshUtil_createMeshOBJFromFile
ext_meshUtil_createMeshOBJFromFile.restype = None
ext_meshUtil_createMeshOBJFromFile.argtypes = [
    ctypes.c_char_p,
    ctypes.POINTER(zExtMesh),
    ]

ext_meshUtil_getMeshFaceCount= DLLFile.ext_meshUtil_getMeshFaceCount
ext_meshUtil_getMeshFaceCount.restype = ctypes.c_int
ext_meshUtil_getMeshFaceCount.argtypes = [
    ctypes.POINTER(zExtMesh),
    ctypes.POINTER(ctypes.c_int),
    ]

ext_meshUtil_getMeshPosition = DLLFile.ext_meshUtil_getMeshPosition
ext_meshUtil_getMeshPosition.restype = ctypes.c_int
ext_meshUtil_getMeshPosition.argtypes = [
    ctypes.POINTER(zExtMesh),
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ]

ext_meshUtil_getMeshFaceConnect = DLLFile.ext_meshUtil_getMeshFaceConnect
ext_meshUtil_getMeshFaceConnect.restype = ctypes.c_int
ext_meshUtil_getMeshFaceConnect.argtypes = [
    ctypes.POINTER(zExtMesh),
    ctypes.POINTER(ctypes.c_int),
    ]

ext_meshUtil_getMeshsFromMeshArray = DLLFile.ext_meshUtil_getMeshsFromMeshArray
ext_meshUtil_getMeshsFromMeshArray.restype = ctypes.c_int
ext_meshUtil_getMeshsFromMeshArray.argtypes = [
    ctypes.POINTER(zExtMeshArray),
     ctypes.POINTER(zExtMesh),
    ]

# Export the zExtMesh struct and its corresponding methods
__all__ = [
    "zExtMesh",
    "ext_meshUtil_createMeshOBJFromArray",
    "ext_meshUtil_createMeshOBJFromFile",
    "ext_meshUtil_getMeshFaceCount",
    "ext_meshUtil_getMeshPosition",
    "ext_meshUtil_getMeshFaceConnect",
    ]
