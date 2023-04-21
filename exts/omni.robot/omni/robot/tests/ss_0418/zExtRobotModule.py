import ctypes
from . import zExtMeshModule
from . import zExtTransformModule

from .zExtMeshModule import zExtMesh
from .zExtMeshModule import zExtMeshArray
from .zExtTransformModule import zExtTransform
from .DllModeule import DLLConfig

DLLFile = DLLConfig.zExternalDLLFile

class zExtRobot(ctypes.Structure):
    _fields_ = [
        ("robot", ctypes.c_void_p),
        ("robotJointTransforms", zExtTransform * 6), 
        ("robotJointRotation", ctypes.c_float * 6),
        ("robotJointRotationMax", ctypes.c_float * 6),
        ("robotJointRotationMin", ctypes.c_float * 6),
        ("robotJointRotationHome", ctypes.c_float * 6),
        ("robotJointRotationMask", ctypes.c_float * 6),
        ("robotJointRotationPulse", ctypes.c_float * 6),
        ("robotJointRotationOffset", ctypes.c_float * 6),
        ]


ext_zTsRobot_createFromFile = DLLFile.ext_zTsRobot_createRobotFromFile
ext_zTsRobot_createFromFile.restype = None
ext_zTsRobot_createFromFile.argtypes = [ctypes.POINTER(zExtRobot), ctypes.c_char_p]

ext_zTsRobot_createRobotJointMeshesfromFile = DLLFile.ext_zTsRobot_createRobotJointMeshesfromFile
ext_zTsRobot_createRobotJointMeshesfromFile.restype = None
ext_zTsRobot_createRobotJointMeshesfromFile.argtypes = [ctypes.POINTER(zExtRobot), ctypes.c_char_p]

ext_zTsRobot_setEndEffector = DLLFile.ext_zTsRobot_setEndEffector
ext_zTsRobot_setEndEffector.restype = None
ext_zTsRobot_setEndEffector.argtypes = [ctypes.POINTER(zExtRobot), ctypes.POINTER(zExtTransform)]

ext_zTsRobot_setFabricationMeshJSONFromDir = DLLFile.ext_zTsRobot_setFabricationMeshJSONFromDir
ext_zTsRobot_setFabricationMeshJSONFromDir.restype = None
ext_zTsRobot_setFabricationMeshJSONFromDir.argtypes = [ctypes.POINTER(zExtRobot), ctypes.c_char_p]

ext_zTsRobot_setFabricationMeshOBJFromDir = DLLFile.ext_zTsRobot_setFabricationMeshOBJFromDir
ext_zTsRobot_setFabricationMeshOBJFromDir.restype = None
ext_zTsRobot_setFabricationMeshOBJFromDir.argtypes = [ctypes.POINTER(zExtRobot), ctypes.c_char_p]

ext_zTsRobot_setFabricationMesh = DLLFile.ext_zTsRobot_setFabricationMesh
ext_zTsRobot_setFabricationMesh.restype = None
ext_zTsRobot_setFabricationMesh.argtypes = [ctypes.POINTER(zExtRobot), ctypes.POINTER(zExtMesh), ctypes.c_int]

ext_zTsRobot_setFabricationPlanes = DLLFile.ext_zTsRobot_setFabricationPlanes
ext_zTsRobot_setFabricationPlanes.restype = None
ext_zTsRobot_setFabricationPlanes.argtypes = [ctypes.POINTER(zExtRobot), ctypes.POINTER(zExtTransform), ctypes.POINTER(zExtTransform)]

ext_zTsRobot_getTargets = DLLFile.ext_zTsRobot_getTargets
ext_zTsRobot_getTargets.restype = None
ext_zTsRobot_getTargets.argtypes = [
    ctypes.POINTER(zExtRobot), 
    ctypes.POINTER(zExtTransform), 
    ctypes.POINTER(ctypes.c_bool), 
    ctypes.POINTER(ctypes.c_int)
    ]

ext_zTsRobot_getFabricationPlanes = DLLFile.ext_zTsRobot_getFabricationPlanes
ext_zTsRobot_getFabricationPlanes.restype = None
ext_zTsRobot_getFabricationPlanes.argtypes = [ctypes.POINTER(zExtRobot), ctypes.POINTER(zExtTransform), ctypes.POINTER(zExtTransform)]

ext_zTsRobot_getFabricationMeshes = DLLFile.ext_zTsRobot_getFabricationMeshes
ext_zTsRobot_getFabricationMeshes.restype = None
ext_zTsRobot_getFabricationMeshes.argtypes = [ctypes.POINTER(zExtRobot), ctypes.POINTER(zExtMeshArray), ctypes.POINTER(ctypes.c_int)]

ext_zTsRobot_forwardKinematics = DLLFile.ext_zTsRobot_forwardKinematics
ext_zTsRobot_forwardKinematics.restype = None
ext_zTsRobot_forwardKinematics.argtypes = [ctypes.POINTER(zExtRobot), ctypes.POINTER(ctypes.c_float)]

ext_zTsRobot_inverseKinematics = DLLFile.ext_zTsRobot_inverseKinematics
ext_zTsRobot_inverseKinematics.restype = ctypes.c_bool
ext_zTsRobot_inverseKinematics.argtypes = [
    ctypes.POINTER(zExtRobot), 
    ctypes.POINTER(zExtTransform), 
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ]


ext_zTsRobot_computeTargetzTsRHWC = DLLFile.ext_zTsRobot_computeTargetzTsRHWC
ext_zTsRobot_computeTargetzTsRHWC.restype = None
ext_zTsRobot_computeTargetzTsRHWC.argtypes = [ctypes.POINTER(zExtRobot), ctypes.POINTER(ctypes.c_int)]

#ext_zTsRobot_computeTargetzTsRHWC(zExtRobot& extRobot, int& targetCount)


#ext_zTsRobot_computeTargetzTsRHWC = DLLFile.ext_zTsRobot_computeTargetzTsRHWC
#ext_zTsRobot_computeTargetzTsRHWC.restype = None
#ext_zTsRobot_computeTargetzTsRHWC.argtypes = [ctypes.POINTER(zExtRobot), ctypes.POINTER(ctypes.c_int)]


#-------------------------------------------------------------------------------------------
'''
ext_zTsRobot_setFabricationMesh = DLLFile.ext_zTsRobot_setFabricationMesh
ext_zTsRobot_setFabricationMesh.restype = None
ext_zTsRobot_setFabricationMesh.argtypes = [ctypes.POINTER(zExtRobot), ctypes.POINTER(zExtMesh), ctypes.c_int, ctypes.POINTER(zExtTransform), ctypes.POINTER(zExtTransform)]

ext_zTsRobot_setFabricationMeshFromDir = DLLFile.ext_zTsRobot_setFabricationMeshFromDir
ext_zTsRobot_setFabricationMeshFromDir.restype = None
ext_zTsRobot_setFabricationMeshFromDir.argtypes = [ctypes.POINTER(zExtRobot), ctypes.c_char_p]

ext_zTsRobot_getFabricationMesh = DLLFile.ext_zTsRobot_getFabricationMesh
ext_zTsRobot_getFabricationMesh.restype = None
ext_zTsRobot_getFabricationMesh.argtypes = [ctypes.POINTER(zExtRobot), ctypes.POINTER(zExtMeshArray), ctypes.POINTER(zExtTransform), ctypes.POINTER(zExtTransform)]

ext_zTsRobot_setFabricationMesh = DLLFile.ext_zTsRobot_setFabricationMesh
ext_zTsRobot_setFabricationMesh.restype = None
ext_zTsRobot_setFabricationMesh.argtypes = [ctypes.POINTER(zExtRobot), ctypes.POINTER(zExtMesh), ctypes.c_int, ctypes.POINTER(zExtTransform), ctypes.POINTER(zExtTransform)]

ext_zTsRobot_computeTargetzTsRHWC = DLLFile.ext_zTsRobot_computeTargetzTsRHWC
ext_zTsRobot_computeTargetzTsRHWC.restype = None
ext_zTsRobot_computeTargetzTsRHWC.argtypes = [ctypes.POINTER(zExtRobot), ctypes.POINTER(ctypes.c_int)]
'''
