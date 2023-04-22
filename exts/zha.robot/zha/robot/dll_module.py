import ctypes
import pathlib
import omni.kit.app

EXTENSION_FOLDER_PATH = pathlib.Path(
    omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
)

class DLLConfig:
    #zExternalDLLFile = ctypes.CDLL(f"{EXTENSION_FOLDER_PATH}/data/zSpace_External.dll")
    zExternalDLLFile = ctypes.CDLL(f"{EXTENSION_FOLDER_PATH}/data/zSpace_External_2.dll")