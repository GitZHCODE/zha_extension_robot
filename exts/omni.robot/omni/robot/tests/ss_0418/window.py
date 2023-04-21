# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
__all__ = ["ZhcodeRobotWindow"]

import omni.ui as ui
from omni.ui import scene as sc
from omni.ui import color as cl
import omni.usd
from pxr import UsdGeom, Usd, Sdf, Gf, Tf, UsdUtils
from pxr.Usd import Stage
import ctypes, ctypes.util
import numpy as np
import pathlib
import omni.kit.app
import carb
import time
import omni.kit.utils
import omni.appwindow
import omni.timeline
import omni.anim
import asyncio

from . import zExtMeshModule
from . import zExtTransformModule
from . import zExtRobotModule

from .zExtMeshModule import zExtMesh, zExtMeshArray 
from .zExtTransformModule import zExtTransform 
from .zExtRobotModule import zExtRobot
from .zListenerModule import zListener

from omni.kit.window.popup_dialog import MessageDialog

from .custom_bool_widget import CustomBoolWidget
from .custom_color_widget import CustomColorWidget
from .custom_combobox_widget import CustomComboboxWidget
from .custom_multifield_widget import CustomMultifieldWidget
from .custom_path_button import CustomPathButtonWidget
from .custom_radio_collection import CustomRadioCollection
from .custom_slider_widget import CustomSliderWidget
from .style import julia_modeler_style, ATTR_LABEL_WIDTH

SPACING = 5

def on_test():
    print("test!") 

class ZhcodeRobotWindow(ui.Window):
    """The class that represents the window"""    

    def __init__(self, title: str, delegate=None, **kwargs):
        self.__label_width = ATTR_LABEL_WIDTH

        super().__init__(title, **kwargs)
        self.frame.style = julia_modeler_style
        self.stage_listener = None
        self.bool_listening = False
        self.joint1_slider = None
        self.joint2_slider = None
        self.joint3_slider = None
        self.joint4_slider = None
        self.joint5_slider = None
        self.joint6_slider = None
        self.frame_slider = None
        self.current_frame = 0
        self.max_frame = 10
        self.targets_num = 0
        self.targets_list = []
        self.reachability_list = []
        self.target_type = []
        self.joints_slider_enabled = False
        self.frame.set_build_fn(self._build_fn)

    def destroy(self):
        # Destroys all the children
        super().destroy()

    @property
    def label_width(self):
        """The width of the attribute label"""
        return self.__label_width

    @label_width.setter
    def label_width(self, value):
        """The width of the attribute label"""
        self.__label_width = value
        self.frame.rebuild()

    def on_export_btn_click(self, path):
        dialog = MessageDialog(
            title="Export Dialog",
            message=f"Export file with path: {path}",
            disable_cancel_button=True,
            ok_handler=lambda dialog: dialog.hide(),            
            #layout=Layout(align_items='center')
        )
        dialog.show()

    def _build_title(self):
        with ui.VStack():
            ui.Spacer(height=10)
            ui.Label("ZHCODE Robot - 1.0", name="window_title")
            ui.Spacer(height=10)

    def _build_collapsable_header(self, collapsed, title):
        """Build a custom title of CollapsableFrame"""
        with ui.VStack():
            ui.Spacer(height=8)
            with ui.HStack():
                ui.Label(title, name="collapsable_name")

                if collapsed:
                    image_name = "collapsable_opened"
                else:
                    image_name = "collapsable_closed"
                ui.Image(name=image_name, width=10, height=10)
            ui.Spacer(height=8)
            ui.Line(style_type_name_override="HeaderLine")

    def _build_fn(self):
        stage = omni.usd.get_context().get_stage()
        prim_J1_path = ["/abb_robot/geometry/r_1"]
        prim_J2_path = ["/abb_robot/geometry/r_2"]
        prim_J3_path = ["/abb_robot/geometry/r_3"]
        prim_J4_path = ["/abb_robot/geometry/r_4"]
        prim_J5_path = ["/abb_robot/geometry/r_5"]
        prim_J6_path = ["/abb_robot/geometry/r_6"]
        prim_EE_path = ["/abb_robot/geometry/r_EE"]
        prim_target_path = ["/abb_robot/geometry/target"]
        prim_cutter_path = ["/abb_robot/geometry/r_EE/cutter"]        
        prim_J1 = [None]
        prim_J2 = [None]
        prim_J3 = [None]
        prim_J4 = [None]
        prim_J5 = [None]
        prim_J6 = [None]
        prim_EE = [None]
        prim_target = [None]
        prim_cutter = [None]      
        #------------------------------------------------------------------------------------
        # Create an instance of the zExtRobot struct
        myRobot = zExtRobot()
        
        jsonFileSTR = "C:/Users/Ling.Mao/Desktop/robot/ABB_IRB_4600_255.json"
        jsonCStr = ctypes.c_char_p(jsonFileSTR.encode())        
        
        zExtRobotModule.ext_zTsRobot_createFromFile(ctypes.byref(myRobot),jsonCStr)
        ee_transform = zExtTransform() 
        eematrix = [  [1, 0, 0, 0],  [0, 1, 0, 0],  [0, 0, 1, 0],  [0, 0, -0.994,1]]
        ee_transform.updateTransformFromListOfLists(eematrix)
        zExtRobotModule.ext_zTsRobot_setEndEffector(ctypes.byref(myRobot), ctypes.byref(ee_transform))
        
        fabMeshDir = "C:/Users/Ling.Mao/Desktop/robot/cut_mesh"
        fabMeshDirCStr = ctypes.c_char_p(fabMeshDir.encode())
        zExtRobotModule.ext_zTsRobot_setFabricationMeshJSONFromDir(ctypes.byref(myRobot),fabMeshDirCStr)

        #fabMeshDir = "C:/Users/Ling.Mao/Desktop/robot/obj_mesh"
        #fabMeshDirCStr = ctypes.c_char_p(fabMeshDir.encode())
        #zExtRobotModule.ext_zTsRobot_setFabricationMeshOBJFromDir(ctypes.byref(myRobot),fabMeshDirCStr)

        #------------------------------------------------------------------------------  
              
        def reset_primMesh():
            print("reset!")
            stage = omni.usd.get_context().get_stage()
            prim_J1[0] = stage.GetPrimAtPath(prim_J1_path[0])
            prim_J2[0] = stage.GetPrimAtPath(prim_J2_path[0])
            prim_J3[0] = stage.GetPrimAtPath(prim_J3_path[0])
            prim_J4[0] = stage.GetPrimAtPath(prim_J4_path[0])
            prim_J5[0] = stage.GetPrimAtPath(prim_J5_path[0])
            prim_J6[0] = stage.GetPrimAtPath(prim_J6_path[0])
            prim_EE[0] = stage.GetPrimAtPath(prim_EE_path[0])
            prim_target[0] = stage.GetPrimAtPath(prim_target_path[0])
            prim_cutter[0] = stage.GetPrimAtPath(prim_cutter_path[0])

        def update_joints_transform():            
            Cur_Matrix =[]
            for i in range(6):
                Cur_Matrix.append(myRobot.robotJointTransforms[i].getGfMatrix())
                       
            xform_1 = UsdGeom.Xformable(prim_J1[0])
            transform_1 = xform_1.MakeMatrixXform()
            transform_1.Set(Cur_Matrix[0])
            
            xform_2 = UsdGeom.Xformable(prim_J2[0])
            transform_2 = xform_2.MakeMatrixXform()
            transform_2.Set(Cur_Matrix[1])

            xform_3 = UsdGeom.Xformable(prim_J3[0])
            transform_3 = xform_3.MakeMatrixXform()
            transform_3.Set(Cur_Matrix[2])

            xform_4 = UsdGeom.Xformable(prim_J4[0])
            transform_4 = xform_4.MakeMatrixXform()
            transform_4.Set(Cur_Matrix[3])

            xform_5 = UsdGeom.Xformable(prim_J5[0])
            transform_5 = xform_5.MakeMatrixXform()
            transform_5.Set(Cur_Matrix[4])

            xform_6 = UsdGeom.Xformable(prim_J6[0])
            transform_6 = xform_6.MakeMatrixXform()
            transform_6.Set(Cur_Matrix[5])

            xform_EE = UsdGeom.Xformable(prim_EE[0])
            transform_EE = xform_EE.MakeMatrixXform()
            transform_EE.Set(Cur_Matrix[5])

        def update_joints_rotation_slider():
            rotate = myRobot.robotJointRotation
            self.joint1_slider.model.set_value(rotate[0])
            self.joint2_slider.model.set_value(rotate[1])
            self.joint3_slider.model.set_value(rotate[2])
            self.joint4_slider.model.set_value(rotate[3])
            self.joint5_slider.model.set_value(rotate[4])
            self.joint6_slider.model.set_value(rotate[5])

        def compute_forwardKinematics():
            joint_rotation = [self.joint1_slider.model.get_value_as_float(), self.joint2_slider.model.get_value_as_float(),self.joint3_slider.model.get_value_as_float(),
                              self.joint4_slider.model.get_value_as_float(), self.joint5_slider.model.get_value_as_float(),self.joint6_slider.model.get_value_as_float()]
            joint_rotation_ctypes = (ctypes.c_float * 6)(*joint_rotation)
            zExtRobotModule.ext_zTsRobot_forwardKinematics(ctypes.byref(myRobot), joint_rotation_ctypes)

        def compute_inverseKinematics():
            xform_target = UsdGeom.Xformable(prim_target[0])
            matrix = xform_target.MakeMatrixXform()
            matrix_list = matrix.Get()
            target_arr = np.asarray(matrix_list, dtype=np.float32).flatten()
            targetTransform = zExtTransform()
            targetTransform.updateTransformFromFloatArray(target_arr)

            dot = ctypes.c_float(0) 
            angle = ctypes.c_float(0) 
            dotZeroZ = ctypes.c_float(0)
            angleZeroZ = ctypes.c_float(0) 
            reached = zExtRobotModule.ext_zTsRobot_inverseKinematics(ctypes.byref(myRobot),
                                                                     ctypes.byref(targetTransform),
                                                                     ctypes.byref(dot),
                                                                     ctypes.byref(angle),
                                                                     ctypes.byref(dotZeroZ),
                                                                     ctypes.byref(angleZeroZ))
            print("dot",dot)
            print("angle",angle)
            print("dotZeroZ",dotZeroZ)
            print("angleZeroZ",angleZeroZ)
            return reached

        def compute_targets():
            robotHomePlane = zExtTransform()
            workPlane = zExtTransform()
            fabMeshArray = zExtMeshArray()
            targetsCount = ctypes.c_int(0)
            zExtRobotModule.ext_zTsRobot_computeTargetzTsRHWC(ctypes.byref(myRobot), ctypes.byref(targetsCount))
            print("targetsCount",targetsCount)
            
            #targets_list = [zExtTransform() for i in range(targetsCount.value)]
            #reachability_list = [False for i in range(targetsCount.value)]
            #types_list = [0 for i in range(targetsCount.value)]

            #targets = (zExtTransform*targetsCount.value)(*targets_list)
            #targetReachability = (ctypes.c_bool*targetsCount.value)(*reachability_list)
            #targetsTypes = (ctypes.c_int*targetsCount.value)(*types_list)
            #zExtRobotModule.ext_zTsRobot_getTargets(ctypes.byref(myRobot),targets,targetReachability,targetsTypes)
            #print("targets",targets)
            #print("targetReachability",targetReachability)
            #print("targetsTypes",targetsTypes)

        def color_target(reached):
            if reached:
                # set target color as white
                prim_target[0].GetAttribute('primvars:displayColor').Set([Gf.Vec3f(1,1,1)])
            if not reached:
                # set target color as red
                prim_target[0].GetAttribute('primvars:displayColor').Set([Gf.Vec3f(1,0,0)])

        def flip_cutter(int_type):
            #cutter_prim = stage.GetPrimAtPath(prim_cutter_path[0])
            #print(prim_cutter[0].GetAttribute('xformOp:rotateXYZ').Get())
            if int_type == 0:
                prim_cutter[0].GetAttribute('xformOp:rotateXYZ').Set(Gf.Vec3d(0,0,0))
            if int_type == 1:
                prim_cutter[0].GetAttribute('xformOp:rotateXYZ').Set(Gf.Vec3d(0,90,0))
            if int_type == 2:
                prim_cutter[0].GetAttribute('xformOp:rotateXYZ').Set(Gf.Vec3d(0,180,0))
            if int_type == 3:
                prim_cutter[0].GetAttribute('xformOp:rotateXYZ').Set(Gf.Vec3d(0,270,0))

        def on_initialize_transform():
            print("1")
            self.current_frame = 0
            self.frame_slider.model.set_value(0)
            self.joint1_slider._restore_default()
            self.joint2_slider._restore_default()
            self.joint3_slider._restore_default()
            self.joint4_slider._restore_default()
            self.joint5_slider._restore_default()
            self.joint6_slider._restore_default()
            print(self.joint1_slider)
            print("2")

            joint_rotation = [myRobot.robotJointRotationHome[0], myRobot.robotJointRotationHome[1],myRobot.robotJointRotationHome[2],
                              myRobot.robotJointRotationHome[3],myRobot.robotJointRotationHome[4],myRobot.robotJointRotationHome[5]]
            joint_rotation_ctypes = (ctypes.c_float * 6)(*joint_rotation)
            zExtRobotModule.ext_zTsRobot_forwardKinematics(ctypes.byref(myRobot), joint_rotation_ctypes)
            update_joints_transform()

        def on_change_joint_rotation():
            if(self.joints_slider_enabled.get_value()):
                compute_forwardKinematics()
                update_joints_transform()

        def on_change_target():
            reached = compute_inverseKinematics()
            update_joints_transform()
            update_joints_rotation_slider()
            if reached:
                print("robot is reached")
            color_target(reached)
            #flip_cutter(3)            

        def test():
            print("test")

        async def on_change_target_async():
            test()

        def on_next_frame():
            self.current_frame += 1 
            if self.current_frame > self.max_frame-1:
                self.current_frame = self.max_frame-1
            self.frame_slider.set_value(self.current_frame)

        def on_previous_frame():
            self.current_frame -= 1 
            if self.current_frame < 0:
                self.current_frame = 0
            self.frame_slider.set_value(self.current_frame)

        def on_change_frame():
            self.current_frame = self.frame_slider.get_value_as_int()
            print(self.current_frame)

        def on_play_animation():            
            timeline = omni.timeline.get_timeline_interface() 
            timecode = timeline.get_current_time() * timeline.get_time_codes_per_seconds()
            print(timecode)
            xform_1 = UsdGeom.Xformable(prim_J1[0])
            transform_1 = xform_1.MakeMatrixXform()
            transform_1.Set(time=10, value= self.current_frame)

            self.current_frame = 0
            for i in range(round(self.max_frame)):
                on_next_frame()

        def on_objects_changed(self, notice: Tf.Notice, sender: Usd.Stage):
            for p in notice.GetChangedInfoOnlyPaths(): 
                if p.IsAbsoluteRootOrPrimPath():
                    print(p)
                    if p.IsPropertyPath():                         
                        #prim_path = p.GetPrimPath()
                        #print(f"prim path: {prim_path}, prop = {p}")
                        if p == f"{prim_target_path}.xformOp:tranform":    
                            asyncio.ensure_future(on_change_target_async)

        def on_listening():            
            bool_listening = not bool_listening
            if(bool_listening):
                self.stage_listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, on_objects_changed, stage)
            if(not bool_listening):
                self.stage_listener = None
               
        # setup my UI interface window
        with ui.ScrollingFrame(name="window_bg",horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF):
            with ui.VStack(height=0):
                self._build_title()

                with ui.CollapsableFrame("initialize".upper(), name="group",build_header_fn=self._build_collapsable_header):
                    with ui.VStack(height=0, spacing=SPACING):
                        ui.Spacer(height=3)
                        ui.Button("Reset Joints & EE PrimMesh !!!",clicked_fn = reset_primMesh)
                        with ui.HStack(height=1):
                            ui.Button("Reset fabMesh")
                            ui.Button("Reset workPlane")
                            ui.Button("Reset basePlane")
                        CustomComboboxWidget(label="Robot Type",options=["ABB_IRB_4600_255", "KUKA_KR30", "NACHI_MZ07"])
                        CustomPathButtonWidget(label="Read Path",path=".../read/mesh1.json",btn_label="Read",btn_callback=self.on_export_btn_click)
                

                with ui.CollapsableFrame("compute".upper(), name="group",
                                build_header_fn=self._build_collapsable_header):
                    with ui.VStack(height=0, spacing=SPACING):
                        ui.Spacer(height=3)                        
                        self.joints_slider_enabled = CustomBoolWidget(label="Joints_Slider_Enabled", default_value=False)
                        ui.Spacer(height=3) 

                        self.joint1_slider = CustomSliderWidget(min=myRobot.robotJointRotationMin[0], max=myRobot.robotJointRotationMax[0], 
                                                num_type = "float",label="J1_Rotation", display_range = True, default_val=myRobot.robotJointRotationHome[0])
                        self.joint1_slider.model.add_value_changed_fn(lambda m : on_change_joint_rotation())
                        
                        self.joint2_slider = CustomSliderWidget(min=myRobot.robotJointRotationMin[1], max=myRobot.robotJointRotationMax[1], 
                                                num_type = "float",label="J2_Rotation", display_range = True, default_val=myRobot.robotJointRotationHome[1])
                        self.joint2_slider.model.add_value_changed_fn(lambda m : on_change_joint_rotation())

                        self.joint3_slider = CustomSliderWidget(min=myRobot.robotJointRotationMin[2], max=myRobot.robotJointRotationMax[2], 
                                                num_type = "float",label="J3_Rotation", display_range = True, default_val=myRobot.robotJointRotationHome[2])
                        self.joint3_slider.model.add_value_changed_fn(lambda m : on_change_joint_rotation())

                        self.joint4_slider = CustomSliderWidget(min=myRobot.robotJointRotationMin[3], max=myRobot.robotJointRotationMax[3], 
                                                num_type = "float",label="J4_Rotation", display_range = True, default_val=myRobot.robotJointRotationHome[3])
                        self.joint4_slider.model.add_value_changed_fn(lambda m : on_change_joint_rotation())

                        self.joint5_slider = CustomSliderWidget(min=myRobot.robotJointRotationMin[4], max=myRobot.robotJointRotationMax[4], 
                                                num_type = "float",label="J5_Rotation", display_range = True, default_val=myRobot.robotJointRotationHome[4])
                        self.joint5_slider.model.add_value_changed_fn(lambda m : on_change_joint_rotation())

                        self.joint6_slider = CustomSliderWidget(min=myRobot.robotJointRotationMin[5], max=myRobot.robotJointRotationMax[5], 
                                                num_type = "float",label="J6_Rotation", display_range = True, default_val=myRobot.robotJointRotationHome[5])
                        self.joint6_slider.model.add_value_changed_fn(lambda m : on_change_joint_rotation())

                        self.frame_slider = CustomSliderWidget(min=0, max=round(self.max_frame-1), num_type = "int",label="Cur_Frame", 
                                                      display_range = True, default_val=0)
                        self.frame_slider.model.add_value_changed_fn(lambda m : on_change_frame()) 
                        ui.Button("Initialize_Joints_Transform",clicked_fn= lambda: on_initialize_transform())

                        ui.Button("Compute Target",clicked_fn = lambda: compute_targets)
                        with ui.HStack(height=10):                            
                            ui.Button("Previous_Frame",clicked_fn= lambda: on_previous_frame)
                            ui.Button("Play_Animation",clicked_fn = lambda: on_play_animation)
                            ui.Button("Next_Frame",clicked_fn= lambda: on_next_frame)
                        ui.Button("Update Target",clicked_fn = lambda: on_change_target)
                        ui.Button("Listening",clicked_fn = lambda: on_listening)

                with ui.CollapsableFrame("output".upper(), name="group",
                                build_header_fn=self._build_collapsable_header):
                    with ui.VStack(height=0, spacing=SPACING):
                        ui.Spacer(height=3) 
                        CustomPathButtonWidget(label="Export Path",path=".../export/mesh1.usd",btn_label="Export",btn_callback=self.on_export_btn_click)

                with ui.CollapsableFrame("graph".upper(), name="group",
                                build_header_fn=self._build_collapsable_header):                        
                        
                        #scene_view = sc.SceneView(aspect_ratio_policy=sc.AspectRatioPolicy.PRESERVE_ASPECT_FIT,height=200)
                        #with scene_view.scene:
                            #sc.Line([-0.5,-0.5,0], [-0.5, 0.5, 0], color=cl.red)
                            #sc.Line([-0.5,-0.5,0], [0.5, -0.5, 0], color=cl.green)
                            #sc.Arc(0.5, color=cl.documentation_nvidia)

                    ui.Spacer(height=10)
               
        