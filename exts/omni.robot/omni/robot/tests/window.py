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
import omni.usd
import omni.kit.utils
import omni.appwindow
import omni.timeline
import omni.anim
import omni.kit.app
import carb.events
from carb.events import IEventStream

from pxr import UsdGeom, Usd, Sdf, Gf, Tf, UsdUtils
from pxr.Usd import Stage

import ctypes, ctypes.util
import numpy as np
import time
import asyncio
import pathlib

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
EXTENSION_FOLDER_PATH = pathlib.Path(
    omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
)

class ZhcodeRobotWindow(ui.Window):
    def __init__(self, title: str, delegate=None, **kwargs):
        self.__label_width = ATTR_LABEL_WIDTH

        super().__init__(title, **kwargs)
        self.frame.style = julia_modeler_style
        self.stage = omni.usd.get_context().get_stage()
        self.prim_J1_path = "/abb_robot/geometry/r_1"
        self.prim_J2_path = "/abb_robot/geometry/r_2"
        self.prim_J3_path = "/abb_robot/geometry/r_3"
        self.prim_J4_path = "/abb_robot/geometry/r_4"
        self.prim_J5_path = "/abb_robot/geometry/r_5"
        self.prim_J6_path = "/abb_robot/geometry/r_6"
        self.prim_EE_path = "/abb_robot/geometry/r_EE"
        self.prim_target_path = "/abb_robot/target"
        self.prim_cutter_path = "/abb_robot/geometry/r_EE/cutter"

        self.prim_J1 = None
        self.prim_J2 = None
        self.prim_J3 = None
        self.prim_J4 = None
        self.prim_J5 = None
        self.prim_J6 = None
        self.prim_EE = None
        self.prim_target = None       
        self.prim_cutter = None

        self.joint1_slider = None
        self.joint2_slider = None
        self.joint3_slider = None
        self.joint4_slider = None
        self.joint5_slider = None
        self.joint6_slider = None
        self.frame_slider = None
        self.bool_widget_fk = None
        
        self.current_frame = 0
        self.max_frame = 100
        self.targets_num = 0
        self.all_targets = []
        self.all_reachability = []
        self.target_type = []
        self.cutter_angle = []

        self.stage_listener = None
        self.bool_listening = False 
        self.timeline = None      
        
        self.robot = zExtRobot()

        jsonFileSTR = f"{EXTENSION_FOLDER_PATH}/data/ABB_IRB_4600_255.json"
        jsonCStr = ctypes.c_char_p(jsonFileSTR.encode())        
        zExtRobotModule.ext_zTsRobot_createFromFile(ctypes.byref(self.robot),jsonCStr)

        ee_transform = zExtTransform() 
        eematrix = [  [1, 0, 0, 0],  [0, 1, 0, 0],  [0, 0, 1, 0],  [0, 0, -0.994,1]]
        ee_transform.updateTransformFromListOfLists(eematrix)
        zExtRobotModule.ext_zTsRobot_setEndEffector(ctypes.byref(self.robot), ctypes.byref(ee_transform))
        
        robotBaseMatrix = [ [0.965926,0,-0.258819,0], [0,1,0,0], [0.258819,0,0.965926,0], [0,0,0,1]]
        robot_transform = zExtTransform()
        robot_transform.updateTransformFromListOfLists(robotBaseMatrix)
        zExtRobotModule.ext_zTsRobot_setRobotBasePlane(ctypes.byref(self.robot), ctypes.byref(robot_transform))        

        homePlane_Matrix = [[ 1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [1.8, -0.25, 0, 1] ]
        homePlane_transform = zExtTransform()
        homePlane_transform.updateTransformFromListOfLists(homePlane_Matrix)
        zExtRobotModule.ext_zTsRobot_setRobotHomePlane(ctypes.byref(self.robot), ctypes.byref(homePlane_transform))


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

    def on_reset_primMesh(self): 
        self.stage = omni.usd.get_context().get_stage()
        self.prim_J1 = self.stage.GetPrimAtPath(self.prim_J1_path)
        self.prim_J2 = self.stage.GetPrimAtPath(self.prim_J2_path)
        self.prim_J3 = self.stage.GetPrimAtPath(self.prim_J3_path)
        self.prim_J4 = self.stage.GetPrimAtPath(self.prim_J4_path)
        self.prim_J5 = self.stage.GetPrimAtPath(self.prim_J5_path)
        self.prim_J6 = self.stage.GetPrimAtPath(self.prim_J6_path)
        self.prim_EE = self.stage.GetPrimAtPath(self.prim_EE_path)
        self.prim_target = self.stage.GetPrimAtPath(self.prim_target_path)            
        self.prim_cutter = self.stage.GetPrimAtPath(self.prim_cutter_path)

    def on_read_fabMesh(self,path):
            fabMeshDir = f"{EXTENSION_FOLDER_PATH}{path}"
            fabMeshDirCStr = ctypes.c_char_p(fabMeshDir.encode())
            zExtRobotModule.ext_zTsRobot_setFabricationMeshJSONFromDir(ctypes.byref(self.robot),fabMeshDirCStr)
            print(fabMeshDir)

    def on_bake_fabPrimMesh(self):
        pass

    def update_joints_transform(self):            
        Cur_Matrix =[]
        for i in range(6):
            Cur_Matrix.append(self.robot.robotJointTransforms[i].getGfMatrix())
                       
        xform_1 = UsdGeom.Xformable(self.prim_J1)
        transform_1 = xform_1.MakeMatrixXform()
        transform_1.Set(Cur_Matrix[0])
            
        xform_2 = UsdGeom.Xformable(self.prim_J2)
        transform_2 = xform_2.MakeMatrixXform()
        transform_2.Set(Cur_Matrix[1])

        xform_3 = UsdGeom.Xformable(self.prim_J3)
        transform_3 = xform_3.MakeMatrixXform()
        transform_3.Set(Cur_Matrix[2])

        xform_4 = UsdGeom.Xformable(self.prim_J4)
        transform_4 = xform_4.MakeMatrixXform()
        transform_4.Set(Cur_Matrix[3])

        xform_5 = UsdGeom.Xformable(self.prim_J5)
        transform_5 = xform_5.MakeMatrixXform()
        transform_5.Set(Cur_Matrix[4])

        xform_6 = UsdGeom.Xformable(self.prim_J6)
        transform_6 = xform_6.MakeMatrixXform()
        transform_6.Set(Cur_Matrix[5])

        xform_EE = UsdGeom.Xformable(self.prim_EE)
        transform_EE = xform_EE.MakeMatrixXform()
        transform_EE.Set(Cur_Matrix[5])

    def update_joints_rotation_slider(self):
        rotate = self.robot.robotJointRotation
        self.joint1_slider.model.set_value(rotate[0])
        self.joint2_slider.model.set_value(rotate[1])
        self.joint3_slider.model.set_value(rotate[2])
        self.joint4_slider.model.set_value(rotate[3])
        self.joint5_slider.model.set_value(rotate[4])
        self.joint6_slider.model.set_value(rotate[5])

    def compute_forwardKinematics(self):
        joint_rotation = [self.joint1_slider.model.get_value_as_float(), self.joint2_slider.model.get_value_as_float(),
                          self.joint3_slider.model.get_value_as_float(), self.joint4_slider.model.get_value_as_float(), 
                          self.joint5_slider.model.get_value_as_float(),self.joint6_slider.model.get_value_as_float()]
        joint_rotation_ctypes = (ctypes.c_float * 6)(*joint_rotation)
        zExtRobotModule.ext_zTsRobot_forwardKinematics(ctypes.byref(self.robot), joint_rotation_ctypes)

    def compute_inverseKinematics(self):
            xform_target = UsdGeom.Xformable(self.prim_target)
            matrix = xform_target.MakeMatrixXform()
            matrix_list = matrix.Get()
            target_arr = np.asarray(matrix_list, dtype=np.float32).flatten()
            targetTransform = zExtTransform()
            targetTransform.updateTransformFromFloatArray(target_arr)
            bool_reach = zExtRobotModule.ext_zTsRobot_inverseKinematics(ctypes.byref(self.robot), ctypes.byref(targetTransform))
            return bool_reach
    
    def color_target(self,reach):
            if reach:
                # set target color as blue
                self.prim_target.GetAttribute('primvars:displayColor').Set([Gf.Vec3f(0,0,1)])
            if not reach:
                # set target color as red
                self.prim_target.GetAttribute('primvars:displayColor').Set([Gf.Vec3f(1,0,0)])
 
    def move_target_transform(self,new_transform):
        xform = UsdGeom.Xformable(self.prim_target)
        transform = xform.MakeMatrixXform()
        transform.Set(new_transform)

    def on_compute_targets(self):
        targetsCount = ctypes.c_int(0)
        zExtRobotModule.ext_zTsRobot_computeTargetzTsRHWC(ctypes.byref(self.robot), ctypes.byref(targetsCount))
        print("targetsCount",targetsCount)

        targets_list = [zExtTransform() for i in range(targetsCount.value)]
        reachability_list = [False for i in range(targetsCount.value)]
        types_list = [0 for i in range(targetsCount.value)]
        angles_list = [0 for i in range(targetsCount.value)]
            
        targets = (zExtTransform*targetsCount.value)(*targets_list)
        targetReachability = (ctypes.c_bool*targetsCount.value)(*reachability_list)
        targetsTypes = (ctypes.c_int*targetsCount.value)(*types_list)
        angles = (ctypes.c_float*targetsCount.value)(*angles_list)

        zExtRobotModule.ext_zTsRobot_getTargets(ctypes.byref(self.robot),targets,targetReachability, targetsTypes,angles)

        self.max_frame = targetsCount.value
        self.targets_num = targetsCount.value
        self.all_targets.clear()
        for i in range(targetsCount.value):
            self.all_targets.append(targets[i].getGfMatrixTransformed())
            print("targets",i,targets[i].getGfMatrixTransformed())
    
    def on_initialize_transform(self):
        self.current_frame = 0
        #model._restore_default()
        self.joint1_slider._restore_default()
        self.joint2_slider._restore_default()
        self.joint3_slider._restore_default()
        self.joint4_slider._restore_default()
        self.joint5_slider._restore_default()
        self.joint6_slider._restore_default()

        joint_rotation = [self.robot.robotJointRotationHome[0], self.robot.robotJointRotationHome[1],
                          self.robot.robotJointRotationHome[2], self.robot.robotJointRotationHome[3],
                          self.robot.robotJointRotationHome[4], self.robot.robotJointRotationHome[5]]
        joint_rotation_ctypes = (ctypes.c_float * 6)(*joint_rotation)
        zExtRobotModule.ext_zTsRobot_forwardKinematics(ctypes.byref(self.robot), joint_rotation_ctypes)
        self.update_joints_transform()

    def on_change_joint_rotation(self):
        if(self.bool_widget_fk.model.get_value()):
            self.compute_forwardKinematics()
            self.update_joints_transform()

    def on_change_target(self):
        reached = self.compute_inverseKinematics()
        self.update_joints_transform()
        self.update_joints_rotation_slider()
        if reached:
            print("robot is reached")
        self.color_target(reached)
        #flip_cutter(3) 
        
    def on_next_frame(self):
        self.current_frame += 1
        if self.current_frame > self.max_frame - 1:
            self.current_frame = self.max_frame - 1
        self.frame_slider.model.set_value(self.current_frame)

    def on_previous_frame(self):
        self.current_frame += 1
        if self.current_frame < 0:
            self.current_frame = 0
        self.frame_slider.model.set_value(self.current_frame)

    def on_change_frame(self):
        self.current_frame = self.frame_slider.model.get_value_as_int()
        self.move_target_transform(self.all_targets[self.current_frame])
        print("current target",self.current_frame,self.all_targets[self.current_frame])
        self.on_change_target()

    def on_play_frame(self):
            #self.timeline = omni.timeline.get_timeline_interface()
            self.timeline.play()
            timecode = self.timeline.get_current_time() * self.timeline.get_time_codes_per_seconds()            
            print(timecode)
            #event_stream = z_timeline[0].get_timeline_event_stream()
            #subscription = event_stream.create_subscription_to_pop(on_update)
 
    def on_pause_frame(self):
        self.timeline.pause()

    def on_create_animation(self):
        self.timeline = omni.timeline.get_timeline_interface() 
        FPS = 24           
        for i in range(self.targets_num):
            xform = UsdGeom.Xformable(self.prim_target)
            transform = xform.MakeMatrixXform()
            transform.Set(time=i*FPS, value = self.all_targets[i])

    def on_listening(self):
        self.bool_listening = not self.bool_listening
        if(self.bool_listening):
            pass
        if(not self.bool_listening):
            pass


    def _build_fn(self):      
        with ui.ScrollingFrame(name="window_bg",horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF):
            with ui.VStack(height=0):
                self._build_title()

                with ui.CollapsableFrame("1. initialize".upper(), name="group",build_header_fn=self._build_collapsable_header):
                    with ui.VStack(height=0, spacing=SPACING):
                        ui.Spacer(height=1)
                        ui.Button("Reset Joints & EE PrimMesh !!!",clicked_fn = self.on_reset_primMesh)
                        CustomPathButtonWidget(label="Read Path",path="/data/meshes/fabMesh",btn_label="Set_fabMesh",btn_callback=self.on_read_fabMesh)
                        with ui.HStack(height=10): 
                            CustomComboboxWidget(label="Robot Type",options=["ABB_IRB_4600_255", "KUKA_KR30(WIP)", "NACHI_MZ07(WIP)"])
                        ui.Button("Bake_fabMeshes")
                          
                with ui.CollapsableFrame("2. compute".upper(), name="group", build_header_fn=self._build_collapsable_header):
                    with ui.VStack(height=0, spacing=SPACING):
                        ui.Spacer(height=3)                        
                        self.bool_widget_fk = CustomBoolWidget(label="FK", default_value=False)
                        ui.Spacer(height=3) 
                        self.joint1_slider = CustomSliderWidget(min=self.robot.robotJointRotationMin[0], max=self.robot.robotJointRotationMax[0], 
                                                num_type = "float",label="J1_Rotation", display_range = True, default_val=self.robot.robotJointRotationHome[0])
                        self.joint1_slider.model.add_value_changed_fn(self.on_change_joint_rotation)
                        
                        self.joint2_slider = CustomSliderWidget(min=self.robot.robotJointRotationMin[1], max=self.robot.robotJointRotationMax[1], 
                                                num_type = "float",label="J2_Rotation", display_range = True, default_val=self.robot.robotJointRotationHome[1])
                        self.joint2_slider.model.add_value_changed_fn(self.on_change_joint_rotation)

                        self.joint3_slider = CustomSliderWidget(min=self.robot.robotJointRotationMin[2], max=self.robot.robotJointRotationMax[2], 
                                                num_type = "float",label="J3_Rotation", display_range = True, default_val=self.robot.robotJointRotationHome[2])
                        self.joint3_slider.model.add_value_changed_fn(self.on_change_joint_rotation)

                        self.joint4_slider = CustomSliderWidget(min=self.robot.robotJointRotationMin[3], max=self.robot.robotJointRotationMax[3], 
                                                num_type = "float",label="J4_Rotation", display_range = True, default_val=self.robot.robotJointRotationHome[3])
                        self.joint4_slider.model.add_value_changed_fn(self.on_change_joint_rotation)

                        self.joint5_slider = CustomSliderWidget(min=self.robot.robotJointRotationMin[4], max=self.robot.robotJointRotationMax[4], 
                                                num_type = "float",label="J5_Rotation", display_range = True, default_val=self.robot.robotJointRotationHome[4])
                        self.joint5_slider.model.add_value_changed_fn(self.on_change_joint_rotation)

                        self.joint6_slider = CustomSliderWidget(min=self.robot.robotJointRotationMin[5], max=self.robot.robotJointRotationMax[5], 
                                                num_type = "float",label="J6_Rotation", display_range = True, default_val=self.robot.robotJointRotationHome[5])
                        self.joint6_slider.model.add_value_changed_fn(self.on_change_joint_rotation)

                        self.frame_slider = CustomSliderWidget(min=0, max=round(self.max_frame-1), num_type = "int",label="Cur_Frame", 
                                                      display_range = True, default_val=0)
                        self.frame_slider.model.add_value_changed_fn(self.on_change_frame)
                        
                        with ui.HStack(height=10): 
                            ui.Button("Initialize_target",clicked_fn= self.on_initialize_transform)
                            ui.Button("Update_target",clicked_fn = self.on_change_target)
                            ui.Button("Compute_target",clicked_fn = self.on_compute_targets)

                with ui.CollapsableFrame("3. animation".upper(), name="group",
                                build_header_fn=self._build_collapsable_header):
                    with ui.VStack(height=0, spacing=SPACING):
                        with ui.HStack(height=10):
                            ui.Button("Previous_frame",clicked_fn= self.on_previous_frame) 
                            ui.Button("Play",clicked_fn = self.on_play_frame)
                            ui.Button("Pause",clicked_fn = self.on_pause_frame)
                            ui.Button("Next_frame",clicked_fn= self.on_next_frame)
                        ui.Button("Create_Animation",clicked_fn = self.on_create_animation)                                               

                with ui.CollapsableFrame("4. export".upper(), name="group",
                                build_header_fn=self._build_collapsable_header):
                    with ui.VStack(height=0, spacing=SPACING):
                        ui.Spacer(height=3) 
                        CustomPathButtonWidget(label="Export Path",path=".../export/gcode.txt",btn_label="Export",btn_callback=self.on_export_btn_click)
                
                ui.Spacer(height=10)