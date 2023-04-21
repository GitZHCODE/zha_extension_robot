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
        boolean = [False]
        myListener = [None]

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
        #------------------------------------------------------------------------------------
        currentFrame = [0]
        maxFrame = [0]
        allMatrix = []        

        with open('C:/Users/Ling.Mao/source/repos/kit-extension-template/exts/omni.robot/meshes/jointMatrix3.txt', 'r') as file:
            content = file.read()
            double_str_list = content.split(',')     
            double_list = []
            for x in double_str_list:
                try:
                    double_list.append(float(x))
                except ValueError:
                    pass

        maxFrame[0] = len(double_list)/(16*6)

        for i in range(0, len(double_list), 16):
            allMatrix.append(Gf.Matrix4d(double_list[i],double_list[i+1],double_list[i+2],double_list[i+3],
                                        double_list[i+4],double_list[i+5],double_list[i+6],double_list[i+7],
                                        double_list[i+8],double_list[i+9],double_list[i+10],double_list[i+11],
                                        double_list[i+12],double_list[i+13],double_list[i+14],double_list[i+15]))
        #------------------------------------------------------------------------------------
        # Create an instance of the zExtRobot struct
        myRobot = zExtRobot()
        jsonFileSTR = "C:/Users/Ling.Mao/Desktop/robot/ABB_IRB_4600_255.json"
        jsonCStr = ctypes.c_char_p(jsonFileSTR.encode())
        #fabMeshDir = "C:/Users/heba.eiz/source/repos/GitZHCODE/zspace_alice/ALICE_PLATFORM/x64/Release/EXE/data/ABB_IRB_4600_255/meshes"
        #fabMeshDirCStr = ctypes.c_char_p(fabMeshDir.encode())
        
        zExtRobotModule.ext_zTsRobot_createFromFile(ctypes.byref(myRobot),jsonCStr)
        ee_transform = zExtTransform() 
        eematrix = [  [1, 0, 0, 0],  [0, 1, 0, 0],  [0, 0, 1, 0],  [0, 0, -0.994,1]]
        ee_transform.updateTransformFromListOfLists(eematrix)
        zExtRobotModule.ext_zTsRobot_setEndEffector(ctypes.byref(myRobot), ctypes.byref(ee_transform))
        #------------------------------------------------------------------------------        
        def reset_primMesh():             
            stage = omni.usd.get_context().get_stage()
            prim_J1[0] = stage.GetPrimAtPath(prim_J1_path[0])
            prim_J2[0] = stage.GetPrimAtPath(prim_J2_path[0])
            prim_J3[0] = stage.GetPrimAtPath(prim_J3_path[0])
            prim_J4[0] = stage.GetPrimAtPath(prim_J4_path[0])
            prim_J5[0] = stage.GetPrimAtPath(prim_J5_path[0])
            prim_J6[0] = stage.GetPrimAtPath(prim_J6_path[0])
            prim_EE[0] = stage.GetPrimAtPath(prim_EE_path[0])
            prim_target[0] = stage.GetPrimAtPath(prim_target_path[0])    

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

        def update_joints_rotation_slider(M1,M2,M3,M4,M5,M6):
            rotate = myRobot.robotJointRotation
            M1.model.set_value(rotate[0])
            M2.model.set_value(rotate[1])
            M3.model.set_value(rotate[2])
            M4.model.set_value(rotate[3])
            M5.model.set_value(rotate[4])
            M6.model.set_value(rotate[5])

        def compute_forwardKinematics(M1,M2,M3,M4,M5,M6):
            joint_rotation = [M1.model.get_value_as_float(), M2.model.get_value_as_float(),M3.model.get_value_as_float(),
                              M4.model.get_value_as_float(), M5.model.get_value_as_float(),M6.model.get_value_as_float()]
            joint_rotation_ctypes = (ctypes.c_float * 6)(*joint_rotation)
            zExtRobotModule.ext_zTsRobot_forwardKinematics(ctypes.byref(myRobot), joint_rotation_ctypes)

        def compute_inverseKinematics():
            xform_target = UsdGeom.Xformable(prim_target[0])
            matrix = xform_target.MakeMatrixXform()
            matrix_list = matrix.Get()
            target_arr = np.asarray(matrix_list, dtype=np.float32).flatten()

            targetTransform = zExtTransform()
            targetTransform.updateTransformFromFloatArray(target_arr)
            zExtRobotModule.ext_zTsRobot_inverseKinematics(ctypes.byref(myRobot),ctypes.byref(targetTransform))

        def color_target(reached):
            if reached:
                # set target color as blue
                prim_target[0].GetAttribute('primvars:displayColor').Set([Gf.Vec3f(0,0,1)])
            if not reached:
                # set target color as red
                prim_target[0].GetAttribute('primvars:displayColor').Set([Gf.Vec3f(1,0,0)])

        def flip_cutter(int_type):
            cutter_prim = stage.GetPrimAtPath(prim_cutter_path[0])
            print(cutter_prim.GetAttribute('xformOp:rotateXYZ').Get())
            if int_type == 0:
                cutter_prim.GetAttribute('xformOp:rotateXYZ').Set(Gf.Vec3d(0,0,0))
            if int_type == 1:
                cutter_prim.GetAttribute('xformOp:rotateXYZ').Set(Gf.Vec3d(0,90,0))
            if int_type == 2:
                cutter_prim.GetAttribute('xformOp:rotateXYZ').Set(Gf.Vec3d(0,180,0))
            if int_type == 3:
                cutter_prim.GetAttribute('xformOp:rotateXYZ').Set(Gf.Vec3d(0,270,0))

        def on_initialize_transform(model,M1,M2,M3,M4,M5,M6):
            currentFrame[0] = 0
            model.set_value(0)
            M1._restore_default()
            M2._restore_default()
            M3._restore_default()
            M4._restore_default()
            M5._restore_default()
            M6._restore_default()

            joint_rotation = [myRobot.robotJointRotationHome[0], myRobot.robotJointRotationHome[1],myRobot.robotJointRotationHome[2],
                              myRobot.robotJointRotationHome[3],myRobot.robotJointRotationHome[4],myRobot.robotJointRotationHome[5]]
            joint_rotation_ctypes = (ctypes.c_float * 6)(*joint_rotation)
            zExtRobotModule.ext_zTsRobot_forwardKinematics(ctypes.byref(myRobot), joint_rotation_ctypes)
            update_joints_transform()

        def on_change_joint_rotation(bool_model,M1,M2,M3,M4,M5,M6):
            if(bool_model.get_value()):
                compute_forwardKinematics(M1,M2,M3,M4,M5,M6)
                update_joints_transform()

        def on_change_target(M1,M2,M3,M4,M5,M6):
            compute_inverseKinematics()
            update_joints_transform()
            update_joints_rotation_slider(M1,M2,M3,M4,M5,M6)
            color_target(False)
            flip_cutter(3)            

        def on_next_frame(model):
            value = currentFrame[0] + 1
            if value > maxFrame[0]-1:
                value = maxFrame[0]-1
            currentFrame[0] = value
            model.set_value(currentFrame[0])

        def on_previous_frame(model):
            value = currentFrame[0] - 1
            if value < 0:
                value = 0
            currentFrame[0] = value
            model.set_value(currentFrame[0])

        def on_change_frame(model):
            currentFrame[0] = model.get_value_as_int()
            value = currentFrame[0]
            xform_1 = UsdGeom.Xformable(prim_J1[0])
            transform_1 = xform_1.MakeMatrixXform()
            transform_1.Set(allMatrix[value*6+0])
            
            xform_2 = UsdGeom.Xformable(prim_J2[0])
            transform_2 = xform_2.MakeMatrixXform()
            transform_2.Set(allMatrix[value*6+1])

            xform_3 = UsdGeom.Xformable(prim_J3[0])
            transform_3 = xform_3.MakeMatrixXform()
            transform_3.Set(allMatrix[value*6+2])

            xform_4 = UsdGeom.Xformable(prim_J4[0])
            transform_4 = xform_4.MakeMatrixXform()
            transform_4.Set(allMatrix[value*6+3])

            xform_5 = UsdGeom.Xformable(prim_J5[0])
            transform_5 = xform_5.MakeMatrixXform()
            transform_5.Set(allMatrix[value*6+4])

            xform_6 = UsdGeom.Xformable(prim_J6[0])
            transform_6 = xform_6.MakeMatrixXform()
            transform_6.Set(allMatrix[value*6+5])

            xform_EE = UsdGeom.Xformable(prim_EE[0])
            transform_EE = xform_EE.MakeMatrixXform()
            transform_EE.Set(allMatrix[value*6+5])

        def on_play_animation(model):
            '''
            app_window = omni.ui.get_app_window()
            frame_count = 0
            currentFrame[0] = 0
            while True:
                app_window.sync_frame()
                frame_count += 1
                if(frame_count%100 == 0):
                    on_next_frame(model)
            '''
            #end_time = UsdUtils.ComputeStageMinMaxTime(stage)
            #print(end_time)
            #anim_player = omni.anim.get_anim_player(stage)
            #anim_player.play()
            #anim_start_time = anim_player.get_start_time()
            #anim_end_time = anim_player.get_end_time()
            #print(f"Animation start time: {anim_start_time}, end time: {anim_end_time}")
            
            currentFrame[0] = 0
            for i in range(round(maxFrame[0])):
                on_next_frame(model)
                
                #omni.usd.utils.wait_for_condition(lambda: True, 1)
                #time.sleep(1)

        def on_test():
            print("test!")           
        
        def on_listening(boolean,Listener,robot,M1,M2,M3,M4,M5,M6):            
            boolean[0] = not boolean[0]
            if(boolean[0]):
                Listener[0] = zListener(prim_target[0], callback = on_test)
                print("listening",len(Listener),Listener[0])
            if(not boolean[0]):
                if Listener[0]:
                    Listener[0].destroy()
                Listener[0] = None
                print("no listening",len(Listener),Listener[0])      

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
                        slider_enabled = CustomBoolWidget(label="Slider_Enabled", default_value=False)
                        ui.Spacer(height=3) 
                        J1 = CustomSliderWidget(min=myRobot.robotJointRotationMin[0], max=myRobot.robotJointRotationMax[0], 
                                                num_type = "float",label="J1_Rotation", display_range = True, default_val=myRobot.robotJointRotationHome[0])
                        J1.model.add_value_changed_fn(lambda m : on_change_joint_rotation(slider_enabled,J1,J2,J3,J4,J5,J6))
                        
                        J2 = CustomSliderWidget(min=myRobot.robotJointRotationMin[1], max=myRobot.robotJointRotationMax[1], 
                                                num_type = "float",label="J2_Rotation", display_range = True, default_val=myRobot.robotJointRotationHome[1])
                        J2.model.add_value_changed_fn(lambda m : on_change_joint_rotation(slider_enabled,J1,J2,J3,J4,J5,J6))

                        J3 = CustomSliderWidget(min=myRobot.robotJointRotationMin[2], max=myRobot.robotJointRotationMax[2], 
                                                num_type = "float",label="J3_Rotation", display_range = True, default_val=myRobot.robotJointRotationHome[2])
                        J3.model.add_value_changed_fn(lambda m : on_change_joint_rotation(slider_enabled,J1,J2,J3,J4,J5,J6))

                        J4 = CustomSliderWidget(min=myRobot.robotJointRotationMin[3], max=myRobot.robotJointRotationMax[3], 
                                                num_type = "float",label="J4_Rotation", display_range = True, default_val=myRobot.robotJointRotationHome[3])
                        J4.model.add_value_changed_fn(lambda m : on_change_joint_rotation(slider_enabled,J1,J2,J3,J4,J5,J6))

                        J5 = CustomSliderWidget(min=myRobot.robotJointRotationMin[4], max=myRobot.robotJointRotationMax[4], 
                                                num_type = "float",label="J5_Rotation", display_range = True, default_val=myRobot.robotJointRotationHome[4])
                        J5.model.add_value_changed_fn(lambda m : on_change_joint_rotation(slider_enabled,J1,J2,J3,J4,J5,J6))

                        J6 = CustomSliderWidget(min=myRobot.robotJointRotationMin[5], max=myRobot.robotJointRotationMax[5], 
                                                num_type = "float",label="J6_Rotation", display_range = True, default_val=myRobot.robotJointRotationHome[5])
                        J6.model.add_value_changed_fn(lambda m : on_change_joint_rotation(slider_enabled,J1,J2,J3,J4,J5,J6))

                        CurFrame = CustomSliderWidget(min=0, max=round(maxFrame[0]-1), num_type = "int",label="Cur_Frame", 
                                                      display_range = True, default_val=0)
                        CurFrame.model.add_value_changed_fn(lambda m : on_change_frame(m))

                        

                        ui.Button("Initialize_Joints_Transform",clicked_fn= lambda: on_initialize_transform(CurFrame.model,J1,J2,J3,J4,J5,J6))
                        with ui.HStack(height=10):                            
                            ui.Button("Previous_Frame",clicked_fn= lambda: on_previous_frame(CurFrame.model))
                            ui.Button("Play_Animation",clicked_fn = lambda: on_play_animation(CurFrame.model))
                            ui.Button("Next_Frame",clicked_fn= lambda: on_next_frame(CurFrame.model))
                        ui.Button("Update Target",clicked_fn = lambda: on_change_target(J1,J2,J3,J4,J5,J6))
                        ui.Button("Listening",clicked_fn = lambda: on_listening(boolean,myListener,myRobot,J1,J2,J3,J4,J5,J6))
                        
                        

                with ui.CollapsableFrame("output".upper(), name="group",
                                build_header_fn=self._build_collapsable_header):
                    with ui.VStack(height=0, spacing=SPACING):
                        ui.Spacer(height=3) 
                        CustomPathButtonWidget(label="Export Path",path=".../export/mesh1.usd",btn_label="Export",btn_callback=self.on_export_btn_click)
                
                ui.Spacer(height=10)
        