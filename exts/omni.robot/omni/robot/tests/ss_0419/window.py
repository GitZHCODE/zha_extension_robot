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
import carb

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
        bool_listening = [False]
        stage_listener = [None]
        stage_event = [None]
        time = 0

        prim_J1_path = ["/abb_robot/geometry/r_1"]
        prim_J2_path = ["/abb_robot/geometry/r_2"]
        prim_J3_path = ["/abb_robot/geometry/r_3"]
        prim_J4_path = ["/abb_robot/geometry/r_4"]
        prim_J5_path = ["/abb_robot/geometry/r_5"]
        prim_J6_path = ["/abb_robot/geometry/r_6"]
        prim_EE_path = ["/abb_robot/geometry/r_EE"]
        prim_target_path = ["/abb_robot/target"]
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
        currentFrame = [0]
        maxFrame = [100]
        allMatrix = []
        animation_target = [Gf.Matrix4d(0, 1, 0, 0,  1, 0, 0, 0,  0, 0, -1, 0,  1, 0, 0.2,1),
                            Gf.Matrix4d(0, 1, 0, 0,  1, 0, 0, 0,  0, 0, -1, 0,  1, 0, 0.6,1),
                            Gf.Matrix4d(0, 1, 0, 0,  1, 0, 0, 0,  0, 0, -1, 0,  2, 0, 0.6,1),
                            Gf.Matrix4d(0, 1, 0, 0,  1, 0, 0, 0,  0, 0, -1, 0,  2, 0, 0.2,1)]
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
        
        robotBaseMatrix = [ [0.965926,0,-0.258819,0], [0,1,0,0], [0.258819,0,0.965926,0], [0,0,0,1]]
        #robotBaseMatrix = [ [1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]
        robot_transform = zExtTransform()
        robot_transform.updateTransformFromListOfLists(robotBaseMatrix)
        zExtRobotModule.ext_zTsRobot_setRobotBasePlane(ctypes.byref(myRobot), ctypes.byref(robot_transform))

        fabMeshDir = "C:/Users/Ling.Mao/Desktop/robot/cut_mesh"
        fabMeshDirCStr = ctypes.c_char_p(fabMeshDir.encode())
        zExtRobotModule.ext_zTsRobot_setFabricationMeshJSONFromDir(ctypes.byref(myRobot),fabMeshDirCStr)

        homePlane_Matrix = [[ 1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [1.8, -0.25, 0, 1] ]
        homePlane_transform = zExtTransform()
        homePlane_transform.updateTransformFromListOfLists(homePlane_Matrix)
        zExtRobotModule.ext_zTsRobot_setRobotHomePlane(ctypes.byref(myRobot), ctypes.byref(homePlane_transform))

        #fabPlane_Matrix = [[ 1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [1.7, -1, -0.8, 1] ]
        #fabPlane_transform = zExtTransform()
        #fabPlane_transform.updateTransformFromListOfLists(fabPlane_Matrix)
        #zExtRobotModule.ext_zTsRobot_setFabricationPlane(ctypes.byref(myRobot), ctypes.byref(fabPlane_transform))

        #--------------------------------------------
        '''
        prim = stage.GetPrimAtPath("/abb_robot/plane")

        points = prim.GetAttribute('points').Get()                    
        faceVertexCounts = prim.GetAttribute('faceVertexCounts').Get()
        faceVertexIndices  = prim.GetAttribute('faceVertexIndices').Get()
        faceNum = len(faceVertexCounts)
        VertNum = len(points)

        VertexPositions = []
        for i in range(len(points)):
            VertexPositions.append(points[i][0])
            VertexPositions.append(points[i][1])
            VertexPositions.append(points[i][2])   
        VertexPositions_array = (ctypes.c_double * len(VertexPositions))(*VertexPositions)      

        polyCounts = []
        for i in range(faceNum):
            polyCounts.append(faceVertexCounts[i])
        polyCounts_array = (ctypes.c_int * faceNum)(*polyCounts)

        polyConnects = []
        for i in range(len(faceVertexIndices)):
            polyConnects.append(faceVertexIndices[i])
        polyConnects_array = (ctypes.c_int * len(faceVertexIndices))(*polyConnects)

        testPlane = zExtMesh()

        zExtMeshModule.ext_meshUtil_createMeshOBJFromArray(VertexPositions_array,polyCounts_array,polyConnects_array,ctypes.c_int(VertNum),ctypes.c_int(faceNum),ctypes.byref(testPlane))
        #zExtRobotModule.ext_zTsRobot_setFabricationMesh(ctypes.byref(myRobot),ctypes.byref(testPlane),1)
        '''
        #--------------------------------------------

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
            #print("dot",dot)
            #print("angle",angle)
            #print("dotZeroZ",dotZeroZ)
            #print("angleZeroZ",angleZeroZ)
            return reached
        
        def color_target(reached):
            if reached:
                # set target color as blue
                prim_target[0].GetAttribute('primvars:displayColor').Set([Gf.Vec3f(0,0,1)])
            if not reached:
                # set target color as red
                prim_target[0].GetAttribute('primvars:displayColor').Set([Gf.Vec3f(1,0,0)])

        def move_target_transform(new_transform):
            xform = UsdGeom.Xformable(prim_target[0])
            transform = xform.MakeMatrixXform()
            transform.Set(new_transform)

        def on_compute_targets(frame_slider,M1,M2,M3,M4,M5,M6):
            targetsCount = ctypes.c_int(0)
            zExtRobotModule.ext_zTsRobot_computeTargetzTsRHWC(ctypes.byref(myRobot), ctypes.byref(targetsCount))
            print("targetsCount",targetsCount)

            targets_list = [zExtTransform() for i in range(targetsCount.value)]
            reachability_list = [False for i in range(targetsCount.value)]
            types_list = [0 for i in range(targetsCount.value)]
            
            targets = (zExtTransform*targetsCount.value)(*targets_list)
            targetReachability = (ctypes.c_bool*targetsCount.value)(*reachability_list)
            targetsTypes = (ctypes.c_int*targetsCount.value)(*types_list)

            zExtRobotModule.ext_zTsRobot_getTargets(ctypes.byref(myRobot),targets,targetReachability, targetsTypes)

            maxFrame[0] = targetsCount.value
            print(maxFrame[0])
            #frame_slider.set_max(maxFrame[0])
            allMatrix.clear()
            for i in range(targetsCount.value):
                allMatrix.append(targets[i].getGfMatrixTransformed())
                print("targets",i,targets[i].getGfMatrixTransformed())
               
            
        def flip_cutter(angle):
           prim_cutter[0].GetAttribute('xformOp:rotateXYZ').Set(Gf.Vec3d(0,180,0))

        def on_initialize_transform(model,M1,M2,M3,M4,M5,M6):
            currentFrame[0] = 0
            #model._restore_default()
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
            reached = compute_inverseKinematics()
            update_joints_transform()
            update_joints_rotation_slider(M1,M2,M3,M4,M5,M6)
            if reached:
                print("robot is reached")
            color_target(reached)
            #flip_cutter(3)            

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

        def on_change_frame(model,M1,M2,M3,M4,M5,M6):
            currentFrame[0] = model.model.get_value_as_int()
            move_target_transform(allMatrix[currentFrame[0]])
            print("current target",currentFrame[0],allMatrix[currentFrame[0]])
            on_change_target(M1,M2,M3,M4,M5,M6)

        def on_create_animation(frame_slider):
            #timeline = omni.timeline.get_timeline_interface() 
            #timecode = timeline.get_current_time() * timeline.get_time_codes_per_seconds()  
            timeline = omni.timeline.acquire_timeline_interface()
            timecode = timeline.get_time_codes_per_seconds()
            print(timecode)

            #print("len(animation_target)",len(animation_target))
            #for i in range(len(animation_target)):
                #xform = UsdGeom.Xformable(prim_target[0])
                #transform = xform.MakeMatrixXform()
                #transform.Set(time=i*20, value = animation_target[i])

            '''
                matrix_list = animation_target[i]
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
                Cur_Matrix =[]
                for i in range(6):
                    Cur_Matrix.append(myRobot.robotJointTransforms[i].getGfMatrix())
                print(Cur_Matrix)
                
                xform_1 = UsdGeom.Xformable(prim_J1[0])
                transform_1 = xform_1.MakeMatrixXform()
                transform_1.Set(time=i*20, value = Cur_Matrix[0]) 
                xform_2 = UsdGeom.Xformable(prim_J2[0])
                transform_2 = xform_2.MakeMatrixXform()
                transform_2.Set(time=i*20, value = Cur_Matrix[1])
                xform_3 = UsdGeom.Xformable(prim_J3[0])
                transform_3 = xform_3.MakeMatrixXform()
                transform_3.Set(time=i*20, value = Cur_Matrix[2])
                xform_4 = UsdGeom.Xformable(prim_J4[0])
                transform_4 = xform_4.MakeMatrixXform()
                transform_4.Set(time=i*20, value = Cur_Matrix[3])
                xform_5 = UsdGeom.Xformable(prim_J5[0])
                transform_5 = xform_5.MakeMatrixXform()
                transform_5.Set(time=i*20, value = Cur_Matrix[4])
                xform_6 = UsdGeom.Xformable(prim_J6[0])
                transform_6 = xform_6.MakeMatrixXform()
                transform_6.Set(time=i*20, value = Cur_Matrix[5])
                xform_EE = UsdGeom.Xformable(prim_EE[0])
                transform_EE = xform_EE.MakeMatrixXform()
                transform_EE.Set(time=i*20, value = Cur_Matrix[5])
            '''

        def test():
            time += 1
            if (time%1000) == 0:
                print("test")

        async def on_change_target_async(self, notice: Tf.Notice, sender: Usd.Stage):
            test()

        def on_stage_event(event):
            if event.type == int(omni.usd.StageEventType.SELECTION_CHANGED):
                print("SELECTION_CHANGED")

        def on_objects_changed(self, notice: Tf.Notice, sender: Usd.Stage):
            for p in notice.GetChangedInfoOnlyPaths(): 
                if p.IsAbsoluteRootOrPrimPath():
                    print(p)
                    #if p.IsPropertyPath():                         
                        #prim_path = p.GetPrimPath()
                        #print(f"prim path: {prim_path}, prop = {p}")
                        #if p == f"{prim_target_path}.xformOp:tranform": 
                            #print(p)
                            #asyncio.ensure_future(on_change_target_async)

        def on_listening(boolean,event,listener):            
            boolean[0] = not boolean[0]
            if(boolean[0]):
                listener[0] = zListener(prim = prim_target[0])
                #listener[0] = Tf.Notice.Register(Usd.Notice.ObjectsChanged, test, stage)
                #event[0] = omni.usd.get_context().get_stage_event_stream()
                #listener = event[0].create_subscription_to_pop(test)
                print("listening",listener[0])
            if(not boolean[0]):    
                #event[0] = None            
                listener[0] = None
                print("no listening",listener[0])

        # setup my UI interface window
        with ui.ScrollingFrame(name="window_bg",horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF):
            with ui.VStack(height=0):
                self._build_title()
                
                with ui.CollapsableFrame("initialize".upper(), name="group",build_header_fn=self._build_collapsable_header):
                    with ui.VStack(height=0, spacing=SPACING):
                        ui.Spacer(height=1)
                        ui.Button("Reset Joints & EE PrimMesh !!!",clicked_fn = reset_primMesh)
                        with ui.HStack(height=1):
                            ui.Button("Reset fabMesh")
                            ui.Button("Reset workPlane")
                            ui.Button("Reset basePlane")
                        ui.Spacer(height=2)
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

                        cur_frame = CustomSliderWidget(min=0, max=round(maxFrame[0]-1), num_type = "int",label="Cur_Frame", 
                                                      display_range = True, default_val=0)
                        cur_frame.model.add_value_changed_fn(lambda m : on_change_frame(cur_frame,J1,J2,J3,J4,J5,J6))
                        
                        with ui.HStack(height=10): 
                            ui.Button("Initialize_Joints_Transform",clicked_fn= lambda: on_initialize_transform(cur_frame,J1,J2,J3,J4,J5,J6))
                            ui.Button("Update Target",clicked_fn = lambda: on_change_target(J1,J2,J3,J4,J5,J6))
                        ui.Spacer(height=10)

                        ui.Button("Compute Target",clicked_fn = lambda: on_compute_targets(cur_frame,J1,J2,J3,J4,J5,J6))
                        with ui.HStack(height=10):                            
                            ui.Button("Previous_Frame",clicked_fn= lambda: on_previous_frame(cur_frame.model))
                            ui.Button("Create_Animation",clicked_fn = lambda: on_create_animation(cur_frame))
                            ui.Button("Next_Frame",clicked_fn= lambda: on_next_frame(cur_frame.model))
                        ui.Spacer(height=10)
                        
                        ui.Button("Listening",clicked_fn = lambda: on_listening(bool_listening,stage_event,stage_listener))
                                                

                with ui.CollapsableFrame("output".upper(), name="group",
                                build_header_fn=self._build_collapsable_header):
                    with ui.VStack(height=0, spacing=SPACING):
                        ui.Spacer(height=3) 
                        CustomPathButtonWidget(label="Export Path",path=".../export/mesh1.usd",btn_label="Export",btn_callback=self.on_export_btn_click)
                
                ui.Spacer(height=10)
                