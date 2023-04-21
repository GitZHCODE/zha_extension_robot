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
        self.frame.set_build_fn(self._build_fn)
        self.listener = None

    def destroy(self):
        # Destroys all the children
        super().destroy()

    @property
    def label_width(self):
        return self.__label_width

    @label_width.setter
    def label_width(self, value):
        self.__label_width = value
        self.frame.rebuild()

    def on_export_btn_click(self, path):
        dialog = MessageDialog(
            title="Export Dialog",
            message=f"Export file with path: {path}",
            disable_cancel_button=True,
            ok_handler=lambda dialog: dialog.hide(),
        )
        dialog.show()

    def _build_title(self):
        with ui.VStack():
            ui.Spacer(height=2)
            ui.Label("ZHA Robot Extension - 0.0.1", name="window_title")
            ui.Spacer(height=5)

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
        bool_listening = [False]
        stage_listener = [None]
        stage_event = [None]
        currentFrame = [0]
        maxFrame = [100]
        all_target = []
        z_timeline = [None]
        #------------------------------------------------------------------------------------
        myRobot = zExtRobot()
        
        jsonFileSTR = f"{EXTENSION_FOLDER_PATH}/data/ABB_IRB_4600_255.json"
        jsonCStr = ctypes.c_char_p(jsonFileSTR.encode())        
        zExtRobotModule.ext_zTsRobot_createFromFile(ctypes.byref(myRobot),jsonCStr)

        ee_transform = zExtTransform() 
        eematrix = [  [1, 0, 0, 0],  [0, 1, 0, 0],  [0, 0, 1, 0],  [0, 0, -0.994,1]]
        ee_transform.updateTransformFromListOfLists(eematrix)
        zExtRobotModule.ext_zTsRobot_setEndEffector(ctypes.byref(myRobot), ctypes.byref(ee_transform))
        
        robotBaseMatrix = [ [0.965926,0,-0.258819,0], [0,1,0,0], [0.258819,0,0.965926,0], [0,0,0,1]]
        robot_transform = zExtTransform()
        robot_transform.updateTransformFromListOfLists(robotBaseMatrix)
        zExtRobotModule.ext_zTsRobot_setRobotBasePlane(ctypes.byref(myRobot), ctypes.byref(robot_transform))        

        homePlane_Matrix = [[ 1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [1.8, -0.25, 0, 1] ]
        homePlane_transform = zExtTransform()
        homePlane_transform.updateTransformFromListOfLists(homePlane_Matrix)
        zExtRobotModule.ext_zTsRobot_setRobotHomePlane(ctypes.byref(myRobot), ctypes.byref(homePlane_transform))

        #fabMeshDir = f"{EXTENSION_FOLDER_PATH}/data/meshes/fabMesh"
        #fabMeshDirCStr = ctypes.c_char_p(fabMeshDir.encode())
        #zExtRobotModule.ext_zTsRobot_setFabricationMeshJSONFromDir(ctypes.byref(myRobot),fabMeshDirCStr)

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
        def on_reset_primMesh():             
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

        def on_read_fabMesh(path):
            fabMeshDir = f"{EXTENSION_FOLDER_PATH}{path}"
            fabMeshDirCStr = ctypes.c_char_p(fabMeshDir.encode())
            zExtRobotModule.ext_zTsRobot_setFabricationMeshJSONFromDir(ctypes.byref(myRobot),fabMeshDirCStr)
 
        def on_bake_fabPrimMesh():
            fabMeshArray = zExtMeshArray()
            zExtRobotModule.ext_zTsRobot_getFabricationMeshes(ctypes.byref(myRobot),ctypes.byref(fabMeshArray))
            fabMeshes = (zExtMesh * (fabMeshArray.arrayCount))()
            zExtMeshModule.ext_meshUtil_getMeshsFromMeshArray(ctypes.byref(fabMeshArray), fabMeshes)

            fabMeshNum = fabMeshArray.arrayCount
            
            for i in range(fabMeshNum):
                #points_1d = (ctypes.c_float * (fabMeshes[i].vCount * 3))()
                #color_1d = (ctypes.c_float * (fabMeshes[i].vCount * 4))()
                #faceCount = (ctypes.c_int * (fabMeshes[i].fCount))()
                #faceConnect = (ctypes.c_int * (fabMeshes[i].fCount))()
            
                #zExtMeshModule.ext_meshUtil_getMeshPosition(ctypes.byref(fabMeshes[i]), points_1d, color_1d)
                #zExtMeshModule.ext_meshUtil_getMeshFaceCount(ctypes.byref(fabMeshes[i]), faceCount)
                #zExtMeshModule.ext_meshUtil_getMeshFaceConnect(ctypes.byref(fabMeshes[i]), faceConnect)

                points = []
                #for k in range(0,len(points_1d),3):
                    #points.append(Gf.Vec3f(points_1d[k],points_1d[k+1],points_1d[k+2]))

                points = [(-12.188835, -4.214836, 16.778866), (-11.555672, -2.3264694, 12.584152), (5.3809586, 7.331726, 16.778866), (7.2693253, 6.6985474, 12.584152), (3.828148, 7.883209, 20.973587), (-12.740318, -5.7676544, 20.973587), (-13.230537, -7.068802, 25.168304), (2.52697, 8.373413, 25.168304), (1.3935928, 8.822762, 29.363018), (-13.679878, -8.202194, 29.363018), (-14.108818, -9.251694, 33.557735), (0.34409332, 9.251694, 33.557735), (9.40934, 6.0040665, 8.389435), (-10.861183, -0.18644714, 8.389435), (-10.125877, 2.1213531, 4.1947174), (11.71714, 5.2687607, 4.1947174), (14.108818, 4.5130234, 0), (-9.37014, 4.5130234, 0)]
                faceCount = [3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3]
                faceConnect = [0, 2, 1, 2, 3, 1, 3, 13, 1, 3, 12, 13, 15, 17, 14, 15, 16, 17, 13, 12, 14, 12, 15, 14, 10, 11, 9, 11, 8, 9, 8, 6, 9, 8, 7, 6, 4, 0, 5, 4, 2, 0, 6, 7, 5, 7, 4, 5]

                stage = omni.usd.get_context().get_stage()
                prim_mesh = stage.DefinePrim(f"/fabMesh{i}", 'Mesh')                

                point_attr = prim_mesh.GetAttribute('points')
                point_attr.Set(points)
                face_vertex_counts_attr = prim_mesh.GetAttribute('faceVertexCounts')
                face_vertex_counts_attr.Set(faceCount)
                face_vertex_indices_attr = prim_mesh.GetAttribute('faceVertexIndices')
                face_vertex_indices_attr.Set(faceConnect)

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
        
        def color_target(reach):
            if reach:
                # set target color as blue
                prim_target[0].GetAttribute('primvars:displayColor').Set([Gf.Vec3f(0,0,1)])
            if not reach:
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
            angles_list = [0 for i in range(targetsCount.value)]
            
            targets = (zExtTransform*targetsCount.value)(*targets_list)
            targetReachability = (ctypes.c_bool*targetsCount.value)(*reachability_list)
            targetsTypes = (ctypes.c_int*targetsCount.value)(*types_list)
            angles = (ctypes.c_float*targetsCount.value)(*angles_list)

            zExtRobotModule.ext_zTsRobot_getTargets(ctypes.byref(myRobot),targets,targetReachability, targetsTypes,angles)

            maxFrame[0] = targetsCount.value
            print(maxFrame[0])
            
            all_target.clear()
            for i in range(targetsCount.value):
                all_target.append(targets[i].getGfMatrixTransformed())
                print("targets",i,targets[i].getGfMatrixTransformed())
                           
        def flip_cutter(angle):
           prim_cutter[0].GetAttribute('xformOp:rotateXYZ').Set(Gf.Vec3d(0,angle,0))

        def on_initialize_transform(frame_slider,M1,M2,M3,M4,M5,M6):
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

        def on_next_frame(frame_slider):
            value = currentFrame[0] + 1
            if value > maxFrame[0]-1:
                value = maxFrame[0]-1
            currentFrame[0] = value
            frame_slider.set_value(currentFrame[0])

        def on_previous_frame(frame_slider):
            value = currentFrame[0] - 1
            if value < 0:
                value = 0
            currentFrame[0] = value
            frame_slider.set_value(currentFrame[0])

        def on_change_frame(frame_slider,M1,M2,M3,M4,M5,M6):
            currentFrame[0] = frame_slider.model.get_value_as_int()
            move_target_transform(all_target[currentFrame[0]])
            print("current target",currentFrame[0],all_target[currentFrame[0]])
            on_change_target(M1,M2,M3,M4,M5,M6)

        def on_update(event):
            if event == omni.timeline._timeline.TimelineEvent.TIME_CHANGED:
               print("ok")

        def on_play_frame(frame_slider):
            z_timeline[0] = omni.timeline.get_timeline_interface()
            z_timeline[0].play()
            timecode = z_timeline[0].get_current_time() * z_timeline[0].get_time_codes_per_seconds()            
            print(timecode)
            #event_stream = z_timeline[0].get_timeline_event_stream()
            #subscription = event_stream.create_subscription_to_pop(on_update)
 
        def on_pause_frame(frame_slider):
            z_timeline[0].pause()

        def on_create_animation(frame_slider):
            timeline = omni.timeline.get_timeline_interface() 
            FPS = 24
           
            for i in range(len(all_target)):
                xform = UsdGeom.Xformable(prim_target[0])
                transform = xform.MakeMatrixXform()
                transform.Set(time=i*FPS, value = all_target[i])

                #-------------------------------------------------------------------
                matrix_list = all_target[i]
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
                print("Cur_Matrix[0]",Cur_Matrix[0])

                #-------------------------------------------------------------------
                #xform_1 = UsdGeom.Xformable(prim_J1[0])
                #transform_1 = xform_1.MakeMatrixXform()
                #transform_1.Set(time=i*FPS, value = Cur_Matrix[0]) 
                #xform_2 = UsdGeom.Xformable(prim_J2[0])
                #transform_2 = xform_2.MakeMatrixXform()
                #transform_2.Set(time=i*FPS, value = Cur_Matrix[1])
                #xform_3 = UsdGeom.Xformable(prim_J3[0])
                #transform_3 = xform_3.MakeMatrixXform()
                #transform_3.Set(time=i*FPS, value = Cur_Matrix[2])
                #xform_4 = UsdGeom.Xformable(prim_J4[0])
                #transform_4 = xform_4.MakeMatrixXform()
                #transform_4.Set(time=i*FPS, value = Cur_Matrix[3])
                #xform_5 = UsdGeom.Xformable(prim_J5[0])
                #transform_5 = xform_5.MakeMatrixXform()
                #transform_5.Set(time=i*FPS, value = Cur_Matrix[4])
                #xform_6 = UsdGeom.Xformable(prim_J6[0])
                #transform_6 = xform_6.MakeMatrixXform()
                #transform_6.Set(time=i*FPS, value = Cur_Matrix[5])
                xform_EE = UsdGeom.Xformable(prim_EE[0])
                transform_EE = xform_EE.MakeMatrixXform()
                transform_EE.Set(time=i*FPS, value = Cur_Matrix[5])
        
        #--------------------------------------------------------
        def test(path):
            print("test")

        async def on_change_target_async(self, notice: Tf.Notice, sender: Usd.Stage):
            test()

        def on_stage_event(event):
            if event.type == int(omni.usd.StageEventType.SELECTION_CHANGED):
                print("SELECTION_CHANGED")

        def on_objects_changed(self, notice, sender):
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
                #listener[0] = zListener(prim = prim_target[0])
                self.listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, on_objects_changed, stage)
                #event[0] = omni.usd.get_context().get_stage_event_stream()
                #listener = event[0].create_subscription_to_pop(test)
                print("listening",self.listener)
            if(not boolean[0]):    
                #event[0] = None            
                self.listener = None
                print("no listening",self.listener)

        # setup my UI interface window
        with ui.ScrollingFrame(name="window_bg",horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF):
            with ui.VStack(height=0):
                self._build_title()
                
                with ui.CollapsableFrame("1. initialize".upper(), name="group",build_header_fn=self._build_collapsable_header):
                    with ui.VStack(height=0, spacing=SPACING):
                        ui.Spacer(height=1)
                        ui.Button("Reset Joints & EE PrimMesh !!!",clicked_fn = on_reset_primMesh)
                        CustomPathButtonWidget(label="Read Path",path="/data/meshes/fabMesh",btn_label="Set_fabMesh",btn_callback=on_read_fabMesh)
                        with ui.HStack(height=10): 
                            CustomComboboxWidget(label="Robot Type",options=["ABB_IRB_4600_255", "KUKA_KR30(WIP)", "NACHI_MZ07(WIP)"])
                        ui.Button("Bake_fabMeshes",clicked_fn = on_bake_fabPrimMesh)
                          
                with ui.CollapsableFrame("2. compute".upper(), name="group",
                                build_header_fn=self._build_collapsable_header):
                    with ui.VStack(height=0, spacing=SPACING):
                        ui.Spacer(height=3)                        
                        bool_fk = CustomBoolWidget(label="FK", default_value=False)
                        ui.Spacer(height=3) 
                        J1 = CustomSliderWidget(min=myRobot.robotJointRotationMin[0], max=myRobot.robotJointRotationMax[0], 
                                                num_type = "float",label="J1_Rotation", display_range = True, default_val=myRobot.robotJointRotationHome[0])
                        J1.model.add_value_changed_fn(lambda m : on_change_joint_rotation(bool_fk,J1,J2,J3,J4,J5,J6))
                        
                        J2 = CustomSliderWidget(min=myRobot.robotJointRotationMin[1], max=myRobot.robotJointRotationMax[1], 
                                                num_type = "float",label="J2_Rotation", display_range = True, default_val=myRobot.robotJointRotationHome[1])
                        J2.model.add_value_changed_fn(lambda m : on_change_joint_rotation(bool_fk,J1,J2,J3,J4,J5,J6))

                        J3 = CustomSliderWidget(min=myRobot.robotJointRotationMin[2], max=myRobot.robotJointRotationMax[2], 
                                                num_type = "float",label="J3_Rotation", display_range = True, default_val=myRobot.robotJointRotationHome[2])
                        J3.model.add_value_changed_fn(lambda m : on_change_joint_rotation(bool_fk,J1,J2,J3,J4,J5,J6))

                        J4 = CustomSliderWidget(min=myRobot.robotJointRotationMin[3], max=myRobot.robotJointRotationMax[3], 
                                                num_type = "float",label="J4_Rotation", display_range = True, default_val=myRobot.robotJointRotationHome[3])
                        J4.model.add_value_changed_fn(lambda m : on_change_joint_rotation(bool_fk,J1,J2,J3,J4,J5,J6))

                        J5 = CustomSliderWidget(min=myRobot.robotJointRotationMin[4], max=myRobot.robotJointRotationMax[4], 
                                                num_type = "float",label="J5_Rotation", display_range = True, default_val=myRobot.robotJointRotationHome[4])
                        J5.model.add_value_changed_fn(lambda m : on_change_joint_rotation(bool_fk,J1,J2,J3,J4,J5,J6))

                        J6 = CustomSliderWidget(min=myRobot.robotJointRotationMin[5], max=myRobot.robotJointRotationMax[5], 
                                                num_type = "float",label="J6_Rotation", display_range = True, default_val=myRobot.robotJointRotationHome[5])
                        J6.model.add_value_changed_fn(lambda m : on_change_joint_rotation(bool_fk,J1,J2,J3,J4,J5,J6))

                        cur_frame = CustomSliderWidget(min=0, max=round(maxFrame[0]-1), num_type = "int",label="Cur_Frame", 
                                                      display_range = True, default_val=0)
                        cur_frame.model.add_value_changed_fn(lambda m : on_change_frame(cur_frame,J1,J2,J3,J4,J5,J6))
                        
                        with ui.HStack(height=10): 
                            ui.Button("Initialize_target",clicked_fn= lambda: on_initialize_transform(cur_frame,J1,J2,J3,J4,J5,J6))
                            ui.Button("Update_target",clicked_fn = lambda: on_change_target(J1,J2,J3,J4,J5,J6))
                            ui.Button("Compute_target",clicked_fn = lambda: on_compute_targets(cur_frame,J1,J2,J3,J4,J5,J6))

                with ui.CollapsableFrame("3. animation".upper(), name="group",
                                build_header_fn=self._build_collapsable_header):
                    with ui.VStack(height=0, spacing=SPACING):
                        with ui.HStack(height=10):
                            ui.Button("Previous_frame",clicked_fn= lambda: on_previous_frame(cur_frame.model)) 
                            ui.Button("Play",clicked_fn = lambda: on_play_frame(cur_frame))
                            ui.Button("Pause",clicked_fn = lambda: on_pause_frame(cur_frame))
                            ui.Button("Next_frame",clicked_fn= lambda: on_next_frame(cur_frame.model))
                        ui.Button("Create_Animation",clicked_fn = lambda: on_create_animation(cur_frame))
                        ui.Button("Listening",clicked_fn = lambda: on_listening(bool_listening,stage_event,stage_listener))                                                

                with ui.CollapsableFrame("4. export".upper(), name="group",
                                build_header_fn=self._build_collapsable_header):
                    with ui.VStack(height=0, spacing=SPACING):
                        ui.Spacer(height=3) 
                        CustomPathButtonWidget(label="Export Path",path=".../export/gcode.txt",btn_label="Export",btn_callback=self.on_export_btn_click)
                
                ui.Spacer(height=10)
                