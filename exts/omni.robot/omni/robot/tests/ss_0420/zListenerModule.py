import omni.usd
from pxr import Sdf, Tf, Usd
from typing import Callable

class zListener:
    
    def __init__(self, prim):
        self.stage_listener = None
        self.usd_context = omni.usd.get_context()
        self.stage: Usd.Stage = self.usd_context.get_stage()        
        self.target_prim = prim
        self.current_prim = None
        self.events = self.usd_context.get_stage_event_stream()
        self.stage_event_delegate = self.events.create_subscription_to_pop(self.on_stage_event)
        #self.update_transform = callback
    
    def destroy(self):        
        if self.stage_listener:
            self.stage_listener.Revoke()
        self.stage_event_delegate.unsubscribe()
        self.stage_event_delegate = None        
        self.events = None
        self.stage_listener = None
        self.target_prim = None
        #self.update_transform = None

    def on_stage_event(self, event):
        if event.type == int(omni.usd.StageEventType.SELECTION_CHANGED):            
            self.current_prim = self.stage.GetPrimAtPath(self.usd_context.get_selection().get_selected_prim_paths()[0])
            if not (self.current_prim == self.target_prim):
                if self.stage_listener:
                    self.stage_listener.Revoke()
                    self.stage_listener = None
                print("not correct prim!",self.current_prim)
                return
            if self.current_prim == self.target_prim:
                #print(self.update_transform)
                try:                    
                    #self.stage_listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self.update_transform, self.stage)
                    self.stage_listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self._update_transform, self.stage)
                except Exception as e:
                    print("Error: ", e)
                print("correct prim!",self.current_prim)

    def _update_transform(self, notice, sender):
        for p in notice.GetChangedInfoOnlyPaths():
                print(p)
                if p == "/abb_robot/target.xformOp:tranform": 
                      print("true",p)
        #print("original method")

   




'''
class zListener:
    
    def __init__(self):
        self._usd_context = omni.usd.get_context()
        self._selection = self._usd_context.get_selection()
        self._selected_paths = []
        self._objects_changed_listener = None
        
        self._stage_event_sub = self._usd_context.get_stage_event_stream().create_subscription_to_pop(self._on_stage_event)
              
    def clean(self):
        # Unsubscribe from stage events
        self._stage_event_sub = None
        # Clear objects changed listener
        self._toggle_objects_changed_listener(False)

    def _on_stage_event(self, evt):
        if evt.type == int(omni.usd.StageEventType.SELECTION_CHANGED):
            self._update_selection()

    def _update_selection(self):
        # Update the selected prim paths
        self._selected_paths = self._selection.get_selected_prim_paths()
        self._toggle_objects_changed_listener(len(self._selected_paths) > 0)
        
    def _toggle_objects_changed_listener(self, toggle: bool):
        if toggle:
            if self._objects_changed_listener is None:
                self._objects_changed_listener = Tf.Notice.Register(
                    Usd.Notice.ObjectsChanged, self._on_objects_changed, self._stage
                )
        else:
            if self._objects_changed_listener is not None:
                self._objects_changed_listener.Revoke()
                self._objects_changed_listener = None

    def _on_objects_changed(self, notice, sender):
        changed_paths_prims = []
        # Get prim paths of changed prims
        changed_paths_prims = [Sdf.Path.GetAbsoluteRootOrPrimPath(i) for i in notice.GetChangedInfoOnlyPaths()]
        print(f"Changed prim paths: {changed_paths_prims}")
'''