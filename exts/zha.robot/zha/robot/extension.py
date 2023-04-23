__all__ = ["ZhcodeRobotExtension"]

import asyncio
from functools import partial

import omni.ext
import omni.kit.ui
import omni.ui as ui

from .style import WIN_WIDTH, WIN_HEIGHT
from .window import ZhcodeRobotWindow


class ZhcodeRobotExtension(omni.ext.IExt):

    WINDOW_NAME = "ZHA Robot"
    MENU_PATH = f"Window/{WINDOW_NAME}"

    def on_startup(self):
        self._window = ZhcodeRobotWindow(ZhcodeRobotExtension.WINDOW_NAME, width=WIN_WIDTH, height=WIN_HEIGHT)
        '''
        ui.Workspace.set_show_window_fn(ZhcodeRobotExtension.WINDOW_NAME, partial(self.show_window, None))
        editor_menu = omni.kit.ui.get_editor_menu()
        if editor_menu:
            self._menu = editor_menu.add_item(
                ZhcodeRobotExtension.MENU_PATH, self.show_window, toggle=True, value=True
            )
        ui.Workspace.show_window(ZhcodeRobotExtension.WINDOW_NAME)
        '''

    def on_shutdown(self):
        #self._menu = None
        if self._window:
            self._window.destroy()
            self._window = None        
        #ui.Workspace.set_show_window_fn(ZhcodeRobotExtension.WINDOW_NAME, None)

    def _set_menu(self, value):
        editor_menu = omni.kit.ui.get_editor_menu()
        if editor_menu:
            editor_menu.set_value(ZhcodeRobotExtension.MENU_PATH, value)

    async def _destroy_window_async(self):
        await omni.kit.app.get_app().next_update_async()
        if self._window:
            self._window.destroy()
            self._window = None

    def _visiblity_changed_fn(self, visible):
        self._set_menu(visible)
        if not visible:
            asyncio.ensure_future(self._destroy_window_async())

    def show_window(self, menu, value):
        if value:
            self._window = ZhcodeRobotWindow(
                ZhcodeRobotExtension.WINDOW_NAME, width=WIN_WIDTH, height=WIN_HEIGHT)
            self._window.set_visibility_changed_fn(self._visiblity_changed_fn)
        elif self._window:
            self._window.visible = False
