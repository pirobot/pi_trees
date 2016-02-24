#!/usr/bin/env python3
#
# Copyright 2008 Jose Fonseca
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU Lesser General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
"""
    Adapted for use with the ROS pi_trees library.
     
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2016 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import sys

from gi.repository import Gtk

import xdot
import time

from watchdog.observers import Observer
from watchdog.events import PatternMatchingEventHandler
import threading

class MyDotWindow(xdot.DotWindow):

    def __init__(self):
        xdot.DotWindow.__init__(self)
        self.dotwidget.connect('clicked', self.on_url_clicked)
        
        dotfile = '/home/patrick/.ros/tree.dot'
        
        with open(dotfile, 'r') as myfile:
            dotcode=myfile.read().replace('\n', '')
            
        self.set_dotcode(dotcode, filename=dotfile)
        
        event_handler = myEventHandler(self.dotwidget, patterns="*.dot", ignore_directories=True, case_sensitive=False)
         
        self.observer = Observer()
        self.observer.schedule(event_handler, '/home/patrick/.ros/')
        self.observer.start()
        
        self.connect('delete-event', Gtk.main_quit)
        Gtk.main()
        
    def on_url_clicked(self, widget, url, event):
        dialog = Gtk.MessageDialog(
                parent = self, 
                buttons = Gtk.ButtonsType.OK,
                message_format="%s clicked" % url)
        dialog.connect('response', lambda dialog, response: dialog.destroy())
        dialog.run()
        return True
    
class myEventHandler(PatternMatchingEventHandler):
    def __init__(self, dotwidget, patterns="*.dot", ignore_directories=True, case_sensitive=False):
        super(myEventHandler, self).__init__(patterns="*.dot", ignore_directories=True, case_sensitive=False)

        self.dotwidget = dotwidget
        
        self.lock = threading.Lock()
                
    def dispatch(self, event):
        
        with self.lock:
            self.dotwidget.update()
        
        return PatternMatchingEventHandler.dispatch(self, event)


if __name__ == '__main__':
    try:
        window = MyDotWindow()
    except KeyboardInterrupt:
        window.observer.stop()
        window.observer.join()

