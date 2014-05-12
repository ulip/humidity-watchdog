#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  unbenannt.py
#
#  Copyright 2014 Uwe Lippmann <uwe.lippmann@web.de>
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#
#

import monitor_ui
import numpy as np
import sys
import serial
import time
from PyQt4 import QtCore, QtGui
import PyQt4.Qwt5 as Qwt


class MonitorWindow(QtGui.QMainWindow):
    def __init__(self):
        QtGui.QMainWindow.__init__(self)
        # self.ser = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=4)
        self.monDlg = monitor_ui.Ui_monitorWindow()
        self.monDlg.setupUi(self)

        self.readTimer = QtCore.QTimer()
        self.readTimer.start(1000.0)

        self.saveTimer = QtCore.QTimer()
        self.saveTimer.start(1800000) # save file every 30 minutes
        # set up data arrays
        self.numPoints = 100 # 17280 # 24 hours with one point every 5 seconds
        self.t = np.linspace(0, 24, self.numPoints)
        self.relHum = np.zeros(self.numPoints)
        self.tempAir = self.t
        self.curve1 = Qwt.QwtPlotCurve() # make a curve
        self.curve1.attach(self.monDlg.history)
        #self.curve2 = Qwt.QwtPlotCurve()
        #self.curve2.attach(self.monDlg.history)

        self.updateValues()

        self.monDlg.hygroAir.setScaleArc(0.0, 270.0)
        self.monDlg.hygroAir.setRange(30.0, 70.0, 10.0)

        needle = Qwt.QwtDialSimpleNeedle(Qwt.QwtDialSimpleNeedle.Arrow)
        self.monDlg.hygroAir.setNeedle(needle)

        self.connect(self.readTimer, QtCore.SIGNAL('timeout()'), self.updateValues)

    def updateValues(self):
        self.t = np.roll(self.t, -1)
        self.relHum = np.roll(self.relHum, -1)
        self.relHum[-1] = 20 + 20*np.sin(2 * np.pi * self.t[-1] / 24.0)
        #self.tempAir = np.roll(self.tempAir, -1)
        self.curve1.setData(self.t, self.relHum)
        #self.curve2.setData(self.t, self.tempAir)
        self.monDlg.history.replot()
        self.monDlg.thermoAir.setValue(self.tempAir[-1])
        self.monDlg.hygroAir.setValue(self.relHum[-1])

    def getMeasurement(self):
        pass


def main():
    app = QtGui.QApplication(sys.argv)
    monWin = MonitorWindow()
    monWin.show()
    return app.exec_()

if __name__ == '__main__':
    main()


