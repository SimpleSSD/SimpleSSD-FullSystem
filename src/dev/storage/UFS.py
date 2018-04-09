# Copyright (C) 2017 CAMELab
#
# This file is part of SimpleSSD.
#
# SimpleSSD is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# SimpleSSD is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with SimpleSSD.  If not, see <http://www.gnu.org/licenses/>.

import sys
from m5.params import *
from m5.proxy import *
from Device import DmaDevice

class UFSInterface(DmaDevice):
    type = 'UFSInterface'
    cxx_header = "dev/storage/ufs_interface.hh"

    pio_addr = Param.Addr("Address for UFS interface registers")
    pio_latency = Param.Latency("10ns", "")
    gic = Param.BaseGic(Parent.any, "Generic Interrupt Controller")
    int_num = Param.UInt32("Interrupt number")
    SSDConfig = Param.String('', "Path to SimpleSSD configuration file.")
