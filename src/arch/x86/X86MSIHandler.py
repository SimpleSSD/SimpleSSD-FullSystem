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

from m5.params import *
from Device import PioDevice
from X86LocalApic import X86LocalApic

class X86MSIHandler(PioDevice):
    type = 'X86MSIHandler'
    cxx_header = "arch/x86/msi_handler.hh"

    lapics = VectorParam.X86LocalApic([], "Interrupt Controller")

    pio_addr = Param.Addr(0xFEE00000,
                          "Base address for X86 Message Data Register")
    pio_size = Param.Addr(0x00100000,
                          "Address Range for X86 Message Data Register")
    pio_latency = Param.Latency('10ns', "Delay for PioDevice")
