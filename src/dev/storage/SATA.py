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

from m5.SimObject import SimObject
from m5.params import *
from PciDevice import PciDevice


class SATAInterface(PciDevice):
    type = 'SATAInterface'
    cxx_header = "dev/storage/sata_interface.hh"
    SSDConfig = Param.String('', "SimpleSSD Configuration File")

    # NAME                      # START DESC
    # PCIe Header
    # See Intel 9 Series Chipset Family Platform Controller Hub Datasheet
    # See Serial ATA Advanced Host Controller Interface Revision 1.3.1
    VendorID = 0x8086           # 00    Intel
    DeviceID = 0x8C80           # 02
    Command = 0x0000            # 04
    Status = 0x0230             # 06    Capability List
    Revision = 0x00             # 08
    ProgIF = 0x01               # 09
    SubClassCode = 0x06         # 0A    AHCI Controller
    ClassCode = 0x01            # 0B    Mass storage controller
    # CacheLineSize             # 0C    Written by system
    # LatencyTimer              # 0D    Master Latency Timer (ZERO)
    # HeaderType                # 0E    Single Function | Header Layout
    # BIST                      # 0F    Built-in Self Test (ZERO)
    BAR0 = 0x00000000           # 10
    BAR1 = 0x00000000           # 14
    BAR2 = 0x00000000           # 18
    BAR3 = 0x00000000           # 1C
    BAR4 = 0x00000000           # 20
    BAR5 = 0x00000000           # 24    AHCI
    # CardbusCIS                # 28    Not supported by NVMe (ZERO)
    SubsystemVendorID = 0x0000  # 2C
    SubsystemID = 0x0000        # 2E
    # ExpansionROM              # 30    Not supported (ZERO)
    CapabilityPtr = 0x80        # 34    First capability pointer
    # Reserved                  # 35
    InterruptLine = 0x0B        # 3C    Interrupt Line
    InterruptPin = 0x01         # 3D    Use INT A
    # MaximumLatency            # 3E    Not supported by NVMe (ZERO)
    # MinimumGrant              # 3F    Not supported by NVMe (ZERO)

    # PMCAP - PCI Power Management Capability
    PMCAPBaseOffset = 0x70      # --    PMCAP capability base
    PMCAPCapId = 0x01           # 70    PMCAP ID
    PMCAPNextCapability = 0x00  # 71    Next capability pointer
    # 72    Device Specific Initialization (No) | Version (1.2)
    PMCAPCapabilities = 0x0003
    PMCAPCtrlStatus = 0x0008    # 74    No Soft Reset

    # MSICAP - Message Signaled Interrupt Capability
    MSICAPBaseOffset = 0x80     # --    MSICAP capability base
    MSICAPCapId = 0x05          # 80    MSICAP ID
    MSICAPNextCapability = 0x70  # 81    Next capability pointer
    MSICAPMsgCtrl = 0x0000      # 82    MSI Message Control
    # MSICAPMsgAddr             # 84    MSI Message Address
    # MSICAPMsgData             # 88    MSI Message Data

    BAR5Size = '1024B'
