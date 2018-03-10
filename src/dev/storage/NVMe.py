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

class NVMeInterface(PciDevice):
    type = 'NVMeInterface'
    cxx_header = "dev/storage/nvme_interface.hh"
    SSDConfig = Param.String('', "Path to SimpleSSD configuration file.")

    # NAME                      # START DESC
    # PCIe Header
    ## See Linux kernel 4.9.32 /drivers/nvme/host/pci.c:2100
    VendorID = 0x144D           # 00    ## For NVMe driver's quirk
    DeviceID = 0x2001           # 02    ## For NVMe driver's quirk
    Command = 0x0000            # 04
    Status = 0x0010             # 06    Capability List
    Revision = 0x00             # 08
    ProgIF = 0x02               # 09    NVM Express
    SubClassCode = 0x08         # 0A    Non-Volatile Memory controller
    ClassCode = 0x01            # 0B    Mass storage controller
    # CacheLineSize             # 0C    Written by system
    # LatencyTimer              # 0D    Master Latency Timer (ZERO)
    # HeaderType                # 0E    Single Function | Header Layout
    # BIST                      # 0F    Built-in Self Test (ZERO)
    BAR0 = 0x00000000           # 10    TYPE = 32bit address space
    BAR1 = 0x00000000           # 14    Should be ZERO
    BAR2 = 0x00000000           # 18    Index/Data pair is not supported
    BAR3 = 0x00000000           # 1C    Not used (RESERVED)
    BAR4 = 0x00000000           # 20    MSI-X Table
    BAR5 = 0x00000000           # 24    MSI-X PBA
    # CardbusCIS                # 28    Not supported by NVMe (ZERO)
    SubsystemVendorID = 0x8086  # 2C    Intel Corporation
    SubsystemID = 0x3704        # 2E    Intel 750 Series NVMe SSD
    # ExpansionROM              # 30    Not supported (ZERO)
    CapabilityPtr = 0x48        # 34    First capability pointer
    # Reserved                  # 35
    InterruptLine = 0x0B        # 3C    Interrupt Line
    InterruptPin = 0x01         # 3D    Use INT A
    # MaximumLatency            # 3E    Not supported by NVMe (ZERO)
    # MinimumGrant              # 3F    Not supported by NVMe (ZERO)

    # MSICAP - Message Signaled Interrupt Capability
    MSICAPBaseOffset = 0x48     # --    MSICAP capability base
    MSICAPCapId = 0x05          # 48    MSICAP ID
    MSICAPNextCapability = 0x60 # 49    Next capability pointer
    MSICAPMsgCtrl = 0x018A      # 4A    MSI Message Control
    # MSICAPMsgAddr             # 4C    MSI Message Address
    # MSICAPMsgUpperAddr        # 50    MSI Message Upper Address
    # MSICAPMsgData             # 54    MSI Message Data
    # MSICAPMaskBits            # 56    MSI Mask Bits
    # MSICAPPendingBits         # 5A    MSI Pending Bits

    # MSIXCAP Message Signaled Interrupt eXtended Capability
    MSIXCAPBaseOffset = 0x60    # --    MSIXCAP capability base
    MSIXCAPCapId = 0x11         # 60    MSIXCAP ID
    MSIXCAPNextCapability = 0x70# 61   Next capability pointer
    MSIXMsgCtrl = 0x01FF        # 62    MSI-X Message Control (512 vectors)
    MSIXTableOffset = 0x00000004# 64   MSI-X Table Offset/BIR (Use BAR4)
    MSIXPbaOffset = 0x00000005  # 68    MSI-X PBA Offset/BIR (Use BAR5)

    # PXCAP - PCI Express Capability
    PXCAPBaseOffset = 0x70      # --    PXCAP capability base
    PXCAPCapId = 0x10           # 70    PXCAP ID
    PXCAPNextCapability = 0xA4  # 71    No next capability
    PXCAPCapabilities = 0x0002  # 72    Version = 2
    PXCAPDevCapabilities = \
        0x10008000              # 74    Function Level Reset Capability | Role-based Error Reporting
    PXCAPDevCtrl = 0x0000       # 78
    PXCAPDevStatus = 0x0000     # 7A
    PXCAPLinkCap = 0x00000000   # 7C
    PXCAPLinkCtrl = 0x0000      # 80
    PXCAPLinkStatus = 0x0001    # 82
    PXCAPDevCap2 = 0x00000010   # 94    Completion Timeout Disable Supported
    PXCAPDevCtrl2 = 0x00000000  # 98

    # PMCAP - PCI Power Management Capability
    PMCAPBaseOffset = 0xA4      # --    PMCAP capability base
    PMCAPCapId = 0x01           # A4    PMCAP ID
    PMCAPNextCapability = 0x00  # A5    Next capability pointer
    PMCAPCapabilities = 0x0003  # A6    Device Specific Initialization (No) | Version (1.2)
    PMCAPCtrlStatus = 0x0008    # A8    No Soft Reset

    BAR0Size = '8192B'  # 8KB (512 queue pairs)
    BAR4Size = '8192B'  # 8KB for MSI-X 512 vectors
    BAR5Size = '4096B'  # 4KB (64B) for MSI-X 512 vectors
