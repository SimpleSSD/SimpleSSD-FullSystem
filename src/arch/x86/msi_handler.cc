/*
 * Copyright (C) 2017 CAMELab
 *
 * This file is part of SimpleSSD.
 *
 * SimpleSSD is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SimpleSSD is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SimpleSSD.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "arch/x86/msi_handler.hh"
#include "debug/MSI.hh"

X86MSIHandler::X86MSIHandler(const X86MSIHandlerParams *p)
    : PioDevice(p), range(p->pio_addr, p->pio_addr + p->pio_size - 1),
      pioDelay(p->pio_latency), lapics(p->lapics) {
  DPRINTF(MSI, "Total %d of LocalAPIC detected\n", lapics.size());
}

AddrRangeList X86MSIHandler::getAddrRanges() const {
  return AddrRangeList({range});
}

Tick X86MSIHandler::read(PacketPtr pkt) {
  warn("Read to Message Data Register.\n");

  pkt->makeAtomicResponse();

  return pioDelay;
}

Tick X86MSIHandler::write(PacketPtr pkt) {
  handleInterrupt(pkt);

  return pioDelay;
}

void X86MSIHandler::handleInterrupt(PacketPtr pkt) {
  Addr addr = pkt->getAddr();

  DPRINTF(MSI, "Got DMA write to Message Address Register %X\n",
          (uint32_t)addr);

  uint8_t dstID = (addr & 0x000FF000) >> 12; // Destination ID
  bool rh = addr & 0x00000008;               // Redirection Hint
  bool dm = addr & 0x00000004;               // Destination Mode

  uint16_t data = pkt->get<uint16_t>();
  bool level = data & 0x8000;          // Trigger Mode
  uint8_t mode = (data & 0x0700) >> 8; // Delivery Mode
  uint8_t vector = data & 0x00FF;      // Interrupt Vector

  uint8_t pid, lid, model;

  if (rh & dm) {
    // Logical Destination Mode
    // Redirect to only those processors that are part of the logical group
    // of processors based on the processor's logical APIC ID and the
    // Destination ID field in the message
    for (auto iter = lapics.begin(); iter != lapics.end(); iter++) {
      (*iter)->getID(pid, lid, model);

      if (model == 0x00) {
        // Cluster model
        if (dstID == 0xFF) {
          panic("Invalid destination ID in message\n");
        }

        if (lid == dstID) { // Is this also & ?
          DPRINTF(MSI, "Send Interrupt to Local APIC %d\n", pid);
          (*iter)->requestInterrupt(vector, mode, level);
        }
      } else {
        // Flat model
        if (lid & dstID) {
          DPRINTF(MSI, "Send Interrupt to Local APIC %d\n", pid);
          (*iter)->requestInterrupt(vector, mode, level);
        }
      }
    }
  } else if (rh & !dm) {
    // Physical Destination Mode
    // Send interrupt to processor that has the matching APIC ID
    if (dstID == 0xFF) {
      panic("Invalid destination ID in message\n");
    }

    for (auto iter = lapics.begin(); iter != lapics.end(); iter++) {
      (*iter)->getID(pid, lid, model);

      if (pid == dstID) {
        DPRINTF(MSI, "Send Interrupt to Local APIC %d\n", pid);
        (*iter)->requestInterrupt(vector, mode, level);
      }
    }
  } else {
    // Message is sent ahead independent of whether the physical or logical
    // destination mode is used
    if (dstID == 0xFF) {
      // Send to all
      for (auto iter = lapics.begin(); iter != lapics.end(); iter++) {
        DPRINTF(MSI, "Send Interrupt to Local APIC %d\n", pid);
        (*iter)->requestInterrupt(vector, mode, level);
      }
    } else {
      for (auto iter = lapics.begin(); iter != lapics.end(); iter++) {
        (*iter)->getID(pid, lid, model);

        if (pid == dstID) {
          DPRINTF(MSI, "Send Interrupt to Local APIC %d\n", pid);
          (*iter)->requestInterrupt(vector, mode, level);
        }
      }
    }
  }

  pkt->makeAtomicResponse();
}

void X86MSIHandler::serialize(CheckpointOut &cp) const {
  DPRINTF(MSI, "Serializing X86MSIHandler\n");

  // Noting to do (Only uses read-only params when initializing)
}

void X86MSIHandler::unserialize(CheckpointIn &cp) {
  DPRINTF(MSI, "Unserializing X86MSIHandler\n");
}

X86MSIHandler *X86MSIHandlerParams::create() { return new X86MSIHandler(this); }
