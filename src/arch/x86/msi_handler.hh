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

#include "arch/x86/interrupts.hh"
#include "dev/io_device.hh"
#include "params/X86MSIHandler.hh"

class X86MSIHandler : public PioDevice {
  private:
  const AddrRange range;
  const Tick pioDelay;

  std::vector<X86ISA::Interrupts *> lapics;

  void handleInterrupt(PacketPtr pkt);

public:
  X86MSIHandler(const X86MSIHandlerParams *p);

  /* Pio Device */
  AddrRangeList getAddrRanges() const override;
  Tick read(PacketPtr pkt) override;
  Tick write(PacketPtr pkt) override;

  void serialize(CheckpointOut &cp) const override;
  void unserialize(CheckpointIn &cp) override;
};
