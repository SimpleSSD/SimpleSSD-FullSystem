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

#ifndef __DEV_STORAGE_UFS_INTERFACE_HH__
#define __DEV_STORAGE_UFS_INTERFACE_HH__

#include <cinttypes>
#include <queue>

#include "base/addr_range.hh"
#include "dev/arm/base_gic.hh"
#include "dev/dma_device.hh"
#include "dev/storage/def.hh"
#include "dev/storage/simplessd/hil/ufs/interface.hh"
#include "dev/storage/simplessd/sim/config_reader.hh"
#include "dev/storage/simplessd/sim/simulator.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/UFSInterface.hh"

class UFSInterface : public DmaDevice,
                     public SimpleSSD::HIL::UFS::Interface,
                     public SimpleSSD::Simulator {
 private:
  AddrRangeList getAddrRanges() const override;
  Tick read(PacketPtr pkt) override;
  Tick write(PacketPtr pkt) override;

  const Addr pioAddr;
  const Addr pioSize;
  const Tick pioDelay;
  const int intn;
  BaseGic *gic;

  SimpleSSD::ConfigReader conf;

  SimpleSSD::ARM::AXI::BUS_WIDTH axiWidth;
  uint64_t axiClock;

  // DMA scheduling
  EventFunctionWrapper dmaReadEvent;
  EventFunctionWrapper dmaWriteEvent;
  std::queue<DMAEntry> dmaReadQueue;
  std::queue<DMAEntry> dmaWriteQueue;
  bool dmaReadPending;
  bool dmaWritePending;

  // Simulator
  std::unordered_map<SimpleSSD::Event, EventFunctionWrapper> eventList;
  SimpleSSD::Event counter;

  // Stats
  EventFunctionWrapper statUpdateEvent;
  Stats::Scalar *pStats;

  void dmaReadDone();
  void submitDMARead();
  void dmaWriteDone();
  void submitDMAWrite();

 public:
  UFSInterface(const UFSInterfaceParams *p);
  ~UFSInterface();

  void serialize(CheckpointOut &cp) const override;
  void unserialize(CheckpointIn &cp) override;

  void regStats() override;
  void resetStats() override;
  void updateStats();

  // Simulator
  uint64_t getCurrentTick() override;

  SimpleSSD::Event allocateEvent(SimpleSSD::EventFunction) override;
  void scheduleEvent(SimpleSSD::Event, uint64_t) override;
  void descheduleEvent(SimpleSSD::Event) override;
  bool isScheduled(SimpleSSD::Event) override;
  void deallocateEvent(SimpleSSD::Event) override;

  // Interface <-> Controller
  void dmaRead(uint64_t, uint64_t, uint8_t *, SimpleSSD::DMAFunction &,
               void * = nullptr) override;
  void dmaWrite(uint64_t, uint64_t, uint8_t *, SimpleSSD::DMAFunction &,
                void * = nullptr) override;
  void generateInterrupt() override;
  void clearInterrupt() override;
};

#endif
