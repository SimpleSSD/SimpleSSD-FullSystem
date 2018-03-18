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

#ifndef __DEV_STORAGE_NVME_INTERFACE_HH__
#define __DEV_STORAGE_NVME_INTERFACE_HH__

#include <cinttypes>
#include <queue>
#include <unordered_map>

#include "dev/io_device.hh"
#include "dev/pci/device.hh"
#include "dev/storage/simplessd/hil/nvme/interface.hh"
#include "dev/storage/simplessd/sim/config_reader.hh"
#include "dev/storage/simplessd/sim/simulator.hh"
#include "params/NVMeInterface.hh"

typedef enum _INTERRUPT_MODE {
  INTERRUPT_PIN,
  INTERRUPT_MSI,
  INTERRUPT_MSIX
} INTERRUPT_MODE;

typedef struct _DMAEntry {
  uint64_t beginAt;
  uint64_t finishedAt;
  uint64_t addr;
  uint64_t size;
  uint8_t *buffer;
  void *context;
  SimpleSSD::DMAFunction &func;

  _DMAEntry(SimpleSSD::DMAFunction &f)
      : beginAt(0),
        finishedAt(0),
        addr(0),
        size(0),
        buffer(nullptr),
        context(nullptr),
        func(f) {}
} DMAEntry;

class NVMeInterface : public PciDevice,
                      public SimpleSSD::HIL::NVMe::Interface,
                      public SimpleSSD::Simulator {
 private:
  std::string configPath;
  SimpleSSD::ConfigReader conf;

  Addr registerTableBaseAddress;
  int registerTableSize;

  // DMA scheduling
  EventFunctionWrapper dmaReadEvent;
  EventFunctionWrapper dmaWriteEvent;
  std::queue<DMAEntry> dmaReadQueue;
  std::queue<DMAEntry> dmaWriteQueue;
  bool dmaReadPending;
  bool dmaWritePending;

  /* Interrupt logics */
  // Pin based
  uint32_t interruptStatus;
  uint32_t oldInterruptStatus;

  // MSI/MSI-X
  uint16_t vectors;

  // MSI-X
  Addr tableBaseAddress;
  int tableSize;
  Addr pbaBaseAddress;
  int pbaSize;

  // Current Interrupt Mode
  INTERRUPT_MODE mode;

  // Stats
  EventFunctionWrapper statUpdateEvent;
  Stats::Scalar *pStats;

  void writeInterrupt(Addr, size_t, uint8_t *);
  void dmaReadDone();
  void submitDMARead();
  void dmaWriteDone();
  void submitDMAWrite();

  // Simulator
  std::unordered_map<SimpleSSD::Event, EventFunctionWrapper> eventList;
  SimpleSSD::Event counter;

 public:
  typedef NVMeInterfaceParams Params;
  const Params *params() const { return (const Params *)_params; }
  NVMeInterface(Params *p);
  ~NVMeInterface();

  Tick writeConfig(PacketPtr pkt) override;
  Tick readConfig(PacketPtr pkt) override;

  Tick read(PacketPtr pkt) override;
  Tick write(PacketPtr pkt) override;

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
  void updateInterrupt(uint16_t, bool) override;
  void getVendorID(uint16_t &, uint16_t &) override;
};

#endif
