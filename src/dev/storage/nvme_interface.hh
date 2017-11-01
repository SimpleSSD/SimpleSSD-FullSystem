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

#include "dev/io_device.hh"
#include "dev/pci/device.hh"
#include "dev/storage/simplessd/hil/nvme/interface.hh"
#include "dev/storage/simplessd/util/config.hh"
#include "params/NVMeInterface.hh"

typedef enum _INTERRUPT_MODE {
  INTERRUPT_PIN,
  INTERRUPT_MSI,
  INTERRUPT_MSIX
} INTERRUPT_MODE;

class NVMeInterface : public PciDevice, public SimpleSSD::HIL::NVMe::Interface {
private:
  std::string configPath;
  SimpleSSD::ConfigReader conf;

  Addr register_addr;
  int register_size;

  Tick periodWork;
  Tick periodQueue;

  /* Interrupt logics */
  // Pin based
  uint32_t IS;
  uint32_t ISold;

  // MSI/MSI-X
  uint16_t vectors;

  // MSI-X
  Addr table_addr;
  int table_size;
  Addr pba_addr;
  int pba_size;

  // Current Interrupt Mode
  INTERRUPT_MODE mode;

  void writeInterrupt(Addr, size_t, uint8_t *);

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

  // Interface <-> Controller
  void dmaRead(uint64_t, uint64_t, uint8_t *);
  void dmaWrite(uint64_t, uint64_t, uint8_t *);
  void updateInterrupt(uint16_t, bool);
  void getVendorID(uint16_t &, uint16_t &);
  void enableController(Tick, Tick);
  void disableController();
  void doQueue();
  friend class EventWrapper<NVMeInterface, &NVMeInterface::doQueue>;
  EventWrapper<NVMeInterface, &NVMeInterface::doQueue> queueEvent;
  void doWork();
  friend class EventWrapper<NVMeInterface, &NVMeInterface::doWork>;
  EventWrapper<NVMeInterface, &NVMeInterface::doWork> workEvent;
};

#endif
