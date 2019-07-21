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

#include "dev/storage/sata_interface.hh"

#include "cpu/intr_control.hh"
#include "dev/storage/simplessd/hil/sata/hba.hh"
#include "dev/storage/simplessd/util/algorithm.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

#undef panic
#undef warn
#undef info

#include "dev/storage/simplessd/util/interface.hh"
#include "dev/storage/simplessd/util/simplessd.hh"

SATAInterface::SATAInterface(Params *p)
    : PciDevice(p),
      EventEngine(this),
      configPath(p->SSDConfig),
      IS(0),
      ISold(0),
      mode(INTERRUPT_PIN),
      dmaReadEvent([this]() { dmaReadDone(); }, name()),
      dmaWriteEvent([this]() { dmaWriteDone(); }, name()),
      dmaReadPending(false),
      dmaWritePending(false),
      statUpdateEvent([this]() { updateStats(); }, name()),
      pStats(nullptr) {
  if (p->SSDConfig.size() == 0) {
    pController = nullptr;

    return;
  }

  conf = initSimpleSSDEngine(this, &std::cout, &std::cerr, configPath);

  pcieGen = (SimpleSSD::PCIExpress::PCIE_GEN)conf.readInt(
      SimpleSSD::CONFIG_SATA, SimpleSSD::HIL::SATA::SATA_PCIE_GEN);
  pcieLane = (uint8_t)conf.readUint(SimpleSSD::CONFIG_SATA,
                                    SimpleSSD::HIL::SATA::SATA_PCIE_LANE);

  pController = new SimpleSSD::HIL::SATA::HBA(this, conf);

  registerExitCallback(new ExitCallback());
}

SATAInterface::~SATAInterface() {
  delete pController;
}

Tick SATAInterface::readConfig(PacketPtr pkt) {
  if (!pController) {
    pkt->makeAtomicResponse();

    return configDelay;
  }

  int offset = pkt->getAddr() & PCI_CONFIG_SIZE;
  int size = pkt->getSize();

  if (offset < PCI_DEVICE_SPECIFIC) {
    return PciDevice::readConfig(pkt);
  }
  else {
    // Read on PCI capabilities
    uint32_t val = 0;
    for (int i = 0; i < size; i++) {
      if (offset + i >= PMCAP_BASE && offset + i < PMCAP_BASE + PMCAP_SIZE) {
        val |= (uint32_t)pmcap.data[offset + i - PMCAP_BASE] << (i * 8);
      }
      else if (offset + i >= MSICAP_BASE &&
               offset + i < MSICAP_BASE + MSICAP_SIZE) {
        val |= (uint32_t)msicap.data[offset + i - MSICAP_BASE] << (i * 8);
      }
      else {
        SimpleSSD::warn("sata_interface: Invalid PCI config read offset: %#x",
                        offset);
      }
    }

    switch (size) {
      case sizeof(uint8_t):
        pkt->setLE<uint8_t>(val);
        break;
      case sizeof(uint16_t):
        pkt->setLE<uint16_t>(val);
        break;
      case sizeof(uint32_t):
        pkt->setLE<uint32_t>(val);
        break;
      default:
        SimpleSSD::warn("sata_interface: Invalid PCI config read size: %d",
                        size);
        break;
    }

    pkt->makeAtomicResponse();
  }

  return configDelay;
}

Tick SATAInterface::writeConfig(PacketPtr pkt) {
  if (!pController) {
    pkt->makeAtomicResponse();

    return configDelay;
  }

  int offset = pkt->getAddr() & PCI_CONFIG_SIZE;
  int size = pkt->getSize();
  uint32_t val = 0;

  if (offset < PCI_DEVICE_SPECIFIC) {
    PciDevice::writeConfig(pkt);

    switch (offset) {
      case PCI0_BASE_ADDR5:
        if (BARAddrs[5] != 0) {
          AHCIRegisters = BARAddrs[5];
        }

        break;
    }
  }
  else {
    // Write on PCI capabilities
    if (offset == PMCAP_BASE + 4 &&
        size == sizeof(uint16_t)) {  // PMCAP Control Status
      val = pkt->getLE<uint16_t>();

      if (val & 0x8000) {
        pmcap.pmcs &= 0x7F00;  // Clear PMES
      }
      pmcap.pmcs &= ~0x1F03;
      pmcap.pmcs |= (val & 0x1F03);
    }
    else if (offset == MSICAP_BASE + 2 &&
             size == sizeof(uint16_t)) {  // MSICAP Message Control
      val = pkt->getLE<uint16_t>();

      mode = (val & 0x0001) ? INTERRUPT_MSI : INTERRUPT_PIN;

      msicap.mc &= ~0x0071;
      msicap.mc |= (val & 0x0071);

      vectors = (uint16_t)powf(2, (msicap.mc & 0x0070) >> 4);

      SimpleSSD::debugprint(
          SimpleSSD::LOG_HIL_SATA, "INTR    | MSI %s | %d vectors",
          mode == INTERRUPT_PIN ? "disabled" : "enabled", vectors);
    }
    else if (offset == MSICAP_BASE + 4 &&
             size == sizeof(uint32_t)) {  // MSICAP Message Address
      msicap.ma = pkt->getLE<uint32_t>() & 0xFFFFFFFC;
    }
    else if (offset == MSICAP_BASE + 8 &&
             size >= sizeof(uint16_t)) {  // MSICAP Message Data
      msicap.md = pkt->getLE<uint16_t>();
    }
    else {
      SimpleSSD::warn(
          "sata_interface: Invalid PCI config write offset: %#x size: %d",
          offset, size);
    }

    pkt->makeAtomicResponse();
  }

  return configDelay;
}

Tick SATAInterface::read(PacketPtr pkt) {
  if (!pController) {
    pkt->makeAtomicResponse();

    return configDelay;
  }

  Addr addr = pkt->getAddr();
  int size = pkt->getSize();
  uint8_t *buffer = pkt->getPtr<uint8_t>();

  if (addr >= AHCIRegisters && addr + size <= AHCIRegisters + 1024) {
    pController->readAHCIRegister(addr - AHCIRegisters, size, buffer);
  }
  else {
    SimpleSSD::panic("sata_interface: Invalid address access!");
  }

  pkt->makeAtomicResponse();

  return configDelay;
}

Tick SATAInterface::write(PacketPtr pkt) {
  if (!pController) {
    pkt->makeAtomicResponse();

    return configDelay;
  }

  Addr addr = pkt->getAddr();
  int size = pkt->getSize();
  uint8_t *buffer = pkt->getPtr<uint8_t>();

  if (addr >= AHCIRegisters && addr + size <= AHCIRegisters + 1024) {
    pController->writeAHCIRegister(addr - AHCIRegisters, size, buffer);
  }
  else {
    SimpleSSD::panic("sata_interface: Invalid address access!");
  }

  pkt->makeAtomicResponse();

  return configDelay;
}

void SATAInterface::writeInterrupt(Addr addr, size_t size, uint8_t *data) {
  Addr dmaAddr = hostInterface.dmaAddr(addr);

  DmaDevice::dmaWrite(dmaAddr, size, NULL, data);
}

void SATAInterface::updateInterrupt(bool post) {
  static uint16_t iv = 0;

  switch (mode) {
    case INTERRUPT_PIN:
      if (post) {
        IS |= (1 << iv);
      }
      else {
        IS &= ~(1 << iv);
      }

      if (IS == ISold) {
        break;
      }

      if (IS == 0) {
        intrClear();

        SimpleSSD::debugprint(SimpleSSD::LOG_HIL_SATA,
                              "INTR    | Pin Interrupt Clear");
      }
      else {
        intrPost();

        SimpleSSD::debugprint(SimpleSSD::LOG_HIL_SATA,
                              "INTR    | Pin Interrupt Post");
      }

      ISold = IS;

      break;
    case INTERRUPT_MSI: {
      uint32_t data = msicap.md;

      if (post) {
        if (vectors > 1) {  // Multiple MSI
          data &= ~(vectors - 1);
          data |= iv & (vectors - 1);
        }

        writeInterrupt(msicap.ma, sizeof(uint32_t), (uint8_t *)&data);

        SimpleSSD::debugprint(SimpleSSD::LOG_HIL_SATA,
                              "INTR    | MSI sent | vector %d", iv);
      }
    }

    break;
    default:
      break;
  }
}

void SATAInterface::dmaRead(uint64_t addr, uint64_t size, uint8_t *buffer,
                            SimpleSSD::DMAFunction &func, void *context) {
  if (size == 0) {
    SimpleSSD::warn("sata_interface: zero-size DMA read request. Ignore.");

    return;
  }

  dmaReadQueue.push(DMAEntry(func));

  auto &iter = dmaReadQueue.back();
  iter.addr = addr;
  iter.size = size;
  iter.buffer = buffer;
  iter.context = context;

  if (!dmaReadPending) {
    submitDMARead();
  }
}

void SATAInterface::dmaReadDone() {
  auto &iter = dmaReadQueue.front();
  uint64_t tick = curTick();

  if (tick < iter.finishedAt) {
    schedule(dmaReadEvent, iter.finishedAt);

    return;
  }

  iter.func(tick, iter.context);
  dmaReadQueue.pop();
  dmaReadPending = false;

  if (dmaReadQueue.size() > 0) {
    submitDMARead();
  }
}

void SATAInterface::submitDMARead() {
  auto &iter = dmaReadQueue.front();

  dmaReadPending = true;

  iter.beginAt = curTick();
  iter.finishedAt = iter.beginAt + SimpleSSD::PCIExpress::calculateDelay(
                                       pcieGen, pcieLane, iter.size);

  if (iter.buffer) {
    DmaDevice::dmaRead(pciToDma(iter.addr), iter.size, &dmaReadEvent,
                       iter.buffer);
  }
  else {
    schedule(dmaReadEvent, iter.finishedAt);
  }
}

void SATAInterface::dmaWrite(uint64_t addr, uint64_t size, uint8_t *buffer,
                             SimpleSSD::DMAFunction &func, void *context) {
  if (size == 0) {
    SimpleSSD::warn("sata_interface: zero-size DMA write request. Ignore.");

    return;
  }

  dmaWriteQueue.push(DMAEntry(func));

  auto &iter = dmaWriteQueue.back();
  iter.addr = addr;
  iter.size = size;
  iter.buffer = buffer;
  iter.context = context;

  if (!dmaWritePending) {
    submitDMAWrite();
  }
}

void SATAInterface::dmaWriteDone() {
  auto &iter = dmaWriteQueue.front();
  uint64_t tick = curTick();

  if (tick < iter.finishedAt) {
    schedule(dmaWriteEvent, iter.finishedAt);

    return;
  }

  iter.func(tick, iter.context);
  dmaWriteQueue.pop();
  dmaWritePending = false;

  if (dmaWriteQueue.size() > 0) {
    submitDMAWrite();
  }
}

void SATAInterface::submitDMAWrite() {
  auto &iter = dmaWriteQueue.front();

  dmaWritePending = true;

  iter.beginAt = curTick();
  iter.finishedAt = iter.beginAt + SimpleSSD::PCIExpress::calculateDelay(
                                       pcieGen, pcieLane, iter.size);

  if (iter.buffer) {
    DmaDevice::dmaWrite(pciToDma(iter.addr), iter.size, &dmaWriteEvent,
                        iter.buffer);
  }
  else {
    schedule(dmaWriteEvent, iter.finishedAt);
  }
}

void SATAInterface::serialize(CheckpointOut &cp) const {
  // FIXME: Not implemented
}

void SATAInterface::unserialize(CheckpointIn &cp) {
  // FIXME: Not implemented
}

void SATAInterface::regStats() {
  PciDevice::regStats();

  std::vector<SimpleSSD::Stats> list;

  if (!pController) {
    return;
  }

  pController->getStatList(list, "");
  SimpleSSD::getCPUStatList(list, "cpu");

  if (list.size() > 0) {
    pStats = new Stats::Scalar[list.size()]();

    for (uint32_t i = 0; i < list.size(); i++) {
      pStats[i].name(name() + "." + list[i].name).desc(list[i].desc);
    }
  }
}

void SATAInterface::resetStats() {
  PciDevice::resetStats();

  if (!pController) {
    return;
  }

  pController->resetStatValues();
  SimpleSSD::resetCPUStatValues();

  updateStats();
}

void SATAInterface::updateStats() {
  if (!pController) {
    return;
  }

  std::vector<double> values;

  pController->getStatValues(values);
  SimpleSSD::getCPUStatValues(values);

  for (uint32_t i = 0; i < values.size(); i++) {
    pStats[i] = values[i];
  }

  if (statUpdateEvent.scheduled()) {
    deschedule(statUpdateEvent);
  }

  schedule(statUpdateEvent, curTick() + STAT_UPDATE_PERIOD);
}

SATAInterface *SATAInterfaceParams::create() {
  return new SATAInterface(this);
}
