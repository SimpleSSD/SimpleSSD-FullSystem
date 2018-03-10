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

#include "dev/storage/nvme_interface.hh"

#include "cpu/intr_control.hh"
#include "dev/storage/simplessd/hil/nvme/controller.hh"
#include "dev/storage/simplessd/hil/nvme/def.hh"
#include "dev/storage/simplessd/util/algorithm.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

#undef panic
#undef warn
#undef info

#include "dev/storage/simplessd/log/log.hh"
#include "dev/storage/simplessd/log/trace.hh"
#include "dev/storage/simplessd/util/interface.hh"

NVMeInterface::NVMeInterface(Params *p)
    : PciDevice(p), configPath(p->SSDConfig), periodWork(0),
      lastReadDMAEndAt(0), lastWriteDMAEndAt(0), IS(0), ISold(0),
      mode(INTERRUPT_PIN), workEvent(this), completionEvent(this) {
  SimpleSSD::Logger::initLogSystem(std::cout, std::cerr,
                                   []() -> uint64_t { return curTick(); });

  if (!conf.init(configPath)) {
    SimpleSSD::Logger::panic("Failed to read SimpleSSD configuration");
  }

  pController = new SimpleSSD::HIL::NVMe::Controller(this, &conf);
}

NVMeInterface::~NVMeInterface() { delete pController; }

Tick NVMeInterface::readConfig(PacketPtr pkt) {
  int offset = pkt->getAddr() & PCI_CONFIG_SIZE;
  int size = pkt->getSize();

  if (offset < PCI_DEVICE_SPECIFIC) {
    return PciDevice::readConfig(pkt);
  } else {
    // Read on PCI capabilities
    uint32_t val = 0;
    for (int i = 0; i < size; i++) {
      if (offset + i >= PMCAP_BASE && offset + i < PMCAP_BASE + PMCAP_SIZE) {
        val |= (uint32_t)pmcap.data[offset + i - PMCAP_BASE] << (i * 8);
      } else if (offset + i >= MSICAP_BASE &&
                 offset + i < MSICAP_BASE + MSICAP_SIZE) {
        val |= (uint32_t)msicap.data[offset + i - MSICAP_BASE] << (i * 8);
      } else if (offset + i >= MSIXCAP_BASE &&
                 offset + i < MSIXCAP_BASE + MSIXCAP_SIZE) {
        val |= (uint32_t)msixcap.data[offset + i - MSIXCAP_BASE] << (i * 8);
      } else if (offset + i >= PXCAP_BASE &&
                 offset + i < PXCAP_BASE + PXCAP_SIZE) {
        val |= (uint32_t)pxcap.data[offset + i - PXCAP_BASE] << (i * 8);
      } else {
        SimpleSSD::Logger::warn(
            "nvme_interface: Invalid PCI config read offset: %#x", offset);
      }
    }

    switch (size) {
    case sizeof(uint8_t):
      pkt->set<uint8_t>(val);
      break;
    case sizeof(uint16_t):
      pkt->set<uint16_t>(val);
      break;
    case sizeof(uint32_t):
      pkt->set<uint32_t>(val);
      break;
    default:
      SimpleSSD::Logger::warn(
          "nvme_interface: Invalid PCI config read size: %d", size);
      break;
    }

    pkt->makeAtomicResponse();
  }

  return configDelay;
}

Tick NVMeInterface::writeConfig(PacketPtr pkt) {
  int offset = pkt->getAddr() & PCI_CONFIG_SIZE;
  int size = pkt->getSize();
  uint32_t val = 0;

  if (offset < PCI_DEVICE_SPECIFIC) {
    PciDevice::writeConfig(pkt);

    // Updates on BAR0/1 address
    if (offset == PCI0_BASE_ADDR0 || offset == PCI0_BASE_ADDR1) {
      register_addr = BARAddrs[0] | ((uint64_t)BARAddrs[1] << 32);
      register_size = BARSize[0];
    } else if (offset == PCI0_BASE_ADDR4) {
      table_addr = BARAddrs[4];
      table_size = BARSize[4];
    } else if (offset == PCI0_BASE_ADDR5) {
      pba_addr = BARAddrs[5];
      pba_size = BARSize[5];
    }
  } else {
    // Write on PCI capabilities
    if (offset == PMCAP_BASE + 4 &&
        size == sizeof(uint16_t)) { // PMCAP Control Status
      val = pkt->get<uint16_t>();

      if (val & 0x8000) {
        pmcap.pmcs &= 0x7F00; // Clear PMES
      }
      pmcap.pmcs &= ~0x1F03;
      pmcap.pmcs |= (val & 0x1F03);
    } else if (offset == MSICAP_BASE + 2 &&
               size == sizeof(uint16_t)) { // MSICAP Message Control
      val = pkt->get<uint16_t>();

      mode = (val & 0x0001) ? INTERRUPT_MSI : INTERRUPT_PIN;

      msicap.mc &= ~0x0071;
      msicap.mc |= (val & 0x0071);

      vectors = (uint16_t)powf(2, (msicap.mc & 0x0070) >> 4);

      SimpleSSD::Logger::debugprint(
          SimpleSSD::Logger::LOG_HIL_NVME, "INTR    | MSI %s | %d vectors",
          mode == INTERRUPT_PIN ? "disabled" : "enabled", vectors);
    } else if (offset == MSICAP_BASE + 4 &&
               size == sizeof(uint32_t)) { // MSICAP Message Address
      msicap.ma = pkt->get<uint32_t>() & 0xFFFFFFFC;
    } else if (offset == MSICAP_BASE + 8 &&
               size == sizeof(uint32_t)) { // MSICAP Message Upper Address
      msicap.mua = pkt->get<uint32_t>();
    } else if (offset == MSICAP_BASE + 12 &&
               size >= sizeof(uint16_t)) { // MSICAP Message Data
      msicap.md = pkt->get<uint16_t>();
    } else if (offset == MSICAP_BASE + 16 &&
               size == sizeof(uint32_t)) { // MSICAP Interrupt Mask Bits
      msicap.mmask = pkt->get<uint32_t>();
    } else if (offset == MSICAP_BASE + 20 &&
               size == sizeof(uint32_t)) { // MSICAP Interrupt Pending Bits
      msicap.mpend = pkt->get<uint32_t>();
    } else if (offset == MSIXCAP_BASE + 2 &&
               size == sizeof(uint16_t)) { // MSIXCAP Message Control
      val = pkt->get<uint16_t>();

      mode = (val & 0x8000) ? INTERRUPT_MSIX : INTERRUPT_PIN;

      msixcap.mxc &= ~0xC000;
      msixcap.mxc |= (val & 0xC000);

      vectors = (msixcap.mxc & 0x07FF) + 1;

      SimpleSSD::Logger::debugprint(
          SimpleSSD::Logger::LOG_HIL_NVME, "INTR    | MSI-X %s | %d vectors",
          mode == INTERRUPT_PIN ? "disabled" : "enabled", vectors);
    } else if (offset == PXCAP_BASE + 8 &&
               size == sizeof(uint16_t)) { // PXCAP Device Capabilities
      pxcap.pxdc = pkt->get<uint16_t>();
    } else if (offset == PXCAP_BASE + 10 &&
               size == sizeof(uint16_t)) { // PXCAP Device Status
      val = pkt->get<uint16_t>();

      if (val & 0x0001) {
        pxcap.pxds &= 0xFFFE;
      }
      if (val & 0x0002) {
        pxcap.pxds &= 0xFFFD;
      }
      if (val & 0x0004) {
        pxcap.pxds &= 0xFFFB;
      }
      if (val & 0x0008) {
        pxcap.pxds &= 0xFFF7;
      }
    } else if (offset == PXCAP_BASE + 16 &&
               size == sizeof(uint16_t)) { // PXCAP Link Control
      pxcap.pxlc = pkt->get<uint16_t>();
    } else if (offset == PXCAP_BASE + 18 &&
               size == sizeof(uint16_t)) { // PXCAP Link Status
      val = pkt->get<uint16_t>();

      if (val & 0x4000) {
        pxcap.pxls &= 0xBFFF;
      }
      if (val & 0x8000) {
        pxcap.pxls &= 0x7FFF;
      }
    } else if (offset == PXCAP_BASE + 40 &&
               size == sizeof(uint32_t)) { // PXCAP Device Control 2
      pxcap.pxdc2 = pkt->get<uint32_t>();
    } else {
      SimpleSSD::Logger::panic(
          "nvme_interface: Invalid PCI config write offset: %#x size: %d",
          offset, size);
    }

    pkt->makeAtomicResponse();
  }

  return configDelay;
}

Tick NVMeInterface::read(PacketPtr pkt) {
  Addr addr = pkt->getAddr();
  int size = pkt->getSize();
  uint8_t *buffer = pkt->getPtr<uint8_t>();
  Tick begin = curTick();
  Tick end = curTick();

  if (addr >= register_addr && addr + size <= register_addr + register_size) {
    int offset = addr - register_addr;

    if (offset >= SimpleSSD::HIL::NVMe::REG_DOORBELL_BEGIN) {
      // Read on doorbell register is vendor specific
      memset(buffer, 0, size);
    } else {
      pController->readRegister(offset, size, buffer, end);
    }
  } else if (addr >= table_addr && addr + size <= table_addr + table_size) {

  } else if (addr >= pba_addr && addr + size <= pba_addr + pba_size) {

  } else {
    SimpleSSD::Logger::panic(
        "nvme_interface: Invalid address access! BAR0 base: %016" PRIX64
        " addr: %016" PRIX64 "",
        register_addr, addr);
  }

  pkt->makeAtomicResponse();

  return end - begin;
}

Tick NVMeInterface::write(PacketPtr pkt) {
  Addr addr = pkt->getAddr();
  int size = pkt->getSize();
  uint8_t *buffer = pkt->getPtr<uint8_t>();
  Tick begin = curTick();
  Tick end = curTick();

  if (addr >= register_addr && addr + size <= register_addr + register_size) {
    int offset = addr - register_addr;

    if (offset >= SimpleSSD::HIL::NVMe::REG_DOORBELL_BEGIN) {
      const int dstrd = 4;
      uint32_t uiTemp, uiMask;
      uint16_t uiDoorbell;

      // Get new doorbell
      memcpy(&uiDoorbell, buffer, 2);

      // Calculate Queue Type and Queue ID from offset
      offset -= SimpleSSD::HIL::NVMe::REG_DOORBELL_BEGIN;
      uiTemp = offset / dstrd;
      uiMask = (uiTemp & 0x00000001); // 0 for Submission Queue Tail Doorbell
                                      // 1 for Completion Queue Head Doorbell
      uiTemp = (uiTemp >> 1);         // Queue ID

      if (uiMask) { // Completion Queue
        pController->ringCQHeadDoorbell(uiTemp, uiDoorbell, end);
      } else { // Submission Queue
        pController->ringSQTailDoorbell(uiTemp, uiDoorbell, end);
      }
    } else {
      pController->writeRegister(offset, size, buffer, end);
    }
  } else if (addr >= table_addr && addr + size <= table_addr + table_size) {
    uint64_t entry = (addr - table_addr) / 16;
    uint64_t offset = (addr - table_addr) % 16;

    memcpy(msix_table.at(entry).data + offset / 4, buffer, size);
  } else if (addr >= pba_addr && addr + size <= pba_addr + pba_size) {
    uint64_t entry = (addr - pba_addr) / 16;
    uint64_t offset = (addr - pba_addr) % 16;

    memcpy(&msix_pba.at(entry) + offset, buffer, size);
  } else {
    SimpleSSD::Logger::panic(
        "nvme_interface: Invalid address access! BAR0 base: %016" PRIX64
        " addr: %016" PRIX64 "",
        register_addr, addr);
  }

  pkt->makeAtomicResponse();

  return end - begin;
}

void NVMeInterface::writeInterrupt(Addr addr, size_t size, uint8_t *data) {
  Addr dmaAddr = hostInterface.dmaAddr(addr);

  DmaDevice::dmaWrite(dmaAddr, size, NULL, data);
}

uint64_t NVMeInterface::dmaRead(uint64_t addr, uint64_t size, uint8_t *buffer,
                                uint64_t &tick) {
  uint64_t latency = SimpleSSD::PCIExpress::calculateDelay(
      SimpleSSD::PCIExpress::PCIE_2_X, 4, size);
  uint64_t delay = 0;

  // DMA Scheduling
  if (tick == 0) {
    tick = lastReadDMAEndAt;
  }

  if (lastReadDMAEndAt <= tick) {
    lastReadDMAEndAt = tick + latency;
  } else {
    delay = lastReadDMAEndAt - tick;
    lastReadDMAEndAt += latency;
  }

  if (buffer) {
    DmaDevice::dmaRead(pciToDma(addr), size, nullptr, buffer);
  }

  delay += tick;
  tick = delay + latency;

  return delay;
}

uint64_t NVMeInterface::dmaWrite(uint64_t addr, uint64_t size, uint8_t *buffer,
                                 uint64_t &tick) {
  uint64_t latency = SimpleSSD::PCIExpress::calculateDelay(
      SimpleSSD::PCIExpress::PCIE_2_X, 4, size);
  uint64_t delay = 0;

  // DMA Scheduling
  if (tick == 0) {
    tick = lastWriteDMAEndAt;
  }

  if (lastWriteDMAEndAt <= tick) {
    lastWriteDMAEndAt = tick + latency;
  } else {
    delay = lastWriteDMAEndAt - tick;
    lastWriteDMAEndAt += latency;
  }

  if (buffer) {
    DmaDevice::dmaWrite(pciToDma(addr), size, nullptr, buffer);
  }

  delay += tick;
  tick = delay + latency;

  return delay;
}

void NVMeInterface::updateInterrupt(uint16_t iv, bool post) {
  switch (mode) {
  case INTERRUPT_PIN:
    if (post) {
      IS |= (1 << iv);
    } else {
      IS &= ~(1 << iv);
    }

    if (IS == ISold) {
      break;
    }

    if (IS == 0) {
      intrClear();

      SimpleSSD::Logger::debugprint(SimpleSSD::Logger::LOG_HIL_NVME,
                                    "INTR    | Pin Interrupt Clear");
    } else {
      intrPost();

      SimpleSSD::Logger::debugprint(SimpleSSD::Logger::LOG_HIL_NVME,
                                    "INTR    | Pin Interrupt Post");
    }

    ISold = IS;

    break;
  case INTERRUPT_MSI: {
    uint32_t mask = 1 << iv;
    uint32_t data = msicap.md;

    if (!(msicap.mmask & mask) && post) {
      if (vectors > 1) { // Multiple MSI
        data &= ~(vectors - 1);
        data |= iv & (vectors - 1);
      }

      writeInterrupt(((uint64_t)msicap.mua << 32) | msicap.ma, sizeof(uint32_t),
                     (uint8_t *)&data);

      SimpleSSD::Logger::debugprint(SimpleSSD::Logger::LOG_HIL_NVME,
                                    "INTR    | MSI sent | vector %d", iv);
    }
  }

  break;
  case INTERRUPT_MSIX: {
    MSIXTable &table = msix_table.at(iv);

    if (!(table.fields.vec_ctrl & 0x0000001) && post) {
      writeInterrupt(((uint64_t)table.fields.addr_hi << 32) |
                         table.fields.addr_lo,
                     sizeof(uint32_t), (uint8_t *)&table.fields.msg_data);

      SimpleSSD::Logger::debugprint(SimpleSSD::Logger::LOG_HIL_NVME,
                                    "INTR    | MSI-X sent | vector %d", iv);
    }
  }

  break;
  }
}

void NVMeInterface::getVendorID(uint16_t &vid, uint16_t &ssvid) {
  vid = config.vendor;
  ssvid = config.subsystemVendorID;
}

void NVMeInterface::doWork() {
  Tick handling = curTick();

  pController->work(handling);

  schedule(workEvent, handling + periodWork);
}

void NVMeInterface::doCompletion() {
  Tick handling = curTick();

  pController->completion(handling);
}

void NVMeInterface::enableController(Tick w) {
  periodWork = w;

  schedule(workEvent, curTick() + periodWork);
}

void NVMeInterface::submitCompletion(Tick tick) {
  if (completionEvent.scheduled()) {
    deschedule(completionEvent);
  }

  schedule(completionEvent, tick);
}

void NVMeInterface::disableController() {
  if (workEvent.scheduled()) {
    deschedule(workEvent);
  }
}

void NVMeInterface::serialize(CheckpointOut &cp) const {
  // FIXME: Not implemented
}

void NVMeInterface::unserialize(CheckpointIn &cp) {
  // FIXME: Not implemented
}

NVMeInterface *NVMeInterfaceParams::create() { return new NVMeInterface(this); }
