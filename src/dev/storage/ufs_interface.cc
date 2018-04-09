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

#include "dev/storage/ufs_interface.hh"

#undef panic
#undef warn
#undef info

#include "dev/storage/simplessd/hil/ufs/host.hh"
#include "dev/storage/simplessd/sim/log.hh"
#include "dev/storage/simplessd/util/interface.hh"

UFSInterface::UFSInterface(const UFSInterfaceParams *p)
    : DmaDevice(p),
      pioAddr(p->pio_addr),
      pioSize(0xFFFF),
      pioDelay(p->pio_latency),
      intn(p->int_num),
      gic(p->gic),
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

  conf = initSimpleSSDEngine(this, std::cout, std::cerr, p->SSDConfig);

  axiWidth = (SimpleSSD::ARM::AXI::BUS_WIDTH)conf.readInt(
      SimpleSSD::CONFIG_UFS, SimpleSSD::HIL::UFS::UFS_HOST_AXI_BUS_WIDTH);
  axiClock = conf.readUint(SimpleSSD::CONFIG_UFS,
                           SimpleSSD::HIL::UFS::UFS_HOST_AXI_CLOCK);

  pController = new SimpleSSD::HIL::UFS::Host(this, conf);

  registerExitCallback(new ExitCallback());
}

UFSInterface::~UFSInterface() {
  delete pController;
  delete[] pStats;
}

AddrRangeList UFSInterface::getAddrRanges() const {
  AddrRangeList ranges;
  ranges.push_back(RangeSize(pioAddr, pioSize));

  return ranges;
}

Tick UFSInterface::read(PacketPtr pkt) {
  uint32_t data = 0;
  uint32_t offset = pkt->getAddr() & 0xFF;

  if (pController) {
    pController->readRegister(offset, data);
  }

  pkt->set<uint32_t>(data);
  pkt->makeResponse();

  return pioDelay;
}

Tick UFSInterface::write(PacketPtr pkt) {
  uint32_t data = pkt->get<uint32_t>();
  uint32_t offset = pkt->getAddr() & 0xFF;

  if (pController && pkt->getSize() == 4) {
    pController->writeRegister(offset, data, curTick() + pioDelay);
  }
  else {
    SimpleSSD::warn("Undefined UFSHCI register write size");
  }

  pkt->makeResponse();

  return pioDelay;
}

void UFSInterface::generateInterrupt() {
  gic->sendInt(intn);
  SimpleSSD::debugprint(SimpleSSD::LOG_HIL_UFS, "INTR    | Interrupt Post");
}

void UFSInterface::clearInterrupt() {
  gic->clearInt(intn);
  SimpleSSD::debugprint(SimpleSSD::LOG_HIL_UFS, "INTR    | Interrupt Clear");
}

void UFSInterface::dmaRead(uint64_t addr, uint64_t size, uint8_t *buffer,
                           SimpleSSD::DMAFunction &func, void *context) {
  if (size == 0) {
    SimpleSSD::warn("ufs_interface: zero-size DMA read request. Ignore.");

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

void UFSInterface::dmaReadDone() {
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

void UFSInterface::submitDMARead() {
  auto &iter = dmaReadQueue.front();

  dmaReadPending = true;

  iter.beginAt = curTick();
  iter.finishedAt = iter.beginAt + SimpleSSD::ARM::AXI::calculateDelay(
                                       axiClock, axiWidth, iter.size);

  if (iter.buffer) {
    DmaDevice::dmaRead(iter.addr, iter.size, &dmaReadEvent, iter.buffer);
  }
  else {
    schedule(dmaReadEvent, iter.finishedAt);
  }
}

void UFSInterface::dmaWrite(uint64_t addr, uint64_t size, uint8_t *buffer,
                            SimpleSSD::DMAFunction &func, void *context) {
  if (size == 0) {
    SimpleSSD::warn("ufs_interface: zero-size DMA write request. Ignore.");

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

void UFSInterface::dmaWriteDone() {
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

void UFSInterface::submitDMAWrite() {
  auto &iter = dmaWriteQueue.front();

  dmaWritePending = true;

  iter.beginAt = curTick();
  iter.finishedAt = iter.beginAt + SimpleSSD::ARM::AXI::calculateDelay(
                                       axiClock, axiWidth, iter.size);
  if (iter.buffer) {
    DmaDevice::dmaWrite(iter.addr, iter.size, &dmaWriteEvent, iter.buffer);
  }
  else {
    schedule(dmaWriteEvent, iter.finishedAt);
  }
}

void UFSInterface::serialize(CheckpointOut &cp) const {}

void UFSInterface::unserialize(CheckpointIn &cp) {}

void UFSInterface::regStats() {
  DmaDevice::regStats();

  if (pController) {
    std::vector<SimpleSSD::Stats> list;

    pController->getStatList(list, "");
    SimpleSSD::getCPUStatList(list, "cpu");

    if (list.size() > 0) {
      pStats = new Stats::Scalar[list.size()]();

      for (uint32_t i = 0; i < list.size(); i++) {
        pStats[i].name(name() + "." + list[i].name).desc(list[i].desc);
      }
    }
  }
}

void UFSInterface::resetStats() {
  DmaDevice::resetStats();

  if (pController) {
    pController->resetStatValues();
    SimpleSSD::resetCPUStatValues();

    updateStats();
  }
}

void UFSInterface::updateStats() {
  if (pController) {
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
}

uint64_t UFSInterface::getCurrentTick() {
  return curTick();
}

SimpleSSD::Event UFSInterface::allocateEvent(SimpleSSD::EventFunction func) {
  std::string name("SimpleSSD_Event_");

  name += std::to_string(counter++);

  auto iter = eventList.insert(
      {counter, EventFunctionWrapper([func]() { func(curTick()); }, name)});

  if (!iter.second) {
    SimpleSSD::panic("Fail to allocate event");
  }

  return counter;
}

void UFSInterface::scheduleEvent(SimpleSSD::Event eid, uint64_t tick) {
  auto iter = eventList.find(eid);

  if (iter != eventList.end()) {
    if (iter->second.scheduled()) {
      SimpleSSD::warn("Event %" PRIu64 " rescheduled from %" PRIu64
                      " to %" PRIu64,
                      eid, iter->second.when(), tick);

      reschedule(iter->second, tick);
    }
    else {
      schedule(iter->second, tick);
    }
  }
  else {
    SimpleSSD::panic("Event %" PRIu64 " does not exists", eid);
  }
}

void UFSInterface::descheduleEvent(SimpleSSD::Event eid) {
  auto iter = eventList.find(eid);

  if (iter != eventList.end()) {
    if (iter->second.scheduled()) {
      deschedule(iter->second);
    }
  }
  else {
    SimpleSSD::panic("Event %" PRIu64 " does not exists", eid);
  }
}

bool UFSInterface::isScheduled(SimpleSSD::Event eid) {
  bool ret = false;
  auto iter = eventList.find(eid);

  if (iter != eventList.end()) {
    ret = iter->second.scheduled();
  }
  else {
    SimpleSSD::panic("Event %" PRIu64 " does not exists", eid);
  }

  return ret;
}

void UFSInterface::deallocateEvent(SimpleSSD::Event eid) {
  auto iter = eventList.find(eid);

  if (iter != eventList.end()) {
    eventList.erase(iter);
  }
  else {
    SimpleSSD::panic("Event %" PRIu64 " does not exists", eid);
  }
}

UFSInterface *UFSInterfaceParams::create() {
  return new UFSInterface(this);
}
