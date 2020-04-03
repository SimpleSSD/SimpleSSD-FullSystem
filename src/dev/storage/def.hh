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

#ifndef __DEV_STORAGE_DEF_HH__
#define __DEV_STORAGE_DEF_HH__

#include <cinttypes>
#include <unordered_map>

#include "base/callback.hh"
#include "dev/storage/simplessd/sim/dma_interface.hh"
#include "dev/storage/simplessd/sim/simulator.hh"
#include "sim/sim_object.hh"

#define STAT_UPDATE_PERIOD 10000000

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
  SimpleSSD::DMAFunction func;

  _DMAEntry(SimpleSSD::DMAFunction &f)
      : beginAt(0), finishedAt(0), addr(0), size(0), buffer(nullptr),
        context(nullptr), func(f) {}
} DMAEntry;

class ExitCallback : public Callback {
protected:
  void autoDestruct() override;

public:
  void process() override;
};

class EventEngine : public SimpleSSD::Simulator {
protected:
  SimObject *pObject;

  // Simulator
  std::unordered_map<SimpleSSD::Event, EventFunctionWrapper> eventList;
  SimpleSSD::Event counter;

public:
  EventEngine(SimObject *);
  virtual ~EventEngine() {}

  uint64_t getCurrentTick() override;

  SimpleSSD::Event allocateEvent(SimpleSSD::EventFunction) override;
  void scheduleEvent(SimpleSSD::Event, uint64_t) override;
  void descheduleEvent(SimpleSSD::Event) override;
  bool isScheduled(SimpleSSD::Event, uint64_t * = nullptr) override;
  void deallocateEvent(SimpleSSD::Event) override;
};

#endif
