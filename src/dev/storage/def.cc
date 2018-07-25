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

#include "dev/storage/def.hh"

#include "dev/storage/simplessd/util/simplessd.hh"

void ExitCallback::autoDestruct() { delete this; }

void ExitCallback::process() { releaseSimpleSSDEngine(); }

EventEngine::EventEngine(SimObject *s) : pObject(s), counter(0) {}

uint64_t EventEngine::getCurrentTick() { return curTick(); }

SimpleSSD::Event EventEngine::allocateEvent(SimpleSSD::EventFunction func) {
  std::string name("SimpleSSD_Event_");

  name += std::to_string(counter++);

  auto iter = eventList.insert(
      {counter, EventFunctionWrapper([func]() { func(curTick()); }, name)});

  if (!iter.second) {
    SimpleSSD::panic("Fail to allocate event");
  }

  return counter;
}

void EventEngine::scheduleEvent(SimpleSSD::Event eid, uint64_t tick) {
  auto iter = eventList.find(eid);

  if (pObject == nullptr) {
    SimpleSSD::panic("SimObject not initialized");
  }

  if (iter != eventList.end()) {
    uint64_t now = curTick();

    if (tick < now) {
      SimpleSSD::warn("Tried to schedule %" PRIu64
                      " < curTick() to event %" PRIu64
                      ". Set tick as curTick().",
                      tick, eid);

      tick = now;
    }

    if (iter->second.scheduled()) {
      SimpleSSD::warn("Event %" PRIu64 " rescheduled from %" PRIu64
                      " to %" PRIu64,
                      eid, iter->second.when(), tick);

      pObject->reschedule(iter->second, tick);
    } else {
      pObject->schedule(iter->second, tick);
    }
  } else {
    SimpleSSD::panic("Event %" PRIu64 " does not exists", eid);
  }
}

void EventEngine::descheduleEvent(SimpleSSD::Event eid) {
  auto iter = eventList.find(eid);

  if (pObject == nullptr) {
    SimpleSSD::panic("SimObject not initialized");
  }

  if (iter != eventList.end()) {
    if (iter->second.scheduled()) {
      pObject->deschedule(iter->second);
    }
  } else {
    SimpleSSD::panic("Event %" PRIu64 " does not exists", eid);
  }
}

bool EventEngine::isScheduled(SimpleSSD::Event eid, uint64_t *pTick) {
  bool ret = false;
  auto iter = eventList.find(eid);

  if (iter != eventList.end()) {
    ret = iter->second.scheduled();

    if (pTick) {
      *pTick = iter->second.when();
    }
  } else {
    SimpleSSD::panic("Event %" PRIu64 " does not exists", eid);
  }

  return ret;
}

void EventEngine::deallocateEvent(SimpleSSD::Event eid) {
  auto iter = eventList.find(eid);

  if (iter != eventList.end()) {
    eventList.erase(iter);
  } else {
    SimpleSSD::panic("Event %" PRIu64 " does not exists", eid);
  }
}
