#!/usr/bin/env python3

# Copyright 2026 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import queue
import threading

_SENTINEL = object()


class SensorPublishWorker:
    """Convert and publish one sensor's data off the simulation tick thread.

    The bridge drives the simulation clock from a single synchronous loop:
    every tick waits for sensor processing to finish before the next
    world.tick(). Converting and publishing large messages inline (for
    example multi-megabyte camera images with reliable QoS, whose publish
    call blocks on DDS flow control) stalls that loop and slows simulation
    time itself, which destabilizes every downstream consumer.

    Each worker owns one daemon thread and a bounded latest-wins queue:
    when the publish side cannot keep up, stale frames are dropped instead
    of blocking the tick loop. Sensor frames are perishable, so dropping
    the oldest frame is the correct degradation.
    """

    def __init__(self, name, logger, queue_size=1):
        self._name = name
        self._logger = logger
        self._queue = queue.Queue(maxsize=queue_size)
        self._dropped = 0
        self._thread = threading.Thread(
            target=self._run, name=f"sensor_publish_{name}", daemon=True
        )
        self._thread.start()

    def submit(self, fn, args):
        """Enqueue a publish call without blocking; drop the stale frame when full."""
        item = (fn, args)
        while True:
            try:
                self._queue.put_nowait(item)
                return
            except queue.Full:
                try:
                    self._queue.get_nowait()
                    self._dropped += 1
                    if self._dropped % 100 == 1:
                        self._logger.debug(
                            f"Publish worker '{self._name}' dropped {self._dropped} stale frames"
                        )
                except queue.Empty:
                    pass

    def _run(self):
        while True:
            item = self._queue.get()
            if item is _SENTINEL:
                return
            fn, args = item
            try:
                fn(*args)
            except Exception as e:  # noqa: B902 - keep the worker alive on publish errors
                self._logger.error(f"Publish worker '{self._name}' failed: {e}")

    def stop(self, timeout=2.0):
        """Stop the worker after letting queued work drain."""
        try:
            self._queue.put(_SENTINEL, timeout=timeout)
        except queue.Full:
            # Worker is wedged inside a publish call; it is a daemon thread,
            # so it will not block process exit.
            return
        self._thread.join(timeout=timeout)
