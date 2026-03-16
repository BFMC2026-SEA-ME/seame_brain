# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived Owner
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

import json
import os
import time
from src.utils.messages.allMessages import Cars, Semaphores
from twisted.internet import protocol
from src.utils.messages.messageHandlerSender import messageHandlerSender

class udpListener(protocol.DatagramProtocol):
    """This class is used to receive the information from the servers.

    Args:
        queue (multiprocessing.queues.Queue): the queue to send the info
    """

    def __init__(self, queuesList, logger, debugging):
        self.semaphoresSender = messageHandlerSender(queuesList, Semaphores)
        self.carsSender = messageHandlerSender(queuesList, Cars)
        self.logger = logger
        self.debugging = debugging
        self._forward_car = os.getenv("SEMAPHORE_FORWARD_CAR", "1").lower() in ("1", "true", "yes", "y")
        car_id_filter = os.getenv("SEMAPHORE_CAR_ID", "0").strip()
        if car_id_filter in ("", "*"):
            self._car_id_filter = None
        else:
            try:
                self._car_id_filter = int(car_id_filter)
            except ValueError:
                self._car_id_filter = None
        self._car_min_period = float(os.getenv("SEMAPHORE_CAR_FORWARD_PERIOD", "0.1"))
        self._last_car_emit_by_id = {}

    def datagramReceived(self, datagram, addr):
        """Specific function for receiving the information. It will select and create different dictionary for each type of data we receive(car or semaphore)

        Args:
            datagram (dictionary): In this we store the data we get from servers.
        """
        dat = datagram.decode("utf-8")
        dat = json.loads(dat)

        if dat["device"] == "semaphore":
            tmp = {"device": "semaphore", "id": dat["id"], "state": dat["state"], "x": dat["x"], "y": dat["y"]}
            if self.debugging:
                self.logger.info(tmp)
            self.semaphoresSender.send(tmp)
            return

        if dat["device"] == "car":
            if not self._forward_car:
                return
            car_id = dat.get("id")
            try:
                car_id_int = int(car_id)
            except Exception:
                return
            if self._car_id_filter is not None and car_id_int != self._car_id_filter:
                return
            now = time.monotonic()
            if self._car_min_period > 0.0:
                last_emit = self._last_car_emit_by_id.get(car_id_int, 0.0)
                if (now - last_emit) < self._car_min_period:
                    return
                self._last_car_emit_by_id[car_id_int] = now
            tmp = {"device": "car", "id": car_id_int, "x": dat["x"], "y": dat["y"]}
            if self.debugging:
                self.logger.info(tmp)
            self.carsSender.send(tmp)
            return

        return

    def stopListening(self):
        super().stopListening() # type: ignore
