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
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWITrueSE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import serialCamera
import time
from queue import Empty

class threadGateway(ThreadWithStop):
    """Thread which will handle processGateway functionalities.\n
    Args:
        queuesList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logger (logging object): Made for debugging.
        debugger (bool): A flag for debugging.
    """

    # ===================================== INIT =========================================

    def __init__(self, queueList, logger, debugging):
        super(threadGateway, self).__init__(pause=0.005)
        self.logger = logger
        self.debugging = debugging
        self.sendingList = {}
        self.queuesList = queueList
        self.messageApproved = []
        self._critical_batch_limit = 32
        self._warning_batch_limit = 32
        self._general_batch_limit = 128
        self._config_batch_limit = 32

    # =================================== SUBSCRIBE ======================================

    def subscribe(self, message):
        """This functin will add the pipe into the approved messages list   and it will be added into the dictionary of sending
        Args:
            message(dictionary): Dictionary received from the multiprocessing queues ( the config one).
        """
        
        # Declaration of variables:
        Owner = message["Owner"]
        Id = message["msgID"]
        To = message["To"]["receiver"]
        Pipe = message["To"]["pipe"]
        if not Owner in self.sendingList.keys():
            self.sendingList[Owner] = {}
        if not Id in self.sendingList[Owner].keys():
            self.sendingList[Owner][Id] = {}
        if not To in self.sendingList[Owner][Id].keys():
            self.sendingList[Owner][Id][To] = Pipe
        self.messageApproved.append((Owner, Id))
        # Debugging( you can comment this):
        if self.debugging:
            self.print_list()

    # ================================== UNSUBSCRIBE =====================================

    def unsubscribe(self, message):
        """This functin will remove the pipe into the approved messages list and it will be added into the dictionary of sending
        Args:
            message(dictionary): Dictionary received from the multiprocessing queues ( the config one).
        """

        Owner = message["Owner"]
        Id = message["msgID"]
        To = message["To"]["receiver"]

        # Tolerate duplicated/unordered unsubscribe events.
        owner_dict = self.sendingList.get(Owner)
        if owner_dict is not None:
            id_dict = owner_dict.get(Id)
            if id_dict is not None and To in id_dict:
                del id_dict[To]
                if not id_dict:
                    del owner_dict[Id]
                if not owner_dict:
                    del self.sendingList[Owner]

        key = (Owner, Id)
        if key in self.messageApproved:
            self.messageApproved.remove(key)
        if self.debugging:
            self.print_list()

    # =================================== SENDING ========================================

    def send(self, message):
        """This functin will send the message on all the pipes that are in the sending list of the message ID.
        Args:
            message(dictionary): Dictionary received from the multiprocessing queues ( the config one).
        """

        Owner = message["Owner"]
        Id = message["msgID"]
        Type = message["msgType"]
        Value = message["msgValue"]
        if (Owner, Id) in self.messageApproved:
            to_remove = []
            # 병렬로 보내는게 아니라, 한줄로 순서대로 보낸다. 병렬로 보내면 메시지 순서가 뒤죽박죽이 될 수 있다.
            for element, pipe in self.sendingList[Owner][Id].items():
                # We send a dictionary that contain the type of the message and message
                try:
                    # sub이 수신안하면 막힐수도있다.(버퍼가 꽉차서)
                    pipe.send({"Type": Type, "value": Value, "id": Id, "Owner": Owner})
                    if self.debugging:
                        self.logger.warning(message)
                except (BrokenPipeError, EOFError, OSError, ConnectionResetError) as error:
                    to_remove.append(element)
                    if self.debugging:
                        self.logger.warning("Dropping dead pipe for %s/%s/%s: %r", Owner, Id, element, error)
            for element in to_remove:
                owner_dict = self.sendingList.get(Owner)
                if owner_dict is None:
                    continue
                id_dict = owner_dict.get(Id)
                if id_dict is None or element not in id_dict:
                    continue
                del id_dict[element]
                if not id_dict:
                    del owner_dict[Id]
                if not owner_dict:
                    del self.sendingList[Owner]

    def _has_subscribers(self, owner, msg_id):
        return (owner, msg_id) in self.messageApproved

    def _drain_queue(self, queue_name, limit):
        processed = 0
        queue_ref = self.queuesList.get(queue_name)
        if queue_ref is None:
            return 0

        while processed < limit:
            try:
                message = queue_ref.get_nowait()
            except Empty:
                break

            self.send(message)
            processed += 1

        return processed

    def _process_config_messages(self, limit):
        processed = 0
        queue_ref = self.queuesList.get("Config")
        if queue_ref is None:
            return 0

        while processed < limit:
            try:
                message = queue_ref.get_nowait()
            except Empty:
                break

            try:
                if str.lower(message["Subscribe/Unsubscribe"]) == "subscribe":
                    self.subscribe(message)
                else:
                    self.unsubscribe(message)
            except Exception as exc:
                if self.debugging:
                    self.logger.warning("Config routing failed: %r", exc)

            processed += 1

        return processed

    # ====================================================================================

    # Function for debugging:
    def print_list(self):
        """Made for debugging"""

        self.logger.warning(self.sendingList)

    # ==================================== RUN ===========================================

    def thread_work(self):
        """This function will take the messages in priority order form the queues.\n
        the prioirty is: Critical > Warning > General
        """
        self._drain_queue("Critical", self._critical_batch_limit)
        self._drain_queue("Warning", self._warning_batch_limit)
        self._drain_queue("General", self._general_batch_limit)

        # Process latest image independently so camera frames are not starved
        # by a constantly non-empty General queue.
        if "Image" in self.queuesList:
            if self._has_subscribers(serialCamera.Owner.value, serialCamera.msgID.value):
                latest_image = None
                while True:
                    try:
                        latest_image = self.queuesList["Image"].get_nowait()
                    except Empty:
                        break
                if latest_image is not None:
                    self.send(latest_image)
        self._process_config_messages(self._config_batch_limit)

        # print(time.perf_counter_ns())


# =====================================================================================
