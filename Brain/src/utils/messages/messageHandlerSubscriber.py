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
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

import inspect
import pickle
import threading
import time
from multiprocessing import Pipe

class messageHandlerSubscriber: 
    """Class which will handle subscriber functionalities.\n
    Args:
        queuesList (dictionar of multiprocessing.queues.Queue): Dictionar of queues where the ID is the type of messages.
        message (enum): A specific message
        deliveryMode (string): Determines how messages are delivered from the queue. ("FIFO" or "LastOnly").
        subscribe (bool): A flag to automatically subscribe the message.
    """
        
    def __init__(self, queuesList, message, deliveryMode="fifo", subscribe=False):
        self._queuesList = queuesList
        self._message = message
        self._deliveryMode = str.lower(deliveryMode)
        self._pipeRecv, self._pipeSend = Pipe(duplex=False)
        self._subscribed = False
        self._last_recover_time = 0.0
        self._recover_backoff_s = 0.1
        self._close_event = threading.Event()
        self._lastonly_lock = threading.Lock()
        self._lastonly_value_event = threading.Event()
        self._lastonly_cached_message = None
        self._lastonly_discarding = False
        self._lastonly_drain_thread = None
        frame = inspect.currentframe().f_back # type: ignore
        if 'self' in frame.f_locals: # type: ignore
            self._receiver = frame.f_locals['self'].__class__.__name__ # type: ignore
        else:
            self._receiver = frame.f_globals.get('__name__', None) # type: ignore
        
        if subscribe == True:
            self.subscribe()

        if self._deliveryMode not in ["fifo", "lastonly"]:
            print("WARNING! Wrong delivery mode supplied.", deliveryMode, "instead of FIFO or LastOnly.", self._message, self._receiver)
            print("WARNING! Switching to FIFO")
            self._deliveryMode = "fifo"

        if self._deliveryMode == "lastonly":
            self._start_lastonly_drain_thread()

    def _start_lastonly_drain_thread(self):
        if self._lastonly_drain_thread is not None and self._lastonly_drain_thread.is_alive():
            return
        self._lastonly_drain_thread = threading.Thread(
            target=self._lastonly_drain_loop,
            name=f"LastOnlyDrain-{self._receiver}-{self._message.msgID.value}",
            daemon=True,
        )
        self._lastonly_drain_thread.start()

    def _lastonly_drain_loop(self):
        while not self._close_event.is_set():
            try:
                if not self._pipeRecv.poll(0.1):
                    continue
            except (EOFError, OSError, BrokenPipeError, ConnectionResetError, pickle.UnpicklingError) as error:
                if self._close_event.is_set():
                    return
                self._recover_pipe(error)
                continue

            message = self._recv_once()
            if message is None:
                if self._close_event.is_set():
                    return
                continue

            with self._lastonly_lock:
                if self._lastonly_discarding:
                    self._lastonly_cached_message = None
                    self._lastonly_value_event.clear()
                else:
                    self._lastonly_cached_message = message
                    self._lastonly_value_event.set()

    def _pop_lastonly_message(self):
        with self._lastonly_lock:
            message = self._lastonly_cached_message
            self._lastonly_cached_message = None
            self._lastonly_value_event.clear()
            return message

    def _recover_pipe(self, error):
        if self._close_event.is_set():
            return
        now = time.monotonic()
        if now - self._last_recover_time < self._recover_backoff_s:
            time.sleep(self._recover_backoff_s - (now - self._last_recover_time))
        self._last_recover_time = time.monotonic()
        if self._deliveryMode == "lastonly":
            with self._lastonly_lock:
                self._lastonly_cached_message = None
                self._lastonly_value_event.clear()

        was_subscribed = self._subscribed
        if was_subscribed:
            try:
                self.unsubscribe()
            except Exception:
                pass

        try:
            self._pipeRecv.close()
        except Exception:
            pass
        try:
            self._pipeSend.close()
        except Exception:
            pass

        self._pipeRecv, self._pipeSend = Pipe(duplex=False)

        if was_subscribed:
            self.subscribe()

        print("WARNING! Pipe reset due to error:", repr(error), self._message, self._receiver)

    def _recv_once(self):
        try:
            return self._pipeRecv.recv()
        except (EOFError, OSError, BrokenPipeError, ConnectionResetError, pickle.UnpicklingError) as error:
            self._recover_pipe(error)
            return None

    def receive(self):
        """
        Receives values from a pipe

        Returns None if there no data in the Pipe
        """
        if self._deliveryMode == "lastonly":
            message = self._pop_lastonly_message()
            if message is None:
                return None
            return self._extract_message_value(message)

        if not self._pipeRecv.poll():
            return None
        else:
            return self.receive_with_block()
        
    def receive_with_block(self):
        """
        Waits until there is an existing message in the pipe 
        
        Returns:
            message's data type: The received message.
        """
        if self._deliveryMode == "lastonly":
            while not self._close_event.is_set():
                message = self._pop_lastonly_message()
                if message is not None:
                    return self._extract_message_value(message)
                self._lastonly_value_event.wait(0.1)
            return None

        
        message = self._recv_once()
        if message is None:
            return None

        if self._deliveryMode == "fifo":
            return self._extract_message_value(message)

    def _extract_message_value(self, message):
        messageType = type(message["value"]).__name__
        if messageType != self._message.msgType.value:
            print("WARNING! Message type and value type are not matching.", self._message, "received:", messageType, "expected:", self._message.msgType.value)
        return message["value"]
        
    def empty(self):
        """
        Empties the receiving pipe of any existing data.
        """
        if self._deliveryMode == "lastonly":
            quiet_window_s = 0.02
            max_wait_s = 0.1

            with self._lastonly_lock:
                self._lastonly_discarding = True
                self._lastonly_cached_message = None
                self._lastonly_value_event.clear()

            started_at = time.monotonic()
            quiet_deadline = started_at + quiet_window_s
            try:
                while not self._close_event.is_set():
                    now = time.monotonic()
                    if now - started_at >= max_wait_s:
                        break

                    try:
                        has_pending = self._pipeRecv.poll(0.005)
                    except (EOFError, OSError, BrokenPipeError, ConnectionResetError, pickle.UnpicklingError) as error:
                        self._recover_pipe(error)
                        quiet_deadline = time.monotonic() + quiet_window_s
                        continue

                    if has_pending:
                        quiet_deadline = time.monotonic() + quiet_window_s
                        time.sleep(0.001)
                        continue

                    if now >= quiet_deadline:
                        break
            finally:
                with self._lastonly_lock:
                    self._lastonly_cached_message = None
                    self._lastonly_value_event.clear()
                    self._lastonly_discarding = False
            return
        while self._pipeRecv.poll():
            if self._recv_once() is None:
                break

    def subscribe(self):
        """
        Subscribes to messages.
        """
        self._queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": self._message.Owner.value,
                "msgID": self._message.msgID.value,
                "To": {"receiver": self._receiver, "pipe": self._pipeSend},
            }
        )
        self._subscribed = True

    def unsubscribe(self):
        """
        Unsubscribes from messages.
        """
        self._queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "unsubscribe",
                "Owner": self._message.Owner.value,
                "msgID": self._message.msgID.value,
                "To": {"receiver": self._receiver}
            }
        )
        self._subscribed = False

    def is_data_in_pipe(self):
        """
        Checks if there is any data in the receiving pipe.

        Returns:
            bool: True if data is available, False otherwise.
        """
        if self._deliveryMode == "lastonly":
            return self._lastonly_value_event.is_set()
        return self._pipeRecv.poll()

    def set_delivery_mode_to_fifo(self):
        """
        Sets delivery mode to FIFO.
        """
        self._deliveryMode = "fifo"

    def set_delivery_mode_to_last_only(self):
        """
        Sets delivery mode to LastOnly.
        """
        self._deliveryMode = "lastonly"
        self._start_lastonly_drain_thread()

    def close(self):
        self._close_event.set()
        self._lastonly_value_event.set()
        try:
            self._pipeRecv.close()
        except Exception:
            pass
        try:
            self._pipeSend.close()
        except Exception:
            pass
        drain_thread = self._lastonly_drain_thread
        if drain_thread is not None and drain_thread.is_alive() and drain_thread is not threading.current_thread():
            drain_thread.join(timeout=0.2)

    def __del__(self): 
        """
        Cleans up by closing the pipes.
        """
        self.close()
