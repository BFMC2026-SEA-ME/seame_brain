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

import json
import time
from threading import Event
from src.utils.messages.allMessages import Location
from src.utils.messages.messageHandlerSender import messageHandlerSender
from twisted.internet import protocol

# The server itself. Creates a new Protocol for each new connection and has the info for all of them.
class tcpClient(protocol.ClientFactory):
    def __init__(self, connectionBrokenCllbck, locsysID, locsysFrequency, queue):
        self.connectiondata = None
        self.connection = None
        self.retry_delay = 1
        self.connectionBrokenCllbck = connectionBrokenCllbck
        self.locsysID = locsysID
        self.locsysFrequency = locsysFrequency
        self.queue = queue
        self.event = Event()
        self.sendLocation = messageHandlerSender(self.queue, Location)

    def clientConnectionLost(self, connector, reason):
        print(f"\033[1;97m[ Traffic Communication ] :\033[0m \033[1;93mWARNING\033[0m - Connection lost with server \033[94m{self.connectiondata}\033[0m")
        try:
            self.connectiondata = None
            self.connection = None
            self.connectionBrokenCllbck()
        except:
            pass

    def clientConnectionFailed(self, connector, reason):
        print(f"\033[1;97m[ Traffic Communication ] :\033[0m \033[1;93mWARNING\033[0m - Connection failed, retrying in \033[94m{self.retry_delay}s\033[0m")
        self.event.wait(self.retry_delay)
        connector.connect()

    def buildProtocol(self, addr):
        conn = SingleConnection()
        conn.factory = self
        return conn

    def send_data_to_server(self, message):
        self.connection.send_data(message) # type: ignore


# One class is generated for each new connection
class SingleConnection(protocol.Protocol):
    def connectionMade(self):
        peer = self.transport.getPeer() # type: ignore
        self.factory.connectiondata = peer.host + ":" + str(peer.port) # type: ignore
        self.factory.connection = self # type: ignore
        self.subscribeToLocaitonData(self.factory.locsysID, self.factory.locsysFrequency) # type: ignore
        print(f"\033[1;97m[ Traffic Communication ] :\033[0m \033[1;92mINFO\033[0m - Connected to server \033[94m{self.factory.connectiondata}\033[0m") # type: ignore

    def dataReceived(self, data):
        dat = data.decode()
        tmp_data = dat.replace("}{","}}{{")
        if tmp_data != dat:
            tmp_dat = tmp_data.split("}{")
            dat = tmp_dat[-1]
        da = json.loads(dat)

        if da["type"] == "location":
            da["id"] = self.factory.locsysID # type: ignore
            da["_rx_time"] = time.time()  # TCP 수신 시각 기록
            self.factory.sendLocation.send(da) # type: ignore
        else:
            print(f"\033[1;97m[ Traffic Communication ] :\033[0m \033[1;92mINFO\033[0m - Message from server \033[94m{self.factory.connectiondata}\033[0m") # type: ignore
    def send_data(self, message):
        msg = json.dumps(message)
        self.transport.write(msg.encode()) # type: ignore
    
    def subscribeToLocaitonData(self, id, frequency):
        # Sends the id you wish to subscribe to and the frequency you want to receive data. Frequency must be between 0.1 and 5. 
        msg = {
            "reqORinfo": "info",
            "type": "locIDsub",
            "locID": id,
            "freq": frequency,
        }
        self.send_data(msg)
    
    def unSubscribeToLocaitonData(self, id, frequency):
        # Unsubscribes from locaiton data. 
        msg = {
            "reqORinfo": "info",
            "type": "locIDubsub",
        }
        self.send_data(msg)
