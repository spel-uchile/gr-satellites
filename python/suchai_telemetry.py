#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2017 Carlos Gonzalez <carlgonz@uchile.cl>.
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

import numpy
import json
from gnuradio import gr
import pmt
import array
import csp_header

class suchai_telemetry(gr.basic_block):
    """
    Convert CSP message data to SUCHAI telemetry message 
    """
    def __init__(self):
        gr.basic_block.__init__(self,
            name="suchai_telemetry",
            in_sig=[],
            out_sig=[])

        self.message_port_register_in(pmt.intern('in'))
        self.set_msg_handler(pmt.intern('in'), self.handle_msg)
        self.message_port_register_out(pmt.intern('out'))

    def handle_msg(self, msg_pmt):
        msg = pmt.cdr(msg_pmt)
        if not pmt.is_u8vector(msg):
            print "[ERROR] Received invalid message type. Expected u8vector"
            return
        packet = array.array("B", pmt.u8vector_elements(msg))
        try:
            header = csp_header.CSP(packet[:4])
            if header.dest_port == 10: # SUCHAI TM port
                data = packet[4:]
                msg_data = ",".join(["0x{:02X}{:02X}".format(data[i+1], data[i]) for i in range(0, len(data), 2)])
                msg_json = {"type": "tm", "data": msg_data}
            else: # send as debug data
                msg_header = str(header).replace("\n",";")
                msg_json = {"type": "debug", "data": msg_header}
                
            msg_json = json.dumps(msg_json)
            self.message_port_pub(pmt.intern('out'), pmt.to_pmt(msg_json))
        except ValueError as e:
            print e
