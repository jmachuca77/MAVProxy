#!/usr/bin/env python
'''
support for a GCS attached DGPS system
Set the ip, port and conntype settings to your requiered connection
The module supports TCP and UDP connections (conntype)

For using with Emlid Reach use TCP connection, set the IP to the IP address of the Reach base

Configure ReachView like so:

Base mode:
TCP, Role: Server, address: localhost, Port: 9000
Set the RTCM Messages to 10Hz and enable any that are needed.
'''

import socket, errno
import time
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings

class DGPSModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(DGPSModule, self).__init__(mpstate, "DGPS", "DGPS injection support for SBP/RTCP/UBC over tcp/udp")
        print "Loading DGPS module"
        self.dgps_settings = mp_settings.MPSettings(
            [('ip', str, "127.0.0.1"),
             ('port', int, 13320),
             ('conntype', str, 'UDP'),
             ('logfile', str, None)
            ])
        self.add_command('dgps', self.cmd_dgps, 'DGPS control',
                         ["<status>",
                          "<start>",
                          "<stop>",
                          "set (DGPSSETTING)"])
        self.add_completion_function('(DGPSSETTING)',
                                     self.dgps_settings.completion)

        self.port = None      
        self.last_pkt = None 
        self.inject_seq_nr = 0

    def log_rtcm(self, data):
        '''optionally log rtcm data'''
        if self.dgps_settings.logfile is None:
            return
        if self.logfile is None:
            self.logfile = open(self.dgps_settings.logfile, 'wb')
        if self.logfile is not None:
            self.logfile.write(data)

    def cmd_dgps(self, args):
        '''dgps command handling'''
        if len(args) <= 0:
            print("Usage: dgps <start|stop|status|set>")
            return
        if args[0] == "start":
            self.cmd_start()
        if args[0] == "stop":
            self.port = None
        elif args[0] == "status":
            self.dgps_status()
        elif args[0] == "set":
            self.dgps_settings.command(args[1:])

    def cmd_start(self):
        '''start dgps link'''
        if self.dgps_settings.conntype == "UDP":
            print("Connecting to UDP RTCM Stream")
            self.connect_udp_rtcm_base(self.dgps_settings.ip, self.dgps_settings.port)
        elif self.dgps_settings.conntype == "TCP":
            print("Connecting to TCP RTCM Stream")
            self.connect_tcp_rtcm_base(self.dgps_settings.ip, self.dgps_settings.port)
        else:
            print("Undefined connection type!")
            return           

    def dgps_status(self):
        '''show dgps status'''
        now = time.time()
        print("DGPS Configuration:")
        print("Connection Type: %s" % self.dgps_settings.conntype)
        print("IP Address: %s" % self.dgps_settings.ip)
        print("Port: %s" % self.dgps_settings.port)
        
        if self.port is None:
            print("DGPS: Not started")
            return
        elif self.last_pkt is None:
            print("DGPS: no data")
            return
        
        print ("Last packet recieved %.3fs ago" % (now - self.last_pkt))
        # frame_size = 0
        # for id in sorted(self.id_counts.keys()):
        #     print(" %4u: %u (len %u)" % (id, self.id_counts[id], len(self.last_by_id[id])))
        #     frame_size += len(self.last_by_id[id])
        # print("dgps: %u packets, %.1f bytes/sec last %.1fs ago framesize %u" % (self.pkt_count, self.rate, now - self.last_pkt, frame_size))

    def connect_udp_rtcm_base(self, ip, port):
        print "UDP Connection"
        self.portnum = 13320
        self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.port.bind(("127.0.0.1", self.portnum))
        mavutil.set_close_on_exec(self.port.fileno())
        self.port.setblocking(0)
        print("DGPS: Listening for RTCM packets on UDP://%s:%s" % ("127.0.0.1", self.portnum))

    def connect_tcp_rtcm_base(self, ip, port):
        print "TCP Connection"
        try:
            self.port = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.port.connect((ip, port))
            self.port.settimeout(1)
        except:
            print "ERROR: Failed to connect to RTCM base over TCP"
        else:
            print "Connected to base using TCP"

    def send_rtcm_msg(self, data):
        msglen = 180;
        
        if (len(data) > msglen * 4):
            print("DGPS: Message too large", len(data))
            return
        
        # How many messages will we send?
        msgs = 0
        if (len(data) % msglen == 0):
            msgs = len(data) // msglen
        else:
            msgs = (len(data) // msglen) + 1

        for a in range(0, msgs):
            
            flags = 0
            
            # Set the fragment flag if we're sending more than 1 packet.
            if (msgs) > 1:
                flags = 1
            
            # Set the ID of this fragment
            flags |= (a & 0x3) << 1
            
            # Set an overall sequence number
            flags |= (self.inject_seq_nr & 0x1f) << 3
            
            
            amount = min(len(data) - a * msglen, msglen)
            datachunk = data[a*msglen : a*msglen + amount]

            self.master.mav.gps_rtcm_data_send(
                flags,
                len(datachunk),
                bytearray(datachunk.ljust(180, b'\0')))
        
        # Send a terminal 0-length message if we sent 2 or 3 exactly-full messages.     
        if (msgs < 4) and (len(data) % msglen == 0) and (len(data) > msglen):
            flags = 1 | (msgs & 0x3)  << 1 | (self.inject_seq_nr & 0x1f) << 3
            self.master.mav.gps_rtcm_data_send(
                flags,
                0,
                bytearray("".ljust(180, '\0')))
            
        self.inject_seq_nr += 1

    def idle_task(self):
        '''called in idle time'''
        if self.port is None:
            return

        try:
            data = self.port.recv(1024) # Attempt to read up to 1024 bytes.
        except socket.error as e:
            if e.errno in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
                return
            raise
        try:
            now = time.time()
            self.send_rtcm_msg(data)
            self.log_rtcm(data)
            self.last_pkt = now

        except Exception as e:
            print("DGPS: GPS Inject Failed:", e)

def init(mpstate):
    '''initialise module'''
    return DGPSModule(mpstate)

