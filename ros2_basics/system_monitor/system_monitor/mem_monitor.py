#!/usr/bin/env python
############################################################################
#    Copyright (C) 2009, Willow Garage, Inc.                               #
#    Copyright (C) 2013 by Ralf Kaestner                                   #
#    ralf.kaestner@gmail.com                                               #
#    Copyright (C) 2013 by Jerome Maye                                     #
#    jerome.maye@mavt.ethz.ch                                              #
#                                                                          #
#    All rights reserved.                                                  #
#                                                                          #
#    Redistribution and use in source and binary forms, with or without    #
#    modification, are permitted provided that the following conditions    #
#    are met:                                                              #
#                                                                          #
#    1. Redistributions of source code must retain the above copyright     #
#       notice, this list of conditions and the following disclaimer.      #
#                                                                          #
#    2. Redistributions in binary form must reproduce the above copyright  #
#       notice, this list of conditions and the following disclaimer in    #
#       the documentation and/or other materials provided with the         #
#       distribution.                                                      #
#                                                                          #
#    3. The name of the copyright holders may be used to endorse or        #
#       promote products derived from this software without specific       #
#       prior written permission.                                          #
#                                                                          #
#    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   #
#    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     #
#    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     #
#    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        #
#    COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  #
#    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  #
#    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      #
#    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      #
#    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    #
#    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN     #
#    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       #
#    POSSIBILITY OF SUCH DAMAGE.                                           #
############################################################################

from __future__ import with_statement

import rospy

import traceback
import threading
from threading import Timer
import sys, os, time
from time import sleep
import subprocess
import string
import socket

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

mem_level_warn = 0.95
mem_level_error = 0.99

stat_dict = { 0: 'OK', 1: 'Warning', 2: 'Error' }

def update_status_stale(stat, last_update_time):
    time_since_update = rospy.get_time() - last_update_time

    stale_status = 'OK'
    if time_since_update > 20 and time_since_update <= 35:
        stale_status = 'Lagging'
        if stat.level == DiagnosticStatus.OK:
            stat.message = stale_status
        elif stat.message.find(stale_status) < 0:
            stat.message = ', '.join([stat.message, stale_status])
        stat.level = max(stat.level, DiagnosticStatus.WARN)
    if time_since_update > 35:
        stale_status = 'Stale'
        if stat.level == DiagnosticStatus.OK:
            stat.message = stale_status
        elif stat.message.find(stale_status) < 0:
            stat.message = ', '.join([stat.message, stale_status])
        stat.level = max(stat.level, DiagnosticStatus.ERROR)


    stat.values.pop(0)
    stat.values.pop(0)
    stat.values.insert(0, KeyValue(key = 'Update Status', value = stale_status))
    stat.values.insert(1, KeyValue(key = 'Time Since Update', value = str(time_since_update)))
    

class MemMonitor():
    def __init__(self, hostname, diag_hostname):
        self._diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size = 100)

        self._mutex = threading.Lock()

        self._mem_level_warn = rospy.get_param('~mem_level_warn', mem_level_warn)
        self._mem_level_error = rospy.get_param('~mem_level_error', mem_level_error)

        self._usage_timer = None
        
        self._usage_stat = DiagnosticStatus()
        self._usage_stat.name = 'Memory Usage (%s)' % diag_hostname
        self._usage_stat.level = 1
        self._usage_stat.hardware_id = hostname
        self._usage_stat.message = 'No Data'
        self._usage_stat.values = [ KeyValue(key = 'Update Status', value = 'No Data' ),
                                    KeyValue(key = 'Time Since Last Update', value = 'N/A') ]

        self._last_usage_time = 0
        self._last_publish_time = 0

        # Start checking everything
        self.check_usage()

    ## Must have the lock to cancel everything
    def cancel_timers(self):
        if self._usage_timer:
            self._usage_timer.cancel()

    def check_memory(self):
        values = []
        level = DiagnosticStatus.OK
        msg = ''

        mem_dict = { 0: 'OK', 1: 'Low Memory', 2: 'Very Low Memory' }

        try:
            p = subprocess.Popen('free -tm',
                                stdout = subprocess.PIPE,
                                stderr = subprocess.PIPE, shell = True)
            stdout, stderr = p.communicate()
            retcode = p.returncode

            if retcode != 0:
                values.append(KeyValue(key = "\"free -tm\" Call Error", value = str(retcode)))
                return DiagnosticStatus.ERROR, values

            rows = stdout.split('\n')
            data = rows[1].split()
            total_mem_physical = data[1]
            used_mem_physical = data[2]
            free_mem_physical = data[3]
            data = rows[2].split()
            total_mem_swap = data[1]
            used_mem_swap = data[2]
            free_mem_swap = data[3]
            data = rows[3].split()
            total_mem = data[1]
            used_mem = data[2]
            free_mem = data[3]

            level = DiagnosticStatus.OK
            mem_usage = float(used_mem_physical)/float(total_mem_physical)
            if (mem_usage < self._mem_level_warn):
                level = DiagnosticStatus.OK
            elif (mem_usage < self._mem_level_error):
                level = DiagnosticStatus.WARN
            else:
                level = DiagnosticStatus.ERROR

            values.append(KeyValue(key = 'Memory Status', value = mem_dict[level]))
            values.append(KeyValue(key = 'Total Memory (Physical)', value = total_mem_physical+"M"))
            values.append(KeyValue(key = 'Used Memory (Physical)', value = used_mem_physical+"M"))
            values.append(KeyValue(key = 'Free Memory (Physical)', value = free_mem_physical+"M"))
            values.append(KeyValue(key = 'Total Memory (Swap)', value = total_mem_swap+"M"))
            values.append(KeyValue(key = 'Used Memory (Swap)', value = used_mem_swap+"M"))
            values.append(KeyValue(key = 'Free Memory (Swap)', value = free_mem_swap+"M"))
            values.append(KeyValue(key = 'Total Memory', value = total_mem+"M"))
            values.append(KeyValue(key = 'Used Memory', value = used_mem+"M"))
            values.append(KeyValue(key = 'Free Memory', value = free_mem+"M"))

            msg = mem_dict[level]
        except Exception, e:
            rospy.logerr(traceback.format_exc())
            msg = 'Memory Usage Check Error'
            values.append(KeyValue(key = msg, value = str(e)))
            level = DiagnosticStatus.ERROR

        return level, mem_dict[level], values

    def check_usage(self):
        if rospy.is_shutdown():
            with self._mutex:
                self.cancel_timers()
            return 

        diag_level = 0
        diag_vals = [ KeyValue(key = 'Update Status', value = 'OK' ),
                      KeyValue(key = 'Time Since Last Update', value = 0 )]
        diag_msgs = []

        # Check memory
        mem_level, mem_msg, mem_vals = self.check_memory()
        diag_vals.extend(mem_vals)
        if mem_level > 0:
            diag_msgs.append(mem_msg)
        diag_level = max(diag_level, mem_level)

        if diag_msgs and diag_level > 0:
            usage_msg = ', '.join(set(diag_msgs))
        else:
            usage_msg = stat_dict[diag_level]

        # Update status
        with self._mutex:
            self._last_usage_time = rospy.get_time()
            self._usage_stat.level = diag_level
            self._usage_stat.values = diag_vals
            
            self._usage_stat.message = usage_msg
            
            if not rospy.is_shutdown():
                self._usage_timer = threading.Timer(5.0, self.check_usage)
                self._usage_timer.start()
            else:
                self.cancel_timers()

    def publish_stats(self):
        with self._mutex:
            # Update everything with last update times
            update_status_stale(self._usage_stat, self._last_usage_time)

            msg = DiagnosticArray()
            msg.header.stamp = rospy.get_rostime()
            msg.status.append(self._usage_stat)

            if rospy.get_time() - self._last_publish_time > 0.5:
                self._diag_pub.publish(msg)
                self._last_publish_time = rospy.get_time()


if __name__ == '__main__':
    hostname = socket.gethostname()
    hostname = hostname.replace('-', '_')

    import optparse
    parser = optparse.OptionParser(usage="usage: mem_monitor.py [--diag-hostname=cX]")
    parser.add_option("--diag-hostname", dest="diag_hostname",
                      help="Computer name in diagnostics output (ex: 'c1')",
                      metavar="DIAG_HOSTNAME",
                      action="store", default = hostname)
    options, args = parser.parse_args(rospy.myargv())

    try:
        rospy.init_node('mem_monitor_%s' % hostname)
    except rospy.exceptions.ROSInitException:
        print >> sys.stderr, 'Memory monitor is unable to initialize node. Master may not be running.'
        sys.exit(0)

    mem_node = MemMonitor(hostname, options.diag_hostname)

    rate = rospy.Rate(1.0)
    try:
        while not rospy.is_shutdown():
            rate.sleep()
            mem_node.publish_stats()
    except KeyboardInterrupt:
        pass
    except Exception, e:
        traceback.print_exc()
        rospy.logerr(traceback.format_exc())

    mem_node.cancel_timers()
    sys.exit(0)