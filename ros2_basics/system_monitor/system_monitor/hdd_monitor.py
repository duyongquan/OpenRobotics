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

import socket

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

hdd_level_warn = 0.95
hdd_level_error = 0.99
hdd_temp_warn = 55.0
hdd_temp_error = 70.0

stat_dict = { 0: 'OK', 1: 'Warning', 2: 'Error' }
temp_dict = { 0: 'OK', 1: 'Hot', 2: 'Critical Hot' }
usage_dict = { 0: 'OK', 1: 'Low Disk Space', 2: 'Very Low Disk Space' }

REMOVABLE = ['/dev/sg1', '/dev/sdb'] # Store removable drives so we can ignore if removed

## Connects to hddtemp daemon to get temp, HDD make.
def get_hddtemp_data(hostname = 'localhost', port = 7634):
    try:
        hdd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        hdd_sock.connect((hostname, port))
        sock_data = ''
        while True:
            newdat = hdd_sock.recv(1024)
            if len(newdat) == 0:
                break
            sock_data = sock_data + newdat
        hdd_sock.close()

        sock_vals = sock_data.split('|')

        # Format of output looks like ' | DRIVE | MAKE | TEMP | ' 
        idx = 0

        drives = []
        makes = []
        temps = []
        while idx + 5 < len(sock_vals):
            this_drive = sock_vals[idx + 1]
            this_make = sock_vals[idx + 2]
            this_temp = sock_vals[idx + 3]

            # Sometimes we get duplicate makes if hard drives are mounted
            # to two different points
            if this_make in makes:
                idx += 5
                continue

            drives.append(this_drive)
            makes.append(this_make)
            temps.append(this_temp)
                        
            idx += 5

        return True, drives, makes, temps
    except:
        rospy.loginfo(traceback.format_exc())
        return False, [ 'Exception' ], [ traceback.format_exc() ], [ 0 ]

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

class hdd_monitor():
    def __init__(self, hostname, diag_hostname):
        self._mutex = threading.Lock()

        self._hostname = hostname
        self._no_temp = rospy.get_param('~no_hdd_temp', False)
        self._no_temp_warn = rospy.get_param('~no_hdd_temp_warn', False)
        self._hdd_level_warn = rospy.get_param('~hdd_level_warn', hdd_level_warn)
        self._hdd_level_error = rospy.get_param('~hdd_level_error', hdd_level_error)
        self._hdd_temp_warn = rospy.get_param('~hdd_temp_warn', hdd_temp_warn)
        self._hdd_temp_error = rospy.get_param('~hdd_temp_error', hdd_temp_error)

        self._diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size = 100)

        self._last_publish_time = 0

        self._last_temp_time = 0
        self._temp_timer = None
        if not self._no_temp:
          self._temp_stat = DiagnosticStatus()
          self._temp_stat.name = "HDD Temperature (%s)" % diag_hostname
          self._temp_stat.level = DiagnosticStatus.ERROR
          self._temp_stat.hardware_id = hostname
          self._temp_stat.message = 'No Data'
          self._temp_stat.values = [ KeyValue(key = 'Update Status', value = 'No Data'),
                                    KeyValue(key = 'Time Since Last Update', value = 'N/A') ]
          self.check_temps()

        self._last_usage_time = 0
        self._usage_timer = None
        self._usage_stat = DiagnosticStatus()
        self._usage_stat.level = DiagnosticStatus.ERROR
        self._usage_stat.hardware_id = hostname
        self._usage_stat.name = 'HDD Usage (%s)' % diag_hostname
        self._usage_stat.values = [ KeyValue(key = 'Update Status', value = 'No Data' ),
                                    KeyValue(key = 'Time Since Last Update', value = 'N/A') ]
        self.check_disk_usage()

    ## Must have the lock to cancel everything
    def cancel_timers(self):
        if self._temp_timer:
            self._temp_timer.cancel()
            self._temp_timer = None

        if self._usage_timer:
            self._usage_timer.cancel()
            self._usage_timer = None

    def check_temps(self):
        if rospy.is_shutdown():
            with self._mutex:
                self.cancel_timers()
            return

        diag_strs = [ KeyValue(key = 'Update Status', value = 'OK' ) ,
                      KeyValue(key = 'Time Since Last Update', value = '0' ) ]
        diag_level = DiagnosticStatus.OK
        diag_message = 'OK'

        temp_ok, drives, makes, temps = get_hddtemp_data()

        for index in range(0, len(drives)):
            temp = temps[index]
            
            if not unicode(temp).isnumeric() and drives[index] not in REMOVABLE:
                temp_level = DiagnosticStatus.ERROR
                temp_ok = False
            elif not unicode(temp).isnumeric() and drives[index] in REMOVABLE:
                temp_level = DiagnosticStatus.OK
                temp = "Removed"
            else:
                temp_level = DiagnosticStatus.OK
                if float(temp) >= self._hdd_temp_warn:
                    temp_level = DiagnosticStatus.WARN
                if float(temp) >= self._hdd_temp_error:
                    temp_level = DiagnosticStatus.ERROR

            diag_level = max(diag_level, temp_level)
            
            diag_strs.append(KeyValue(key = 'Disk %d Temperature Status' % index, value = temp_dict[temp_level]))
            diag_strs.append(KeyValue(key = 'Disk %d Mount Pt.' % index, value = drives[index]))
            diag_strs.append(KeyValue(key = 'Disk %d Device ID' % index, value = makes[index]))
            diag_strs.append(KeyValue(key = 'Disk %d Temperature' % index, value = str(temp)+"DegC"))

        if not temp_ok:
            diag_level = DiagnosticStatus.ERROR

        with self._mutex:
            self._last_temp_time = rospy.get_time()
            self._temp_stat.values = diag_strs
            self._temp_stat.level = diag_level

            # Give No Data message if we have no reading
            self._temp_stat.message = temp_dict[diag_level]
            if not temp_ok:
                self._temp_stat.message = 'Error'

            if self._no_temp_warn and temp_ok:
                self._temp_stat.level = DiagnosticStatus.OK

            if not rospy.is_shutdown():
                self._temp_timer = threading.Timer(10.0, self.check_temps)
                self._temp_timer.start()
            else:
                self.cancel_timers()

    def check_disk_usage(self):
        if rospy.is_shutdown():
            with self._mutex:
                self.cancel_timers()
            return

        diag_vals = [ KeyValue(key = 'Update Status', value = 'OK' ),
                      KeyValue(key = 'Time Since Last Update', value = '0' ) ]
        diag_level = DiagnosticStatus.OK
        diag_message = 'OK'

        try:
            p = subprocess.Popen(["df", "-Pht", "ext4"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = p.communicate()
            retcode = p.returncode

            if (retcode == 0 or retcode == 1):
                diag_vals.append(KeyValue(key = 'Disk Space Reading', value = 'OK'))
                rows = stdout.split('\n')
                del rows[0]
                row_count = 0
                
                for row in rows:
                    if len(row.split()) < 2:
                        continue
                    if unicode(row.split()[0]) == "none":
                        continue

                    row_count += 1
                    g_available = row.split()[-3]
                    g_use = row.split()[-2]
                    name = row.split()[0]
                    size = row.split()[1]
                    mount_pt = row.split()[-1]

                    hdd_usage = float(g_use.replace("%", ""))*1e-2
                    if (hdd_usage < self._hdd_level_warn):
                        level = DiagnosticStatus.OK
                    elif (hdd_usage < self._hdd_level_error):
                        level = DiagnosticStatus.WARN
                    else:
                        level = DiagnosticStatus.ERROR

                    diag_vals.append(KeyValue(
                            key = 'Disk %d Name' % row_count, value = name))
                    diag_vals.append(KeyValue(
                            key = 'Disk %d Size' % row_count, value = size))
                    diag_vals.append(KeyValue(
                            key = 'Disk %d Available' % row_count, value = g_available))
                    diag_vals.append(KeyValue(
                            key = 'Disk %d Use' % row_count, value = g_use))
                    diag_vals.append(KeyValue(
                            key = 'Disk %d Status' % row_count, value = stat_dict[level]))
                    diag_vals.append(KeyValue(
                            key = 'Disk %d Mount Point' % row_count, value = mount_pt))

                    diag_level = max(diag_level, level)
                    diag_message = usage_dict[diag_level]

            else:
                diag_vals.append(KeyValue(key = 'Disk Space Reading', value = 'Failed'))
                diag_level = DiagnosticStatus.ERROR
                diag_message = stat_dict[diag_level]


        except:
            rospy.logerr(traceback.format_exc())

            diag_vals.append(KeyValue(key = 'Disk Space Reading', value = 'Exception'))
            diag_vals.append(KeyValue(key = 'Disk Space Ex', value = traceback.format_exc()))

            diag_level = DiagnosticStatus.ERROR
            diag_message = stat_dict[diag_level]

        # Update status
        with self._mutex:
            self._last_usage_time = rospy.get_time()
            self._usage_stat.values = diag_vals
            self._usage_stat.message = diag_message
            self._usage_stat.level = diag_level
            
            if not rospy.is_shutdown():
                self._usage_timer = threading.Timer(5.0, self.check_disk_usage)
                self._usage_timer.start()
            else:
                self.cancel_timers()


    def publish_stats(self):
        with self._mutex:
            msg = DiagnosticArray()
            msg.header.stamp = rospy.get_rostime()
            
            if not self._no_temp:
              update_status_stale(self._temp_stat, self._last_temp_time)
              msg.status.append(self._temp_stat)
              
            update_status_stale(self._usage_stat, self._last_usage_time)
            msg.status.append(self._usage_stat)

            if rospy.get_time() - self._last_publish_time > 0.5:
                self._diag_pub.publish(msg)
                self._last_publish_time = rospy.get_time()




##\todo Need to check HDD input/output too using iostat

if __name__ == '__main__':
    hostname = socket.gethostname()
    hostname = hostname.replace('-', '_')

    import optparse
    parser = optparse.OptionParser(usage="usage: hdd_monitor.py [--diag-hostname=cX]")
    parser.add_option("--diag-hostname", dest="diag_hostname",
                      help="Computer name in diagnostics output (ex: 'c1')",
                      metavar="DIAG_HOSTNAME",
                      action="store", default = hostname)
    options, args = parser.parse_args(rospy.myargv())

    try:
        rospy.init_node('hdd_monitor_%s' % hostname)
    except rospy.exceptions.ROSInitException:
        print 'HDD monitor is unable to initialize node. Master may not be running.'
        sys.exit(0)
        
    hdd_monitor = hdd_monitor(hostname, options.diag_hostname)
    rate = rospy.Rate(1.0)

    try:
        while not rospy.is_shutdown():
            rate.sleep()
            hdd_monitor.publish_stats()
    except KeyboardInterrupt:
        pass
    except Exception, e:
        traceback.print_exc()

    hdd_monitor.cancel_timers()
    sys.exit(0)