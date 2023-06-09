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

import rclpy
from rclpy.node import Node

import traceback
import threading
from threading import Timer
import sys, os, time
from time import sleep
import subprocess
import string
import multiprocessing

import socket

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

cpu_load_warn = 0.9
cpu_load_error = 1.1
cpu_load1_warn = 0.9
cpu_load5_warn = 0.8
cpu_temp_warn = 85.0
cpu_temp_error = 90.0

stat_dict = { 0: 'OK', 1: 'Warning', 2: 'Error' }

def update_status_stale(stat, last_update_time):
    time_since_update = rclpy.get_time() - last_update_time

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


class CPUMonitor(Node):
    def __init__(self, hostname, diag_hostname):
        super().__init__('cpu_monitor')
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 100)
        self._mutex = threading.Lock()

        self._check_core_temps = rospy.get_param('~check_core_temps', True)

        self._cpu_load_warn = rospy.get_param('~cpu_load_warn', cpu_load_warn)
        self._cpu_load_error = rospy.get_param('~cpu_load_error', cpu_load_error)
        self._cpu_load1_warn = rospy.get_param('~cpu_load1_warn', cpu_load1_warn)
        self._cpu_load5_warn = rospy.get_param('~cpu_load5_warn', cpu_load5_warn)
        self._cpu_temp_warn = rospy.get_param('~cpu_temp_warn', cpu_temp_warn)
        self._cpu_temp_error = rospy.get_param('~cpu_temp_error', cpu_temp_error)

        self._num_cores = multiprocessing.cpu_count()

        self._temps_timer = None
        self._usage_timer = None
        
        # Get temp_input files
        self._temp_vals = self.get_core_temp_names()

        # CPU stats
        self._temp_stat = DiagnosticStatus()
        self._temp_stat.name = 'CPU Temperature (%s)' % diag_hostname
        self._temp_stat.level = 1
        self._temp_stat.hardware_id = hostname
        self._temp_stat.message = 'No Data'
        self._temp_stat.values = [ KeyValue(key = 'Update Status', value = 'No Data' ),
                                   KeyValue(key = 'Time Since Last Update', value = 'N/A') ]

        self._usage_stat = DiagnosticStatus()
        self._usage_stat.name = 'CPU Usage (%s)' % diag_hostname
        self._usage_stat.level = 1
        self._usage_stat.hardware_id = hostname
        self._usage_stat.message = 'No Data'
        self._usage_stat.values = [ KeyValue(key = 'Update Status', value = 'No Data' ),
                                    KeyValue(key = 'Time Since Last Update', value = 'N/A') ]

        self._last_temp_time = 0
        self._last_usage_time = 0
        self._last_publish_time = 0

        self._usage_old = 0
        self._has_warned_mpstat = False
        self._has_error_core_count = False

        # Start checking everything
        self.check_temps()
        self.check_usage()

    # Restart temperature checking 
    def _restart_temp_check(self):
        rospy.logerr('Restarting temperature check thread in cpu_monitor. This should not happen')
        try:
            with self._mutex:
                if self._temps_timer:
                    self._temps_timer.cancel()
                
            self.check_temps()
        except Exception, e:
            rospy.logerr('Unable to restart temp thread. Error: %s' % traceback.format_exc())
            

    ## Must have the lock to cancel everything
    def cancel_timers(self):
        if self._temps_timer:
            self._temps_timer.cancel()

        if self._usage_timer:
            self._usage_timer.cancel()

    ##\brief Check CPU core temps
    ##
    ## Use 'find /sys -name temp1_input' to find cores
    ## Read from every core, divide by 1000
    def check_core_temps(self, sys_temp_strings):
        diag_vals = []
        diag_level = 0
        diag_msgs = []

        for index, temp_str in enumerate(sys_temp_strings):
            if len(temp_str) < 5:
                continue

            cmd = 'cat %s' % temp_str
            p = subprocess.Popen(cmd, stdout = subprocess.PIPE,
                                stderr = subprocess.PIPE, shell = True)
            stdout, stderr = p.communicate()
            retcode = p.returncode

            if retcode != 0:
                diag_level = DiagnosticStatus.ERROR
                diag_msg = [ 'Core Temperature Error' ]
                diag_vals = [ KeyValue(key = 'Core Temperature Error', value = stderr),
                              KeyValue(key = 'Output', value = stdout) ]
                return diag_vals, diag_msgs, diag_level

            tmp = stdout.strip()
            if unicode(tmp).isnumeric():
                temp = float(tmp) / 1000
                diag_vals.append(KeyValue(key = 'Core %d Temperature' % index, value = str(temp)+"DegC"))

                if temp >= self._cpu_temp_warn:
                    diag_level = max(diag_level, DiagnosticStatus.WARN)
                    diag_msgs.append('Warm')
                elif temp >= self._cpu_temp_error:
                    diag_level = max(diag_level, DiagnosticStatus.ERROR)
                    diag_msgs.append('Hot')
            else:
                diag_level = max(diag_level, DiagnosticStatus.ERROR) # Error if not numeric value
                diag_vals.append(KeyValue(key = 'Core %s Temperature' % index, value = tmp))

        return diag_vals, diag_msgs, diag_level

    ## Checks clock speed from reading from CPU info
    def check_clock_speed(self):
        vals = []
        msgs = []
        lvl = DiagnosticStatus.OK

        try:
            p = subprocess.Popen('cat /proc/cpuinfo | grep MHz',
                                stdout = subprocess.PIPE,
                                stderr = subprocess.PIPE, shell = True)
            stdout, stderr = p.communicate()
            retcode = p.returncode

            if retcode != 0:
                lvl = DiagnosticStatus.ERROR
                msgs = [ 'Clock speed error' ]
                vals = [ KeyValue(key = 'Clock speed error', value = stderr),
                        KeyValue(key = 'Output', value = stdout) ]

                return (vals, msgs, lvl)

            for index, ln in enumerate(stdout.split('\n')):
                words = ln.split(':')
                if len(words) < 2:
                    continue

                speed = words[1].strip().split('.')[0] # Conversion to float doesn't work with decimal
                vals.append(KeyValue(key = 'Core %d Clock Speed' % index, value = speed+"MHz"))

        except Exception, e:
            rospy.logerr(traceback.format_exc())
            lvl = DiagnosticStatus.ERROR
            msgs.append('Exception')
            vals.append(KeyValue(key = 'Exception', value = traceback.format_exc()))

        return vals, msgs, lvl


    # Add msgs output, too
    ##\brief Uses 'uptime' to see load average
    def check_uptime(self):
        level = DiagnosticStatus.OK
        vals = []

        load_dict = { 0: 'OK', 1: 'High Load', 2: 'Very High Load' }

        try:
            p = subprocess.Popen('uptime', stdout = subprocess.PIPE,
                                stderr = subprocess.PIPE, shell = True)
            stdout, stderr = p.communicate()
            retcode = p.returncode

            if retcode != 0:
                vals.append(KeyValue(key = 'uptime Failed', value = stderr))
                return DiagnosticStatus.ERROR, vals

            upvals = stdout.split()
            load1 = float(upvals[-3].rstrip(','))/self._num_cores
            load5 = float(upvals[-2].rstrip(','))/self._num_cores
            load15 = float(upvals[-1])/self._num_cores

            # Give warning if we go over load limit
            if load1 > self._cpu_load1_warn or load5 > self._cpu_load5_warn:
                level = DiagnosticStatus.WARN

            vals.append(KeyValue(key = 'Load Average Status', value = load_dict[level]))
            vals.append(KeyValue(key = 'Load Average (1min)', value = str(load1*1e2)+"%"))
            vals.append(KeyValue(key = 'Load Average (5min)', value = str(load5*1e2)+"%"))
            vals.append(KeyValue(key = 'Load Average (15min)', value = str(load15*1e2)+"%"))

        except Exception, e:
            rospy.logerr(traceback.format_exc())
            level = DiagnosticStatus.ERROR
            vals.append(KeyValue(key = 'Load Average Status', value = traceback.format_exc()))

        return level, load_dict[level], vals

    ##\brief Use mpstat to find CPU usage
    ##
    def check_mpstat(self):
        vals = []
        mp_level = DiagnosticStatus.OK

        load_dict = { 0: 'OK', 1: 'High Load', 2: 'Error' }

        try:
            p = subprocess.Popen('mpstat -P ALL 1 1',
                                stdout = subprocess.PIPE,
                                stderr = subprocess.PIPE, shell = True)
            stdout, stderr = p.communicate()
            retcode = p.returncode

            if retcode != 0:
                if not self._has_warned_mpstat:
                    rospy.logerr("mpstat failed to run for cpu_monitor. Return code %d.", retcode)
                    self._has_warned_mpstat = True

                mp_level = DiagnosticStatus.ERROR
                vals.append(KeyValue(key = '\"mpstat\" Call Error', value = str(retcode)))
                return mp_level, 'Unable to Check CPU Usage', vals

            # Check which column '%idle' is, #4539
            # mpstat output changed between 8.06 and 8.1
            rows = stdout.split('\n')
            col_names = rows[2].split()
            idle_col = -1 if (len(col_names) > 2 and col_names[-1] == '%idle') else -2

            num_cores = 0
            cores_loaded = 0
            for index, row in enumerate(stdout.split('\n')):
                if index < 3:
                    continue

                # Skip row containing 'all' data
                if row.find('all') > -1:
                    continue

                lst = row.split()
                if len(lst) < 8:
                    continue

                ## Ignore 'Average: ...' data
                if lst[0].startswith('Average'):
                    continue

                cpu_name = '%d' % (num_cores)
                idle = lst[idle_col]
                user = lst[3]
                nice = lst[4]
                system = lst[5]

                core_level = 0
                usage = (float(user)+float(nice))*1e-2
                if usage > 10.0: # wrong reading, use old reading instead
                    rospy.logwarn('Read CPU usage of %f percent. Reverting to previous reading of %f percent'%(usage, self._usage_old))
                    usage = self._usage_old
                self._usage_old = usage

                if usage >= self._cpu_load_warn:
                    cores_loaded += 1
                    core_level = DiagnosticStatus.WARN
                elif usage >= self._cpu_load_error:
                    core_level = DiagnosticStatus.ERROR

                vals.append(KeyValue(key = 'Core %s Status' % cpu_name, value = load_dict[core_level]))
                vals.append(KeyValue(key = 'Core %s User' % cpu_name, value = user+"%"))
                vals.append(KeyValue(key = 'Core %s Nice' % cpu_name, value = nice+"%"))
                vals.append(KeyValue(key = 'Core %s System' % cpu_name, value = system+"%"))
                vals.append(KeyValue(key = 'Core %s Idle' % cpu_name, value = idle+"%"))

                num_cores += 1

            # Warn for high load only if we have <= 2 cores that aren't loaded
            if num_cores - cores_loaded <= 2 and num_cores > 2:
                mp_level = DiagnosticStatus.WARN

            if not self._num_cores:
              self._num_cores = num_cores

            # Check the number of cores if self._num_cores > 0, #4850
            if self._num_cores != num_cores:
                mp_level = DiagnosticStatus.ERROR
                if not self._has_error_core_count:
                    rospy.logerr('Error checking number of cores. Expected %d, got %d. Computer may have not booted properly.',
                                  self._num_cores, num_cores)
                    self._has_error_core_count = True
                return DiagnosticStatus.ERROR, 'Incorrect number of CPU cores', vals

        except Exception, e:
            mp_level = DiagnosticStatus.ERROR
            vals.append(KeyValue(key = 'mpstat Exception', value = str(e)))

        return mp_level, load_dict[mp_level], vals

    ## Returns names for core temperature files
    ## Returns list of names, each name can be read like file
    def get_core_temp_names(self):
        temp_vals = []
        try:
            p = subprocess.Popen('find /sys/devices -name temp1_input',
                                stdout = subprocess.PIPE,
                                stderr = subprocess.PIPE, shell = True)
            stdout, stderr = p.communicate()
            retcode = p.returncode

            if retcode != 0:
                rospy.logerr('Error find core temp locations: %s' % stderr)
                return []

            for ln in stdout.split('\n'):
                temp_vals.append(ln.strip())

            return temp_vals
        except:
            rospy.logerr('Exception finding temp vals: %s' % traceback.format_exc())
            return []

    ## Call every 10sec at minimum
    def check_temps(self):
        if rospy.is_shutdown():
            with self._mutex:
                self.cancel_timers()
            return

        diag_vals = [ KeyValue(key = 'Update Status', value = 'OK' ),
                      KeyValue(key = 'Time Since Last Update', value = str(0) ) ]
        diag_msgs = []
        diag_level = 0

        if self._check_core_temps:
            core_vals, core_msgs, core_level = self.check_core_temps(self._temp_vals)
            diag_vals.extend(core_vals)
            diag_msgs.extend(core_msgs)
            diag_level = max(diag_level, core_level)

        diag_log = set(diag_msgs)
        if len(diag_log) > 0:
            message = ', '.join(diag_log)
        else:
            message = stat_dict[diag_level]

        with self._mutex:
            self._last_temp_time = rospy.get_time()
            
            self._temp_stat.level = diag_level
            self._temp_stat.message = message
            self._temp_stat.values = diag_vals
            
            if not rospy.is_shutdown():
                self._temps_timer = threading.Timer(5.0, self.check_temps)
                self._temps_timer.start()
            else:
                self.cancel_timers()

    def check_usage(self):
        if rospy.is_shutdown():
            with self._mutex:
                self.cancel_timers()
            return 

        diag_level = 0
        diag_vals = [ KeyValue(key = 'Update Status', value = 'OK' ),
                      KeyValue(key = 'Time Since Last Update', value = 0 )]
        diag_msgs = []

        # Check clock speed
        clock_vals, clock_msgs, clock_level = self.check_clock_speed()
        diag_vals.extend(clock_vals)
        diag_msgs.extend(clock_msgs)
        diag_level = max(diag_level, clock_level)

        # Check mpstat
        mp_level, mp_msg, mp_vals = self.check_mpstat()
        diag_vals.extend(mp_vals)
        if mp_level > 0:
            diag_msgs.append(mp_msg)
        diag_level = max(diag_level, mp_level)
            
        # Check uptime
        uptime_level, up_msg, up_vals = self.check_uptime()
        diag_vals.extend(up_vals)
        if uptime_level > 0:
            diag_msgs.append(up_msg)
        diag_level = max(diag_level, uptime_level)
        
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
            update_status_stale(self._temp_stat, self._last_temp_time)
            update_status_stale(self._usage_stat, self._last_usage_time)

            msg = DiagnosticArray()
            msg.header.stamp = rospy.get_rostime()
            msg.status.append(self._temp_stat)
            msg.status.append(self._usage_stat)

            if rospy.get_time() - self._last_publish_time > 0.5:
                self._diag_pub.publish(msg)
                self._last_publish_time = rospy.get_time()

        
        # Restart temperature checking if it goes stale, #4171
        # Need to run this without mutex
        if rospy.get_time() - self._last_temp_time > 90: 
            self._restart_temp_check()


if __name__ == '__main__':
    hostname = socket.gethostname()
    hostname = hostname.replace('-', '_')

    import optparse
    parser = optparse.OptionParser(usage="usage: cpu_monitor.py [--diag-hostname=cX]")
    parser.add_option("--diag-hostname", dest="diag_hostname",
                      help="Computer name in diagnostics output (ex: 'c1')",
                      metavar="DIAG_HOSTNAME",
                      action="store", default = hostname)
    options, args = parser.parse_args(rospy.myargv())

    try:
        rospy.init_node('cpu_monitor_%s' % hostname)
    except rospy.exceptions.ROSInitException:
        print >> sys.stderr, 'CPU monitor is unable to initialize node. Master may not be running.'
        sys.exit(0)

    cpu_node = CPUMonitor(hostname, options.diag_hostname)

    rate = rospy.Rate(1.0)
    try:
        while not rospy.is_shutdown():
            rate.sleep()
            cpu_node.publish_stats()
    except KeyboardInterrupt:
        pass
    except Exception, e:
        traceback.print_exc()
        rospy.logerr(traceback.format_exc())

    cpu_node.cancel_timers()
    sys.exit(0)