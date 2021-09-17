#!/usr/bin/env python

# /*******************************************************************************
# * Copyright (c) 2018 Elhay Rauper
# * All rights reserved.
# *
# * Redistribution and use in source and binary forms, with or without
# * modification, are permitted (subject to the limitations in the disclaimer
# * below) provided that the following conditions are met:
# *
# *     * Redistributions of source code must retain the above copyright notice,
# *     this list of conditions and the following disclaimer.
# *
# *     * Redistributions in binary form must reproduce the above copyright
# *     notice, this list of conditions and the following disclaimer in the
# *     documentation and/or other materials provided with the distribution.
# *
# *     * Neither the name of the copyright holder nor the names of its
# *     contributors may be used to endorse or promote products derived from this
# *     software without specific prior written permission.
# *
# * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
# * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
# * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
# * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# * POSSIBILITY OF SUCH DAMAGE.
# *******************************************************************************/

import rospy
import psutil
from psutil_ros.msg import Cpu
from psutil_ros.msg import VirtualMemory
from psutil_ros.msg import Network
from psutil_ros.msg import Temperatures
from psutil_ros.msg import SensorTemp

def if_valid(incoming_value,default_value):
    """Returns the incoming_value if it is valid, otherwise returns the default"""
    if incoming_value is not None:
        return incoming_value
    return default_value

if __name__ == '__main__':

    rospy.init_node('psutil_ros_node', anonymous=True)

    # parameters
    loop_rate = rospy.get_param('~loop_rate', 1)
    publish_cpu = rospy.get_param('~publish_cpu', True)
    publish_memo = rospy.get_param('~publish_memo', True)
    publish_temps = rospy.get_param('~publish_temps', True)
    publish_network = rospy.get_param('~publish_network', True)

    # publishers
    cpu_pub = rospy.Publisher('psutil_ros/cpu', Cpu, queue_size=10)
    mem_pub = rospy.Publisher(
        'psutil_ros/virtual_memory', VirtualMemory, queue_size=10)
    temps_pub = rospy.Publisher(
        'psutil_ros/temps', Temperatures, queue_size=10)
    network_pub = rospy.Publisher('psutil_ros/network', Network, queue_size=10)

    rate = rospy.Rate(loop_rate)
    is_first = True

    while not rospy.is_shutdown():

        # messages
        cpu_msg = Cpu()
        memo_msg = VirtualMemory()
        temps_msg = Temperatures()
        network_msg = Network()

        # ********* CPU *********
        cpu_msg.usage = if_valid(psutil.cpu_percent(interval=0),cpu_msg.usage)
        cpu_msg.usage_per = if_valid(psutil.cpu_percent(interval=0,percpu=True),cpu_msg.usage_per)

        #ARM processors return None
        cpu_msg.physical_cores = if_valid(psutil.cpu_count(logical=False),cpu_msg.physical_cores)
        cpu_msg.cores = if_valid(psutil.cpu_count(), cpu_msg.cores)
        frequency = psutil.cpu_freq()
        cpu_msg.current_freq = if_valid(frequency.current, cpu_msg.current_freq)
        cpu_msg.min_freq = if_valid(frequency.min, cpu_msg.min_freq)
        cpu_msg.max_freq = if_valid(frequency.max, cpu_msg.max_freq)
        frequency_per=psutil.cpu_freq(percpu=True)
        # require more than one, otherwise it is the same as current_freq
        if frequency_per and len(frequency_per)>1:
            currents_per=[]
            for freq in frequency_per:
                currents_per.append(freq.current)
            cpu_msg.current_freq_per=currents_per
        
        # read cpu data only after first cycle, because we are using interval=0, and psutil'''
        # documentation says in this case, the first read should be ignored'''
        if not is_first and publish_cpu:
            cpu_pub.publish(cpu_msg)

        # **** Virtual Memo *****
        if publish_memo:
            vm = psutil.virtual_memory()
            memo_msg.total = if_valid(vm.total, memo_msg.total)
            memo_msg.available = if_valid(vm.available, memo_msg.available)
            memo_msg.used = if_valid(vm.used, memo_msg.used)
            memo_msg.free = if_valid(vm.free, memo_msg.free)
            memo_msg.percent = if_valid(vm.percent, memo_msg.percent)

            mem_pub.publish(memo_msg)

        # ***** Temperatures ****
        if publish_temps:
            temps = psutil.sensors_temperatures()

            for key, value in temps.iteritems():
                sensor_temp_msg = SensorTemp()
                sensor_temp_msg.label = key
                if len(value) > 0:
                    first_value = value[0]
                    if first_value:
                        sensor_temp_msg.current = if_valid(first_value.current,sensor_temp_msg.current)
                        sensor_temp_msg.high = if_valid(first_value.high,sensor_temp_msg.high)
                        sensor_temp_msg.critical = if_valid(first_value.critical,sensor_temp_msg.critical)
                temps_msg.temps.append(sensor_temp_msg)

            temps_pub.publish(temps_msg)

        # ******* Network *******
        if publish_network:
            net = psutil.net_io_counters()
            network_msg.bytes_sent = if_valid(net.bytes_sent,network_msg.bytes_sent)
            network_msg.bytes_recv = if_valid(net.bytes_recv,network_msg.bytes_recv )
            network_msg.packets_sent = if_valid(net.packets_sent,network_msg.packets_sent)
            network_msg.packets_recv = if_valid(net.packets_recv,network_msg.packets_recv)
            network_msg.errin = if_valid(net.errin,network_msg.errin)
            network_msg.errout = if_valid(net.errout,network_msg.errout)
            network_msg.dropin = if_valid(net.dropin,network_msg.dropin)
            network_msg.dropout = if_valid(net.dropout,network_msg.dropout)

            network_pub.publish(network_msg)

        rate.sleep()
        is_first = False
