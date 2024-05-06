# Copyright (c) 2009 Advanced Micro Devices, Inc.
# Copyright (c) 2016 Georgia Institute of Technology
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath

from .Ruby import (
    create_directories,
    create_topology,
)


#
# Declare caches used by the protocol
#
class L2Cache(RubyCache):
    pass


def define_options(parser):
    return


def create_system(
    options, full_system, system, dma_ports, bootmem, ruby_system, cpus
):
    if buildEnv["PROTOCOL"] != "Synfull_standalone":
        panic("This script requires Synfull_standalone protocol to be built.")

    cpu_sequencers = []

    #
    # The Synfull_standalone protocol does not support fs nor dma
    #
    assert dma_ports == []

    #
    # The ruby network creation expects the list of nodes in the system to be
    # consistent with the NetDest list.
    # Therefore the l1 controller nodes must be listed before
    # the directory nodes and directory nodes before dma nodes, etc.
    l2_cntrl_nodes = []

    #
    # Must create the individual controllers before the network to ensure the
    # controller constructors are called before the network constructor
    #

    for i in range(options.num_cpus):
        #
        # First create the Ruby objects associated with this cpu
        # Only one cache exists for this protocol, so by default use the L1D
        # config parameters.
        #
        cache = L2Cache(size=options.l2_size, assoc=options.l2_assoc)

        #
        # Only one unified L1 cache exists.  Can cache instructions and data.
        #
        l2_cntrl = L2Cache_Controller(
            version=i, cacheMemory=cache, ruby_system=ruby_system
        )

        cpu_seq = RubySequencer(
            dcache=cache, ruby_system=ruby_system
        )

        l2_cntrl.sequencer = cpu_seq
        exec("ruby_system.l2_cntrl%d = l2_cntrl" % i)

        # Add controllers and sequencers to the appropriate lists
        cpu_sequencers.append(cpu_seq)
        l2_cntrl_nodes.append(l2_cntrl)

        # Connect the L1 controllers and the network
        l2_cntrl.mandatoryQueue = MessageBuffer()
        l2_cntrl.RequestFromCache = MessageBuffer()
        l2_cntrl.RequestFromCache.out_port = ruby_system.network.in_port
        l2_cntrl.ResponseFromCache = MessageBuffer()
        l2_cntrl.ResponseFromCache.out_port = ruby_system.network.in_port
        l2_cntrl.RequestToCache = MessageBuffer()
        l2_cntrl.RequestToCache.in_port = ruby_system.network.out_port
        l2_cntrl.ResponseToCache = MessageBuffer()
        l2_cntrl.ResponseToCache.in_port = ruby_system.network.out_port

    mem_dir_cntrl_nodes, rom_dir_cntrl_node = create_directories(
        options, bootmem, ruby_system, system
    )
    dir_cntrl_nodes = mem_dir_cntrl_nodes[:]
    if rom_dir_cntrl_node is not None:
        dir_cntrl_nodes.append(rom_dir_cntrl_node)
    for dir_cntrl in dir_cntrl_nodes:
        # Connect the directory controllers and the network
        dir_cntrl.RequestToDir = MessageBuffer()
        dir_cntrl.RequestToDir.in_port = ruby_system.network.out_port
        dir_cntrl.ResponseToDir = MessageBuffer()
        dir_cntrl.ResponseToDir.in_port = ruby_system.network.out_port
        dir_cntrl.RequestToCache = MessageBuffer()
        dir_cntrl.RequestToCache.out_port = ruby_system.network.in_port
        dir_cntrl.ResponseToCache = MessageBuffer()
        dir_cntrl.ResponseToCache.out_port = ruby_system.network.in_port

    all_cntrls = l2_cntrl_nodes + dir_cntrl_nodes
    ruby_system.network.number_of_virtual_networks = 3
    topology = create_topology(all_cntrls, options)
    return (cpu_sequencers, mem_dir_cntrl_nodes, topology)
