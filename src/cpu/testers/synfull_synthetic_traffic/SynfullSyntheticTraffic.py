from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *

#build/Synfull/gem5.debug configs/example/synfull_synth_traffic.py --num-cpus=16 --num-dirs=16 --network=garnet --topology=Mesh_XY --mesh-rows=4 --sim-cycles=100000000
class SynfullSyntheticTraffic(ClockedObject):
    type = "SynfullSyntheticTraffic"
    cxx_header = (
        "cpu/testers/synfull_synthetic_traffic/SynfullSyntheticTraffic.hh"
    )
    cxx_class = "gem5::SynfullSyntheticTraffic"

    block_offset = Param.Int(6, "block offset in bits")
    num_dest = Param.Int(1, "Number of Destinations")
    memory_size = Param.Int(65536, "memory size")
    sim_cycles = Param.Int(1000, "Number of simulation cycles")
    num_packets_max = Param.Int(
        -1,
        "Max number of packets to send. \
                        Default is to keep sending till simulation ends",
    )
    single_sender = Param.Int(
        -1,
        "Send only from this node. \
                                   By default every node sends",
    )
    single_dest = Param.Int(
        -1,
        "Send only to this dest. \
                                 Default depends on traffic_type",
    )    
    inj_rate = Param.Float(0.1, "Packet injection rate")
    inj_vnet = Param.Int(
        -1,
        "Vnet to inject in. \
                              0 and 1 are 1-flit, 2 is 5-flit. \
                                Default is to inject in all three vnets",
    )
    precision = Param.Int(
        3,
        "Number of digits of precision \
                              after decimal point",
    )
    response_limit = Param.Cycles(
        5000000,
        "Cycles before exiting \
                                            due to lack of progress",
    )
    test = RequestPort("Port to the memory system to test")
    system = Param.System(Parent.any, "System we belong to")
