#include "cpu/testers/synfull_synthetic_traffic/Global.hh"
#include "cpu/testers/synfull_synthetic_traffic/Messages.hh"
#include "cpu/testers/synfull_synthetic_traffic/ModelRead.hh"
#include "cpu/testers/synfull_synthetic_traffic/PacketQueue.hh"
#include "cpu/testers/synfull_synthetic_traffic/SynfullSyntheticTraffic.hh"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <list>
#include <set>
#include <string>
#include <vector>

#include "base/logging.hh"
#include "base/random.hh"
#include "base/statistics.hh"
#include "debug/SynfullSyntheticTraffic.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "sim/sim_events.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

namespace gem5
{

int TESTER_NETWORK=0;
bool modelLoaded=false;
std::list<InjectReqMsg> packets;

unsigned long long next_interval;
unsigned long long next_hinterval;
unsigned long long scycle;

int state = 1;
int lastState = 1;
int lastHState = 1;
int messageId = 0;
int stateC = 0;

std::map<int, std::map<int, int> > steadyState;
std::map<int, int> hSteadyState;
std::map<int, double> acceptable_mse;
double acceptable_hmse;

std::map<int, InjectReqMsg> inTransitPackets;
std::map<int, transaction_t> inTransitTransactions;
PacketQueue packet_queue;

bool
SynfullSyntheticTraffic::CpuPort::recvTimingResp(PacketPtr pkt)
{
    scycle = curTick() / 1000;
    tester->completeRequest(pkt);
    return true;
}

void
SynfullSyntheticTraffic::CpuPort::recvReqRetry()
{
    tester->doRetry();
}

void 
SynfullSyntheticTraffic::sendPacket(InjectReqMsg& req) 
{
	req.id = messageId;

	if((int) req.address == -1) {
		req.address = messageId;
		inTransitTransactions[req.address].source = req.source;
		inTransitTransactions[req.address].dest = req.dest;
		inTransitTransactions[req.address].invs_sent = 0;
		inTransitTransactions[req.address].acks_received = 0;
	}
	messageId++;

	inTransitPackets[req.id] = req;
}

void
SynfullSyntheticTraffic::sendPkt(PacketPtr pkt, InjectReqMsg msg)
{
    if ((int)(msg.source / 2) == id && (int)msg.source % 2 == 0) {
        if (!cachePort.sendTimingReq(pkt)) {
            retryPkt = pkt;
        }
        numPacketsSent++;
        std::cout << "CPU: "<< id << std::endl;
        printRequest(pkt->req);
        printPacket(pkt);
        printMessage(msg);
        std::cout << "--------------------------------------------" << std::endl;
    }

}

SynfullSyntheticTraffic::SynfullSyntheticTraffic(const Params &p)
    : ClockedObject(p),
      tickEvent([this]{ tick(); }, "SynfullSyntheticTraffic tick",
                false, Event::CPU_Tick_Pri),
      cachePort("SynfullSyntheticTraffic", this),
      retryPkt(NULL),
      size(p.memory_size),
      blockSizeBits(p.block_offset),
      numDestinations(p.num_dest),
      simCycles(p.sim_cycles),
      numPacketsMax(p.num_packets_max),
      numPacketsSent(0),
      singleSender(p.single_sender),
      singleDest(p.single_dest),
      injRate(p.inj_rate),
      injVnet(p.inj_vnet),
      precision(p.precision),
      responseLimit(p.response_limit),
      requestorId(p.system->getRequestorId(this))
{
    noResponseCycles = 0;
    schedule(tickEvent, 0);

    id = TESTER_NETWORK++;
    DPRINTF(SynfullSyntheticTraffic,"Config Created: Name = %s, and id = %d\n",
            name(), id);
    if (!modelLoaded){
        // load model
        std::ifstream modelFile("/home/jenius/gem5/src/cpu/testers/synfull_synthetic_traffic/generated-models/barnes.model");      
        if(!modelFile.good()) {
            fatal("Could not open model file \n");
        }
        
        // Parses the file and stores all information in global variables
        ReadModel(modelFile);            
        // Close the file stream
        modelFile.close();
        modelLoaded = true;
    }
    
}

Port &
SynfullSyntheticTraffic::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "test")
        return cachePort;
    else
        return ClockedObject::getPort(if_name, idx);
}

void 
SynfullSyntheticTraffic::init()
{
    numPacketsSent = 0;
}

void 
SynfullSyntheticTraffic::TranslateIndex(int index, int &source, int &destination)
{
	source = (int)index / 32; // Truncate remainder
	destination = index - (source * 32);
}

bool 
SynfullSyntheticTraffic::InHSteadyState(int numCycles)
{
	std::vector<double> predict;
	int sum = 0;
	for (std::map<int, int>::iterator it = hSteadyState.begin();
		 it != hSteadyState.end(); ++it)
	{
		double value = it->second;
		sum += value;
		predict.push_back(value);
	}

	for (unsigned int i = 0; i < predict.size(); i++)
	{
		predict[i] = ((double)predict[i] / sum);
	}

	double mse = calculate_mse(predict, g_hierSState);
	if (mse >= 0 && mse < acceptable_hmse && scycle > numCycles * 0.3)
	{
		return true;
	}

	if (scycle > numCycles * 0.7)
	{
		return true;
	}

	return false;
}

double 
SynfullSyntheticTraffic::calculate_mse(std::vector<double> predict, std::vector<double> actual)
{
	if (predict.size() != actual.size())
	{
		return -1;
	}

	double sum = 0;
	for (unsigned int i = 0; i < predict.size(); i++)
	{
		sum += (predict[i] - actual[i]) * (predict[i] - actual[i]);
	}

	return ((double)sum / predict.size());
}

void
SynfullSyntheticTraffic::completeRequest(PacketPtr pkt)
{

    DPRINTF(SynfullSyntheticTraffic,
            "Completed injection of %s packet for address %x\n",
            pkt->isWrite() ? "write" : "read\n",
            pkt->req->getPaddr());
    
    Addr des = pkt->req->getPaddr();
    des >>= blockSizeBits;
    // std::cout << "DIR:des: "<< des << std::endl;
    std::cout << "DIR: "<< des % 16 << std::endl;
    printPacket(pkt);
    InjectReqMsg msg = GarConvertToSyn(pkt);
    printMessage(msg);
    std::cout << "--------------------------------------------" << std::endl;
    react(msg);
    assert(pkt->isResponse());
    noResponseCycles = 0;
    delete pkt;
    
}

void
SynfullSyntheticTraffic::tick()
{

    if (++noResponseCycles >= responseLimit) {
        fatal("%s deadlocked at cycle %d\n", name(), curTick());
    }
    bool sendAllowedThisCycle;
    double injRange = pow((double) 10, (double) precision);
    unsigned trySending = random_mt.random<unsigned>(0, (int) injRange);
    if (trySending < injRate*injRange)
        sendAllowedThisCycle = true;
    else
        sendAllowedThisCycle = false;

    if (sendAllowedThisCycle) {
        bool senderEnable = true;

        if (numPacketsMax >= 0 && numPacketsSent >= numPacketsMax)
            senderEnable = false;

        if (singleSender >= 0 && id != singleSender)
            senderEnable = false;

        if (senderEnable) {
            // Calculate an acceptable MSE for the Markovian Steady-State
            double sensitivity = 1.04;
            std::vector<double> predict;
            for (unsigned int i = 0; i < g_hierSState.size(); i++) {
                predict.push_back(((double)g_hierSState[i] * sensitivity));
            }
            acceptable_hmse = calculate_mse(predict, g_hierSState);
            scycle = curTick() / 1000;
            if (stateC != scycle) {
                packets = packet_queue.DeQueue(scycle);
                stateC = scycle;
            }

            generatePkt();

        }
    }

    // Schedule wakeup
    if (curTick() >= simCycles)
        exitSimLoop("Network Tester completed simCycles");
    else {
        if (!tickEvent.scheduled())
            schedule(tickEvent, clockEdge(Cycles(1)));
    }
}

void 
SynfullSyntheticTraffic::UniformInject(int writes, int reads, int ccrs, int dcrs)
{
	int source, destination;
	UniformDistribution uni_dist(0, g_resolution / 2 - 1);

	int delta = 0;
	for (int i = 0; i < writes; i++)
	{
		delta = uni_dist.Generate() * 2;
		source = g_writeSpat[g_hierClass][state].Generate();
		source = source * 2;

		destination = g_writeDest[g_hierClass][state][source].Generate();
		destination = destination * 2 + 1;

		QueuePacket(source, destination, REQUEST, WRITE, CONTROL_SIZE,
					scycle + delta, -1);
	}

	for (int i = 0; i < reads; i++)
	{
		delta = uni_dist.Generate() * 2;
		source = g_readSpat[g_hierClass][state].Generate();
		source = source * 2;

		destination = g_readDest[g_hierClass][state][source].Generate();
		destination = destination * 2 + 1;

		QueuePacket(source, destination, REQUEST, READ, CONTROL_SIZE,
					scycle + delta, -1);
	}

	for (int i = 0; i < ccrs; i++)
	{
		delta = uni_dist.Generate() * 2;
		source = g_ccrSpat[g_hierClass][state].Generate();
		source = source * 2;

		destination = g_ccrDest[g_hierClass][state][source].Generate();
		destination = destination * 2 + 1;

		QueuePacket(source, destination, REQUEST, PUTC, CONTROL_SIZE,
					scycle + delta, -1);
	}

	for (int i = 0; i < dcrs; i++)
	{
		delta = uni_dist.Generate() * 2;
		source = g_dcrSpat[g_hierClass][state].Generate();
		source = source * 2;

		destination = g_dcrDest[g_hierClass][state][source].Generate();
		destination = destination * 2 + 1;

		QueuePacket(source, destination, REQUEST, PUTD, DATA_SIZE,
					scycle + delta, -1);
	}
}


void
SynfullSyntheticTraffic::generatePkt()
{
    curCyc = curTick() / 1000;
    if (curCyc >= next_hinterval) {
        next_hinterval += g_timeSpan;

        hSteadyState[g_hierClass]++;

        if (curCyc != 0) {
            lastHState = g_hierClass;
            g_hierClass = g_hierState[g_hierClass].Generate() + 1;
            reset_ss();
        }

        if (InHSteadyState(curCyc)) {
            std::cout << "Ending simulation at steady state: " << curCyc << std::endl;
        }
      
        // std::cout << "Current hierarchical state: " << g_hierClass << std::endl;
    }

    if (curCyc >= next_interval) {
        next_interval += g_resolution;

        // Track state history for markovian steady state
        steadyState[g_hierClass][state]++;

        if (curCyc != 0) {
            // Update state
            lastState = state;
            state = g_states1[g_hierClass][state].Generate() + 1;
        }

        // Queue up initiating messages for injection
        InitiateMessages();
    }
    // Inject all of this cycles' messages into the network
    Inject(curCyc);
}

RequestPtr
SynfullSyntheticTraffic::SynConvertToGar(InjectReqMsg& msg, MemCmd::Command& requestType)
{
    Request::Flags flags;
    Addr paddr = (int)(msg.dest / 2);
    paddr <<= blockSizeBits;
    // dir
    if ((int)msg.dest % 2 == 1)
        paddr += 1024;
    unsigned size = msg.packetSize;
    if (msg.msgType == REQUEST) {
        switch (msg.coType) {
            // LD
            case READ:
                requestType = MemCmd::ReadReq;
                break;
            case WRITE:
                requestType = MemCmd::ReadReq;
                break;
            // ST
            case PUTC:
                // requestType = MemCmd::WriteLineReq;
                requestType = MemCmd::WriteReq;
                break;
            case PUTD:
                // requestType = MemCmd::WritebackDirty;
                requestType = MemCmd::WriteReq;
                break;
            // IFETCH
            case INV:
                // requestType = MemCmd::InvalidateReq;
                requestType = MemCmd::ReadReq;
                flags.set(Request::INST_FETCH);
                break;
            default:
                panic("Unsupported coType %d in SynConvertToGar", msg.coType);
        }
    } else if (msg.msgType == RESPONSE) {
        switch (msg.coType) {
            case DATA:
                requestType = MemCmd::ReadResp;
                break;
            case ACK:
                requestType = MemCmd::WriteCompleteResp;
                break;
            case WB_ACK:  // ?
                requestType = MemCmd::WritebackClean;
                break;
            case UNBLOCK: // ?
                requestType = MemCmd::SoftPFReq;
                break;
            default:
                panic("Unsupported coType %d in SynConvertToGar", msg.coType);
        }
    } else {
        panic("Unsupported msgType %d in SynConvertToGar", msg.msgType);
    }

    RequestPtr req = std::make_shared<Request>(paddr, size, flags, requestorId);
    req->setContext(id);

    return req;
}

InjectReqMsg 
SynfullSyntheticTraffic::GarConvertToSyn(PacketPtr pkt)
{
    int pid = pkt->msgId;
    std::map<int, InjectReqMsg>::iterator it = inTransitPackets.find(pid);
	if(it == inTransitPackets.end()) {
		std::cerr << "Error: couldn't find in transit packet " << pid << std::endl;
		exit(-1);
	}

	InjectReqMsg request = it->second;

    inTransitPackets.erase(it);
    return request;
}

void 
SynfullSyntheticTraffic::Inject(Tick curCyc)
{
    // get packets at current cycle
    std::list<InjectReqMsg>::iterator it;
    MemCmd::Command requestType;
    
	for(it = packets.begin(); it != packets.end(); ++it) {
		sendPacket(*it);
        RequestPtr req = SynConvertToGar(*it, requestType);
        PacketPtr pkt = new Packet(req, requestType);
        pkt->msgId = it->id;
        pkt->dataDynamic(new uint8_t[req->getSize()]);
        pkt->senderState = NULL;
        sendPkt(pkt, *it);
	}
    packet_queue.CleanUp(curCyc - 1);
}

//todo: how to react() at dir
void 
SynfullSyntheticTraffic::react(InjectReqMsg request)
{
	std::map<int, transaction_t>::iterator trans = inTransitTransactions.find(request.address);

	if (request.msgType == REQUEST &&
		(request.coType == WRITE || request.coType == READ))
	{
		// Handle Read/Write Requests
		if ((int)request.address == request.id)
		{
			// This is an initiating request. Should we forward it or go to
			// memory?
			bool isForwarded = g_toForward[g_hierClass][request.dest][request.coType].Generate() == 0;

			if (isForwarded)
			{
				int destination = g_forwardDest[g_hierClass][state][request.dest].Generate();
				destination = destination * 2;
				if (destination % 2 != 0)
				{
					std::cerr << "Error: Invalid destination for forwarded request." << std::endl;
				}

				QueuePacket(request.dest, destination, REQUEST, request.coType,
							CONTROL_SIZE, scycle + 1, request.address);

				if (request.coType == WRITE)
				{
					// How many invalidates to send
					int numInv = g_numInv[g_hierClass][state][request.dest].Generate();
					int s = state;

					if (numInv <= 0)
					{
						return;
					}

					// Ensure invalidate destinations are unique (i.e. no two
					// invalidate messages to the same destination)
					std::set<int> destinations;
					destinations.insert(destination); // Request already forwarded here
					while (destinations.size() != (unsigned int)numInv)
					{
						int dest = g_invDest[g_hierClass][s][request.dest].Generate();
						dest = dest * 2;
						destinations.insert(dest);
					}

					for (std::set<int>::iterator it = destinations.begin();
						 it != destinations.end(); ++it)
					{
						QueuePacket(request.dest, *it, REQUEST, INV,
									CONTROL_SIZE, scycle + 1, request.address);
						trans->second.invs_sent++;
					}
				}
			}
			else
			{
				// Access memory, queue up a data response for the future
				QueuePacket(request.dest, request.source, RESPONSE, DATA,
							DATA_SIZE, scycle + 80, request.address);
			}

			return;
		}
		else
		{
			// This is not an initiating request, so it's a forwarded request

			// Respond with Data
			QueuePacket(request.dest,
						trans->second.source, RESPONSE,
						DATA, DATA_SIZE, scycle + 1, request.address);
		}
	}
	else if (request.msgType == REQUEST &&
			 (request.coType == PUTC || request.coType == PUTD))
	{
		// Respond with WB_ACK
		QueuePacket(request.dest, request.source, RESPONSE, WB_ACK,
					CONTROL_SIZE, scycle + 1, request.address);
	}
	else if (request.msgType == REQUEST && request.coType == INV)
	{
		// Respond with Ack
		QueuePacket(request.dest, trans->second.source,
					RESPONSE, ACK, CONTROL_SIZE, scycle + 1, request.address);
	}
	else if (request.msgType == RESPONSE && request.coType == DATA)
	{
		trans->second.data_received = true;
		// Send unblock
		QueuePacket(inTransitTransactions[request.address].source,
					inTransitTransactions[request.address].dest, RESPONSE, UNBLOCK,
					CONTROL_SIZE, scycle + 1, request.address);
	}
	else if (request.msgType == RESPONSE && request.coType == ACK)
	{
		trans->second.acks_received++;
	}
	else if (request.msgType == RESPONSE && request.coType == UNBLOCK)
	{
		trans->second.unblock_received = true;
	}

	if (trans->second.Completed())
	{
		inTransitTransactions.erase(trans);
	}
}

void
SynfullSyntheticTraffic::doRetry()
{
    if (cachePort.sendTimingReq(retryPkt)) {
        retryPkt = NULL;
    }
}

void
SynfullSyntheticTraffic::printAddr(Addr a)
{
    cachePort.printAddr(a);
}

void 
SynfullSyntheticTraffic::InitiateMessages()
{
	int writes = g_writes[g_hierClass][state].Generate();
	int reads = g_reads[g_hierClass][state].Generate();
	int ccrs = g_ccrs[g_hierClass][state].Generate();
	int dcrs = g_dcrs[g_hierClass][state].Generate();

	UniformInject(writes, reads, ccrs, dcrs);
}

void 
SynfullSyntheticTraffic::reset_ss()
{
	for (std::map<int, int>::iterator it = steadyState[g_hierClass].begin();
		 it != steadyState[g_hierClass].end(); ++it)
	{
		it->second = 0;
	}
	state = 1;
}

void
SynfullSyntheticTraffic::printRequest(RequestPtr req) 
{
    std::cout << "Garnet -> Request parameters:" << std::endl;
    std::cout << "  Requestor ID: " << req->requestorId() << std::endl;
    std::cout << "  Context ID: " << req->contextId() << std::endl;
    std::cout << "  Flags: " << req->getFlags() << std::endl;
    std::cout << "  Paddr: " << req->getPaddr() << std::endl;
    std::cout << "  Size: " << req->getSize() << std::endl;
    std::cout << "" << std::endl;
}

void
SynfullSyntheticTraffic::printPacket(PacketPtr pkt) 
{
    std::cout << "Garnet -> Packet parameters:" << std::endl;
    std::cout << "  id: " << pkt->msgId << std::endl;
    std::cout << "  Command: " << pkt->cmdString() << std::endl;
    std::cout << "  Address: " << pkt->getAddr() << std::endl;
    std::cout << "  Size: " << pkt->getSize() << std::endl;
    std::cout << "" << std::endl;
}

void 
SynfullSyntheticTraffic::printMessage(InjectReqMsg msg) {
	std::cout << "Synfull -> Message parameters:" << std::endl;
	std::cout << "  message id: " << msg.id << std::endl;
	std::cout << "  cycle: " << scycle << std::endl;
    std::cout << "  source: " << msg.source << std::endl;
    std::cout << "  destination: " << msg.dest << std::endl;
    std::cout << "  packetSize: " << msg.packetSize << std::endl;
    std::cout << "  msgType: " << msg.msgType << std::endl;
    std::cout << "  coType: " << msg.coType << std::endl;
    std::cout << "  address: " << msg.address << std::endl;
    std::cout << "" << std::endl;
}

void 
SynfullSyntheticTraffic::QueuePacket(int source, int destination, int msgType, int coType,
			int packetSize, int time, int address)
{
	InjectReqMsg packet;
	packet.source = source;
	packet.dest = destination;
	packet.cl = 0;
	packet.network = 0;
	packet.packetSize = packetSize;
	packet.msgType = msgType;
	packet.coType = coType;
	packet.address = address;

	packet_queue.Enqueue(packet, time);
}

} // namespace gem5