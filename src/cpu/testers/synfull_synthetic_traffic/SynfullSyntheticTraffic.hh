#ifndef __CPU_SYNFULL_SYNTHETIC_TRAFFIC_HH__
#define __CPU_SYNFULL_SYNTHETIC_TRAFFIC_HH__

#include "base/statistics.hh"
#include "cpu/testers/synfull_synthetic_traffic/Messages.hh"
#include "cpu/testers/synfull_synthetic_traffic/PacketQueue.hh"
#include "mem/port.hh"
#include "params/SynfullSyntheticTraffic.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"
#include "sim/sim_exit.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"

namespace gem5
{

struct transaction_t
{
	int source;
	int dest;
	int invs_sent;
	int acks_received;
	bool data_received;
	bool unblock_received;

	bool Completed()
	{
		return (invs_sent == acks_received) && data_received && unblock_received;
	}

	transaction_t() : source(-1), dest(-1), invs_sent(0), acks_received(0),
					  data_received(false), unblock_received(false) {}
};

class Packet;
class SynfullSyntheticTraffic : public ClockedObject
{
  public:
    typedef SynfullSyntheticTrafficParams Params;
    SynfullSyntheticTraffic(const Params &p);

    void init() override;

    void tick();

    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;
    
    void printAddr(Addr a);

    
  protected:
    EventFunctionWrapper tickEvent;

    class CpuPort : public RequestPort
    {
        SynfullSyntheticTraffic *tester;
      
      public:

        CpuPort(const std::string &_name, SynfullSyntheticTraffic *_tester)
            : RequestPort(_name), tester(_tester)
        { }

      protected:

        virtual bool recvTimingResp(PacketPtr pkt);

        virtual void recvReqRetry();
    };

    CpuPort cachePort;

    class SynfullSyntheticTrafficSenderState : public Packet::SenderState
    {
      public:
        SynfullSyntheticTrafficSenderState(uint8_t *_data)
            : data(_data)
        { }

        uint8_t *data;
    };

    

    PacketPtr retryPkt;
    unsigned size;
    int id;
    Tick curCyc;

    unsigned blockSizeBits;

    Tick noResponseCycles;

    int numDestinations;
    Tick simCycles;
    int numPacketsMax;
    int numPacketsSent;
    int singleSender;
    int singleDest;

    double injRate;
    int injVnet;
    int precision;

    const Cycles responseLimit;

    RequestorID requestorId;

    friend class MemCompleteEvent;

    RequestPtr SynConvertToGar(InjectReqMsg& msg, MemCmd::Command& requestType);

    InjectReqMsg GarConvertToSyn(PacketPtr pkt);

    void TranslateIndex(int index, int &source, int &destination);

    double calculate_mse(std::vector<double> predict, std::vector<double> actual);

    void Inject(Tick curCyc);

    void react(InjectReqMsg request);

    void completeRequest(PacketPtr pkt);
    
    void generatePkt();

    void sendPacket(InjectReqMsg &req);

    void sendPkt(PacketPtr pkt, InjectReqMsg msg);

    void UniformInject(int writes, int reads, int ccrs, int dcrs);

    void doRetry();

    void printRequest(RequestPtr req);

    void printPacket(PacketPtr pkt);

    void printMessage(InjectReqMsg msg);

    void QueuePacket(int source, int destination, int msgType, int coType,
                     int packetSize, int time, int address);

    void reset_ss();

    void InitiateMessages();

    bool InHSteadyState(int numCycles);
};

} // namespace gem5

#endif // __CPU_SYNFULL_SYNTHETIC_TRAFFIC_HH__