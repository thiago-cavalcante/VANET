    /*
 * wifitest.cc
 *
 *  Created on: Nov 18, 2015
 *      Author: ufam
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/stats-module.h"
#include "ns3/wifi-module.h"
#include "ns3/netanim-module.h"
#include "Vehicle.h"
#include <iostream>



using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("Main");

class Experiment
{
public:
  Experiment ();
  Experiment (std::string name);
  Gnuplot2dDataset Run (const WifiHelper &wifi, const YansWifiPhyHelper &wifiPhy,
                        const NqosWifiMacHelper &wifiMac, const YansWifiChannelHelper &wifiChannel);
private:
  void ReceivePacket (Ptr<Socket> socket);
  void SetPosition (Vehicle* v, Vector position);
  Vector GetPosition (Vehicle* v);
  void AdvancePositionX (Vehicle* v, double* vel, double finalPosition);
  void AdvancePositionY (Vehicle* v, double* vel, double finalPosition);
  Ptr<Socket> SetupPacketReceive (Vehicle* v);

  uint32_t m_bytesTotal;
  Gnuplot2dDataset m_output;
};

Experiment::Experiment ()
{
}

Experiment::Experiment (std::string name)
  : m_output (name)
{
  m_output.SetStyle (Gnuplot2dDataset::LINES);
}

//void Leave(const uint32_t& my_id) {
//  Ptr<MobilityModel> mobility = Config::Instance()->GetMobileNodes().Get(my_id)->GetObject<MobilityModel>();
//  NS_ASSERT(mobility);
//  mobility->SetPosition(Vector(mobility->GetPosition().x, mobility->GetPosition().y, 384e6));
//  nodes[my_id].on_leave = true;
//}

void
Experiment::SetPosition (Vehicle* v, Vector position)
{
  Ptr<MobilityModel> mobility = v->getNode()->GetObject<MobilityModel> ();
  mobility->SetPosition (position);
}

Vector
Experiment::GetPosition (Vehicle* v)
{
  Ptr<MobilityModel> mobility = v->getNode()->GetObject<MobilityModel> ();
  return mobility->GetPosition ();
}

void
Experiment::AdvancePositionX (Vehicle* v, double* vel, double finalPosition)
{
  //speed in kph
  double speed=*vel;

  double time;
  //advance position in meters
  double advance = 0.1;
  //time = advance/(speedA/3.6);
  Vector pos = GetPosition (v);
//  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
//  ns3::Ptr<const ns3::MobilityModel> p = pos;
//  double d = mobility->GetDistanceFrom(p);
//  std::cout << "distance="<<d << std::endl;
  double mbs = ((m_bytesTotal * 8.0) / 1000000);
  m_bytesTotal = 0;
  m_output.Add (pos.x, mbs);

  pos.x += advance;
	  time = advance/(speed/3.6);
  if (pos.x >= finalPosition)
    {
	  SetPosition (v, Vector(384e6, 384e6, 384e6));
      return;
    }
  SetPosition (v, pos);
  //std::cout << "x="<<pos.x << std::endl;
  Simulator::Schedule (Seconds (time), &Experiment::AdvancePositionX, this, v,vel,finalPosition);
}


void
Experiment::AdvancePositionY (Vehicle* v, double* vel, double finalPosition)
{
  //speed in kph
  double speed=*vel;

  double time;
  //advance position in meters
  double advance = 0.1;
  time = advance/(speed/3.6);
  Vector pos = GetPosition (v);
  double mbs = ((m_bytesTotal * 8.0) / 1000000);
  m_bytesTotal = 0;
  m_output.Add (pos.y, mbs);

  pos.y -= advance;
  if (pos.y <= 0.0)
    {
	  pos.y = 0.0;
	  speed = 35;
	  pos.x += advance;
	  time = advance/(speed/3.6);
	  if (pos.x >= finalPosition)
	      {
		    SetPosition (v, Vector(384e6, 384e6, 384e6));
	        return;
	      }
	  //SetPosition (node, pos);
	  //return;
    }
  SetPosition (v, pos);
  //std::cout << "x="<<pos.x << std::endl;
  Simulator::Schedule (Seconds (time), &Experiment::AdvancePositionY, this, v,vel,finalPosition);
}

void
Experiment::ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  while ((packet = socket->Recv ()))
    {
      m_bytesTotal += packet->GetSize ();
    }
}

Ptr<Socket>
Experiment::SetupPacketReceive (Vehicle* v)
{
  TypeId tid = TypeId::LookupByName ("ns3::PacketSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (v->getNode(), tid);
  sink->Bind ();
  sink->SetRecvCallback (MakeCallback (&Experiment::ReceivePacket, this));
  return sink;
}

Gnuplot2dDataset
Experiment::Run (const WifiHelper &wifi, const YansWifiPhyHelper &wifiPhy,
                 const NqosWifiMacHelper &wifiMac, const YansWifiChannelHelper &wifiChannel)
{

  double velA = 45.0;
  double velB = 40.0;
  double velC = 80.0;
  double finalPositionA = 100;
  double finalPositionB = 100;
  double finalPositionC = 100;
  Vehicle* v1 = new Vehicle(velA, Vector (0.0, 0.0, 0.0));
  Vehicle* v2 = new Vehicle(velB, Vector (10.0, 0.0, 0.0));
  Vehicle* v3 = new Vehicle(0, Vector (50.0, -10.0, 0.0));
  Vehicle* v4 = new Vehicle(velC, Vector (50.0, 100.0, 0.0));

  m_bytesTotal = 0;


  NodeContainer c;
  c.Add(v1->getNode());
  c.Add(v2->getNode());
  c.Add(v3->getNode());
  c.Add(v4->getNode());

  PacketSocketHelper packetSocket;
  packetSocket.Install (c);

  YansWifiPhyHelper phy = wifiPhy;
  phy.SetChannel (wifiChannel.Create ());

  NqosWifiMacHelper mac = wifiMac;
  NetDeviceContainer devices = wifi.Install (phy, mac, c);

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (v1->GetInitialPosition());
  positionAlloc->Add (v2->GetInitialPosition());
  positionAlloc->Add (v3->GetInitialPosition());
  positionAlloc->Add (v4->GetInitialPosition());

  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  mobility.Install (c);

  PacketSocketAddress socket;
  socket.SetSingleDevice (devices.Get (1)->GetIfIndex ());
  socket.SetPhysicalAddress (devices.Get (2)->GetAddress ());
  socket.SetProtocol (1);

  OnOffHelper onoff ("ns3::PacketSocketFactory", Address (socket));
  onoff.SetConstantRate (DataRate (60000000));
  onoff.SetAttribute ("PacketSize", UintegerValue (2000));

  ApplicationContainer apps = onoff.Install (c);
  apps.Start (Seconds (0.0));
  apps.Stop (Seconds (500.0));


  Simulator::Schedule (Seconds (0.0), &Experiment::AdvancePositionX, this, v2, &velA, finalPositionA);
  Simulator::Schedule (Seconds (3.0), &Experiment::AdvancePositionX, this, v1, &velB, finalPositionB);
  Simulator::Schedule (Seconds (0.0), &Experiment::AdvancePositionY, this, v4, &velC, finalPositionC);
  Ptr<Socket> recvSink = SetupPacketReceive (v3);


  Simulator::Run ();

  Simulator::Destroy ();

  return m_output;
}

int main (int argc, char *argv[])
{
  // disable fragmentation
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));

  CommandLine cmd;
  cmd.Parse (argc, argv);

  Gnuplot gnuplot = Gnuplot ("reference-rates.png");

  Experiment experiment;
  //set the technology to communicate
  WifiHelper wifi = WifiHelper::Default ();
  wifi.SetStandard (WIFI_PHY_STANDARD_80211a);
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Gnuplot2dDataset dataset;

  wifiMac.SetType ("ns3::AdhocWifiMac");

  NS_LOG_DEBUG ("54");
  experiment = Experiment ("54mb");
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue ("OfdmRate54Mbps"));
  dataset = experiment.Run (wifi, wifiPhy, wifiMac, wifiChannel);
  gnuplot.AddDataset (dataset);

  gnuplot.GenerateOutput (std::cout);

  return 0;
}
