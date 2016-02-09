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

double speedA = 40;
double speedB = 60;
double speedC = 80;

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
  void SetPosition (Ptr<Node> node, Vector position);
  Vector GetPosition (Ptr<Node> node);
  void AdvancePositionX (Ptr<Node> node, double* vel, double finalPosition);
  void AdvancePositionY (Ptr<Node> node, double* vel, double finalPosition);
  Ptr<Socket> SetupPacketReceive (Ptr<Node> node);

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
Experiment::SetPosition (Ptr<Node> node, Vector position)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  mobility->SetPosition (position);
}

Vector
Experiment::GetPosition (Ptr<Node> node)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  return mobility->GetPosition ();
}

void
Experiment::AdvancePositionX (Ptr<Node> node, double* vel, double finalPosition)
{
  //speed in kph
  double speed=*vel;

  double time;
  //advance position in meters
  double advance = 0.1;
  //time = advance/(speedA/3.6);
  Vector pos = GetPosition (node);
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
	  SetPosition (node, Vector(384e6, 384e6, 384e6));
      return;
    }
  SetPosition (node, pos);
  //std::cout << "x="<<pos.x << std::endl;
  Simulator::Schedule (Seconds (time), &Experiment::AdvancePositionX, this, node,vel,finalPosition);
}


void
Experiment::AdvancePositionY (Ptr<Node> node, double* vel, double finalPosition)
{
  //speed in kph
  double speed=*vel;

  double time;
  //advance position in meters
  double advance = 0.1;
  time = advance/(speed/3.6);
  Vector pos = GetPosition (node);
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
		    SetPosition (node, Vector(384e6, 384e6, 384e6));
	        return;
	      }
	  //SetPosition (node, pos);
	  //return;
    }
  SetPosition (node, pos);
  //std::cout << "x="<<pos.x << std::endl;
  Simulator::Schedule (Seconds (time), &Experiment::AdvancePositionY, this, node,vel,finalPosition);
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
Experiment::SetupPacketReceive (Ptr<Node> node)
{
  TypeId tid = TypeId::LookupByName ("ns3::PacketSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);
  sink->Bind ();
  sink->SetRecvCallback (MakeCallback (&Experiment::ReceivePacket, this));
  return sink;
}

Gnuplot2dDataset
Experiment::Run (const WifiHelper &wifi, const YansWifiPhyHelper &wifiPhy,
                 const NqosWifiMacHelper &wifiMac, const YansWifiChannelHelper &wifiChannel)
{

  double velA = 40.0;
  double velB = 45.0;
  double velC = 80.0;
  double finalPositionA = 100;
  double finalPositionB = 100;
  double finalPositionC = 100;
  Vehicle* v1;
  Vehicle* v2;
  Vehicle* v3;
  Vehicle* v4;
//  Ptr<Vehicle> v1 = CreateObject<Vehicle> ();
//  Ptr<Vehicle> v2 = CreateObject<Vehicle> ();
//  Ptr<Vehicle> v3 = CreateObject<Vehicle> ();
//  Ptr<Vehicle> v4 = CreateObject<Vehicle> ();


  m_bytesTotal = 0;

  v1=new Vehicle();
  v2=new Vehicle();
  v3=new Vehicle();
  v4=new Vehicle();

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
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (10.0, 0.0, 0.0));
  positionAlloc->Add (Vector (50.0, -10.0, 0.0));
  positionAlloc->Add (Vector (50.0, 100.0, 0.0));
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


  Simulator::Schedule (Seconds (0.0), &Experiment::AdvancePositionX, this, c.Get (1), &velA, finalPositionA);
  Simulator::Schedule (Seconds (3.0), &Experiment::AdvancePositionX, this, c.Get (0), &velB, finalPositionB);
  Simulator::Schedule (Seconds (0.0), &Experiment::AdvancePositionY, this, c.Get (3), &velC, finalPositionC);
  Ptr<Socket> recvSink = SetupPacketReceive (c.Get (2));

//  v1->getNode() = c.Get (0);
//  v2->getNode() = c.Get (1);
//  v3->getNode() = c.Get (2);
//  v4->getNode() = c.Get (3);

//  Simulator::Schedule (Seconds (0.0), &Experiment::AdvancePositionX, this, v2->getNode(), &velA, finalPositionA);
//  Simulator::Schedule (Seconds (3.0), &Experiment::AdvancePositionX, this, v1->getNode(), &velB, finalPositionB);
//  Simulator::Schedule (Seconds (0.0), &Experiment::AdvancePositionY, this, v4->getNode(), &velC, finalPositionC);
//  Ptr<Socket> recvSink = SetupPacketReceive (v3->getNode());



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
