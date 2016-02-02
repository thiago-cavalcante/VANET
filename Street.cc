/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005-2009 Old Dominion University [ARBABI]
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Hadi Arbabi <marbabi@cs.odu.edu>
 */

#include <iostream>
#include "Street.h"
#include "ns3/simulator.h"
#include <math.h>
#include "random-variable-stream.h"

namespace ns3
{
  TypeId Street::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::Street")
    .SetParent<Object> ()
    .AddConstructor<Street> ()
    ;
    return tid;
  }

  void Street::InitStreet()
  {

    m_vehicleId = 1;
    bool init = false;
    
	if(!m_initVehicle.IsNull())
	  {
        init = m_initVehicle(Ptr<Street>(this), m_vehicleId);
	  }

//    if(init==true)
//      {
//        for(int i=0;i<m_numberOfLanes;i++)
//          {
//            m_vehicles[i].sort(ns3::Vehicle::Compare);
//            if(m_twoDirectional==true)
//              m_vehiclesOpp[i].sort(ns3::Vehicle::Compare);
//          }
//      }

    if(m_autoInject==true) 
	  InjectVehicles(m_injectionSafetyGap, (int)m_sedanTruckPerc);
  }


  Ptr<Vehicle> Street::GetVehicle(std::list<Ptr<Vehicle> > v, int index)
  {
	std::list<Ptr<Vehicle> >::iterator i=v.begin();
    advance(i,index);
    return *i;
  }

  double Street::GetYForLane(int lane, int dir)
  {
    if(dir == 1)
	  {
		return m_laneWidth * (lane + 0.5);
	  }
    return ((2 * m_numberOfLanes * m_laneWidth) + m_medianGap - GetYForLane(lane, 1));
  }

  double Street::GetMedianGap()
  {
    return m_medianGap;
  }

  void Street::SetMedianGap(double value)
  {
    m_medianGap=value;
  }

  double Street::GetInjectionGap()
  {
    return m_injectionSafetyGap;
  }

  void Street::SetInjectionGap(double value)
  {
    if(value > 0 )
      m_injectionSafetyGap= value;
  }

  void Street::InjectVehicles(double minGap, int p)
  {
    UniformVariable uRnd(0,100);
	UniformVariable uRnd2(0,100);
    double rate = 10;//test value
	int last;
	double gap;
	double vel = 0.0;

	last = m_vehicles[m_currentLane].size()-1;
	if (last < 0)
	  {
        gap = minGap + 1;
	  }
	else
	  {
        gap = GetVehicle(m_vehicles[m_currentLane],last)->GetPosition().x;
        vel = m_velocity;//GetVehicle(m_vehicles[m_currentLaneDirPos],last)->GetVelocity();
      }

    m_remainder += rate;
		if(m_remainder >= 1 && gap > minGap)
	      {
			m_remainder -= 1;
            if (uRnd.GetValue() <= p)        
              {
                Ptr<Vehicle> temp=CreateObject<Vehicle>();
				if(uRnd2.GetValue() <= m_penetrationRate) 
				{
					temp->IsEquipped = true;
					temp->SetupWifi(m_wifiHelper, m_wifiPhyHelper, m_wifiMacHelper);
				}
				else
				{
					temp->IsEquipped=false;
				}
                temp->SetVehicleId(m_vehicleId++);
                temp->SetDirection(1);
                temp->SetPosition(Vector(-4,GetYForLane(m_currentLane,1),0));
                temp->SetLane(m_currentLane);
                temp->SetVelocity(vel);
                temp->SetAcceleration(0.0);
				Ptr<Model> tempModel=CreateSedanModel();
				tempModel->SetDesiredVelocity(m_RVSpeed.GetValue());
                temp->SetModel(tempModel);
                temp->SetLength(4);
                temp->SetWidth(2);
				if(temp->IsEquipped ==true)
				{
                temp->SetReceiveCallback(m_receiveData);
                temp->SetDevTxTraceCallback(m_devTxTrace);
                temp->SetDevRxTraceCallback(m_devRxTrace);
                temp->SetPhyRxOkTraceCallback(m_phyRxOkTrace);
                temp->SetPhyRxErrorTraceCallback(m_phyRxErrorTrace);
                temp->SetPhyTxTraceCallback(m_phyTxTrace);
                temp->SetPhyStateTraceCallback(m_phyStateTrace);
				}
                m_vehicles[m_currentLaneDirPos].push_back(temp);
              }
	        m_currentLane++;
	        if(m_currentLane >= m_numberOfLanes) m_currentLane = 0;
		  }
  }
  void Street::TranslateVehicles()
  {
    if(m_stopped==true) return;
    static int loop=0;
    // NOTE: ORDER OF CALLING THIS FUNCTIONS IS VERY VERY IMPORTANT (EFFECT OF CURRENT SPEED, POSITION, DECISION)
    if(loop==10) loop=0;

    TranslatePositionVelocity(m_vehicles, m_dt);
	Accelerate(m_vehicles, m_dt);
    
    if(m_autoInject==true) 
	  InjectVehicles(m_injectionSafetyGap, (int)m_sedanTruckPerc);
 
    loop++;
	Simulator::Schedule(Seconds(m_dt), &Street::Step, Ptr<Street>(this));
  }

  void Street::Accelerate(std::list<Ptr<Vehicle> > vehicles[], double dt)
  {
    for (int i = 0; i < m_numberOfLanes; i++)
      {
        for (uint j = 0; j < vehicles[i].size(); j++)
          {
            Ptr<Vehicle> veh=GetVehicle(vehicles[i],j);
            bool controled=false;
            if(!m_controlVehicle.IsNull()) 
		      controled=m_controlVehicle(Ptr<Street>(this), veh, dt);
            if(controled==false)
              {
                if (j == 0)
                  {
                    veh->Accelerate(0);
                    continue;
                  }
                veh->Accelerate(GetVehicle(vehicles[i],j - 1));
              }
           }
      }
  }

  void Street::TranslatePositionVelocity(std::list<Ptr<Vehicle> > vehicles[], double dt)
  {
    std::list<Ptr<Vehicle> > reachedEnd;
    for (int i = 0; i < m_numberOfLanes; i++)
      {
        for (uint j = 0; j < vehicles[i].size(); j++)
          {
            Ptr<Vehicle> veh=GetVehicle(vehicles[i],j);
            veh->TranslatePosition(dt);
            veh->TranslateVelocity(dt);
            if(veh->GetPosition().x > m_streetLength && veh->GetDirection()==1)
			  reachedEnd.push_back(veh);
            else if(veh->GetPosition().x <0 && veh->GetDirection()==-1) 
			  reachedEnd.push_back(veh);
          }

        for(uint r=0; r<reachedEnd.size(); r++)
          {
            Ptr<Vehicle> rm=GetVehicle(reachedEnd, r);
            vehicles[i].remove(rm);
            if(rm->IsEquipped==true) rm->GetReceiveCallback().Nullify();
            // to put vehicle's node far away from the street
            // we cannot dispose the vehicle here because its node may still be involved in send and receive process
            rm->SetPosition(Vector(10000, 10000, 10000)); 
            rm=0;
          }

        reachedEnd.clear();
      }
  }

  void Street::ChangeLane(std::list<Ptr<Vehicle> > vehicles[])
  {
    if (m_numberOfLanes <= 1)
      {
        return;
      }

    for (int i = 0; i < m_numberOfLanes; i++)
      {
        if (i < 1)
          {
            DoChangeLaneIfPossible(vehicles,i, i + 1);
            continue;
          }
        if (i + 1 >= m_numberOfLanes)
          {
            DoChangeLaneIfPossible(vehicles,i, i - 1);
            continue;
          }
        DoChangeLaneIfPossible(vehicles, i, i + 1);
        DoChangeLaneIfPossible(vehicles, i, i - 1);
      }     
  }

  void Street::DoChangeLaneIfPossible(std::list<Ptr<Vehicle> > vehicles[], int curLane, int desLane)
  {
	std::list<Ptr<Vehicle> > canChange;
    canChange.clear();
    
	for (uint j = 0; j < vehicles[curLane].size(); j++)
      {
        Ptr<Vehicle> fOld = 0;
        if (j > 0)
          {
            fOld = GetVehicle(vehicles[curLane],j - 1);
          }

        FindSideVehicles(vehicles, GetVehicle(vehicles[curLane],j), desLane);
        if (GetVehicle(vehicles[curLane],j)->CheckLaneChange(fOld, m_tempVehicles[0], m_tempVehicles[1], (curLane < desLane) ? true : false))
          {
            canChange.push_back(GetVehicle(vehicles[curLane],j));
          }               
      }

    for (uint j = 0; j < canChange.size(); j++)
      {
        Vector position=GetVehicle(canChange,j)->GetPosition();
        position.y=GetYForLane(desLane, GetVehicle(canChange,j)->GetDirection());
        GetVehicle(canChange,j)->SetLane(desLane);
        GetVehicle(canChange,j)->SetPosition(position);
        vehicles[curLane].remove(GetVehicle(canChange,j));
        vehicles[desLane].push_back(GetVehicle(canChange,j));
      }

	m_vehicles[desLane].sort(ns3::Vehicle::Compare);
  }

  void Street::FindSideVehicles(std::list<Ptr<Vehicle> > vehicles[], Ptr<Vehicle> veh, int sideLane)
  {
    int front=-1, back=-1;
    m_tempVehicles[0] = 0;
	m_tempVehicles[1] = 0;
    for (uint i = 0; i < vehicles[sideLane].size(); i++)
      {
        if(veh->GetDirection() == 1)
          {	
            if (GetVehicle(vehicles[sideLane],i)->GetPosition().x <= veh->GetPosition().x)
              {
                back = i;
                front = back - 1;
                break;
              }
          }
        else
          {
            if (GetVehicle(vehicles[sideLane],i)->GetPosition().x >= veh->GetPosition().x)
            {
              back = i;
              front = back - 1;
              break;
            }
          }
      }

    if (back < 0)
      {
        front = vehicles[sideLane].size()-1;
      }
    if (back > -1)
      {
        m_tempVehicles[1] = GetVehicle(vehicles[sideLane], back);
      }
    if (front > -1)
      {
        m_tempVehicles[0] = GetVehicle(vehicles[sideLane], front);
      }
  }

  Street::Street()
  {
    m_dt=0.1;
    m_vehicleId=1;
    m_numberOfLanes=1;
    m_streetLength=1000;
    m_laneWidth=5;
    m_laneChangeSedan=0;
    m_laneChangeTruck=0;
    m_autoInject=true;
    m_medianGap=5;
    m_injectionSafetyGap=50;
    m_changeLaneSet=false;
	m_flow=1;
	m_velocity=0;
	m_velocity=0;
	m_currentLane=0;
	m_remainder=0;
	m_penetrationRate=100;

    // Setup Wifi
    m_wifiHelper = WifiHelper::Default();	
    m_wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211a);
    m_wifiMacHelper = NqosWifiMacHelper::Default();
    m_wifiPhyHelper = YansWifiPhyHelper::Default();
    m_wifiChannelHelper = YansWifiChannelHelper::Default ();
    m_wifiMacHelper.SetType ("ns3::AdhocWifiMac");
    m_wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue ("wifia-6mbs"));
    //m_wifiChannelHelper.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");
	m_wifiChannel = m_wifiChannelHelper.Create();
    m_wifiPhyHelper.SetChannel (m_wifiChannel);
    m_wifiPhyHelper.Set("TxPowerStart",DoubleValue(21.5));	// 250-300 meter transmission range 
    m_wifiPhyHelper.Set("TxPowerEnd",DoubleValue(21.5));      // 250-300 meter transmission range 			
    m_wifiPhyHelper.Set("TxPowerLevels",UintegerValue(1)); 
    m_wifiPhyHelper.Set("TxGain",DoubleValue(2)); 
    m_wifiPhyHelper.Set("RxGain",DoubleValue(2));  
    m_wifiPhyHelper.Set("EnergyDetectionThreshold", DoubleValue(-101.0));
  }

  Street::~Street()
  {
    m_tempVehicles[0]=0; 
	m_tempVehicles[1]=0;

    for(int i=0;i<m_numberOfLanes;i++)
      {
        m_vehicles[i].erase(m_vehicles[i].begin(), m_vehicles[i].end());
      }
  }
  void Street::Step(Ptr<Street> street)
  {
    street->TranslateVehicles();
  }
  void Street::Start()
  {
    m_stopped=false;
    InitStreet();
    Simulator::Schedule(Seconds(0.0), &Step, Ptr<Street>(this));
  }

  void Street::Stop()
  {
    m_stopped=true;
  }
 
  double Street::GetDeltaT()
  {
    return m_dt;
  }

  void Street::SetDeltaT(double value)
  {
    if(value<=0) 
	  value=0.1;

    m_dt=value;
  }

  int Street::GetNumberOfLanes()
  {
    return m_numberOfLanes;
  }

  void Street::SetNumberOfLanes(int value)
  {
    if(value<1) 
	  value=1;
    else if(value>5) 
	  value=5;

    m_numberOfLanes=value;
  }

  double Street::GetStreetLength()
  {
    return m_streetLength;
  }

  void Street::SetStreetLength(double value)
  {
    if(value<0) 
	  value=10000;

    m_streetLength=value;
  }

  double Street::GetLaneWidth()
  {
   return m_laneWidth;
  }

  void Street::SetLaneWidth(double value)
  {
    if(value<0) 
	  value=5;

    m_laneWidth=value;
  }

  void Street::SetPenetrationRate(double value)
  {
    if(value>100) m_penetrationRate=100;
	else if(value<0) m_penetrationRate=0;
	else m_penetrationRate=value;
  }

  double Street::GetPenetrationRate()
  {
	  return m_penetrationRate;
  }

  void Street::PrintVehicles()
  {
    std::cout << "Lane 2----------------" << Simulator::Now()<< "--------" << std::endl;
    for(uint i=0; i<m_vehicles[1].size();i++)
      {
        Ptr<Vehicle> v=GetVehicle(m_vehicles[1],i);
		std::cout<< v->GetVehicleId() << ":" << v->GetPosition().x 
                 << ":" << v->GetPosition().y << ":" << v->GetVelocity() << std::endl;
      }
    std::cout << "----------------------" << std::endl;
  }

  void Street::SetFlow(double value)
  {
    if(value>=0) m_flowDirPos = value;
  }

	  
  void Street::SetVelocity(double value)
  {
    if(value>=0) m_velocityDirPos = value;
  }


  bool Street::GetAutoInject()
  {
    return m_autoInject;
  }

  void Street::SetAutoInject(bool value)
  {
    m_autoInject=value; 
  }

  int Street::GetLastVehicleId()
  {
    return m_vehicleId;
  }

  WifiHelper Street::GetWifiHelper()
  {
    return m_wifiHelper;
  }

  NqosWifiMacHelper Street::GetNqosWifiMacHelper()
  {
    return m_wifiMacHelper;
  }

  YansWifiPhyHelper Street::GetYansWifiPhyHelper()
  {
	return m_wifiPhyHelper;
  }

  void Street::SetYansWifiPhyHelper(YansWifiPhyHelper yWifiPhyHelper)
  {
	m_wifiPhyHelper = yWifiPhyHelper;
  }
  	  
  Ptr<YansWifiChannel> Street::GetWifiChannel()
  {
    return m_wifiChannel;
  }

  void Street::AddVehicle(Ptr<Vehicle> vehicle)
  {
    int lane=vehicle->GetLane();
    int dir=vehicle->GetDirection();
    if(lane < m_numberOfLanes && lane >= 0)
      {
		m_vehicles[lane].push_back(vehicle);
      }
  }

  Ptr<Vehicle> Street::FindVehicle(int vid)
  {
    Ptr<Vehicle> v=0;

    for(int i=0;i<m_numberOfLanes;i++)
      {
        for(uint j=0;j<m_vehicles[i].size();j++)
          {	
            v=GetVehicle(m_vehicles[i],j);
            if(v->GetVehicleId()==vid) 
			  return v;
          }
      }
    return v;
  }

  std::list<Ptr<Vehicle> > Street::FindVehiclesInRange(Ptr<Vehicle> vehicle, double range)
  {
    std::list<Ptr<Vehicle> > neighbors;
    if(range<=0) 
	  return neighbors;

    Ptr<Vehicle> v=0;
    double diff=0;
    Vector pos, p;
    p=vehicle->GetPosition();
    for(int i=0;i<m_numberOfLanes;i++)
      {
        for(uint j=0;j<m_vehicles[i].size();j++)
          {	
            v=GetVehicle(m_vehicles[i],j);
            pos=v->GetPosition();
            if(v->GetVehicleId()==vehicle->GetVehicleId()) 
			  continue;
            
			diff= sqrt(pow(pos.x-p.x,2) + pow(pos.y-p.y,2));
            if(diff < range) neighbors.push_back(v);
          }
      }

  return neighbors;
  }

  std::list<Ptr<Vehicle> > Street::FindVehiclesInSegment(double x1, double x2, int lane)
  {
    std::list<Ptr<Vehicle> > segment;
    Ptr<Vehicle> v=0;
    Vector pos;
    if(lane< 5 && lane>=0)
      {
        for(uint j=0;j<m_vehicles[lane].size();j++)
          {	
            v=GetVehicle(m_vehicles[lane],j);
            pos=v->GetPosition();
            if(pos.x >= x1 && pos.x < x2) segment.push_back(v);
          }
      }
    else if(m_twoDirectional==true && lane < 5 && lane >= 0)
      {
        for(uint j=0;j<m_vehiclesOpp[lane].size();j++)
          { 	
            v=GetVehicle(m_vehiclesOpp[lane],j);
            pos=v->GetPosition();
            if(pos.x >= x1 && pos.x < x2) 
			  segment.push_back(v);
          }
      }

    return segment;		
  }
  Callback<void, Ptr<Vehicle>, Ptr<const Packet>, Address> Street::GetReceiveDataCallback()
  {
    return m_receiveData;
  }

  void Street::SetReceiveDataCallback(Callback<void, Ptr<Vehicle>, Ptr<const Packet>, Address> receiveData)
  {
    m_receiveData = receiveData;
  }

  DeviceTraceCallback Street::GetDevTxTraceCallback()
  {
    return m_devTxTrace;
  }

  void Street::SetDevTxTraceCallback(DeviceTraceCallback devTxTrace)
  {
    m_devTxTrace = devTxTrace;
  }

  DeviceTraceCallback Street::GetDevRxTraceCallback()
  {
    return m_devRxTrace;
  }
  
  void Street::SetDevRxTraceCallback(DeviceTraceCallback devRxTrace)
  {
    m_devRxTrace = devRxTrace;
  }

  PhyRxOkTraceCallback Street::GetPhyRxOkTraceCallback()
  {
    return m_phyRxOkTrace;
  }

  void Street::SetPhyRxOkTraceCallback(PhyRxOkTraceCallback phyRxOkTrace)
  {
    m_phyRxOkTrace = phyRxOkTrace;
  }

  PhyRxErrorTraceCallback Street::GetPhyRxErrorTraceCallback()
  {
    return m_phyRxErrorTrace; 
  }

  void Street::SetPhyRxErrorTraceCallback(PhyRxErrorTraceCallback phyRxErrorTrace)
  {
    m_phyRxErrorTrace = phyRxErrorTrace;
  }

  PhyTxTraceCallback Street::GetPhyTxTraceCallback()
  {
    return m_phyTxTrace;
  }

  void Street::SetPhyTxTraceCallback(PhyTxTraceCallback phyTxTrace)
  {
    m_phyTxTrace = phyTxTrace;
  }

  PhyStateTraceCallback Street::GetPhyStateTraceCallback()
  {
    return m_phyStateTrace;
  }

  void Street::SetPhyStateTraceCallback(PhyStateTraceCallback phyStateTrace)
  {
    m_phyStateTrace = phyStateTrace;
  }

  Callback<bool, Ptr<Street> ,Ptr<Vehicle> , double> Street::GetControlVehicleCallback()
  {
    return m_controlVehicle;
  }

  void Street::SetControlVehicleCallback(Callback<bool, Ptr<Street> ,Ptr<Vehicle> , double> controlVehicle)
  {
    m_controlVehicle = controlVehicle;
  }

  Callback<bool, Ptr<Street>, int&> Street::GetInitVehicleCallback()
  {
    return m_initVehicle;
  }

  void Street::SetInitVehicleCallback(Callback<bool, Ptr<Street>, int&> initVehicle)
  {
    m_initVehicle = initVehicle;
  }
}
