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

#include "Vehicle.h"

namespace ns3
{	
  TypeId Vehicle::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::Vehicle")
    .SetParent<Object> ()
    .AddConstructor<Vehicle> ()
    ;
    return tid;
  }

  Vehicle::Vehicle()
  {
    m_node=CreateObject<Node>();
    m_vehicleId = m_node->GetId();
    m_velocity = 0;
	m_CrossingTime=10;
	m_street=0;
	m_initialPosition = Vector (0.0, 0.0, 0.0);
  }

  Vehicle::Vehicle(double vel, Vector initialPosition)
  {
    m_node=CreateObject<Node>();
    m_vehicleId = m_node->GetId();
    m_velocity = vel;
	m_CrossingTime=10;
	m_street=0;
	m_initialPosition=initialPosition;
  }

  Vehicle::~Vehicle()
  {
  }

  int Vehicle::GetVehicleId()
  {
    return m_vehicleId;
  }

  void Vehicle::SetVehicleId(int value)
  {
    m_vehicleId=value;
  }

//  Vector Vehicle::GetPosition()
//  {
//    return m_node->GetObject<MobilityModel>()->GetPosition();
//  }
//
//  void Vehicle::SetPosition(Vector value)
//  {
//    m_node->GetObject<MobilityModel>()->SetPosition(value);
//  }

  double Vehicle::GetVelocity()
  {
    return m_velocity;
  }

  void Vehicle::SetVelocity(double value)
  {
    m_velocity=value;
  }

	double Vehicle::getCrossingTime() const {
		return m_CrossingTime;
	}

	void Vehicle::setCrossingTime(double crossingTime) {
		m_CrossingTime = crossingTime;
	}

	Ptr<Node> Vehicle::getNode(){
		return m_node;
	}

	void Vehicle::setNode(Ptr<Node> node) {
		m_node = node;
	}

	int Vehicle::getStreet(){
		return m_street;
	}

	void Vehicle::setStreet(int street) {
		m_street = street;
	}

	Vector Vehicle::GetInitialPosition()
	  {
	    return m_initialPosition;
	  }

	  void Vehicle::SetInitialPosition(Vector value)
	  {
	    m_node->GetObject<MobilityModel>()->SetPosition(value);
	  }

}
