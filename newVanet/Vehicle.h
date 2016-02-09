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
// eu comentei os parametros de length e width do carro
// nao vamos precisar agora
#ifndef CLASS_VEHICLE_
#define CLASS_VEHICLE_

#include "ns3/ptr.h"
#include "ns3/object.h"
#include "ns3/vector.h"
#include "ns3/ptr.h"
#include "ns3/core-module.h"
#include "ns3/wifi-module.h"

namespace ns3
{
  
  class Vehicle : public ns3::Object
  {
    private: 
    	
	  int m_vehicleId;              // vehicle's id
	  double m_velocity;            // vehicle's velocity.
	  double m_positionX;			// X coordinate for the vehicle.
	  double m_positionY;		    // Y coordinate for the vehicle.
	  double m_CrossingTime;        // time to access the crossing area.
      int m_street;					// vehicle's street.
      Ptr<Node> m_node;             // vehicle has a node

    public:

      /// Override TypeId.
      static TypeId GetTypeId (void);
      /// Constructor to initialize values of all variables to zero except VehicleId to one.
      Vehicle();
	  /// Destructor [does nothing].
      ~Vehicle();
      /**
      * \returns the Vehicle Id.
	  *
      */
      int GetVehicleId();
      /**
      * \param value a Vehicle Id.
	  *
	  * A Vehicle can have an Id. It is good that this Id be unique in the VANETs Highway.  
      */
      void SetVehicleId(int value);

//      /**
//      * \returns the position of Vehicle's Node which is located at the center back of the Vehicle.
//      */
//      Vector GetPosition();
//      /**
//      * \param value a position Vector.
//	  *
//	  * This function sets the position of Vehicle's Node. Vehicle's position is its Node's position.
//	  * This position Vector must point to the center back of the Vehicle.
//      */
//      void SetPosition(Vector value);

      /**
      * \returns the current velocity (speed) of the Vehicle.
      */
      double GetVelocity();
      /**
      * \param value the current velocity (speed) of the Vehicle.
      */
      void SetVelocity(double value);

      double getCrossingTime() const;

      void setCrossingTime(double crossingTime);

      Ptr<Node> getNode();

      void setNode(Ptr<Node> node);

      double getPositionX();

      void setPositionX(double positionX);

      double getPositionY();

      void setPositionY(double positionY);

      int getStreet();

      void setStreet(int street);

};
};
#endif
