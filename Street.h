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

#ifndef CLASS_STREET_
#define CLASS_STREET_

#include "ns3/callback.h"
#include "ns3/ptr.h"
#include "ns3/object.h"
#include "ns3/vector.h"
#include "Vehicle.h"
#include "Model.h"
#include "LaneChange.h"
#include <list>

namespace ns3
{
  /**
  * \brief Street is a place holder of the Vehicle (s) which manages each step of the Vehicle mobility.
  * 
  * A Street has up to total 10 lists (queue of lanes), maximum 5 (lanes) for each direction. At each step (interval dt), Street
  * browse vehicles of each lane in order of their poistions. Street moves each Vehicle or does the change of lane based on the
  * required information given by each vehicle and its adjacent vehicles following IDM Model and LaneChange rules. 
  * It is possible to add vehicles to the Street manually, or we can set the Street to do so automatically (AutoInjection).
  * Vehicle depoloyments can happen at InitStreet or through handling the InitVehicle event raised by Street. Although, it is possible to
  * add vehicles to the street at anytime of simulation.
  * Also, Street raises a ControlVehicle event at each passed interval dt to give possibility to access/control Vehicles of the Street.
  * Vehicles can communicate wirelessly at anytime using the shared WifiChannel given by Street,
  * although they may want to use and apply other channels. It is up to user how to catch the Street events and deal with them.
  */
  class Street: public ns3::Object
  {
    private: 

      std::list<Ptr<Vehicle> > m_vehicles[2];    // list of vehicles up to maximum 2 lanes.
      int m_numberOfLanes;                  // number of lanes, maximum value is 2.
      double m_streetLength;	            // the length of the street.
      double m_laneWidth;                   // the width of each lane in the roadway.
      double m_medianGap;                   // the width of the median.
      double m_injectionSafetyGap;          // the entrance gap criteria in meter.
      double m_dt;                          // the mobility step interval. (duration between each step)
      int m_vehicleId;                      // auto increment vehicle id which will be assigned to vehicles.
      bool m_stopped;	                    // true, if the street manager is stopped.
      bool m_autoInject;                    // true, if we desire that vehicles automatically being injected into the street.
      int m_currentLane;                    // current lane number.
	  double m_flow;                        // traffic flow veh/s at entrance.
	  double m_velocity;                    // traffic velocity at entrance.
	  double m_remainder;                   // remainder of vehicles.
	  double m_penetrationRate;             // street equipped vehicle penetration rate;
      Ptr<Vehicle> m_tempVehicles[2];       // temp vehicles.       
      WifiHelper m_wifiHelper;              // a wifi helper apply to setup vehicles Wifi
      NqosWifiMacHelper m_wifiMacHelper;    // a wifi mac helper apply to setup vehicles Wifi
      YansWifiPhyHelper m_wifiPhyHelper;    // a wifi phy helper apply to setup vehicles Wifi
      YansWifiChannelHelper m_wifiChannelHelper; // a wifi channel helper apply to setup vehicles Wifi
      Ptr<YansWifiChannel> m_wifiChannel;   //the common Wifi Channel created by Street which is being shared by the vehicles to communicate.

      /// Initializes the Street and raises the event InitVehicle.
      void InitStreet();
      /// Injects Vehicles based on given minimum gap and percentage p.
      void InjectVehicles(double minGap, int p);
      /// Translates the Vehicles to the new position.
      void TranslateVehicles();
	  /// Calculates the position and velocity of each vehicle for the passed step and the next step. 
      void TranslatePositionVelocity(std::list<Ptr<Vehicle> > vehicles[], double dt);
	  /// Calculates the acceleration of the vehicles in passed step and for the next step.
      void Accelerate(std::list<Ptr<Vehicle> > vehicles[], double dt);
	  /// Find the Vehicles on the Side of the current vehicle veh.
      void FindSideVehicles(std::list<Ptr<Vehicle> > vehicles[], Ptr<Vehicle> veh, int sideLane);
      /// Prints all vehicles in Street.
      void PrintVehicles();

	  /**
      * An event called for each step of mobility for each Vehicle inside the Street.
      * It gives the Street, the Vehicle, and value of dt.
      * If we return true, it means the Vehicle at this step is being controlled manually by the user.
      * If we return false, the Vehicle mobility (position, velocity, acceleration) is ruled by the car following conditions.
	  * this callback must point to the function which handles such event.
      */
      Callback<bool, Ptr<Street> ,Ptr<Vehicle> , double> m_controlVehicle;
      /**
      * An event called at Street initialization (InitStreet).
      * It gives the Street and the current Vehicle Id value.
	  * This callback must point to the function which handles such event.
      */
      Callback<bool, Ptr<Street>, int&> m_initVehicle;
      /// For Catching an event when a packet is received by any vehicles in the Street.
      VehicleReceiveCallback m_receiveData;
      /// For Catching DevTxTrace.
      DeviceTraceCallback m_devTxTrace;
      /// For Catching DevRxTrace.
      DeviceTraceCallback m_devRxTrace;
      /// For Catching PhyRxOkTrace.
      PhyRxOkTraceCallback m_phyRxOkTrace;
      /// For Catching PhyRxErrorTrace.
      PhyRxErrorTraceCallback m_phyRxErrorTrace;
      /// For Catching PhyTxTrace.
      PhyTxTraceCallback m_phyTxTrace;
      /// For Catching PhyStateTrace.
      PhyStateTraceCallback m_phyStateTrace;

    public: 

      /// Override TypeId.
      static TypeId GetTypeId (void);
      /**
	  * Setting the default values:
      * dt=1.0 , VehicleId=1, Number of Lanes=1, Street Length=1000(m), Width of Lanes=5(m), Median Gap=5(m).
      * Injection Safety Gap=5(m).
	  * Nqos Mac, Yans Phy, and 6-mb WIFI_PHY_STANDARD_80211a with Transmission range around 250-300(m).
	  */
      Street();
      /// Destructor to clean up the lists.
      ~Street();
      /**
      * Starts the Street.
      */
      void Start();
      /**
      * Stops the street. Therefore no vehicles mobility after.
      */
      void Stop();

      /**
      * \returns the Number of Lanes in the Street.
      */
      int GetNumberOfLanes();
      /**
      * \param value the Number of Lanes the street can have for each direction [min=0, max=2].
      */
      void SetNumberOfLanes(int value);

      /**
      * \returns the Length of the Street.
      */
      double GetStreetLength();
      /**
      * \param value the Length of the Street.
      */
      void SetStreetLength(double value);
      /**
      * \returns the Width of the Lanes.
      */
      double GetLaneWidth();
      /**
      * \param value the Width of the Lanes.
      */
      void SetLaneWidth(double value);
      /**
	  * \param value the traffic flow veh/sat entrance.
	  */
	  void SetFlow(double value);

      /**
	  * \param value the traffic velocity m/s at entrance.
	  */	 
	  void SetVelocity(double value);

	  /**
      * \returns the Median Gap. (width of the median)
      */
      double GetMedianGap();
      /**
      * \param value the Median Gap. (width of the median)
      */
      void SetMedianGap(double value);

      /**
      * \returns the y (center) of the wanted lane and desired direction. 
      */
      double GetYForLane(int lane,int dir);
      /**
      * \returns true if auto-injection is on, otherwise false.
      */
      bool GetAutoInject();
      /**
      * \param value true will turn the auto-injection on, false will turn it off.
      */
      void SetAutoInject(bool value);
      /**
      * \returns the Injection Gap (meters) needed by the injection at entrance (InjectVehicles()) of the Street.
      */
      double GetInjectionGap();
      /**
      * \param value the Injection Gap (meters) used in InjectVehicles() for entrance. 
      */
      void SetInjectionGap(double value);

      /**
      * \returns true if changing lanes in Street is on, false if off.
      */
      bool GetChangeLane();
      /**
      * \param value true to turn the changing lanes on, false for off.
      */
      void SetChangeLane(bool value);
      /**
      * \returns the value of interval dt, the duration of each mobility step. A interval between each steps.
      */
      double GetDeltaT(void);
      /**
      * \param value the interval dt, the duration of each mobility step. A interval between each steps.
      */
      void SetDeltaT(double value);
	  /**
      * \param value penetration rate of equipped vehicles in street, percentage (0-100)
	  */
	  void SetPenetrationRate(double value);
	  /**
	  * \returns penetration rate of equipped vehicles in street, percentage (0-100)
	  */
	  double GetPenetrationRate();

	  /**
      * it will add the vehicle in to the Street based on the vehicle lane and direction to the appropriate Street list.
      */
      void AddVehicle(Ptr<Vehicle> vehicle);
      /**
      * \returns the last (currently) value of auto-incremented Vehicle Id.
      */
      int GetLastVehicleId();
	  /**
	  * \returns the WifiHelper used by the street.
	  */
      WifiHelper GetWifiHelper();
	  /**
	  * \returns the NqosWifiMacHelepr used by the Street.
	  */
      NqosWifiMacHelper GetNqosWifiMacHelper();
	  /**
	  * \returns the YansWifiPhyHelper used by the Street.
	  */
      YansWifiPhyHelper GetYansWifiPhyHelper();
	  /** 
	  * \param YansWifiPhyHelper which Street will use as default
	  */
	  void SetYansWifiPhyHelper(YansWifiPhyHelper yWifiPhyHelper);
	  /**
	  * \returns the shared WifiChannel used by/in/for the Street.
	  */
      Ptr<YansWifiChannel> GetWifiChannel();
      /**
      * \returns the retrieved Vehicle at the specific Index from the list of street vehicles.
      */
      Ptr<Vehicle> GetVehicle(std::list<Ptr<Vehicle> > v, int index);
      /**
      * \returns the Vehicle from the Street given its VehicleId (vid).
      */
      Ptr<Vehicle> FindVehicle(int vid);
      /**
      * \returns the list of vehicles within the specific Range from a desired Vehicle in the Street.
      */
      std::list<Ptr<Vehicle> > FindVehiclesInRange(Ptr<Vehicle> vehicle, double range);
      /**
      * \returns the list of vehicles for each Lane and Direction in a specific segment of the Street from x1 to x2.
      */
      std::list<Ptr<Vehicle> > FindVehiclesInSegment(double x1, double x2, int lane);

	  
	  /// Returns the Street's Receive Data callback.
      VehicleReceiveCallback GetReceiveDataCallback();
      /// Sets the Street's Receive Data callback.
      void SetReceiveDataCallback(VehicleReceiveCallback receiveData);
      /// Returns the Street's DevTxTrace callback.
      DeviceTraceCallback GetDevTxTraceCallback();
      /// Sets the Street's DevTxTrace callback.
      void SetDevTxTraceCallback(DeviceTraceCallback devTxTrace);
      /// Returns the Street's DevRxTrace callback.
      DeviceTraceCallback GetDevRxTraceCallback();
      /// Sets the Street's DevRxTrace callback.
      void SetDevRxTraceCallback(DeviceTraceCallback devRxTrace);
      /// Returns the Street's PhyRxOkTrace callback.
      PhyRxOkTraceCallback GetPhyRxOkTraceCallback();
      /// Sets the Street's PhyRxOkTrace callback.
      void SetPhyRxOkTraceCallback(PhyRxOkTraceCallback phyRxOkTrace);
      /// Returns the Street's PhyRxErrorTrace callback.
      PhyRxErrorTraceCallback GetPhyRxErrorTraceCallback();
      /// Sets the Street's PhyRxErrorTrace callback.
      void SetPhyRxErrorTraceCallback(PhyRxErrorTraceCallback phyRxErrorTrace);
      /// Returns the Street's PhyTxTrace callback.
      PhyTxTraceCallback GetPhyTxTraceCallback();
      /// Sets the Street's PhyTxTrace callback.
      void SetPhyTxTraceCallback(PhyTxTraceCallback phyTxTrace);
      /// Returns the Street's PhyStateTrace callback.
      PhyStateTraceCallback GetPhyStateTraceCallback();
      /// Sets the Street's PhyStateTrace callback.
      void SetPhyStateTraceCallback(PhyStateTraceCallback phyStateTrace);
      /// Returns the Street Control Vehicle callback.
      Callback<bool, Ptr<Street> ,Ptr<Vehicle> , double> GetControlVehicleCallback();
      /// Sets the Street Control Vehicle callback.
      void SetControlVehicleCallback(Callback<bool, Ptr<Street> ,Ptr<Vehicle> , double> controlVehicle);
      /// Returns the Street Init Vehicle callback.
      Callback<bool, Ptr<Street>, int&> GetInitVehicleCallback();
      /// Sets the Street Init Vehicle callback.
      void SetInitVehicleCallback(Callback<bool, Ptr<Street>, int&> initVehicle);
	  /**
      * Runs one mobility Step for the given Street.
	  * This function is called each interval dt to simulated the mobility through TranslateVehicles().
      */
      static void Step(Ptr<Street> street);
  };
};
#endif
