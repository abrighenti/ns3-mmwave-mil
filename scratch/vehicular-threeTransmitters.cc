/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
*   Copyright (c) 2020 University of Padova, Dep. of Information Engineering,
*   SIGNET lab.
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License version 2 as
*   published by the Free Software Foundation;
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include "ns3/mmwave-sidelink-spectrum-phy.h"
#include "ns3/mmwave-vehicular-net-device.h"
#include "ns3/mmwave-vehicular-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/mmwave-spectrum-value-helper.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/core-module.h"

NS_LOG_COMPONENT_DEFINE ("VehicularThreeTransmitters");

using namespace ns3;
using namespace millicar;

/**
  In this exampls, we consider one groups of 4 vehicles traveling in the same direction
  Vehicles are moving at a constant speed of [speed] m/s and keeping a safety distance of
  [intraGroupDistance] m. In the group, the front vehicle is receiving data packets generated 
  by the other 3 vehicles. We considered an ON-OFF traffic model, in which a UDP source keeps 
  switching between the ON and the OFF states. During the ON state, the source generates 
  packets at a   constant rate for [onPeriod] ms, while in the OFF state it stays idle 
  for a random amount of time, which follows an exponential distribution with mean
  [offPeriod] ms. All vehicles operate at 28 GHz with a bandwidth of 100 MHz,
  possibly interfering in case of concurrent transmissions, and are equipped
  with a Uniform Planar Array (UPA) of [numAntennaElements] antenna elements to
  establish directional communications.
  The simulation runs for [stopTime] ms, at outputs the overall Packet Reception
  Ratio.
*/

uint32_t g_txPacketsGroup1 = 0; // tx packet counter for group 1
uint32_t g_rxPacketsGroup1 = 0; // rx packet counter for group 1

static void Tx (Ptr<OutputStreamWrapper> stream, int device, Ptr<const Packet> p)
{
  *stream->GetStream () << "Tx\t" << device << "\t" << Simulator::Now ().GetSeconds () << "\t" << p->GetSize () << std::endl;
  ++g_txPacketsGroup1;
}

static void Rx (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> packet, const Address& from)
{
  Ptr<Packet> newPacket = packet->Copy ();
  SeqTsHeader seqTs;
  newPacket->RemoveHeader (seqTs);
  if (seqTs.GetTs ().GetNanoSeconds () != 0)
  {
    uint64_t delayNs = Simulator::Now ().GetNanoSeconds () - seqTs.GetTs ().GetNanoSeconds ();
    *stream->GetStream () << "Rx\t" << Simulator::Now ().GetSeconds () << "\t" << packet->GetSize() << "\t" <<  delayNs << std::endl;
  }
  else
  {
    *stream->GetStream () << "Rx\t" << Simulator::Now ().GetSeconds () << "\t" << packet->GetSize() << "\t" << -1 << std::endl;
  }

  //uint64_t delayNs = Simulator::Now ().GetNanoSeconds () - seqTs.GetTs ().GetNanoSeconds ();
  //*stream->GetStream () << "Rx\t" << Simulator::Now ().GetSeconds () << "\t" << packet->GetSize() << "\t" <<  delayNs << std::endl;
  ++g_rxPacketsGroup1;
}

int main (int argc, char *argv[])
{
  uint32_t startTime = 100; // application start time in milliseconds
  uint32_t stopTime = 2000; // application stop time in milliseconds
  uint32_t onPeriod = 100; // on period duration in milliseconds
  uint32_t offPeriod = 100; // mean duration of the off period in milliseconds
  double dataRate = 200e6; // data rate in bps
  uint32_t mcs = 28; // modulation and coding scheme
  bool csma = true;

  double intraGroupDistance = 5.0; // distance between cars belonging to the same group in meters
  double speed = 20; // speed m/s

  uint32_t numAntennaElements = 4; // number of antenna elements
  double intThreshold=1.5e-15;

  CommandLine cmd;
  cmd.AddValue ("startTime", "application stop time in milliseconds", startTime);
  cmd.AddValue ("stopTime", "application stop time in milliseconds", stopTime);
  cmd.AddValue ("onPeriod", "on period duration in milliseconds", onPeriod);
  cmd.AddValue ("offPeriod", "mean duration of the off period in milliseconds", offPeriod);
  cmd.AddValue ("dataRate", "data rate in bps", dataRate);
  cmd.AddValue ("mcs", "modulation and coding scheme", mcs);
  cmd.AddValue ("CSMA", "Usage of csma", csma);
  cmd.AddValue ("intraGroupDistance", "distance between vehicles in the group in meters", intraGroupDistance);
  cmd.AddValue ("threshold", "interference threshold to declare channel idle", intThreshold);
  cmd.AddValue ("speed", "the speed of the vehicles in m/s", speed);
  cmd.AddValue ("numAntennaElements", "number of antenna elements", numAntennaElements);
  cmd.Parse (argc, argv);

  Config::SetDefault ("ns3::MmWaveSidelinkMac::UseAmc", BooleanValue (false));
  Config::SetDefault ("ns3::MmWaveSidelinkMac::UseCSMA", BooleanValue (csma));
  Config::SetDefault ("ns3::MmWaveSidelinkSpectrumPhy::InterferenceThreshold", DoubleValue (intThreshold));
  Config::SetDefault ("ns3::MmWaveSidelinkMac::Mcs", UintegerValue (mcs));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue (28.0e9));
  Config::SetDefault ("ns3::MmWaveVehicularPropagationLossModel::ChannelCondition", StringValue ("l"));
  Config::SetDefault ("ns3::MmWaveVehicularNetDevice::RlcType", StringValue("LteRlcUm"));
  Config::SetDefault ("ns3::MmWaveVehicularHelper::SchedulingPatternOption", EnumValue(2)); // use 2 for SchedulingPatternOption=OPTIMIZED, 1 or SchedulingPatternOption=DEFAULT
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (500*1024));

  Config::SetDefault ("ns3::MmWaveVehicularAntennaArrayModel::AntennaElements", UintegerValue (numAntennaElements));
  Config::SetDefault ("ns3::MmWaveVehicularAntennaArrayModel::AntennaElementPattern", StringValue ("3GPP-V2V"));
  Config::SetDefault ("ns3::MmWaveVehicularAntennaArrayModel::IsotropicAntennaElements", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveVehicularAntennaArrayModel::NumSectors", UintegerValue (2));

  // create the nodes
  NodeContainer group1;
  group1.Create (4);

  // create the mobility models
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (group1);

  
  group1.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,0,0));
  group1.Get (0)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (speed, 0, 0));

  group1.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (intraGroupDistance,0,0));
  group1.Get (1)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (speed, 0, 0));

  group1.Get (2)->GetObject<MobilityModel> ()->SetPosition (Vector (2*intraGroupDistance,0,0));
  group1.Get (2)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (speed, 0, 0));

  group1.Get (3)->GetObject<MobilityModel> ()->SetPosition (Vector (3*intraGroupDistance,0,0));
  group1.Get (3)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (speed, 0, 0));

  // create and configure the helper
  Ptr<MmWaveVehicularHelper> helper = CreateObject<MmWaveVehicularHelper> ();
  helper->SetNumerology (3);
  helper->SetPropagationLossModelType ("ns3::MmWaveVehicularPropagationLossModel");
  helper->SetSpectrumPropagationLossModelType ("ns3::MmWaveVehicularSpectrumPropagationLossModel");
  NetDeviceContainer devs1 = helper->InstallMmWaveVehicularNetDevices (group1);

  InternetStackHelper internet;
  internet.Install (group1);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");

  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devs1);

  helper->PairDevices(devs1);

  Ipv4StaticRoutingHelper ipv4RoutingHelper;

  Ptr<Ipv4StaticRouting> staticRouting = ipv4RoutingHelper.GetStaticRouting (group1.Get (0)->GetObject<Ipv4> ());
  staticRouting->SetDefaultRoute (group1.Get (3)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal () , 2 );

  staticRouting = ipv4RoutingHelper.GetStaticRouting (group1.Get (1)->GetObject<Ipv4> ());
  staticRouting->SetDefaultRoute (group1.Get (3)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal () , 2 );

  staticRouting = ipv4RoutingHelper.GetStaticRouting (group1.Get (2)->GetObject<Ipv4> ());
  staticRouting->SetDefaultRoute (group1.Get (3)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal () , 2 );

  NS_LOG_DEBUG("IPv4 Address node 0 group 1: " << group1.Get (0)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ());
  NS_LOG_DEBUG("IPv4 Address node 1 group 1: " << group1.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ());
  NS_LOG_DEBUG("IPv4 Address node 2 group 1: " << group1.Get (2)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ());
  NS_LOG_DEBUG("IPv4 Address node 3 group 1: " << group1.Get (3)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ());


  // create the random variables used to setup the applications
  Ptr<ConstantRandomVariable> onPeriodRv = CreateObjectWithAttributes<ConstantRandomVariable> ("Constant", DoubleValue (onPeriod / 1000.0));
  Ptr<ExponentialRandomVariable> offPeriodRv = CreateObjectWithAttributes<ExponentialRandomVariable> ("Mean", DoubleValue (offPeriod / 1000.0));

  // create the appplications for group 1
  uint32_t port = 1234;
  OnOffHelper onoff ("ns3::UdpSocketFactory", Address (InetSocketAddress (group1.Get (3)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), port)));
  onoff.SetConstantRate (DataRate (std::to_string (dataRate)+"b/s"));
  onoff.SetAttribute ("OnTime", PointerValue (onPeriodRv));
  onoff.SetAttribute ("OffTime", PointerValue (offPeriodRv));
  onoff.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
  ApplicationContainer onOffApps = onoff.Install (group1.Get (0));
  onOffApps.Add (onoff.Install (group1.Get (1)));
  onOffApps.Add (onoff.Install (group1.Get (2)));

  PacketSinkHelper sink ("ns3::UdpSocketFactory", Address (InetSocketAddress (Ipv4Address::GetAny (), port)));
  ApplicationContainer packetSinkApps = sink.Install (group1.Get (3));


  onOffApps.Start (MilliSeconds (startTime));
  onOffApps.Stop (MilliSeconds (stopTime));

  packetSinkApps.Start (MilliSeconds (0.0));

  // connect the trace sources to the sinks
  AsciiTraceHelper asciiTraceHelper;
  Ptr<OutputStreamWrapper> stream = asciiTraceHelper.CreateFileStream ("plotsTHR/group-1.txt");
  onOffApps.Get (0)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&Tx, stream, 0));
  onOffApps.Get (1)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&Tx, stream, 1));
  onOffApps.Get (2)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&Tx, stream, 2));
  packetSinkApps.Get (0)->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&Rx, stream));

  Simulator::Stop (MilliSeconds(stopTime + 1000));
  Simulator::Run ();
  Simulator::Destroy ();
  
  std::cout << "PRR " << double(g_rxPacketsGroup1) / double(g_txPacketsGroup1) << std::endl;

  return 0;
}
