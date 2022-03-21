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

#include <iomanip>
#include "ns3/kitti-trace-burst-generator.h"

#include "ns3/seq-ts-size-frag-header.h"
#include "ns3/bursty-helper.h"
#include "ns3/burst-sink-helper.h"
#include "ns3/bursty-app-stats-calculator.h"

NS_LOG_COMPONENT_DEFINE ("VehicularThreeTransmittersNew");

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

uint32_t g_rxDev1 = 0; // rx packet counter for dev 1
uint32_t g_rxDev2 = 0; // rx packet counter for dev 2
uint32_t g_rxDev3 = 0; // rx packet counter for dev 3

uint32_t g_txDev1 = 0; // tx packet counter for dev 1
uint32_t g_txDev2 = 0; // tx packet counter for dev 2
uint32_t g_txDev3 = 0; // tx packet counter for dev 3

/*static void Tx (Ptr<OutputStreamWrapper> stream, int device, Ptr<const Packet> p)
{
  *stream->GetStream () << "Tx\t" << device << "\t" << Simulator::Now ().GetSeconds () << "\t" << p->GetSize () << std::endl;
  ++g_txPacketsGroup1;

  switch(device){
    case 1:
      ++g_txDev1;
      break;
    case 2:
      ++g_txDev2;
      break;
    case 3:
      ++g_txDev3;
      break;
    default:
      break;
  }
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

  ++g_rxPacketsGroup1;
  std::string fromstr=AddressToString(from);
  if (fromstr == "10.1.1.1"){
    g_rxDev1+=1;
  }else if (fromstr == "10.1.1.2"){
    g_rxDev2+=1;
  }else if (fromstr == "10.1.1.3"){
    g_rxDev3+=1;
  }
}*/

std::string
AddressToString (const Address &addr)
{
  std::stringstream addressStr;
  addressStr << InetSocketAddress::ConvertFrom (addr).GetIpv4 ();
  return addressStr.str ();
}

void
BurstRx (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> burst, const Address &from, const Address &to,
         const SeqTsSizeFragHeader &header)
{
  *stream->GetStream() << "Received burst seq=" << header.GetSeq () << " of " << header.GetSize ()
                                     << " bytes transmitted at " << std::setprecision (9)
                                     << header.GetTs ().As (Time::S);
}

static void
RxBurstCallback (uint32_t nodeId, Ptr<BurstyAppStatsCalculator> statsCalculator, Ptr<const Packet> burst, const Address &from,
         const Address &to, const SeqTsSizeFragHeader &header)
{
  std::cout << AddressToString(from) << " " << AddressToString(to) << std::endl;
  statsCalculator->RxBurst (nodeId, burst, from, to, header);
}

static void
TxBurstCallback (uint32_t nodeId, Ptr<BurstyAppStatsCalculator> statsCalculator, Ptr<const Packet> burst,
                 const Address &from, const Address &to, const SeqTsSizeFragHeader &header)
{
  
  statsCalculator->TxBurst (nodeId, burst, from, to, header);
}

int main (int argc, char *argv[])
{
  uint32_t stopTime = 2000; // application stop time in milliseconds
  double dataRate = 200e6; // data rate in bps
  uint32_t mcs = 28; // modulation and coding scheme
  bool csma = true;

  double intraGroupDistance = 5.0; // distance between cars belonging to the same group in meters
  double speed = 20; // speed m/s

  uint32_t numAntennaElements = 4; // number of antenna elements
  double intThreshold=0.0;

  CommandLine cmd;
  cmd.AddValue ("stopTime", "application stop time in milliseconds", stopTime);
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

  Ipv4Address serverAddress = i.GetAddress (3);
  Ipv4Address sinkAddress = Ipv4Address::GetAny (); // 0.0.0.0

  std::string traceFolder = "input/"; // example traces can be found here
  std::string traceFile = "kitti-dataset.csv";
  // create the appplications for group 1
  uint32_t port = 50000;
  BurstyHelper burstyHelper ("ns3::UdpSocketFactory",
                             InetSocketAddress (serverAddress, port));
  burstyHelper.SetAttribute ("FragmentSize", UintegerValue (1200));
  burstyHelper.SetBurstGenerator ("ns3::KittiTraceBurstGenerator",
                                  "TraceFile", StringValue (traceFolder + traceFile));

  // Install bursty application
  ApplicationContainer clientApps;
  ApplicationContainer serverApps;
  
  Ptr<BurstyAppStatsCalculator> statsCalculator = CreateObject<BurstyAppStatsCalculator>();

  // Create burst sink helper
  for (int i = 0; i < 3 ; i++)
  {
    clientApps.Add (burstyHelper.Install (group1.Get (i)));
    Ptr<BurstyApplication> burstyApp = clientApps.Get (i)->GetObject<BurstyApplication> ();
    burstyApp->TraceConnectWithoutContext ("BurstTx", MakeBoundCallback (&TxBurstCallback, 1+group1.Get (i)->GetId(), statsCalculator));

  BurstSinkHelper burstSinkHelper ("ns3::UdpSocketFactory",
                                    InetSocketAddress (sinkAddress, port+i));

  // Install HTTP client
    serverApps.Add(burstSinkHelper.Install (group1.Get (3)));
    Ptr<BurstSink> burstSink = serverApps.Get (serverApps.GetN () - 1)->GetObject<BurstSink> ();
  burstSink->TraceConnectWithoutContext ("BurstRx", MakeBoundCallback (&RxBurstCallback, 1+group1.Get(3)->GetId(), statsCalculator));
    // Link the burst generator to the bursty sink to process the correct reception delay
    Ptr<KittiTraceBurstGenerator> ktb =
        DynamicCast<KittiTraceBurstGenerator> (burstyApp->GetBurstGenerator ());
    burstSink->ConnectBurstGenerator (ktb);
  }

  // TraceFileBurstGenerator stops automatically, but we add an early stop to make the example quicker
  Ptr<UniformRandomVariable> rv = CreateObjectWithAttributes<UniformRandomVariable> (
      "Min", DoubleValue (0), "Max", DoubleValue (1.0));
  clientApps.StartWithJitter (Seconds (1.0), rv);
  clientApps.Stop (Seconds(40));
  //Simulator::Stop (MilliSeconds(stopTime + 1000));
  Simulator::Run ();
  Simulator::Destroy ();

  // Stats
  // std::cout << "Dev1 Total RX bursts: " << burstyApp1->GetTotalTxBursts () << "/"
  //           << burstSink->GetTotalRxBursts () << std::endl;
  // std::cout << "Dev1 Total RX fragments: " << burstyApp1->GetTotalTxFragments () << "/"
  //           << burstSink->GetTotalRxFragments () << std::endl;
  // std::cout << "Dev1 Total RX bytes: " << burstyApp1->GetTotalTxBytes () << "/"
  //           << burstSink->GetTotalRxBytes () << std::endl;

  // connect the trace sources to the sinks
  /*
  std::cout << "PRR " << double(g_rxPacketsGroup1) / double(g_txPacketsGroup1) << std::endl;
  std::cout << "PRR dev1" << double(g_rxDev1) / double(g_txDev1) << std::endl;
  std::cout << "PRR dev2" << double(g_rxDev2) / double(g_txDev2) << std::endl;
  std::cout << "PRR dev3" << double(g_rxDev3) / double(g_txDev3) << std::endl;*/

  return 0;
}
