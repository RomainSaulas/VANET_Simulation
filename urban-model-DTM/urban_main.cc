// Basic C++ library
#include <iostream>
#include <fstream>
#include <sstream>

#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/position-allocator.h"


// Mobility Helper modules
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"

// needed to work with SUMO config files
#include "ns3/ns2-mobility-helper.h"

// for fix mobility
#include "ns3/mobility-model.h"
#include "ns3/mobility-helper.h"

// Wave handler inclusion
#include "ns3/wave-net-device.h"
#include "ns3/wave-mac-helper.h"

// Wifi 802_11p handler inclusion
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-helper.h"

// Ipv4 and internet stack helper includes
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"

// Wifi simulator model include
#include "ns3/yans-wifi-helper.h"

// needed for animation
#include "ns3/netanim-module.h"

#include "ns3/dtm-packet.h"
#include "ns3/dtm-wqueue.h"
#include "ns3/dtm-ptable.h"

using namespace ns3;

// debug log definebbb
NS_LOG_COMPONENT_DEFINE ("UrbanModelScript");


// define global position table and message queue for nodes
dtm::PositionTable * m_neighbors;
dtm::Queue * m_queue;

dtm::PositionTable *
InitializeNeighbors (uint32_t size)
{
   return new dtm::PositionTable[size];
}

dtm::Queue *
InitializeQueue (uint32_t size)
{
   return new dtm::Queue[size];
}


uint32_t m_messageId = 0;

// Set up the global broadcast address
uint16_t beacon_port = 80;
/// Differents times
const Time m_beacon_interval = Seconds (15); 
const Time m_purge_interval = Seconds (0.50);
const Time m_tmax = Seconds (0.05);
const double m_maxRange = 300.0;

InetSocketAddress broadcastAddr = InetSocketAddress (Ipv4Address ("255.255.255.255"), beacon_port);

NodeContainer nodes;
NetDeviceContainer devices;

// Measures variables
std::vector<uint32_t> m_receivers;
std::vector<uint32_t> m_rebroadcast;
uint32_t m_warningBytes = 0;
uint32_t m_helloBytes = 0;

// Prints actual position and velocity when a course change event occurs
static void
CourseChange (std::ostream *os, std::string foo, Ptr<const MobilityModel> mobility)
{
   //NS_LOG_INFO ("CourseChange");

   Vector pos = mobility->GetPosition (); // Get position
   Vector vel = mobility->GetVelocity (); // Get velocity

   Ptr<const Object> object = mobility;
   Ptr<Node> node = object->GetObject<Node> ();

   // Prints position and velocities
   *os << Simulator::Now () << " POS: x=" << pos.x << ", y=" << pos.y
   << ", z=" << pos.z << "; VEL:" << vel.x << ", y=" << vel.y
   << ", z=" << vel.z << std::endl;
}


/// Calculate the Distance to mean for the node and its neighbors
double CalculateDTM (Ptr<Node> node, Vector spatialDist, double maxRangeArea) 
{
   //NS_LOG_LOGIC (" Distance To Mean function: ");

   //NS_LOG_INFO (" spatialDist.X: " << spatialDist.x << " spatialDist.Y: " << spatialDist.y);

   Ptr<Object> object = node;
   Ptr<MobilityModel> mobility = object->GetObject<MobilityModel> ();

   Vector position = mobility->GetPosition (); 
   //NS_LOG_INFO (" position.X: " << position.x << " position.Y: " << position.y);


   double M = (1/maxRangeArea) * sqrt (pow (position.x - spatialDist.x, 2) + pow (position.y - spatialDist.y, 2)); 

   //NS_LOG_INFO (" Distance To Mean : " << M);

   return M;
}

void PurgeQueue (uint32_t nodeId, uint32_t messageId)
{
   m_queue[nodeId].Purge (messageId);
}

void Forward (Ptr<Node> recvNode, dtm::QueueEntry& entry)
{
   if (!(entry == m_queue[recvNode->GetId ()].GetEntry (entry.GetMessageId ())))
      return;

   Vector spatialDist = m_queue[recvNode->GetId ()].CalculateSpatialDist (entry.GetMessageId ());
   double dtm = CalculateDTM (recvNode, spatialDist, m_maxRange);
   double threshold = m_neighbors[recvNode->GetId ()].GetThreshold ();

   if (dtm <= threshold) {
      Simulator::Schedule (m_queue[recvNode->GetId ()].GetQueueTimeOut (), &PurgeQueue, recvNode->GetId (), entry.GetMessageId ());
      return;
   }

   // Update nb of rebroadcast
   m_rebroadcast.at (entry.GetMessageId ()) += 1;

   //NS_LOG_INFO( " Id: " << recvNode->GetId () << " Forward packet: " << entry.GetMessageId ());

   TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

   Ptr<Socket> forwardSocket = Socket::CreateSocket (recvNode, tid);
   forwardSocket->Connect (broadcastAddr);
   forwardSocket->SetAllowBroadcast (true);

   Ptr<Packet> packet = entry.GetPacket ();

   // Get mobility of receiver node
   Ptr<Object> object = recvNode;
   Ptr<MobilityModel> mobility = object->GetObject<MobilityModel> ();


   // Create and Add Warning Header
   dtm::WarningHeader wHeader = dtm::WarningHeader (
      entry.GetSourceId (),
      recvNode->GetId (),
      entry.GetHopCount ()+1,
      entry.GetMessageId (),
      (uint64_t)mobility->GetPosition ().x,
      (uint64_t)mobility->GetPosition ().y
      );
   packet->AddHeader (wHeader);

   // Create and Add Type Header
   dtm::TypeHeader tHeader = dtm::TypeHeader (dtm::DTM_WARNING);
   packet->AddHeader (tHeader);

   // indicate entry as forwarded
   m_queue[recvNode->GetId ()].GetEntry (entry.GetMessageId ()).SetForwarded (true);
   //NS_LOG_INFO (" FORWARDED MODIFIED: " << m_queue[recvNode->GetId ()].IsAlreadyForwarded (entry.GetMessageId ())); 

   // update bytes send
   m_warningBytes += packet->GetSize ();

   // Rebroadcast Packet
   forwardSocket->Send (packet);

   // Close Socket
   forwardSocket->Close ();

   // Declare entry as forwarded
   Simulator::Schedule (m_queue[recvNode->GetId ()].GetQueueTimeOut (), &PurgeQueue, recvNode->GetId (), entry.GetMessageId ());
}

Time CalculateBackOffTime (Vector recvPosition, Vector senderPosition, double maxRangeArea, Time tmax)
{
   //NS_LOG_LOGIC(" CalculateBackOffTime: ");

   double distance = sqrt ( pow (recvPosition.x - senderPosition.x,2) + pow (recvPosition.y - senderPosition.y,2));

   double coeff = (1.0 -(double) distance/maxRangeArea);

   return Seconds (coeff*tmax.GetSeconds ());
}

void GenerateHelloMessage (Ptr<Node> node)
{
   //NS_LOG_INFO ("GenerateHelloMessage");

   Ptr<Object> object = node;
   Ptr<MobilityModel> mobility = object->GetObject<MobilityModel> ();
   if (mobility == 0) 
      {
         NS_FATAL_ERROR ("The requested mobility model is not a mobility model");
      }
   else 
      {     
         Ptr<Packet> packet = Create<Packet> (200);

         uint32_t id = node->GetId ();
         Vector position = mobility->GetPosition ();
         Vector speed = mobility->GetVelocity ();

         /*
         NS_LOG_INFO (" Id " << id 
                        << " X " << position.x
                        << " Y " << position.y
                        << " Speed X " << speed.x
                        << " Speed Y " << speed.y);
         */
         dtm::HelloHeader hHeader =  dtm::HelloHeader (id, 
                                       (uint64_t) position.x, 
                                       (uint64_t) position.y, 
                                       (uint64_t) speed.x, 
                                       (uint64_t) speed.y);

         dtm::TypeHeader tHeader = dtm::TypeHeader (dtm::DTM_HELLO);

         packet->AddHeader (hHeader);
         packet->AddHeader (tHeader);

         m_neighbors[id].Purge ();

          // Beacon source Socket
         TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
         Ptr<Socket> socket = Socket::CreateSocket (node, tid);

         socket->Connect (broadcastAddr);
         socket->SetAllowBroadcast (true);

         // update bytes send
         m_helloBytes += packet->GetSize ();

         socket->Send (packet);

         socket->Close ();

         Simulator::Schedule (m_beacon_interval, &GenerateHelloMessage, node);
      } 
}

void GenerateWarningMessage (Ptr<Node> node) 
{
   //NS_LOG_INFO (" GenerateWarningMessage ");

   Ptr<Object> object = node;
   Ptr<MobilityModel> mobility = object->GetObject<MobilityModel> ();
   if (mobility == 0) 
      {
         NS_FATAL_ERROR ("The requested mobility model is not a mobility model");
      }
   else 
      {     
         TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

         Ptr<Packet> packet = Create<Packet> ();

         Vector position = mobility->GetPosition ();

         //NS_LOG_INFO (" MessageId: " << m_messageId);
         dtm::WarningHeader wHeader = dtm::WarningHeader(node->GetId (),
                                           node->GetId (), 
                                           0, 
                                           m_messageId++, 
                                           position.x, 
                                           position.y);
         dtm::TypeHeader tHeader = dtm::TypeHeader (dtm::DTM_WARNING);

         packet->AddHeader (wHeader);
         packet->AddHeader (tHeader);

         Ptr<Socket> socket = Socket::CreateSocket (node, tid);
         socket->Connect (broadcastAddr);
         socket->SetAllowBroadcast (true);

         // update bytes send
         m_warningBytes += packet->GetSize ();

         socket->Send(packet);

         socket->Close ();

         //Create reachability variable
         m_receivers.insert (m_receivers.end (), 0);
         m_rebroadcast.insert (m_rebroadcast.end (), 0);
      }
}

void ReceivePacket (Ptr<Socket> socket)
{
   //NS_LOG_INFO ("ReceivePacket");

   Ptr<Node> recvNode = socket->GetNode ();
   Ptr<Packet> packet = socket->Recv();

   dtm::TypeHeader tHeader (dtm::DTM_HELLO);
   packet->RemoveHeader (tHeader);
   if (!tHeader.IsValid ())
      {
      //NS_LOG_DEBUG ("dtm message " << packet->GetUid () << " with unknown type received: " << tHeader.Get () << ". Ignored");
         return;
      }
   switch (tHeader.Get ())
      {
         case (dtm::DTM_HELLO):
            {
               dtm::HelloHeader hHeader;
               packet->RemoveHeader (hHeader);
               
               uint32_t id;
               Vector position;
               Vector speed;
   
               id = hHeader.GetId ();
               position.x = hHeader.GetOriginPosx ();
               position.y = hHeader.GetOriginPosy ();
               speed.x = hHeader.GetSpeedx ();
               speed.y = hHeader.GetSpeedy ();

               /*
               NS_LOG_INFO("Receive Hello:");
               NS_LOG_INFO (" Id " << id 
                        << " X " << position.x
                        << " Y " << position.y
                        << " Speed X " << speed.x
                        << " Speed Y " << speed.y);

               NS_LOG_INFO(" \n Neighbors of :" << recvNode->GetId () 
                     << " :: " << m_neighbors[recvNode->GetId ()]);
               */
               m_neighbors[recvNode->GetId ()].AddEntry (id, 
                                                position, 
                                                speed);


               break;
            }
            case (dtm::DTM_WARNING):
            {// To Code
               
               dtm::WarningHeader wHeader = dtm::WarningHeader ();
               packet->RemoveHeader (wHeader);

               //NS_LOG_INFO("Receive Warning: " << wHeader.GetMessageId () << " from: " << wHeader.GetPrevHopId ());

               // if receiver node == source node
               if (recvNode->GetId () == wHeader.GetSourceId ())
                  {
                     //NS_LOG_INFO ("packet ignored: I am source node");
                     return;
                  }
               
               // if received message already received
               if (m_queue[recvNode->GetId ()].Find (wHeader.GetMessageId (), wHeader.GetPrevHopId ()))
                  {
                     //NS_LOG_INFO ("packet ignored: already received");
                     return;
                  }

               // if packet already forwarded by this node
               if (m_queue[recvNode->GetId ()].IsAlreadyForwarded (wHeader.GetMessageId ()))
                  {
                     //NS_LOG_INFO ("packet ignored: already forwarded");
                     return;
                  }

               if (! (m_queue[recvNode->GetId ()].Exist (wHeader.GetMessageId ())))
                  {
                     // Update nb of bytes received and receivers
                     m_receivers.at (wHeader.GetMessageId ()) += 1;
                  }

               Ptr<Object> object = recvNode;
               Ptr<MobilityModel> mobility = object->GetObject<MobilityModel> ();

               // Calculate backoff time
               Time backoffTime = CalculateBackOffTime (
                  mobility->GetPosition (),
                  Vector((double)wHeader.GetPositionx (), (double)wHeader.GetPositiony (), 0),
                  m_maxRange,
                  m_tmax
                  );

               dtm::QueueEntry en = dtm::QueueEntry (
                  Vector((double)wHeader.GetPositionx (), (double)wHeader.GetPositiony (), 0),
                  backoffTime + Simulator::Now (),
                  packet,
                  wHeader.GetSourceId (),
                  wHeader.GetMessageId (),
                  wHeader.GetPrevHopId (),
                  wHeader.GetHopCount (),
                  false);

               m_queue[recvNode->GetId ()].Add (en);

               Simulator::Schedule (en.GetBackOffTime (), &Forward, recvNode, en);
               
               break;
            }
         default:
         NS_LOG_DEBUG (" dtm message " << packet->GetUid () << " with unknown type received: " << tHeader.Get () << ". Ignored");
      }    
}

NodeContainer CreateNodes (std::string traceFile, uint32_t nodeNum)
{

   // Create all nodes.
   //NS_LOG_INFO ("Creating Topology");
   
   NodeContainer nodes;
   nodes.Create (nodeNum);

   // Create Ns2MobilityHelper with the specified trace log file as parameter
   Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
   ns2.Install (); // configure movements for each node, while reading trace file

   /*
   MobilityHelper mobility;   

   mobility.SetPositionAllocator ("ns3::RandomRectanglePositionAllocator",
      "X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"),
      "Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=10.0]"));
  
   mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");

   mobility.Install (nodes);
   */   

   return nodes;
}

NetDeviceContainer CreateDevice (NodeContainer nodes)
{
   NetDeviceContainer devices;

   // Create Wave handler and network model
   // Create Wifi network model physical layer
   //YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
   YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
   wifiPhy.Set ("TxPowerStart", DoubleValue (25));  // 250-300 meter transmission range
   wifiPhy.Set ("TxPowerEnd", DoubleValue (25));  // 250-300 meter transmission range
   wifiPhy.Set ("TxPowerLevels",UintegerValue (1));
   wifiPhy.Set ("TxGain",DoubleValue (0));
   wifiPhy.Set ("RxGain",DoubleValue (0));
   wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (-101.0));

   // Create Wifi network model
   YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();

   // Create wave default wifichannel and apply it to network physical layer
   wifiPhy.SetChannel (wifiChannel.Create ());

   // Set wifi using ieee wifi802.11p Helper
   wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);

   NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
   Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();

   // Define Wifi Remote Station Manager
   std::string phyMode ("OfdmRate6MbpsBW10MHz");
   wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode));


   // Create set of nodes with the right network characteristics
   //devices = waveHelper.Install (wavePhy, waveMac, nodes);
   devices = wifi80211p.Install (wifiPhy, wifi80211pMac, nodes);


   // Enable physical level tracing
   wifiPhy.EnablePcap ("wave-simple-80211p", devices);


   return devices;
}

int main (int argc, char *argv[]) {

   //LogComponentEnable ("Ns2MobilityHelper", LOG_ALL);

   std::string traceFile;
   std::string logFile;

   int nodeNum;
   double duration;

   // Setting attributes
   //traceFile = "scratch/urban-model/ns2-mobility-model/urban.mobility.tcl";
   //nodeNum = 20;
   duration = 300.0;
   logFile = "scratch/urban-model/logs/ns2-mobility-trace.txt";

   // Enable logging from the ns2 helper
   //LogComponentEnable ("Ns2MobilityHelper",LOG_LEVEL_DEBUG);

   // Parse command line attribute
   CommandLine cmd;
   cmd.AddValue ("i", "Ns2 movement trace file", traceFile);
   cmd.AddValue ("n", "Number of nodes", nodeNum);
   cmd.AddValue ("logFile", "Log file", logFile);
   cmd.Parse (argc,argv);

 
   nodes = CreateNodes (traceFile, nodeNum);

   // Create net device.
   devices = CreateDevice (nodes);

   // Set internet stack helper (set ipv4)
   InternetStackHelper internet;
   internet.Install (nodes);

   Ipv4AddressHelper ipv4;
   NS_LOG_INFO ("Assign IP Addresses.");
   ipv4.SetBase ("10.1.0.0", "255.255.0.0");
   Ipv4InterfaceContainer ipv4Container = ipv4.Assign (devices);

   TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

   // Create tab of Position Table
   m_neighbors = InitializeNeighbors (devices.GetN ());
   // Create tab of queue
   m_queue = InitializeQueue (devices.GetN ());

   // Register callback function when receiving
   for (uint32_t i=0; i != devices.GetN (); ++i) 
   {
      // Get node position
      Ptr<Object> object = nodes.Get (i);
      Ptr<ConstantVelocityMobilityModel> mobility = object->GetObject<ConstantVelocityMobilityModel> ();

      //mobility->SetVelocity (Vector (20.0+ (i%10), 0.0, 0.0));

      // Initialize table for each node
      m_neighbors[i] = dtm::PositionTable ();

      // Initialize queue for each node
      m_queue[i] = dtm::Queue (50, Seconds(10));

      // Beacon sink on every nodes
      Ptr<Socket> recv = Socket::CreateSocket (nodes.Get (i), tid);
      recv->Bind (InetSocketAddress (nodes.Get (i)
                    ->GetObject<ns3::Ipv4> ()
                    ->GetAddress (1,0)
                     .GetLocal (), beacon_port));
      
      NS_LOG_INFO ("Addresse local: " << nodes.Get (i)->GetObject<ns3::Ipv4> ()->GetAddress (1,0).GetLocal ());
      //device.allocate
      //device->SetReceiveCallback( MakeCallback( &WaveNetDevice))
      recv->SetRecvCallback (MakeCallback (&ReceivePacket));

      Simulator::ScheduleWithContext (i, Seconds (1.0 + i*0.1), &GenerateHelloMessage, nodes.Get (i));
   }


   uint32_t randN;
   for (uint32_t mes = 50; mes < 250; mes+=5) 
      {
         randN = rand () % nodes.GetN ();
         Simulator::ScheduleWithContext(randN, Seconds(mes), &GenerateWarningMessage, nodes.Get (randN));
      }

   // open log file for output
   std::ofstream os;
   os.open (logFile.c_str ());

   // Configure callback for logging
   Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
                MakeBoundCallback (&CourseChange, &os));

   // Create netanim animation file
   AnimationInterface anim ("animation.xml");

   Simulator::Stop (Seconds (duration));
   Simulator::Run ();
   Simulator::Destroy ();

   // Calculate res
   double reach_mean = 0.0;
   std::vector<uint32_t>::iterator recv = m_receivers.begin ();
   for (; recv != m_receivers.end (); recv++)
      {
         reach_mean += *recv;
      }
   reach_mean = reach_mean/((double) m_receivers.size ()*nodeNum);

   double rebroad_mean = 0.0;
   
   for (uint32_t rebr = 0; rebr < m_rebroadcast.size (); rebr++)
      {
         if (m_receivers.at(rebr) != 0) {
            NS_LOG_INFO (" rebr: " << rebr 
               << " m_rebr " << m_rebroadcast.at (rebr)
               << " m_recv " << m_receivers.at (rebr));
            rebroad_mean += m_rebroadcast.at (rebr) / (double) m_receivers.at (rebr);  
         }
      }
   rebroad_mean = rebroad_mean/(double) m_rebroadcast.size ();

   NS_LOG_INFO ("\n nb of nodes " << nodeNum
      << "\n Reachability " << reach_mean
      << "\n rebroadcast per covered nodes " << rebroad_mean
      << "\n nb of warning bytes transmitted " << m_warningBytes
      << "\n nb of hello bytes transmitted " << m_helloBytes);

   // close log file
   os.close();

   return 0;
}
