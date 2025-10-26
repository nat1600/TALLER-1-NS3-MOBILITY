/**
 * Taller 1 - Modelos estocásticos y simulación en computación y comunicaciones
 */
#include <fstream>
#include <iostream>
#include <ctime>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/olsr-module.h"
#include "ns3/csma-module.h"
#include "ns3/ssid.h"
#include "ns3/applications-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/stats-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("Taller1v4");

struct SimulationResult
{
    // useful for saving simulation statistics and useful data
    double throughput;
    double lossRate;
};

// Define main class (Architecture)
class Taller1Experiment
{
public:
    // Define default constructor
    Taller1Experiment();

    // Define default process
    SimulationResult Run();

    // Handle commandline arguments
    void HandleCommandLineArgs(int, char **, double[]);

    // UDP sender port number
    int port;

    // Number of levels
    int nLevels;

    // Data for first level
    int nClusters_1st_level, nNodes_pC_1st_level;

    // Data for second level
    int nClusters_2nd_level, nNodes_pC_2nd_level;

    // Data for third level
    int nClusters_3rd_level, nNodes_pC_3rd_level;

    // Area bounds
    double width, height;

    // Statistics
    int receivedCount = 0; // Packets
    int sentCount = 0;     // Packets

    // Speed parameters for mobility models
    double nodeMinSpeed_lvl1, nodeMaxSpeed_lvl1;
    double nodeMinSpeed_lvl2, nodeMaxSpeed_lvl2;

    // Second layer resources
    std::string secondLayerResources = "OfdmRate48Mbps";

    // Resources array for first layer's clusters
    std::vector<double> firstLayerResources;

    // Probability
    double probability = 0.5;

    // Mean offTime
    double meanOffTime = 10; // Seconds

    // Traffic ratio for nodes
    double trafficRatio = 0.99;

    // Simulation time
    double simulationTime = 30.0; // Seconds
};

// Save a specific node useful info (resources actually)
class ClusterNode
{
public:
    // Mean for OffTime (distribute with a exponential random variable)
    double trafficRatio;

    // Data rate for OnOffModel
    double dataRate;

    // Save node index, as a utility
    int index;

    // Save a reference to ns3::Node
    Ptr<Node> node;

    // Referenc parent experiment
    Taller1Experiment *parent = NULL;

    // Whether this node was already configured as receiver in past or not
    bool configuredAsReceiver = false;

    // Whether this node was already configured as sender in past or not
    bool configuredAsSender = false;

    // Default constructor
    ClusterNode() {}

    // Construct with resources and offtime's mean
    // Bool -> Whether the second parameter represents resources or dataRate
    ClusterNode(int, bool, double, double, Ptr<Node>);

    // Calculate resources
    double getResources();

    // Generate and track traffic
    ApplicationContainer connectWithNode(ClusterNode &, Taller1Experiment *);

    // Callbacks
    void ReceivePacket(Ptr<Socket> socket);
    void OnPacketSent(Ptr<const Packet> packet);
    void configureAsReceiver(Taller1Experiment *);
};

// Collection of nodes with a head
class Cluster
{
public:
    NodeContainer ns3Nodes;
    NodeContainer ns3NodesExcludingHead;
    NodeContainer headContainer;
    NetDeviceContainer ns3Devices;
    int index;
    std::vector<ClusterNode> nodes;

    Cluster(int);
    void separateHead(uint32_t);
    void createClusterNodes(double, double, double);
    void generateNodes(int);
    void setNodes(NodeContainer);
    double getResources();
};

// Class to represent a cluster of clusters
class Level
{
public:
    std::vector<Cluster> clusters;
    Level() {}
    double getResources();
};

double TruncatedDistribution(int, double, double, int);

ClusterNode::ClusterNode(
    int _index,
    bool includesResources,
    double _trafficRatio,
    double arg2,
    Ptr<Node> _node)
{
    node = _node;
    index = _index;
    trafficRatio = _trafficRatio;

    if (includesResources)
    {
        dataRate = arg2 / trafficRatio;
    }
    else
    {
        dataRate = arg2;
    }
}

double ClusterNode::getResources()
{
    return trafficRatio * dataRate;
}

ApplicationContainer ClusterNode::connectWithNode(ClusterNode &receiver, Taller1Experiment *_parent)
{
    parent = _parent;

    OnOffHelper onoff("ns3::UdpSocketFactory", Address());

    std::stringstream ssOffTime;
    ssOffTime << "ns3::ExponentialRandomVariable[Mean="
              << parent->meanOffTime
              << "]";

    onoff.SetAttribute("OffTime", StringValue(ssOffTime.str()));

    std::stringstream ss;
    ss << "ns3::ExponentialRandomVariable[Mean="
       << (trafficRatio * 0.1 / (1 - trafficRatio))
       << "]";
    onoff.SetAttribute("OnTime", StringValue(ss.str()));

    onoff.SetAttribute("DataRate", DataRateValue(DataRate(dataRate)));

    Ptr<Node> receiverNs3Node = receiver.node;

    uint32_t pktSize = 1024;
    onoff.SetAttribute("PacketSize", UintegerValue(pktSize));

    Ipv4Address remoteAddr = receiverNs3Node->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();

    AddressValue remoteAddress(InetSocketAddress(remoteAddr, parent->port));
    onoff.SetAttribute("Remote", remoteAddress);

    ApplicationContainer sendApp = onoff.Install(node);
    sendApp.Start(Seconds(0.0));
    sendApp.Stop(Seconds(parent->simulationTime));

    receiver.configureAsReceiver(parent);

    if (!configuredAsSender)
    {
        std::string path = "/NodeList/" + std::to_string(node->GetId()) + "/ApplicationList/*/$ns3::OnOffApplication/Tx";
        Config::ConnectWithoutContext(path, MakeCallback(&ClusterNode::OnPacketSent, this));
    }
    else
    {
        configuredAsSender = true;
    }

    return sendApp;
}

void ClusterNode::configureAsReceiver(Taller1Experiment *_parent)
{
    if (configuredAsReceiver)
        return;

    parent = _parent;

    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    Ptr<Socket> recvSink = Socket::CreateSocket(node, tid);

    Ipv4Address remoteAddr = node->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();

    InetSocketAddress local = InetSocketAddress(remoteAddr, parent->port);
    recvSink->Bind(local);
    recvSink->SetRecvCallback(MakeCallback(&ClusterNode::ReceivePacket, this));

    configuredAsReceiver = true;
}

void ClusterNode::OnPacketSent(Ptr<const Packet> packet)
{
    parent->sentCount++;
}

void ClusterNode::ReceivePacket(Ptr<Socket> socket)
{
    while (socket->GetRxAvailable() > 0)
    {
        socket->Recv();
        parent->receivedCount++;
    }
}

Cluster::Cluster(int _index)
{
    index = _index;
}

void Cluster::generateNodes(int nNodes)
{
    ns3Nodes.Create(nNodes);
}

void Cluster::setNodes(NodeContainer _nodes)
{
    ns3Nodes = _nodes;
}

void Cluster::createClusterNodes(double trafficRatio, double totalResouces, double probability)
{
    int length = ns3Nodes.GetN();

    for (int j = 0; j < length; j++)
    {
        double nodeResources = TruncatedDistribution(
            length, totalResouces, probability, j);

        nodes.push_back(ClusterNode(j, true, trafficRatio, nodeResources, ns3Nodes.Get(j)));
    }
}

void Cluster::separateHead(uint32_t headIndex)
{
    Ptr<Node> head = ns3Nodes.Get(headIndex);

    for (uint32_t i = 0; i < ns3Nodes.GetN(); i++)
    {
        if (i == headIndex)
            continue;

        ns3NodesExcludingHead.Add(ns3Nodes.Get(i));
    }

    headContainer.Add(head);
}

double Cluster::getResources()
{
    double resources = 0;

    for (ClusterNode node : nodes)
    {
        resources += node.getResources();
    }

    return resources;
}

double Level::getResources()
{
    double resources = 0;

    for (Cluster cluster : clusters)
    {
        resources += cluster.getResources();
    }

    return resources;
}

double TruncatedDistribution(
    int nPoints, double totalResources, double probability, int nodeIndex)
{
    double portion = probability * pow(1 - probability, nodeIndex) / (1 - pow(1 - probability, nPoints));
    return portion * totalResources;
}

Taller1Experiment::Taller1Experiment()
    : port(9),
      nLevels(2),
      nClusters_1st_level(6),
      nNodes_pC_1st_level(6),
      nClusters_2nd_level(1),
      nNodes_pC_2nd_level(6),
      nClusters_3rd_level(1),
      nNodes_pC_3rd_level(1),
      width(100),
      height(100),
      nodeMinSpeed_lvl1(0.1),
      nodeMaxSpeed_lvl1(1.0),
      nodeMinSpeed_lvl2(0.1),
      nodeMaxSpeed_lvl2(5.0)
{
}

void Taller1Experiment::HandleCommandLineArgs(int argc, char **argv, double resources[])
{
    CommandLine cmd(__FILE__);

    double nc1l = nClusters_1st_level;
    double nn1l = nNodes_pC_1st_level;
    double nc2l = nClusters_2nd_level;
    double nn2l = nNodes_pC_2nd_level;
    double nc3l = nClusters_3rd_level;
    double nn3l = nNodes_pC_3rd_level;
    double nlevels = nLevels;

    cmd.AddValue("nLevels", "Number of levels of this cluster", nlevels);
    cmd.AddValue("nClusters_1st_level", "Number of clusters in 1st level", nc1l);
    cmd.AddValue("nNodes_pC_1st_level", "Number of nodes per cluster in 1st level", nn1l);
    cmd.AddValue("nClusters_2nd_level", "Number of clusters in 2nd level", nc2l);
    cmd.AddValue("nNodes_pC_2nd_level", "Number of nodes per cluster in 2nd level", nn2l);
    cmd.AddValue("nClusters_3rd_level", "Number of clusters in 3rd level", nc3l);
    cmd.AddValue("nNodes_pC_3rd_level", "Number of nodes per cluster in 3rd level", nn3l);
    cmd.AddValue("secondLayerResources", "Resources for second layer", secondLayerResources);
    cmd.AddValue("trafficRatio", "Traffic ratio per node", trafficRatio);
    cmd.AddValue("meanOffTime", "Mean offtime per node", meanOffTime);
    cmd.AddValue("width", "Width of the space", width);
    cmd.AddValue("height", "Height of the space", height);
    cmd.AddValue("simulationTime", "Simulation time in seconds", simulationTime);
    
    // Speed parameters for mobility models
    cmd.AddValue("nodeMinSpeed_lvl1", "Minimum speed for level 1 nodes (m/s)", nodeMinSpeed_lvl1);
    cmd.AddValue("nodeMaxSpeed_lvl1", "Maximum speed for level 1 nodes (m/s)", nodeMaxSpeed_lvl1);
    cmd.AddValue("nodeMinSpeed_lvl2", "Minimum speed for level 2 nodes (m/s)", nodeMinSpeed_lvl2);
    cmd.AddValue("nodeMaxSpeed_lvl2", "Maximum speed for level 2 nodes (m/s)", nodeMaxSpeed_lvl2);

    cmd.Parse(argc, argv);

    nLevels = (int)nlevels;
    nClusters_1st_level = (int)nc1l;
    nNodes_pC_1st_level = (int)nn1l;
    nClusters_2nd_level = (int)nc2l;
    nNodes_pC_2nd_level = (int)nn2l;
    nClusters_3rd_level = (int)nc3l;
    nNodes_pC_3rd_level = (int)nn3l;

    firstLayerResources = std::vector<double>(
        resources, resources + nClusters_1st_level);
}

SimulationResult Taller1Experiment::Run()
{
    bool verbose = true;

    std::srand(std::time(nullptr));
    RngSeedManager::SetSeed(std::rand());

    if (verbose)
        std::cout << "Starting configuration..." << std::endl;

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    channel.AddPropagationLoss("ns3::FriisPropagationLossModel");
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

    YansWifiPhyHelper phy;
    phy.Set("TxPowerStart", DoubleValue(100.0));
    phy.Set("TxPowerEnd", DoubleValue(100.0));

    // Separate speed definitions for each level
    std::stringstream ssSpeedLvl1, ssSpeedLvl2;
    ssSpeedLvl1 << "ns3::UniformRandomVariable[Min=" << nodeMinSpeed_lvl1 << "|Max=" << nodeMaxSpeed_lvl1 << "]";
    ssSpeedLvl2 << "ns3::UniformRandomVariable[Min=" << nodeMinSpeed_lvl2 << "|Max=" << nodeMaxSpeed_lvl2 << "]";

    std::stringstream ssPause;
    double nodePause = 0.0;
    ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";

    std::string sSpeedLvl1 = ssSpeedLvl1.str();
    std::string sSpeedLvl2 = ssSpeedLvl2.str();
    std::string sPause = ssPause.str();

    OlsrHelper olsr;
    InternetStackHelper internet;
    internet.SetRoutingHelper(olsr);

    Ipv4AddressHelper ipAddrs1stLayer;
    ipAddrs1stLayer.SetBase("10.0.0.0", "255.255.255.0");

    Ipv4AddressHelper ipAddrs2ndLayer;
    ipAddrs2ndLayer.SetBase("192.168.0.0", "255.255.255.0");

    Ipv4AddressHelper ipAddrs3rdLayer;
    ipAddrs3rdLayer.SetBase("172.16.0.0", "255.255.255.0");

    Ipv4AddressHelper ipAddrs4thLayer;
    ipAddrs4thLayer.SetBase("172.17.0.0", "255.255.255.0");

    MobilityHelper mobilityAdhoc;

    Level first_level, second_level, third_level, fourth_level;

    if (verbose)
        std::cout << "Creating first level clusters..." << std::endl;

    for (int i = 0; i < nClusters_1st_level; i++)
    {
        if (verbose)
            std::cout << "[Lvl 1] Creating cluster #" << i << std::endl;

        Cluster cluster(i);
        cluster.generateNodes(nNodes_pC_1st_level);
        cluster.separateHead(0);

        WifiHelper nodesWifi;
        nodesWifi.SetRemoteStationManager("ns3::AarfWifiManager");
        phy.SetChannel(channel.Create());

        WifiMacHelper nodesMac;
        std::string ssidString("wifi-cluster-");
        std::stringstream ss;
        ss << i;
        ssidString += ss.str();

        Ssid ssid = Ssid(ssidString);

        nodesMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));

        NetDeviceContainer ns3DevicesExcludingHead = nodesWifi.Install(
            phy, nodesMac, cluster.ns3NodesExcludingHead);

        nodesMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));

        NetDeviceContainer headDevice = nodesWifi.Install(phy, nodesMac, cluster.headContainer);

        cluster.ns3Devices.Add(headDevice);
        cluster.ns3Devices.Add(ns3DevicesExcludingHead);

        internet.Install(cluster.ns3Nodes);

        Ipv4InterfaceContainer assignedAddresses = ipAddrs1stLayer.Assign(cluster.ns3Devices);

        ipAddrs1stLayer.NewNetwork();

        cluster.createClusterNodes(trafficRatio, firstLayerResources[i], probability);
        
        if (verbose)
            std::cout << "Resources in this cluster: " << cluster.getResources() << std::endl;

        first_level.clusters.push_back(cluster);
    }
    
    if (verbose)
        std::cout << "[Lvl 1] Finished clusters creation..." << std::endl;

    if (nLevels == 2)
    {
        nClusters_2nd_level = 1;
        nNodes_pC_2nd_level = nClusters_1st_level;
    }

    if (verbose)
        std::cout << "Creating second level clusters..." << std::endl;

    for (int i = 0; i < nClusters_2nd_level; i++)
    {
        Cluster cluster(i + nClusters_1st_level);

        NodeContainer nodes;

        for (int j = 0; j < nNodes_pC_2nd_level; j++)
        {
            nodes.Add(first_level.clusters[i * nNodes_pC_2nd_level + j].headContainer.Get(0));
        }

        cluster.setNodes(nodes);

        WifiHelper nodesWifi;
        nodesWifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                          "DataMode", StringValue(secondLayerResources));

        phy.SetChannel(channel.Create());

        WifiMacHelper nodesMac;
        nodesMac.SetType("ns3::AdhocWifiMac");

        cluster.ns3Devices = nodesWifi.Install(phy, nodesMac, cluster.ns3Nodes);

        Ipv4InterfaceContainer assignedAddresses = ipAddrs2ndLayer.Assign(cluster.ns3Devices);

        ObjectFactory pos;
        pos.SetTypeId("ns3::RandomRectanglePositionAllocator");

        std::stringstream ssMaxX;
        ssMaxX << "ns3::UniformRandomVariable[Min=0.0|Max=" << width << "]";
        pos.Set("X", StringValue(ssMaxX.str()));

        std::stringstream ssMaxY;
        ssMaxY << "ns3::UniformRandomVariable[Min=0.0|Max=" << height << "]";
        pos.Set("Y", StringValue(ssMaxY.str()));

        Ptr<PositionAllocator> taPositionAlloc = pos.Create()->GetObject<PositionAllocator>();

        // Apply level 2 speed to second level nodes
        mobilityAdhoc.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                                       "Speed", StringValue(sSpeedLvl2),
                                       "Pause", StringValue(sPause),
                                       "PositionAllocator", PointerValue(taPositionAlloc));
        mobilityAdhoc.SetPositionAllocator(taPositionAlloc);

        mobilityAdhoc.Install(cluster.ns3Nodes);

        ipAddrs2ndLayer.NewNetwork();

        second_level.clusters.push_back(cluster);
    }

    if (verbose)
        std::cout << "[Lvl 2] Finished clusters creation..." << std::endl;

    if (verbose)
        std::cout << "[Lvl 1] Placing mobility models..." << std::endl;

    for (int i = 0; i < nClusters_1st_level; i++)
    {
        Cluster cluster = first_level.clusters[i];

        Ptr<ListPositionAllocator> subnetAlloc = CreateObject<ListPositionAllocator>();
        for (uint8_t j = 0; j < nNodes_pC_1st_level; j++)
        {
            subnetAlloc->Add(Vector(0.0, j, 0.0));
        }

        mobilityAdhoc.PushReferenceMobilityModel(cluster.headContainer.Get(0));
        mobilityAdhoc.SetPositionAllocator(subnetAlloc);
        
        // Apply level 1 speed to first level nodes
        mobilityAdhoc.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
                                       "Bounds", RectangleValue(Rectangle(-10, 10, -10, 10)),
                                       "Speed", StringValue(sSpeedLvl1),
                                       "Pause", StringValue(sPause));
        mobilityAdhoc.Install(cluster.ns3Nodes);
    }

    if (nLevels > 2)
    {
        if (nLevels == 3)
        {
            nClusters_3rd_level = 1;
            nNodes_pC_3rd_level = nClusters_2nd_level;
        }

        if (verbose)
            std::cout << "Creating third level clusters..." << std::endl;

        for (int i = 0; i < nClusters_3rd_level; i++)
        {
            if (verbose)
                std::cout << "[Lvl 3] Creating cluster #" << i << std::endl;

            Cluster cluster(i + nClusters_1st_level + nClusters_2nd_level);

            NodeContainer nodes;

            for (int j = 0; j < nNodes_pC_3rd_level; j++)
            {
                nodes.Add(first_level.clusters[i * nNodes_pC_3rd_level * nNodes_pC_2nd_level].ns3Nodes.Get(1));
            }

            cluster.setNodes(nodes);

            WifiHelper nodesWifi;
            nodesWifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                              "DataMode", StringValue("OfdmRate54Mbps"));

            phy.SetChannel(channel.Create());

            WifiMacHelper nodesMac;
            nodesMac.SetType("ns3::AdhocWifiMac");

            cluster.ns3Devices = nodesWifi.Install(phy, nodesMac, cluster.ns3Nodes);

            Ipv4InterfaceContainer assignedAddresses = ipAddrs3rdLayer.Assign(cluster.ns3Devices);

            ipAddrs3rdLayer.NewNetwork();

            third_level.clusters.push_back(cluster);
        }

        if (verbose)
            std::cout << "[Lvl 3] Finished clusters creation..." << std::endl;

        if (nLevels > 3)
        {
            if (verbose)
                std::cout << "Creating fourth level cluster..." << std::endl;

            if (verbose)
                std::cout << "[Lvl 4] Creating cluster #0" << std::endl;

            Cluster cluster(nClusters_1st_level + nClusters_2nd_level + nClusters_3rd_level);

            NodeContainer nodes;

            for (int j = 0; j < nClusters_3rd_level; j++)
            {
                nodes.Add(first_level.clusters[j * nNodes_pC_2nd_level * nNodes_pC_3rd_level].ns3Nodes.Get(2));
            }

            cluster.setNodes(nodes);

            WifiHelper nodesWifi;
            nodesWifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                              "DataMode", StringValue("OfdmRate54Mbps"));

            phy.SetChannel(channel.Create());

            WifiMacHelper nodesMac;
            nodesMac.SetType("ns3::AdhocWifiMac");

            cluster.ns3Devices = nodesWifi.Install(phy, nodesMac, cluster.ns3Nodes);

            Ipv4InterfaceContainer assignedAddresses = ipAddrs4thLayer.Assign(cluster.ns3Devices);

            fourth_level.clusters.push_back(cluster);

            if (verbose)
                std::cout << "[Lvl 4] Finished clusters creation..." << std::endl;
        }
    }

    if (verbose)
        std::cout << "Preparing random traffic for simulation..." << std::endl;

    int k = 20;
    for (int i = 0; i < k; i++)
    {
        int senderNodeIndex = rand() % nNodes_pC_1st_level;
        int receiverNodeIndex = rand() % nNodes_pC_1st_level;

        int senderClusterIndex = rand() % nClusters_1st_level;
        int receiverClusterIndex = rand() % nClusters_1st_level;

        while (senderNodeIndex == receiverNodeIndex && receiverClusterIndex == senderClusterIndex)
        {
            receiverNodeIndex = rand() % nNodes_pC_1st_level;
            receiverClusterIndex = rand() % nClusters_1st_level;
        }

        if (verbose)
            std::cout << "Connecting IP Address: "
                      << first_level.clusters[senderClusterIndex].nodes[senderNodeIndex].node->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal()
                      << " with IP Address: "
                      << first_level.clusters[receiverClusterIndex].nodes[receiverNodeIndex].node->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal()
                      << std::endl;

        ClusterNode senderNode = first_level.clusters[senderClusterIndex].nodes[senderNodeIndex];
        ClusterNode receiverNode = first_level.clusters[receiverClusterIndex].nodes[receiverNodeIndex];
        senderNode.connectWithNode(receiverNode, this);
    }

    if (verbose)
        std::cout << "Running simulation..." << std::endl;

    Simulator::Stop(Seconds(simulationTime));
    Simulator::Run();

    std::cout << "Simulation finished" << std::endl;
    std::cout << "Level of resources in first layer: " << first_level.getResources() << std::endl;

    std::cout << "Total packets received: " << receivedCount << std::endl;
    std::cout << "Total packets sent: " << sentCount << std::endl;
    
    double throughput = receivedCount / Simulator::Now().GetSeconds();
    double lossRate = (sentCount - receivedCount) / (double)sentCount;
    
    // Calculate and report growth factor j
    double u1_avg = (nodeMinSpeed_lvl1 + nodeMaxSpeed_lvl1) / 2.0;
    double u2_avg = (nodeMinSpeed_lvl2 + nodeMaxSpeed_lvl2) / 2.0;
    double j = u2_avg / u1_avg;
    
    std::cout << "Average speed Level 1: " << u1_avg << " m/s" << std::endl;
    std::cout << "Average speed Level 2: " << u2_avg << " m/s" << std::endl;
    std::cout << "Growth factor j (u2/u1): " << j << std::endl;

    SimulationResult results;
    results.throughput = throughput;
    results.lossRate = lossRate;

    Simulator::Destroy();

    return results;
}

int testPhyRatio(int argc, char *argv[])
{
    int ncases = 50;
    for (int i = 0; i < ncases; i++)
    {
        Taller1Experiment experiment;

        double resourcesForClusters[experiment.nClusters_1st_level];

        double minResourceValue = 500000;
        double maxResourceValue = 1200000;

        for (int j = 0; j < experiment.nClusters_1st_level; j++)
        {
            resourcesForClusters[j] = ((double)rand() / (RAND_MAX)) *
                                          (maxResourceValue - minResourceValue) +
                                      minResourceValue;
        }

        experiment.HandleCommandLineArgs(argc, argv, resourcesForClusters);

        std::cout << "Case " << i << std::endl;
        SimulationResult experimentResult = experiment.Run();
        std::cout << "Resources: " << std::endl;

        for (int i = 0; i < (int)experiment.firstLayerResources.size(); i++)
        {
            std::cout << experiment.firstLayerResources[i] << " ";
        }
        std::cout << std::endl;
        std::cout << "Throughput: " << experimentResult.throughput << " Pkt/s" << std::endl;
        std::cout << "Loss rate: " << experimentResult.lossRate << std::endl;
    }

    return 0;
}

int main(int argc, char **argv)
{
    Taller1Experiment experiment;

    double resourcesForClusters[experiment.nClusters_1st_level];

    double minResourceValue = 500000;
    double maxResourceValue = 1200000;

    for (int j = 0; j < experiment.nClusters_1st_level; j++)
    {
        resourcesForClusters[j] = ((double)rand() / (RAND_MAX)) *
                                      (maxResourceValue - minResourceValue) +
                                  minResourceValue;
    }

    experiment.HandleCommandLineArgs(argc, argv, resourcesForClusters);

    SimulationResult experimentResult = experiment.Run();
    std::cout << "Resources: " << std::endl;

    for (int i = 0; i < (int)experiment.firstLayerResources.size(); i++)
    {
        std::cout << experiment.firstLayerResources[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "Throughput: " << experimentResult.throughput << " Pkt/s" << std::endl;
    std::cout << "Loss rate: " << experimentResult.lossRate << std::endl;
    
    return 0;
}