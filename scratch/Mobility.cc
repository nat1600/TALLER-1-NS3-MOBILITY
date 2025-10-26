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

    

    // Second layer resources
    // Note they aren't calculated with OnOffModel
    // Its just the datarate value for shared wifi channel
    // Note also this is represented by a String which match a
    // well-known datarate value denotated as a string

    // Valueas which can be used:
    // OfdmRate6Mbps
    // OfdmRate9Mbps
    // OfdmRate12Mbps
    // OfdmRate18Mbps
    // OfdmRate24Mbps
    // OfdmRate1_5MbpsBW5MHz
    // OfdmRate2_25MbpsBW5MHz
    // OfdmRate3MbpsBW5MHz
    // OfdmRate4_5MbpsBW5MHz
    // OfdmRate6MbpsBW5MHz
    // OfdmRate9MbpsBW5MHz
    // OfdmRate12MbpsBW5MHz
    // OfdmRate13_5MbpsBW5MHz
    // OfdmRate3MbpsBW10MHz
    // OfdmRate4_5MbpsBW10MHz
    // OfdmRate6MbpsBW10MHz
    // OfdmRate9MbpsBW10MHz
    // OfdmRate12MbpsBW10MHz
    // OfdmRate18MbpsBW10MHz
    // OfdmRate24MbpsBW10MHz
    // OfdmRate27MbpsBW10MHz

    // In case we want to go further:
    // OfdmRate36Mbps
    // OfdmRate48Mbps
    // OfdmRate54Mbps
    std::string secondLayerResources = "OfdmRate48Mbps";

    // Resources array for first layer's clusters
    // Note this parameter can't be passed through console :P
    std::vector<double> firstLayerResources;

    // Probability
    // We can of course set each cluster a different probability, but firstly
    // we will assume they all have the same probability for truncated geometric distribution
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

    // Finally, the resources on this node are calculated with the following formula
    // resources = DataRate * trafficRatio
    // We will say trafficRatio will be a constant passed as argument for this class
    // dataRate will be calculated then since we can have a specific number of resources

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

    // On packet receive
    void ReceivePacket(Ptr<Socket> socket);

    // On packet sent
    void OnPacketSent(Ptr<const Packet> packet);

    // Configuration as node receiver
    void configureAsReceiver(Taller1Experiment *);
};

// Collection of nodes with a head
class Cluster
{
public:
    // All nodes within this cluster
    NodeContainer ns3Nodes;

    // All nodes excluding head
    NodeContainer ns3NodesExcludingHead;

    // Container for head
    NodeContainer headContainer;

    // All devices (physic interfaces within this cluster)
    NetDeviceContainer ns3Devices;

    // Cluster Index
    int index;

    // Save an array of clusterNodes

    // Created only for first level clusters, then tested with different values by higher levels
    std::vector<ClusterNode> nodes;

    // Default constructor
    Cluster(int);

    // Separate cluster's head
    // This is an utility for first layer only, further layers won't need this
    void separateHead(uint32_t);

    // Create cluster nodes having needed data
    void createClusterNodes(
        double, double, double);

    // First layer will generate all nodes
    void generateNodes(int);

    // Other layers will take nodes only
    void setNodes(NodeContainer);

    // Calculate resources
    double getResources();
};

// Class to represent a cluster of clusters
class Level
{
public:
    std::vector<Cluster> clusters;

    // Default constructor
    Level() {}

    // Calculate expected value of resources
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
    // Always must be passed as argument
    node = _node;
    index = _index;
    trafficRatio = _trafficRatio;

    if (includesResources)
    {
        // arg2 represents resources
        dataRate = arg2 / trafficRatio;
    }
    else
    {
        // arg2 represents dataRate directly
        dataRate = arg2;
    }
}

double ClusterNode::getResources()
{
    return trafficRatio * dataRate;
}

// Configure random packet sending
ApplicationContainer ClusterNode::connectWithNode(ClusterNode &receiver, Taller1Experiment *_parent)
{
    // Firstly, update parent
    parent = _parent;

    // Configure sender node
    OnOffHelper onoff("ns3::UdpSocketFactory", Address());

    // According to:
    // https://revistas.udistrital.edu.co/index.php/Tecnura/article/view/6754/8337
    // The correct portion of time a node is sending data is calculated as:
    // P = (u_on / (u_on + u_off)), where u_on and u_off represent the portion of time
    // the node is sending data and the portion of time the node is not sending data
    // respectively. Both are distributed exponentially
    // Configure OnOff properties

    std::stringstream ssOffTime;
    ssOffTime << "ns3::ExponentialRandomVariable[Mean="
              << parent->meanOffTime
              << "]";

    // Lets set OffTIme as always 1.0 to simplify calculations
    onoff.SetAttribute("OffTime", StringValue(ssOffTime.str()));

    // trafficRatio should be the expected probability
    // of a node being on
    // So, we can calculate the mean of the exponential distribution

    std::stringstream ss;
    ss << "ns3::ExponentialRandomVariable[Mean="
       << (trafficRatio * 0.1 / (1 - trafficRatio)) // This is A_y_i
       << "]";
    onoff.SetAttribute("OnTime", StringValue(ss.str()));

    // Set onoff rate
    // Both Data rate and off time are components of resources
    // Configure data rate (bps)
    onoff.SetAttribute("DataRate", DataRateValue(DataRate(dataRate)));

    // // Configure receiver node
    Ptr<Node> receiverNs3Node = receiver.node;

    // // Configure packet size
    uint32_t pktSize = 1024;
    onoff.SetAttribute("PacketSize", UintegerValue(pktSize));

    // Note that head nodes have their "external" address assignated first
    // So this packet will be sent there on that case
    Ipv4Address remoteAddr = receiverNs3Node->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();

    // Configure sender node
    AddressValue remoteAddress(InetSocketAddress(remoteAddr, parent->port));
    onoff.SetAttribute("Remote", remoteAddress);

    ApplicationContainer sendApp = onoff.Install(node);
    sendApp.Start(Seconds(0.0));
    sendApp.Stop(Seconds(parent->simulationTime));

    receiver.configureAsReceiver(parent);

    // Check if current node hasn't been configured as a sender node yet
    if (!configuredAsSender)
    {
        // Configure packet sink tracker
        std::string path = "/NodeList/" + std::to_string(node->GetId()) + "/ApplicationList/*/$ns3::OnOffApplication/Tx";
        Config::ConnectWithoutContext(path, MakeCallback(&ClusterNode::OnPacketSent, this));
    }
    else
    {
        // Otherwise sent packets would be counted twice!
        configuredAsSender = true;
    }

    return sendApp;
}

// Configure node as receiver
void ClusterNode::configureAsReceiver(Taller1Experiment *_parent)
{
    if (configuredAsReceiver)
        return;

    parent = _parent;

    // Configure packet sink tracker
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    Ptr<Socket> recvSink = Socket::CreateSocket(node, tid);

    Ipv4Address remoteAddr = node->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();

    InetSocketAddress local = InetSocketAddress(remoteAddr, parent->port);
    recvSink->Bind(local);
    recvSink->SetRecvCallback(MakeCallback(&ClusterNode::ReceivePacket, this));

    configuredAsReceiver = true;
}

// Callback for packet sent BY node
void ClusterNode::OnPacketSent(Ptr<const Packet> packet)
{
    parent->sentCount++; // Propagate callback to parent
}

// Callback for packet received BY node
void ClusterNode::ReceivePacket(Ptr<Socket> socket)
{
    while (socket->GetRxAvailable() > 0)
    {
        // Multiple packets could have reached, they must be "read" by means of Recv()
        socket->Recv();
        parent->receivedCount++; // Propagate callback to parent
    }
}

// Create nodes contaner with specified number of nodes
Cluster::Cluster(int _index)
{
    // Save cluster index, useful for some tricks like SSID assignation
    index = _index;
}

// Generate nodes when there are not nodes available
void Cluster::generateNodes(int nNodes)
{
    // Create nodes
    ns3Nodes.Create(nNodes);
}

// Save data from existent nodes
void Cluster::setNodes(NodeContainer _nodes)
{
    ns3Nodes = _nodes;
}

// Create cluster nodes (here we will save some useful data, like node's resources)
void Cluster::createClusterNodes(double trafficRatio, double totalResouces, double probability)
{
    int length = ns3Nodes.GetN();

    for (int j = 0; j < length; j++)
    {
        // Get resources for node
        double nodeResources = TruncatedDistribution(
            length, totalResouces, probability, j);

        nodes.push_back(ClusterNode(j, true, trafficRatio, nodeResources, ns3Nodes.Get(j)));
    }
}

// Set cluster head (comes from level container)
void Cluster::separateHead(uint32_t headIndex)
{
    Ptr<Node> head = ns3Nodes.Get(headIndex);

    // Refactor containers to have consistency
    for (uint32_t i = 0; i < ns3Nodes.GetN(); i++)
    {
        if (i == headIndex)
            continue;

        ns3NodesExcludingHead.Add(ns3Nodes.Get(i));
    }

    // Update head-only container
    headContainer.Add(head);
}

// Calculate expected resources for this cluster
double Cluster::getResources()
{
    double resources = 0;

    for (ClusterNode node : nodes)
    {
        resources += node.getResources();
    }

    return resources;
}

// Calculate level resources
double Level::getResources()
{
    double resources = 0;

    for (Cluster cluster : clusters)
    {
        resources += cluster.getResources();
    }

    return resources;
}

// Truncated distribution assigner
double
TruncatedDistribution(
    int nPoints, double totalResources, double probability, int nodeIndex)
{
    // Here joins probability density function for truncated geometric distribution
    // Portion of total resources this node will take

    double portion = probability * pow(1 - probability, nodeIndex) / (1 - pow(1 - probability, nPoints));
    // Return resources to assign to node
    return portion * totalResources;
}

// Default constructor
Taller1Experiment::Taller1Experiment()
    // Default port to 9
    : port(9),
      // Default number of levels to 2
      nLevels(2),
      // Default number of clusters in 1st level to 6
      nClusters_1st_level(6),
      // Default number of nodes per cluster in 1st level to 6
      nNodes_pC_1st_level(6),
      // Default number of clusters in 2nd level to 2
      nClusters_2nd_level(1),
      // Default number of nodes per cluster in 2nd level to 2
      nNodes_pC_2nd_level(6),
      // Default number of clusters in 3rd level to 1
      nClusters_3rd_level(1),
      // Default number of nodes per cluster in 3rd level to 2
      nNodes_pC_3rd_level(1),
      // Default width to 500
      width(100),
      // Default height to 500
      height(100)
{
}

// Receive and set command line arguments
void Taller1Experiment::HandleCommandLineArgs(int argc, char **argv, double resources[])
{
    /*
     * Get console parameters
     */
    CommandLine cmd(__FILE__);

    // Solving a bug which doesn't allow you to read integers from command line
    double nc1l = nClusters_1st_level;
    double nn1l = nNodes_pC_1st_level;
    double nc2l = nClusters_2nd_level;
    double nn2l = nNodes_pC_2nd_level;
    double nc3l = nClusters_3rd_level;
    double nn3l = nNodes_pC_3rd_level;
    double nlevels = nLevels;

    // Number of hierarchy levels
    cmd.AddValue("nLevels", "Number of levels of this cluster", nlevels);

    // Data for first level
    cmd.AddValue("nClusters_1st_level", "Number of clusters in 1st level", nc1l);
    cmd.AddValue("nNodes_pC_1st_level", "Number of nodes per cluster in 1st level", nn1l);

    // Data for second level
    cmd.AddValue("nClusters_2nd_level", "Number of clusters in 2nd level", nc2l);
    cmd.AddValue("nNodes_pC_2nd_level", "Number of nodes per cluster in 2nd level", nn2l);

    // Data for third level
    cmd.AddValue("nClusters_3rd_level", "Number of clusters in 3rd level", nc3l);
    cmd.AddValue("nNodes_pC_3rd_level", "Number of nodes per cluster in 3rd level", nn3l);

    // Second level resources
    cmd.AddValue("secondLayerResources", "Resources for second layer", secondLayerResources);

    // Traffic ratio and mean offtime params
    cmd.AddValue("trafficRatio", "Traffic ratio per node", trafficRatio);
    cmd.AddValue("meanOffTime", "Mean offtime per node", meanOffTime);

    // Space bounds
    cmd.AddValue("width", "Width of the space", width);
    cmd.AddValue("height", "Height of the space", height);

    // Simulation time
    cmd.AddValue("simulationTime", "Simulation time in seconds", simulationTime);

    // Parse arguments
    cmd.Parse(argc, argv);

    // Set values to class attributes
    nLevels = (int)nlevels;
    nClusters_1st_level = (int)nc1l;
    nNodes_pC_1st_level = (int)nn1l;
    nClusters_2nd_level = (int)nc2l;
    nNodes_pC_2nd_level = (int)nn2l;
    nClusters_3rd_level = (int)nc3l;
    nNodes_pC_3rd_level = (int)nn3l;

    // Set further arguments

    // Set values to vector of resources on First Layer
    // Its size should match the number of first layer cluster

    firstLayerResources = std::vector<double>(
        resources, resources + nClusters_1st_level);
}

SimulationResult Taller1Experiment::Run()
{
    bool verbose = true;

    // Randomize
    std::srand(std::time(nullptr));
    RngSeedManager::SetSeed(std::rand());

    if (verbose)
        std::cout << "Starting configuration..." << std::endl;

    //
    // Configure physical layer
    //

    // Wifi channel
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();

    // Using friss propagation loss model
    // It considers variables such as waves distortion due to obstacles, diffraction and related phenomena
    channel.AddPropagationLoss(
        "ns3::FriisPropagationLossModel");

    // Use constant speed propagation delay model
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

    // Configure transmission channel
    YansWifiPhyHelper phy;

    phy.Set("TxPowerStart", DoubleValue(100.0));
    phy.Set("TxPowerEnd", DoubleValue(100.0));

    // Define speed (Which is distributed uniformly between 0 and 1 (units are m/s))
    double nodeMinSpeed = 0.0, nodeMaxSpeed = 1.0;
    std::stringstream ssSpeed;
    ssSpeed << "ns3::UniformRandomVariable[Min=" << nodeMinSpeed << "|Max=" << nodeMaxSpeed << "]";

    // Pause refers to the time a node waits before changing direction
    // (Node remains static while this time passes)
    std::stringstream ssPause;
    double nodePause = 0.0;
    ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";

    // Configure mobility model

    // Firstly, get speed and pause as strings:
    std::string sSpeed = ssSpeed.str();
    std::string sPause = ssPause.str();

    //
    // Configure network stack
    //

    // Enable OLSR
    OlsrHelper olsr;

    // Install network stack
    InternetStackHelper internet;
    internet.SetRoutingHelper(olsr); // has effect on the next Install ()

    // Assign IPv4 addresses (First layer)
    Ipv4AddressHelper ipAddrs1stLayer;
    ipAddrs1stLayer.SetBase("10.0.0.0", "255.255.255.0");

    // Assign IPv4 addresses (Second layer)
    Ipv4AddressHelper ipAddrs2ndLayer;
    ipAddrs2ndLayer.SetBase("192.168.0.0", "255.255.255.0");

    // Assign IPv4 addresses (Third layer)
    Ipv4AddressHelper ipAddrs3rdLayer;
    ipAddrs3rdLayer.SetBase("172.16.0.0", "255.255.255.0");

    // Assign IPv4 addresses (Fourth layer)
    Ipv4AddressHelper ipAddrs4thLayer;
    ipAddrs3rdLayer.SetBase("172.17.0.0", "255.255.255.0");

    // Mobility helper
    MobilityHelper mobilityAdhoc;

    // We are now able to create nodes

    // Initialize all levels
    Level first_level, second_level, third_level, fourth_level;

    if (verbose)
        std::cout << "Creating first level clusters..." << std::endl;

    // Always create nodes for the first level (note actually all nodes instances will be created here)
    // Create nodes for each cluster in first level
    for (int i = 0; i < nClusters_1st_level; i++)
    {
        if (verbose)
            std::cout << "[Lvl 1] Creating cluster #" << i << std::endl;
        //  Create cluster
        Cluster cluster(i);

        // Since we are in the first layer, create nodes
        cluster.generateNodes(nNodes_pC_1st_level);

        // Group nodes by defining head
        cluster.separateHead(0); // Note node #0 is the one with highest resources

        // Physical layer
        WifiHelper nodesWifi;
        nodesWifi.SetRemoteStationManager("ns3::AarfWifiManager");

        // phy.Set("ChannelNumber", UintegerValue(1));
        phy.SetChannel(channel.Create());

        // Data link layer
        WifiMacHelper nodesMac;

        // Each subnetwork (cluster) will be identified by a different SSID
        std::string ssidString("wifi-cluster-");
        std::stringstream ss;
        ss << i; // Each SSID has the format: wifi-cluster-i
        ssidString += ss.str();

        Ssid ssid = Ssid(ssidString);

        nodesMac.SetType("ns3::StaWifiMac",
                         "Ssid", SsidValue(ssid));

        // Nodes will connect to their cluster head (which is actually an AP on this case)
        NetDeviceContainer ns3DevicesExcludingHead = nodesWifi.Install(
            phy, nodesMac, cluster.ns3NodesExcludingHead);

        // Setup heads as APs
        nodesMac.SetType("ns3::ApWifiMac",
                         "Ssid", SsidValue(ssid));

        NetDeviceContainer headDevice = nodesWifi.Install(phy, nodesMac, cluster.headContainer);

        // Total cluster devices
        cluster.ns3Devices.Add(headDevice);
        cluster.ns3Devices.Add(ns3DevicesExcludingHead);

        // All nodes are including in OLSR protocol
        internet.Install(cluster.ns3Nodes);

        // It is kinda useful to save interfaces for future connections
        Ipv4InterfaceContainer assignedAddresses = ipAddrs1stLayer.Assign(
            cluster.ns3Devices);

        // Step next subnet
        ipAddrs1stLayer.NewNetwork();

        // Create nodes
        cluster.createClusterNodes(
            trafficRatio,
            firstLayerResources[i],
            probability);
        if (verbose)
            std::cout << "Resources in this cluster: " << cluster.getResources() << std::endl;

        // Add this cluster to first level clusters
        first_level.clusters.push_back(cluster);

        // Mobility will be set later after configuring heads mobility
    }
    if (verbose)
        std::cout << "[Lvl 1] Finished clusters creation..." << std::endl;

    if (nLevels == 2)
    {
        // In a two layer architecture, there is actually one single cluster in second level
        // And its nodes are sublayer's clusters heads
        nClusters_2nd_level = 1;
        nNodes_pC_2nd_level = nClusters_1st_level;
    }

    if (verbose)
        std::cout << "Creating second level clusters..." << std::endl;

    // Get nodes for second cluster (Heads on first level)
    for (int i = 0; i < nClusters_2nd_level; i++)
    {
        // std::cout << "[Lvl 2] Creating cluster #" << i << std::endl;
        //  Create cluster
        Cluster cluster(i + nClusters_1st_level);

        NodeContainer nodes;

        // Get nodes for this cluster, they were created in previous step
        for (int j = 0; j < nNodes_pC_2nd_level; j++)
        {
            // Add head as an element from cluster
            nodes.Add(first_level.clusters[i * nNodes_pC_2nd_level + j].headContainer.Get(0));
        }

        // Set nodes in cluster
        cluster.setNodes(nodes);

        // Physical layer
        WifiHelper nodesWifi;
        nodesWifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                          "DataMode", StringValue(secondLayerResources));

        phy.SetChannel(channel.Create());

        //
        // Configure data link layer
        //
        WifiMacHelper nodesMac;
        nodesMac.SetType("ns3::AdhocWifiMac");

        // Create physical interfaces between 2nd layer nodes
        cluster.ns3Devices = nodesWifi.Install(
            phy, nodesMac, cluster.ns3Nodes);

        // Note internet stack is already installed on nodes
        Ipv4InterfaceContainer assignedAddresses = ipAddrs2ndLayer.Assign(
            cluster.ns3Devices);

        // Create position allocator, random at start
        ObjectFactory pos;
        pos.SetTypeId("ns3::RandomRectanglePositionAllocator");

        // Define boundaries for our area (By default 500x500), units are meters
        std::stringstream ssMaxX;
        ssMaxX << "ns3::UniformRandomVariable[Min=0.0|Max=" << width << "]";
        pos.Set("X", StringValue(ssMaxX.str()));

        std::stringstream ssMaxY;
        ssMaxY << "ns3::UniformRandomVariable[Min=0.0|Max=" << height << "]";
        pos.Set("Y", StringValue(ssMaxY.str()));

        // Create position allocators based on geometrical boundaries already defined
        // int64_t streamIndex = 0; // used to get consistent mobility across scenarios
        Ptr<PositionAllocator> taPositionAlloc = pos.Create()->GetObject<PositionAllocator>();

        // Set random way mobility on head nodes
        mobilityAdhoc.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                                       "Speed", StringValue(sSpeed),
                                       "Pause", StringValue(sPause),
                                       "PositionAllocator", PointerValue(taPositionAlloc));
        mobilityAdhoc.SetPositionAllocator(taPositionAlloc);

        // Remind that on fitst layer we didn't configured mobility for heads
        mobilityAdhoc.Install(cluster.ns3Nodes);

        // Step next subnet
        ipAddrs2ndLayer.NewNetwork();

        // Add this cluster to second level clusters
        second_level.clusters.push_back(cluster);
    }

    if (verbose)
        std::cout << "[Lvl 2] Finished clusters creation..." << std::endl;

    // Now set mobility for lvl 1 nodes
    if (verbose)
        std::cout << "[Lvl 1] Placing mobility models..." << std::endl;

    for (int i = 0; i < nClusters_1st_level; i++)
    {
        Cluster cluster = first_level.clusters[i];

        // Configure mobility model, nodes will follow head within a certain rectangle movement
        // We consider cleaner to use a simplier model for internal nodes movement within a head
        // also its even easier to manage movement bounds
        Ptr<ListPositionAllocator> subnetAlloc =
            CreateObject<ListPositionAllocator>();
        for (uint8_t j = 0; j < nNodes_pC_1st_level; j++)
        {
            subnetAlloc->Add(Vector(0.0, j, 0.0));
        }

        // Nodes move around cluster's head
        mobilityAdhoc.PushReferenceMobilityModel(cluster.headContainer.Get(0));
        mobilityAdhoc.SetPositionAllocator(subnetAlloc);
        mobilityAdhoc.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
                                       "Bounds", RectangleValue(Rectangle(-10, 10, -10, 10)),
                                       "Speed", StringValue(sSpeed),
                                       "Pause", StringValue(sPause));
        mobilityAdhoc.Install(cluster.ns3Nodes);
    }

    // Config third layer only if needed
    if (nLevels > 2)
    {
        if (nLevels == 3)
        {
            // In a three layer architecture, there is actually one single cluster in third level
            // And its nodes are sublayer's clusters heads
            nClusters_3rd_level = 1;
            nNodes_pC_3rd_level = nClusters_2nd_level;
        }

        if (verbose)
            std::cout << "Creating third level clusters..." << std::endl;

        for (int i = 0; i < nClusters_3rd_level; i++)
        {
            if (verbose)
                std::cout << "[Lvl 3] Creating cluster #" << i << std::endl;

            // Create cluster
            Cluster cluster(i + nClusters_1st_level + nClusters_2nd_level);

            NodeContainer nodes;

            // Get nodes for this cluster, they were created in previous step
            for (int j = 0; j < nNodes_pC_3rd_level; j++)
            {
                // Add head as an element for cluster
                nodes.Add(
                    first_level.clusters[i * nNodes_pC_3rd_level * nNodes_pC_2nd_level]
                        .ns3Nodes.Get(1));
            }

            // Set nodes in cluster
            cluster.setNodes(nodes);

            // Physical layer
            WifiHelper nodesWifi;
            nodesWifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                              "DataMode", StringValue("OfdmRate54Mbps"));

            phy.SetChannel(channel.Create());

            //
            // Configure data link layer
            //
            WifiMacHelper nodesMac;
            nodesMac.SetType("ns3::AdhocWifiMac");

            // Create physical interfaces between 2nd layer nodes
            cluster.ns3Devices = nodesWifi.Install(
                phy, nodesMac, cluster.ns3Nodes);

            // Note internet stack is already installed on nodes
            Ipv4InterfaceContainer assignedAddresses = ipAddrs3rdLayer.Assign(
                cluster.ns3Devices);

            // Mobility model is already setted for heads

            // Step next subnet
            ipAddrs3rdLayer.NewNetwork();

            // Add this cluster to second level clusters
            third_level.clusters.push_back(cluster);
        }

        if (verbose)
            std::cout << "[Lvl 3] Finished clusters creation..." << std::endl;

        if (nLevels > 3)
        {
            // Finally, there is a maximum of four levels in case third layer
            // has multiple clusters

            if (verbose)
                std::cout << "Creating fourth level cluster..." << std::endl;

            if (verbose)
                std::cout << "[Lvl 4] Creating cluster #0" << std::endl;

            // Create cluster
            Cluster cluster(nClusters_1st_level + nClusters_2nd_level + nClusters_3rd_level);

            NodeContainer nodes;

            // Get nodes for this cluster, they were created in previous steps
            for (int j = 0; j < nClusters_3rd_level; j++)
            {
                // Add head as an element for cluster
                nodes.Add(
                    first_level.clusters[j * nNodes_pC_2nd_level * nNodes_pC_3rd_level]
                        .ns3Nodes.Get(2));
            }

            // Set nodes in cluster
            cluster.setNodes(nodes);

            // Physical layer
            WifiHelper nodesWifi;
            nodesWifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                              "DataMode", StringValue("OfdmRate54Mbps"));

            phy.SetChannel(channel.Create());

            //
            // Configure data link layer
            //
            WifiMacHelper nodesMac;
            nodesMac.SetType("ns3::AdhocWifiMac");

            // Create physical interfaces between 2nd layer nodes
            cluster.ns3Devices = nodesWifi.Install(
                phy, nodesMac, cluster.ns3Nodes);

            // Note internet stack is already installed on nodes
            Ipv4InterfaceContainer assignedAddresses = ipAddrs4thLayer.Assign(
                cluster.ns3Devices);

            // Mobility model is already setted for heads

            // Add this cluster to second level clusters
            fourth_level.clusters.push_back(cluster);

            if (verbose)
                std::cout << "[Lvl 4] Finished clusters creation..." << std::endl;
        }
    }

    // Preparate nodes for simulation
    if (verbose)
        std::cout << "Preparing random traffic for simulation..." << std::endl;

    // Make k random connections between nodes in first level
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

    // Run simulation
    Simulator::Stop(Seconds(simulationTime));
    Simulator::Run();

    std::cout << "Simulation finished" << std::endl;
    std::cout << "Level of resources in first layer: " << first_level.getResources() << std::endl;

    // Show performance results
    std::cout << "Total packets received: " << receivedCount << std::endl;
    std::cout << "Total packets sent: " << sentCount << std::endl;
    double throughput = receivedCount / Simulator::Now().GetSeconds(); // Pkt / s
    double lossRate = (sentCount - receivedCount) / (double)sentCount;

    // Generate simulation results data container
    SimulationResult results;
    results.throughput = throughput;
    results.lossRate = lossRate;

    Simulator::Destroy();

    return results;
}

// Useful for resources testing
int testPhyRatio(int argc, char *argv[])
{
    // Time::SetResolution(Time::US);
    int ncases = 50;
    // Create experiment
    for (int i = 0; i < ncases; i++)
    {
        Taller1Experiment experiment;

        double resourcesForClusters[experiment.nClusters_1st_level];

        // Set minimum resource value
        double minResourceValue = 500000;
        double maxResourceValue = 1200000;

        // Generate random resources for clusters
        for (int j = 0; j < experiment.nClusters_1st_level; j++)
        {
            resourcesForClusters[j] = ((double)rand() / (RAND_MAX)) *
                                          (maxResourceValue - minResourceValue) +
                                      minResourceValue;
        }

        // Receive command line args
        experiment.HandleCommandLineArgs(argc, argv, resourcesForClusters);

        // Run experiment
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

    // Set minimum resource value
    double minResourceValue = 500000;
    double maxResourceValue = 1200000;

    // Generate random resources for clusters
    for (int j = 0; j < experiment.nClusters_1st_level; j++)
    {
        resourcesForClusters[j] = ((double)rand() / (RAND_MAX)) *
                                      (maxResourceValue - minResourceValue) +
                                  minResourceValue;
    }

    // Receive command line args
    experiment.HandleCommandLineArgs(argc, argv, resourcesForClusters);

    // Run experiment
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