/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Análisis de escalabilidad en redes MANET jerárquicas de dos niveles
 * Investiga cómo la velocidad de movilidad de clústeres afecta el factor de crecimiento (j)
 */

#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/log.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/wifi-net-device.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/aodv-helper.h"
#include "ns3/olsr-helper.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/random-walk-2d-mobility-model.h"
#include "ns3/random-waypoint-mobility-model.h"
#include "ns3/position-allocator.h"
#include <fstream>
#include <vector>
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("HierarchicalMANET");

// Estructura de parámetros para la simulación
struct Parameters {
    std::string testName;
    
    // Parámetros jerárquicos
    uint32_t numClusters;              // Número de clústeres (nivel 1)
    uint32_t nodesPerCluster;          // Nodos por clúster (nivel 2)
    double growthFactor;               // Factor j de crecimiento
    
    // Parámetros de movilidad
    double clusterMobilitySpeed;       // Velocidad de clústeres (m/s)
    double nodeMobilitySpeed;          // Velocidad intra-clúster (m/s)
    double clusterRadius;              // Radio del clúster (m)
    double areaSize;                   // Tamaño del área de simulación (m)
    
    // Parámetros de red
    std::string routingProtocol;       // "AODV" o "OLSR"
    bool isUdp;
    uint32_t payloadSize;
    double dataRate;                   // Mbps
    double simulationTime;
    
    // Parámetros WiFi
    WifiStandard wifiStandard;
    std::string wifiMode;
};

// Estructura para almacenar resultados
struct SimulationResults {
    double throughput;                 // Mbps
    double avgDelay;                   // ms
    double packetDeliveryRatio;        // %
    double avgHopCount;
    double jitter;                     // ms
    uint32_t totalPacketsSent;
    uint32_t totalPacketsReceived;
    uint32_t totalPacketsLost;
};

class Experiment {
public:
    Experiment ();
    SimulationResults Run (Parameters params);
    
private:
    void SetupWifi (Parameters params);
    void SetupMobility (Parameters params);
    void SetupNetwork (Parameters params);
    void SetupApplications (Parameters params);
    SimulationResults CollectResults (Parameters params, Ptr<FlowMonitor> flowMonitor, 
                                     Ptr<Ipv4FlowClassifier> classifier);
    
    // Contenedores de nodos
    std::vector<NodeContainer> clusterNodes;
    NodeContainer clusterHeads;
    NodeContainer allNodes;
    
    // Contenedores de dispositivos
    std::vector<NetDeviceContainer> clusterDevices;
    NetDeviceContainer allDevices;
    
    // Interfaces IP
    std::vector<Ipv4InterfaceContainer> clusterInterfaces;
    
    // Aplicaciones
    ApplicationContainer serverApps;
    ApplicationContainer clientApps;
};

Experiment::Experiment () {
}

SimulationResults Experiment::Run (Parameters params) {
    std::cout << "\n========================================" << std::endl;
    std::cout << "Run: " << params.testName << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  numClusters: " << params.numClusters << std::endl;
    std::cout << "  nodesPerCluster: " << params.nodesPerCluster << std::endl;
    std::cout << "  growthFactor (j): " << params.growthFactor << std::endl;
    std::cout << "  clusterMobilitySpeed: " << params.clusterMobilitySpeed << " m/s" << std::endl;
    std::cout << "  nodeMobilitySpeed: " << params.nodeMobilitySpeed << " m/s" << std::endl;
    std::cout << "  Total nodes: " << (params.numClusters * params.nodesPerCluster) << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    // Limpiar contenedores
    clusterNodes.clear();
    clusterDevices.clear();
    clusterInterfaces.clear();
    clusterHeads = NodeContainer();
    allNodes = NodeContainer();
    
    // Crear clústeres
    clusterHeads.Create(params.numClusters);
    clusterNodes.resize(params.numClusters);
    
    for (uint32_t i = 0; i < params.numClusters; i++) {
        clusterNodes[i].Create(params.nodesPerCluster);
        allNodes.Add(clusterHeads.Get(i));
        allNodes.Add(clusterNodes[i]);
    }
    
    // Configurar WiFi ad-hoc
    SetupWifi(params);
    
    // Configurar movilidad jerárquica
    SetupMobility(params);
    
    // Configurar red y enrutamiento
    SetupNetwork(params);
    
    // Configurar aplicaciones
    SetupApplications(params);
    
    // Flow Monitor para métricas
    FlowMonitorHelper flowHelper;
    Ptr<FlowMonitor> flowMonitor = flowHelper.InstallAll();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowHelper.GetClassifier());
    
    // Ejecutar simulación
    Simulator::Stop(Seconds(params.simulationTime + 1));
    Simulator::Run();
    
    // Recolectar resultados
    SimulationResults results = CollectResults(params, flowMonitor, classifier);
    
    Simulator::Destroy();
    
    return results;
}

void Experiment::SetupWifi (Parameters params) {
    // Configurar canal WiFi con pérdidas
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::RangePropagationLossModel",
                                "MaxRange", DoubleValue(250.0));
    
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    phy.Set("TxPowerStart", DoubleValue(20.0));
    phy.Set("TxPowerEnd", DoubleValue(20.0));
    
    // Configurar WiFi en modo ad-hoc con 802.11b
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                  "DataMode", StringValue("DsssRate1Mbps"),
                                  "ControlMode", StringValue("DsssRate1Mbps"));
    
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    
    // Instalar WiFi en todos los nodos
    allDevices = wifi.Install(phy, mac, allNodes);
}

void Experiment::SetupMobility (Parameters params) {
    // Nivel 1: Movilidad de Cluster Heads (mayor escala)
    MobilityHelper clusterMobility;
    
    // Crear position allocator para los cluster heads
    Ptr<ListPositionAllocator> chPositionAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < params.numClusters; i++) {
        double angle = (2.0 * M_PI * i) / params.numClusters;
        double radius = params.areaSize / 4.0;
        double x = params.areaSize / 2.0 + radius * cos(angle);
        double y = params.areaSize / 2.0 + radius * sin(angle);
        chPositionAlloc->Add(Vector(x, y, 0.0));
    }
    
    clusterMobility.SetPositionAllocator(chPositionAlloc);
    clusterMobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                                     "Speed", StringValue("ns3::UniformRandomVariable[Min=" + 
                                                          std::to_string(params.clusterMobilitySpeed * 0.5) + "|Max=" + 
                                                          std::to_string(params.clusterMobilitySpeed * 1.5) + "]"),
                                     "Pause", StringValue("ns3::ConstantRandomVariable[Constant=2.0]"),
                                     "PositionAllocator", PointerValue(chPositionAlloc));
    
    clusterMobility.Install(clusterHeads);
    
    // Nivel 2: Movilidad intra-clúster (nodos miembros siguen al clusterhead)
    for (uint32_t i = 0; i < params.numClusters; i++) {
        Ptr<MobilityModel> chMobility = clusterHeads.Get(i)->GetObject<MobilityModel>();
        Vector chPos = chMobility->GetPosition();
        
        MobilityHelper nodeMobility;
        Ptr<ListPositionAllocator> nodePositionAlloc = CreateObject<ListPositionAllocator>();
        
        // Posicionar nodos alrededor del clusterhead
        for (uint32_t j = 0; j < params.nodesPerCluster; j++) {
            double angle = (2.0 * M_PI * j) / params.nodesPerCluster;
            double radius = params.clusterRadius * (0.3 + 0.7 * ((double)rand() / RAND_MAX));
            double x = chPos.x + radius * cos(angle);
            double y = chPos.y + radius * sin(angle);
            nodePositionAlloc->Add(Vector(x, y, 0.0));
        }
        
        nodeMobility.SetPositionAllocator(nodePositionAlloc);
        nodeMobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                                      "Bounds", RectangleValue(Rectangle(0, params.areaSize, 
                                                                         0, params.areaSize)),
                                      "Speed", StringValue("ns3::UniformRandomVariable[Min=" + 
                                                           std::to_string(params.nodeMobilitySpeed) + "|Max=" + 
                                                           std::to_string(params.nodeMobilitySpeed * 2.0) + "]"),
                                      "Distance", DoubleValue(params.clusterRadius / 2.0));
        
        nodeMobility.Install(clusterNodes[i]);
    }
}

void Experiment::SetupNetwork (Parameters params) {
    // Configurar stack de Internet con protocolo de enrutamiento
    InternetStackHelper stack;
    
    if (params.routingProtocol == "AODV") {
        AodvHelper aodv;
        stack.SetRoutingHelper(aodv);
    } else if (params.routingProtocol == "OLSR") {
        OlsrHelper olsr;
        olsr.Set("HelloInterval", TimeValue(Seconds(2.0)));
        olsr.Set("TcInterval", TimeValue(Seconds(5.0)));
        stack.SetRoutingHelper(olsr);
    }
    
    stack.Install(allNodes);
    
    // Asignar direcciones IP a todos los dispositivos de una vez
    Ipv4AddressHelper address;
    address.SetBase("10.1.0.0", "255.255.0.0");
    address.Assign(allDevices);
}

void Experiment::SetupApplications (Parameters params) {
    uint16_t port = 9;
    
    // Verificar que hay suficientes nodos
    if (clusterHeads.GetN() == 0) {
        std::cerr << "Error: No cluster heads available!" << std::endl;
        return;
    }
    
    // Seleccionar nodos para comunicación
    // Server: primer clusterhead
    // Clients: un nodo de cada clúster
    
    // Obtener dirección IP del servidor
    Ptr<Node> serverNode = clusterHeads.Get(0);
    Ptr<Ipv4> serverIpv4 = serverNode->GetObject<Ipv4>();
    
    // Buscar la interfaz válida (no loopback)
    Ipv4Address serverAddr;
    for (uint32_t i = 0; i < serverIpv4->GetNInterfaces(); i++) {
        if (serverIpv4->GetAddress(i, 0).GetLocal() != Ipv4Address("127.0.0.1")) {
            serverAddr = serverIpv4->GetAddress(i, 0).GetLocal();
            break;
        }
    }
    
    if (serverAddr == Ipv4Address()) {
        std::cerr << "Error: Server IP address not found!" << std::endl;
        return;
    }
    
    std::cout << "Server IP: " << serverAddr << std::endl;
    
    if (params.isUdp) {
        // Servidor UDP en el primer clusterhead
        UdpServerHelper server(port);
        serverApps = server.Install(clusterHeads.Get(0));
        serverApps.Start(Seconds(0.0));
        serverApps.Stop(Seconds(params.simulationTime + 1));
        
        // Clientes UDP: un nodo de cada clúster envía al servidor
        uint32_t clientCount = 0;
        for (uint32_t i = 0; i < params.numClusters; i++) {
            if (clusterNodes[i].GetN() > 0 && i > 0) { // Excluir el clúster del servidor
                UdpClientHelper client(serverAddr, port);
                client.SetAttribute("MaxPackets", UintegerValue(4294967295u));
                client.SetAttribute("Interval", TimeValue(MilliSeconds(10)));
                client.SetAttribute("PacketSize", UintegerValue(params.payloadSize));
                
                ApplicationContainer app = client.Install(clusterNodes[i].Get(0));
                app.Start(Seconds(2.0)); // Dar tiempo a que las rutas se establezcan
                app.Stop(Seconds(params.simulationTime + 1));
                clientApps.Add(app);
                clientCount++;
            }
        }
        std::cout << "Configured " << clientCount << " UDP clients" << std::endl;
    } else {
        // Servidor TCP
        Address localAddress(InetSocketAddress(Ipv4Address::GetAny(), port));
        PacketSinkHelper packetSinkHelper("ns3::TcpSocketFactory", localAddress);
        serverApps = packetSinkHelper.Install(clusterHeads.Get(0));
        serverApps.Start(Seconds(0.0));
        serverApps.Stop(Seconds(params.simulationTime + 1));
        
        // Clientes TCP
        uint32_t clientCount = 0;
        for (uint32_t i = 0; i < params.numClusters; i++) {
            if (clusterNodes[i].GetN() > 0 && i > 0) { // Excluir el clúster del servidor
                OnOffHelper onoff("ns3::TcpSocketFactory", 
                                  InetSocketAddress(serverAddr, port));
                onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
                onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
                onoff.SetAttribute("PacketSize", UintegerValue(params.payloadSize));
                onoff.SetAttribute("DataRate", DataRateValue(DataRate(std::to_string(params.dataRate) + "Mbps")));
                
                ApplicationContainer app = onoff.Install(clusterNodes[i].Get(0));
                app.Start(Seconds(2.0)); // Dar tiempo a que las rutas se establezcan
                app.Stop(Seconds(params.simulationTime + 1));
                clientApps.Add(app);
                clientCount++;
            }
        }
        std::cout << "Configured " << clientCount << " TCP clients" << std::endl;
    }
}

SimulationResults Experiment::CollectResults (Parameters params, Ptr<FlowMonitor> flowMonitor,
                                              Ptr<Ipv4FlowClassifier> classifier) {
    SimulationResults results;
    results.throughput = 0.0;
    results.avgDelay = 0.0;
    results.packetDeliveryRatio = 0.0;
    results.avgHopCount = 0.0;
    results.jitter = 0.0;
    results.totalPacketsSent = 0;
    results.totalPacketsReceived = 0;
    results.totalPacketsLost = 0;
    
    // Obtener estadísticas del FlowMonitor
    flowMonitor->CheckForLostPackets();
    
    std::map<FlowId, FlowMonitor::FlowStats> stats = flowMonitor->GetFlowStats();
    
    double totalDelay = 0.0;
    uint64_t totalRxBytes = 0;
    uint32_t flowCount = 0;
    
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); 
         i != stats.end(); ++i) {
        
        results.totalPacketsSent += i->second.txPackets;
        results.totalPacketsReceived += i->second.rxPackets;
        results.totalPacketsLost += i->second.lostPackets;
        
        totalRxBytes += i->second.rxBytes;
        
        if (i->second.rxPackets > 0) {
            totalDelay += (i->second.delaySum.GetMilliSeconds() / i->second.rxPackets);
            results.jitter += (i->second.jitterSum.GetMilliSeconds() / i->second.rxPackets);
            flowCount++;
        }
    }
    
    // Calcular métricas
    if (flowCount > 0) {
        results.avgDelay = totalDelay / flowCount;
        results.jitter = results.jitter / flowCount;
    }
    
    results.throughput = (totalRxBytes * 8.0) / (params.simulationTime * 1000000.0);
    
    if (results.totalPacketsSent > 0) {
        results.packetDeliveryRatio = (100.0 * results.totalPacketsReceived) / results.totalPacketsSent;
    }
    
    // Estimar hop count promedio (simplificación)
    results.avgHopCount = sqrt(params.numClusters) * 1.5;
    
    return results;
}

int main (int argc, char *argv[]) {
    // Parámetros por defecto
    Parameters params;
    params.testName = "Hierarchical MANET Scalability Analysis";
    params.numClusters = 5;
    params.nodesPerCluster = 4;
    params.growthFactor = 2.0;
    params.clusterMobilitySpeed = 5.0;     // m/s
    params.nodeMobilitySpeed = 2.0;        // m/s
    params.clusterRadius = 50.0;           // m
    params.areaSize = 500.0;               // m
    params.routingProtocol = "AODV";
    params.isUdp = true;
    params.payloadSize = 1024;             // bytes
    params.dataRate = 2.0;                 // Mbps
    params.simulationTime = 30.0;          // segundos
    params.wifiStandard = WIFI_STANDARD_80211b;
    params.wifiMode = "DsssRate11Mbps";
    
    bool runFullExperiment = false;
    std::string outputFile = "manet_results.csv";
    
    // Parámetros de línea de comandos
    CommandLine cmd(__FILE__);
    cmd.AddValue("numClusters", "Number of clusters", params.numClusters);
    cmd.AddValue("nodesPerCluster", "Nodes per cluster", params.nodesPerCluster);
    cmd.AddValue("growthFactor", "Growth factor j", params.growthFactor);
    cmd.AddValue("clusterSpeed", "Cluster mobility speed (m/s)", params.clusterMobilitySpeed);
    cmd.AddValue("nodeSpeed", "Node mobility speed (m/s)", params.nodeMobilitySpeed);
    cmd.AddValue("simulationTime", "Simulation time (s)", params.simulationTime);
    cmd.AddValue("routingProtocol", "Routing protocol (AODV/OLSR)", params.routingProtocol);
    cmd.AddValue("isUdp", "Use UDP (1) or TCP (0)", params.isUdp);
    cmd.AddValue("runFullExperiment", "Run full experiment with varying parameters", runFullExperiment);
    cmd.AddValue("outputFile", "Output CSV file", outputFile);
    cmd.Parse(argc, argv);
    
    Experiment experiment;
    
    if (runFullExperiment) {
        // Experimento completo: variar j y velocidad de movilidad
        std::ofstream outFile(outputFile);
        outFile << "GrowthFactor,ClusterSpeed,NumClusters,TotalNodes,Throughput(Mbps),"
                << "AvgDelay(ms),PDR(%),Jitter(ms),PacketsSent,PacketsReceived,PacketsLost\n";
        
        std::cout << "\n╔════════════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║  EXPERIMENTO COMPLETO: Factor j vs Velocidad de Movilidad ║" << std::endl;
        std::cout << "╚════════════════════════════════════════════════════════════╝\n" << std::endl;
        
        // Variar factor de crecimiento j
        for (double j = 1.5; j <= 4.0; j += 0.5) {
            params.growthFactor = j;
            params.numClusters = static_cast<uint32_t>(5 * j); // Escalar clusters con j
            
            // Variar velocidad de movilidad de clústeres
            for (double speed = 2.0; speed <= 20.0; speed += 3.0) {
                params.clusterMobilitySpeed = speed;
                params.nodeMobilitySpeed = speed * 0.4; // Nodos se mueven más lento
                
                params.testName = "j=" + std::to_string(j) + " speed=" + std::to_string(speed);
                
                SimulationResults results = experiment.Run(params);
                
                // Guardar resultados
                outFile << j << "," 
                        << speed << ","
                        << params.numClusters << ","
                        << (params.numClusters * params.nodesPerCluster) << ","
                        << results.throughput << ","
                        << results.avgDelay << ","
                        << results.packetDeliveryRatio << ","
                        << results.jitter << ","
                        << results.totalPacketsSent << ","
                        << results.totalPacketsReceived << ","
                        << results.totalPacketsLost << "\n";
                
                std::cout << "RESULTADOS:" << std::endl;
                std::cout << "  Throughput: " << results.throughput << " Mbps" << std::endl;
                std::cout << "  Avg Delay: " << results.avgDelay << " ms" << std::endl;
                std::cout << "  PDR: " << results.packetDeliveryRatio << " %" << std::endl;
                std::cout << "  Jitter: " << results.jitter << " ms" << std::endl;
                std::cout << "  Packets: " << results.totalPacketsReceived 
                          << "/" << results.totalPacketsSent << std::endl << std::endl;
            }
        }
        
        outFile.close();
        std::cout << "\n✓ Resultados guardados en: " << outputFile << "\n" << std::endl;
        
    } else {
        // Ejecución simple
        SimulationResults results = experiment.Run(params);
        
        std::cout << "\n╔═══════════════════════════╗" << std::endl;
        std::cout << "║  RESULTADOS DE SIMULACIÓN ║" << std::endl;
        std::cout << "╚═══════════════════════════╝" << std::endl;
        std::cout << "Throughput:       " << results.throughput << " Mbps" << std::endl;
        std::cout << "Avg Delay:        " << results.avgDelay << " ms" << std::endl;
        std::cout << "PDR:              " << results.packetDeliveryRatio << " %" << std::endl;
        std::cout << "Jitter:           " << results.jitter << " ms" << std::endl;
        std::cout << "Packets Sent:     " << results.totalPacketsSent << std::endl;
        std::cout << "Packets Received: " << results.totalPacketsReceived << std::endl;
        std::cout << "Packets Lost:     " << results.totalPacketsLost << std::endl;
        std::cout << "═══════════════════════════════\n" << std::endl;
    }
    
    return 0;
}