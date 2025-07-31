//
// Created by 고기호 on 25. 7. 5.
//
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/nr-module.h"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("CellularMobilityExample");

// CSV 파일의 한 줄을 저장하기 위한 구조체
struct WaypointData
{
    double time;
    double x;
    double y;
    double z;
    double speed;
};

// UE의 위치와 속도를 출력하는 함수
void PrintUeInfo(Ptr<Node> ueNode)
{
    Ptr<MobilityModel> mob = ueNode->GetObject<MobilityModel>();
    Vector pos = mob->GetPosition();
    Vector vel = mob->GetVelocity();

    NS_LOG_UNCOND("Time: " << Simulator::Now().GetSeconds() << "s");
    NS_LOG_UNCOND("UE Position: x=" << pos.x << ", y=" << pos.y << ", z=" << pos.z);
    NS_LOG_UNCOND("UE Velocity: x=" << vel.x << ", y=" << vel.y << ", z=" << vel.z << " (m/s)");
    Simulator::Schedule (Seconds (1.0), &PrintUeInfo, ueNode);
}

int main(int argc, char* argv[])
{
    // 1. CSV 파일 읽기
    // ==========================================================
    std::vector<WaypointData> waypoints;
    std::string csvFileName = "/Users/koh-mac/Documents/ns-simulator/ns-3-v2x/scratch/final_3d_trace.csv";
    std::ifstream file(csvFileName);

    if (!file.is_open())
    {
        NS_LOG_ERROR("Could not open CSV file: " << csvFileName);
        return 2;
    }

    std::string line;
    // 헤더 라인 무시
    std::getline(file, line);

    double maxTime = 0.0;
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string value;
        WaypointData data;

        // 각 열 파싱
        std::getline(ss, value, ','); // time
        data.time = std::stod(value);
        std::getline(ss, value, ','); // vehicle_id (skip)
        std::getline(ss, value, ','); // x
        data.x = std::stod(value);
        std::getline(ss, value, ','); // y
        data.y = std::stod(value);
        std::getline(ss, value, ','); // z
        data.z = std::stod(value);
        std::getline(ss, value, ','); // speed
        data.speed = std::stod(value);
        std::getline(ss, value, ','); // lon (skip)
        std::getline(ss, value, ','); // lat (skip)

        waypoints.push_back(data);
        if (data.time > maxTime) {
            maxTime = data.time;
        }
    }
    file.close();
    NS_LOG_UNCOND("Successfully read " << waypoints.size() << " waypoints from CSV.");


    // 2. 5G-LENA 네트워크 설정
    // ==========================================================
    NodeContainer ueNodes;
    ueNodes.Create(1);


    // 3. 모빌리티 설정 (가장 중요)
    // ==========================================================
    MobilityHelper mobility;

    // UE는 WaypointMobilityModel 사용
    mobility.SetMobilityModel("ns3::WaypointMobilityModel");
    mobility.Install(ueNodes);

    Ptr<WaypointMobilityModel> ueMobility = ueNodes.Get(0)->GetObject<WaypointMobilityModel>();

    // 읽어온 CSV 데이터를 Waypoint로 추가
    for (const auto& data : waypoints)
    {
        Waypoint waypoint(Seconds(data.time), Vector(data.x, data.y, data.z));
        ueMobility->AddWaypoint(waypoint);
    }

    // 4. 시뮬레이션 스케줄링 및 실행
    // ==========================================================
    // UE를 gNB에 연결

    // 시뮬레이션 시작 시점에 UE 정보 출력 함수를 호출하도록 예약
    Simulator::Schedule(Seconds(0.0), &PrintUeInfo, ueNodes.Get(0));

    // 전체 시뮬레이션 시간 설정
    Simulator::Stop(Seconds(maxTime + 1.0));

    NS_LOG_INFO("Running simulation...");
    Simulator::Run();
    NS_LOG_INFO("Simulation finished.");
    Simulator::Destroy();

    return 0;
}