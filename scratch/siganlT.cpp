#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/spectrum-module.h"
// ===== 추가된 헤더파일 =====
#include "ns3/itu-r-1411-nlos-over-rooftop-propagation-loss-model.h"

using namespace ns3;

// UE가 측정한 RSRP 값을 출력하는 콜백 함수
void
UeMeasCallback(uint16_t cellId, uint16_t imsi, uint16_t rnti, double rsrp, uint8_t bwpId)
{
    std::cout << "\n======================================================\n";
    std::cout << "🏢 UE의 RSRP 측정 결과 (모델 적용됨) 🏢\n";
    std::cout << "측정 시간: " << Simulator::Now().GetSeconds() << " s\n";
    std::cout << "Cell ID: " << cellId << ", RNTI: " << rnti << "\n";
    std::cout << "측정된 RSRP: " << rsrp << " dBm\n";
    std::cout << "======================================================\n";
    // RSRP = TxPower - PathLoss. 이 값이 낮다는 것은 PathLoss가 크다는 의미입니다.
}

int
main(int argc, char* argv[])
{
    // --- 시뮬레이션 시간 설정 ---
    Time simTime = Seconds(1.0);

    // --- 1. 전파 손실 모델 객체 생성 및 설정 ---
    // 사용하려는 PropagationLossModel 객체를 직접 생성합니다.
    Ptr<ItuR1411NlosOverRooftopPropagationLossModel> lossModel = CreateObject<ItuR1411NlosOverRooftopPropagationLossModel>();

    // 모델의 파라미터를 설정하여 가상 건물/도시 환경을 정의합니다.
    lossModel->SetAttribute("Frequency", DoubleValue(3.5e9)); // gNB 주파수와 일치
    lossModel->SetAttribute("RooftopLevel", DoubleValue(20.0)); // 건물 옥상 높이: 20m
    lossModel->SetAttribute("StreetsWidth", DoubleValue(30.0)); // 도로 폭: 30m
    lossModel->SetAttribute("StreetsOrientation", DoubleValue(30.0)); // 도로 방향: 30도
    lossModel->SetAttribute("BuildingSeparation", DoubleValue(50.0)); // 건물 간 간격: 50m

    // --- 2. 채널 객체에 전파 손실 모델 설정 ---
    // NrHelper를 통해 채널을 설정하기 전에, 채널 객체를 만들고 위에서 만든 모델을 할당합니다.
    Ptr<MultiModelSpectrumChannel> channel =
        CreateObject<MultiModelSpectrumChannel>();

    // 3) 손실 모델과 지연 모델 등록
    channel->AddPropagationLossModel(lossModel);
    Ptr<ConstantSpeedPropagationDelayModel> delay =
        CreateObject<ConstantSpeedPropagationDelayModel>();
    channel->SetPropagationDelayModel(delay);
    
    // --- 3. 헬퍼 설정 및 채널 할당 ---
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();


    // --- 4. gNB 노드 생성 및 설정 ---
    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    
    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> gnbPosAlloc = CreateObject<ListPositionAllocator>();
    gnbPosAlloc->Add(Vector(0.0, 0.0, 10.0)); // gNB 위치: 지상 10m
    gnbMobility.SetPositionAllocator(gnbPosAlloc);
    gnbMobility.Install(gnbNodes);

    // gNB 대역 및 BWP 설정
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 1e8, 1, BandwidthPartInfo::UMi_StreetCanyon_LoS);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    nrHelper->InitializeOperationBand(&band);
    BandwidthPartInfoPtrVector bwp = CcBwpCreator::GetAllBwps({band});
    
    // gNB NetDevice 설치
    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, bwp);

    // --- 5. UE 노드 생성 및 설정 ---
    NodeContainer ueNodes;
    ueNodes.Create(1);

    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> uePosAlloc = CreateObject<ListPositionAllocator>();
    uePosAlloc->Add(Vector(50.0, 0.0, 1.5)); // UE 위치: 지상 1.5m, gNB로부터 50m 거리
    ueMobility.SetPositionAllocator(uePosAlloc);
    ueMobility.Install(ueNodes);

    // UE NetDevice 설치
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, bwp);

    // (1) gNB SpectrumPhy 에 채널 연결
    for (uint32_t i = 0; i < gnbDevs.GetN(); ++i) {
        Ptr<NrGnbNetDevice> gnbDev = gnbDevs.Get(i)->GetObject<NrGnbNetDevice>();
        Ptr<NrSpectrumPhy>  gnbSpecPhy = gnbDev->GetPhy(0)->GetSpectrumPhy();  //  [oai_citation:1‡cttc-lena.gitlab.io](https://cttc-lena.gitlab.io/nr/html/classns3_1_1_nr_phy.html?utm_source=chatgpt.com)
        gnbSpecPhy->SetChannel(channel);
    }

    // (2) UE SpectrumPhy 에 채널 연결
    for (uint32_t i = 0; i < ueDevs.GetN(); ++i) {
        Ptr<NrUeNetDevice>  ueDev     = ueDevs.Get(i)->GetObject<NrUeNetDevice>();
        Ptr<NrSpectrumPhy>  ueSpecPhy = ueDev->GetPhy(0)->GetSpectrumPhy();  //  [oai_citation:2‡cttc-lena.gitlab.io](https://cttc-lena.gitlab.io/nr/html/classns3_1_1_nr_phy.html?utm_source=chatgpt.com)
        ueSpecPhy->SetChannel(channel);
    }

    // --- 6. RSRP 측정 콜백 연결 및 기지국 접속 ---
    Ptr<NrUeNetDevice> ueDevice = ueDevs.Get(0)->GetObject<NrUeNetDevice>();
    ueDevice->GetPhy(0)->TraceConnectWithoutContext("ReportRsrp", MakeCallback(&UeMeasCallback));

    Ipv6AddressHelper ipv6h;
    ipv6h.SetBase("fd00:a516:7c1b:17cd::", Ipv6Prefix(64));
    epcHelper->AssignUeIpv6Address(ueDevs);
    epcHelper->AssignUeIpv6Address(gnbDevs);

    nrHelper->AttachToClosestEnb(ueDevs, gnbDevs);
    
    // --- 시뮬레이션 실행 ---
    Simulator::Stop(simTime);
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}