// Fill out your copyright notice in the Description page of Project Settings.

#include "DataSenderActor.h"
#include "IPAddress.h"
#include "Runtime/Networking/Public/Interfaces/IPv4/IPv4Address.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

using namespace rapidjson;

// Sets default values
ADataSenderActor::ADataSenderActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ADataSenderActor::BeginPlay()
{
	Super::BeginPlay();
	
	connectStatus = false;
	FTimerHandle ReceiverHandler;
	GetWorldTimerManager().SetTimer(ReceiverHandler, this, &ADataSenderActor::Send, 0.005f, true);
}

// Called every frame
void ADataSenderActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

bool ADataSenderActor::Conn(int IP_A, int IP_B, int IP_C, int IP_D, int port)
{
	Socket = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateSocket(NAME_Stream, TEXT("default"), false);

	FIPv4Address ip(IP_A, IP_B, IP_C, IP_D);
	TSharedRef<FInternetAddr> addr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
	addr->SetIp(ip.Value);
	addr->SetPort(port);

	bool success = Socket->Connect(*addr);
	if (success) GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red, FString::Printf(TEXT("Sender Connected!")));
	return success;
}

void ADataSenderActor::Send()
{
	if (connectStatus) {
		FVector loc = RWrist->RelativeLocation;
		FRotator rot = RWrist->RelativeRotation;
		FQuat rotQuat = FQuat(rot);

		Document jsonDoc;
		Document::AllocatorType &allocator = jsonDoc.GetAllocator();
		jsonDoc.SetObject();

		Value locObj(rapidjson::kObjectType);
		locObj.AddMember("X", loc.X + 0.114, allocator);
		locObj.AddMember("Y", -loc.Y, allocator);
		locObj.AddMember("Z", loc.Z, allocator);
		jsonDoc.AddMember("location", locObj, allocator);

		Value rotObj(rapidjson::kObjectType);
		rotObj.AddMember("X", rotQuat.X, allocator);
		rotObj.AddMember("Y", rotQuat.Y, allocator);
		rotObj.AddMember("Z", rotQuat.Z, allocator);
		rotObj.AddMember("W", rotQuat.W, allocator);
		jsonDoc.AddMember("rotation", rotObj, allocator);

		StringBuffer buffer;
		Writer<StringBuffer> writer(buffer);
		jsonDoc.Accept(writer);

		std::string strJson = buffer.GetString();

		FString content(strJson.c_str());
		content += '\n';
		TCHAR *serializedChar = content.GetCharArray().GetData();
		int32 size = FCString::Strlen(serializedChar);
		int32 sent = 0;

		bool result = Socket->Send((uint8*)TCHAR_TO_UTF8(serializedChar), size, sent);
	}
}

void ADataSenderActor::SetPara(bool inStatus, UStaticMeshComponent* inMesh)
{
	connectStatus = inStatus;
	RWrist = inMesh;
}

