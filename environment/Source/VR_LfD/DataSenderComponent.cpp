// Fill out your copyright notice in the Description page of Project Settings.

#include "DataSenderComponent.h"
#include "Public/TimerManager.h"
#include "IPAddress.h"
#include "Runtime/Networking/Public/Interfaces/IPv4/IPv4Address.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

using namespace rapidjson;

// Sets default values for this component's properties
UDataSenderComponent::UDataSenderComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void UDataSenderComponent::BeginPlay()
{
	Super::BeginPlay();

	// ...
	connectState = false;
}


// Called every frame
void UDataSenderComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// ...
}

bool UDataSenderComponent::Conn(int IP_A, int IP_B, int IP_C, int IP_D, int port)
{
	Socket = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateSocket(NAME_Stream, TEXT("default"), false);

	FIPv4Address ip(IP_A, IP_B, IP_C, IP_D);
	TSharedRef<FInternetAddr> addr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
	addr->SetIp(ip.Value);
	addr->SetPort(port);

	bool success =  Socket->Connect(*addr);
	if (success) GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red, FString::Printf(TEXT("Sender Connected!")));
	return success;
}

bool UDataSenderComponent::Send(bool reset)
{
	FVector loc = wayPoint->RelativeLocation;
	FQuat rot = wayPoint->RelativeRotation.Quaternion();

	Document jsonDoc;
	Document::AllocatorType &allocator = jsonDoc.GetAllocator();
	jsonDoc.SetObject();

	int resetSignal = 0;
	if (reset) resetSignal = 1;
	jsonDoc.AddMember("reset", resetSignal, allocator);

	Value locObj(rapidjson::kObjectType);
	locObj.AddMember("X", loc.X, allocator);
	locObj.AddMember("Y", -loc.Y, allocator);
	locObj.AddMember("Z", loc.Z, allocator);
	jsonDoc.AddMember("location", locObj, allocator);

	Value rotObj(rapidjson::kObjectType);
	rotObj.AddMember("X", rot.X, allocator);
	rotObj.AddMember("Y", -rot.Y, allocator);
	rotObj.AddMember("Z", rot.Z, allocator);
	rotObj.AddMember("W", -rot.W, allocator);
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
	return result;
}

void UDataSenderComponent::SetPara(bool inState, USceneComponent* inPoint)
{
	connectState = inState;
	wayPoint = inPoint;
}
