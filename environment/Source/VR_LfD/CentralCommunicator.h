// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <string>
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include "SocketInterface.h"
#include "BaseRobot.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "CentralCommunicator.generated.h"

using namespace rapidjson;

UCLASS()
class VR_LFD_API ACentralCommunicator : public AActor, public ISocketInterface
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ACentralCommunicator();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	virtual void BeginDestroy() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Handle the TCP connection with provided socket interface
	bool OnTcpListenerConnectionAccepted(FSocket* Socket, const FIPv4Endpoint& Endpoint);

	int32 OnTcpReceived(const uint8* Data, int32 BytesReceived);

	void BuildListenerConnection();

private:

	TMap<FString, ABaseRobot*> Name2RobotMap;
	
};
