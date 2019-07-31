// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <string>
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include "SocketInterface.h"
#include "BaseRobot.h"

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DataReciverComponent.generated.h"

using namespace rapidjson;

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class VR_LFD_API UDataReciverComponent : public UActorComponent, public ISocketInterface
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UDataReciverComponent();

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

	virtual void BeginDestroy() override;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	UFUNCTION(BlueprintCallable, Category = "Reciver")
		void InitReciver(int IP_A, int IP_B, int IP_C, int IP_D, int port);

	UFUNCTION(BlueprintCallable, Category = "Reciver")
		int GetLastROSExecResult();

	UFUNCTION(BlueprintCallable, Category = "Reciver")
		void ClearLastROSExecResult();

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Reciver")
		FString baxterName;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Reciver")
		bool newWaypointSent;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Reciver")
		bool connected;

	bool OnTcpListenerConnectionAccepted(FSocket* Socket, const FIPv4Endpoint& Endpoint);

	int32 OnTcpReceived(const uint8* Data, int32 BytesReceived);

	void BuildListenerConnection();

private:

	TMap<FString, ABaseRobot*> Name2RobotMap;
	
	bool socketInitialized;
	int LastROSExecResult;

};
