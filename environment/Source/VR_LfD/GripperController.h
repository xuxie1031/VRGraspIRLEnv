// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include "SocketInterface.h"
#include "SmoothActionExecuteActor.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include <string>

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GripperController.generated.h"

using namespace rapidjson;

UCLASS()
class VR_LFD_API AGripperController : public AActor, public ISocketInterface
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	AGripperController();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	virtual void BeginDestroy() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	UFUNCTION(BlueprintCallable, Category = "Reciver")
		void InitReciver(int IP_A, int IP_B, int IP_C, int IP_D, int port);

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Reciver")
		bool connected;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Reciver")
		int actionCnt;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Reciver")
		bool terminate;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Reciver")
		bool lFingerTouched;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Reciver")
		bool rFingerTouched;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Reciver")
		bool fingerFirstFlipped;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Reciver")
		TSubclassOf<AActor> ClassToFind;

	UFUNCTION(BlueprintCallable, Category = "Sender")
		bool SenderConn(int IP_A, int IP_B, int IP_C, int IP_D, int port);


	UFUNCTION(BlueprintCallable, Category = "Sender")
		void Init(UStaticMeshComponent* inPlayDesk,
			UStaticMeshComponent* inGrabTarget,
			UStaticMeshComponent* inGripperBase,
			UStaticMeshComponent* inLFinger,
			UStaticMeshComponent* inRFinger,
			ASmoothActionExecuteActor* inExecutor);

	UFUNCTION(BlueprintCallable, Category = "Sender")
		void Reset();

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "GripperController")
		FString lFingerTouchedName;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "GripperController")
		FString rFingerTouchedName;

	bool OnTcpListenerConnectionAccepted(FSocket* Socket, const FIPv4Endpoint& Endpoint);

	int32 OnTcpReceived(const uint8* Data, int32 BytesReceived);

	void BuildListenerConnection();

	void CheckAnimQueue();

	void Send(std::string cmd);

	FVector LocRelativeToTarget(int meshNo);

private:

	float stateAsFloat[9];
	float actionAsFloat[4];

	std::string lastOutJsonStr;

	TQueue<int>waitSignalQ;

	int waitForSmoothAction;

	bool socketInitialized;

	bool firstReset;

	FSocket * Socket;
	bool senderConnectStatus;

	ASmoothActionExecuteActor* smoothActionExecutor;

	UStaticMeshComponent* playDesk;
	UStaticMeshComponent* grabTarget;
	UStaticMeshComponent* gripperBase;
	UStaticMeshComponent* lFinger;
	UStaticMeshComponent* rFinger;

	FTransform playDeskStartTF;
	FTransform grabTargetStartTF;
	FTransform gripperBaseStartTF;
	FTransform lFingerStartTF;
	FTransform rFingerStartTF;
};