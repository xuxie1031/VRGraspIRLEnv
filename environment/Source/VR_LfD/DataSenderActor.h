// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Sockets.h"
#include "SocketSubsystem.h"
#include "Engine/Engine.h"
#include "Components/StaticMeshComponent.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DataSenderActor.generated.h"

UCLASS()
class VR_LFD_API ADataSenderActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ADataSenderActor();

	UFUNCTION(BlueprintCallable, Category = "Sender")
		bool Conn(int IP_A, int IP_B, int IP_C, int IP_D, int port);

	UFUNCTION(BlueprintCallable, Category = "Sender")
		void Send();

	UFUNCTION(BlueprintCallable, Category = "Sender")
		void SetPara(bool inStatus, UStaticMeshComponent* inMesh);

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

private:
	FSocket * Socket;
	UStaticMeshComponent* RWrist;
	bool connectStatus;
	
	
};
