// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Containers/Queue.h"
#include "DocParser.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "BaseRobot.generated.h"

UCLASS()
class VR_LFD_API ABaseRobot : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ABaseRobot();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	TQueue<std::string> PendingData;
	int PendingDataSize;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	virtual void LaunchAnimation();

	virtual void ParseRobotData();

	void RecvRobotData(std::string RobotData);

	
};
