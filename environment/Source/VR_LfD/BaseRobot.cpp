// Fill out your copyright notice in the Description page of Project Settings.

#include "BaseRobot.h"


// Sets default values
ABaseRobot::ABaseRobot()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ABaseRobot::BeginPlay()
{
	Super::BeginPlay();
	
	PendingDataSize = 0;
}

// Called every frame
void ABaseRobot::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void ABaseRobot::LaunchAnimation()
{
	ParseRobotData();
}

void ABaseRobot::ParseRobotData()
{
	std::string data;
	if (!PendingData.IsEmpty())
	{
		PendingData.Dequeue(data);
		if (PendingData.IsEmpty())
			PendingData.Enqueue(data);
	}

	//parsing the current json object, override
}

void ABaseRobot::RecvRobotData(std::string RobotData)
{
	PendingData.Enqueue(RobotData);
	PendingDataSize++;
}
