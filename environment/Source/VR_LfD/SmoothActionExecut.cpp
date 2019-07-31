// Fill out your copyright notice in the Description page of Project Settings.

#include "SmoothActionExecut.h"
#include "Public/TimerManager.h"

// Sets default values for this component's properties
USmoothActionExecut::USmoothActionExecut()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void USmoothActionExecut::BeginPlay()
{
	Super::BeginPlay();

	// ...
}


// Called every frame
void USmoothActionExecut::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// ...
}

void USmoothActionExecut::PushAction(UStaticMeshComponent* inMesh, FVector inMovement)
{
	PendingMesh.Enqueue(inMesh);
	PendingMovement.Enqueue(inMovement);
}

bool USmoothActionExecut::QueueIsEmpty()
{
	return PendingMesh.IsEmpty();
}

//void USmoothActionExecut::LaunchAnimation()
//{
//
//}

