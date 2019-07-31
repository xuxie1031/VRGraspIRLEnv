// Fill out your copyright notice in the Description page of Project Settings.

#include "SmoothActionExecuteActor.h"
#include "Components/StaticMeshComponent.h"
#include "Public/TimerManager.h"

// Sets default values
ASmoothActionExecuteActor::ASmoothActionExecuteActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ASmoothActionExecuteActor::BeginPlay()
{
	Super::BeginPlay();

	pushingState = false;
	cnt = 0;
}

// Called every frame
void ASmoothActionExecuteActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void ASmoothActionExecuteActor::Init(bool inTrainingMode, float inRate, UStaticMeshComponent* inPlayDesk, UStaticMeshComponent* inGrabTarget, UStaticMeshComponent* inGripperBase, UStaticMeshComponent* inLFinger, UStaticMeshComponent* inRFinger)
{
	FTimerHandle ReceiverHandler;
	GetWorldTimerManager().SetTimer(ReceiverHandler, this, &ASmoothActionExecuteActor::LaunchAnimation, inRate, true);

	trainingMode = inTrainingMode;

	if (trainingMode) {
		playDesk = inPlayDesk;
		grabTarget = inGrabTarget;
	}
	gripperBase = inGripperBase;
	lFinger = inLFinger;
	rFinger = inRFinger;
}

void ASmoothActionExecuteActor::HandleFingerMovementinRFinger(float totalMovement, int stepsToComplete)
{
	float stepLen = totalMovement / stepsToComplete;

	SetPushingState(true);
	for (int i = 0; i < stepsToComplete; i++) {
		PushAction(1);
		PushMovement(FVector(0, 0, 0));
		PushMovement(FVector(0, 0, stepLen));
		PushMovement(FVector(0, 0, -stepLen));
	}
	SetPushingState(false);
}

void ASmoothActionExecuteActor::PushAction(int inActionNo)
{
	PendingAction.Enqueue(inActionNo);
}

void ASmoothActionExecuteActor::PushMovement(FVector inMovement)
{
	PendingMovement.Enqueue(inMovement);
}

void ASmoothActionExecuteActor::PushTF(FTransform inTF)
{
	PendingTF.Enqueue(inTF);
}

bool ASmoothActionExecuteActor::QueueIsEmpty()
{
	return PendingAction.IsEmpty();
}

void ASmoothActionExecuteActor::ClearQueue()
{
	PendingAction.Empty();
	PendingMovement.Empty();
	PendingTF.Empty();
}

void ASmoothActionExecuteActor::LaunchAnimation()
{
	if (pushingState) return;

	int thisActionNo;
	

	if (!PendingAction.IsEmpty())
	{
		PendingAction.Dequeue(thisActionNo);
		if (thisActionNo  == 0) {
			FTransform thisTF;

			PendingTF.Dequeue(thisTF); 
			gripperBase->SetWorldTransform(thisTF);
			
			PendingTF.Dequeue(thisTF); 
			lFinger->SetWorldTransform(thisTF);
			
			PendingTF.Dequeue(thisTF); 
			rFinger->SetWorldTransform(thisTF);

			PendingTF.Dequeue(thisTF);
			playDesk->SetWorldTransform(thisTF);

			PendingTF.Dequeue(thisTF); 
			grabTarget->SetWorldTransform(thisTF);
		}
		else if (thisActionNo == 1){
			FVector thisMovement;

			if (trainingMode) {
				if (!grabTarget->IsSimulatingPhysics()) {
					if (cnt <= 10) cnt++;
					else {
						playDesk->SetSimulatePhysics(true);
						grabTarget->SetSimulatePhysics(true);
						cnt = 0;
					}
				}
			}

			PendingMovement.Dequeue(thisMovement);
			gripperBase->AddRelativeLocation(thisMovement);

			PendingMovement.Dequeue(thisMovement); 
			lFinger->AddRelativeLocation(thisMovement);

			PendingMovement.Dequeue(thisMovement); 
			rFinger->AddRelativeLocation(thisMovement);
		}
		else if (thisActionNo == 2) {
			playDesk->SetSimulatePhysics(false);
			grabTarget->SetSimulatePhysics(false);
		}
	}
}

void ASmoothActionExecuteActor::SetPushingState(bool state) 
{
	pushingState = state;
}

