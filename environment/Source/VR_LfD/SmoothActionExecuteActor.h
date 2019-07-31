// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Containers/Queue.h"
#include <string>

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SmoothActionExecuteActor.generated.h"

UCLASS()
class VR_LFD_API ASmoothActionExecuteActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ASmoothActionExecuteActor();

	UFUNCTION(BlueprintCallable, Category = "SmoothAction")
		void Init(bool inTrainingMode, float inRate, UStaticMeshComponent* inPlayDesk, UStaticMeshComponent* inGrabTarget, UStaticMeshComponent* inGripperBase, UStaticMeshComponent* inLFinger, UStaticMeshComponent* inRFinger);

	UFUNCTION(BlueprintCallable, Category = "SmoothAction")
		void HandleFingerMovementinRFinger(float totalMovement, int stepsToComplete);

	UFUNCTION(BlueprintCallable, Category = "SmoothAction")
		void PushAction(int inActionNo);

	UFUNCTION(BlueprintCallable, Category = "SmoothAction")
		void PushMovement(FVector inMovement);

	UFUNCTION(BlueprintCallable, Category = "SmoothAction")
		void PushTF(FTransform inTF);

	UFUNCTION(BlueprintCallable, Category = "SmoothAction")
		bool QueueIsEmpty();

	UFUNCTION(BlueprintCallable, Category = "SmoothAction")
		void ClearQueue();

	void SetPushingState(bool state);

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

private:
	void LaunchAnimation();

	int cnt;

	TQueue<int> PendingAction;
	TQueue<FVector> PendingMovement;
	TQueue<FTransform> PendingTF;

	UStaticMeshComponent* playDesk;
	UStaticMeshComponent* grabTarget;
	UStaticMeshComponent* gripperBase;
	UStaticMeshComponent* lFinger;
	UStaticMeshComponent* rFinger;

	bool pushingState;
	bool trainingMode;
};
