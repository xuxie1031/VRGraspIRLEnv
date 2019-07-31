// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Containers/Queue.h"
#include <string>

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "SmoothActionExecut.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class VR_LFD_API USmoothActionExecut : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	USmoothActionExecut();

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	UFUNCTION(BlueprintCallable, Category = "SmoothAction")
		void PushAction(UStaticMeshComponent* inMesh, FVector inMovement);

	UFUNCTION(BlueprintCallable, Category = "SmoothAction")
		bool QueueIsEmpty();


private:
	void LaunchAnimation();

	TQueue<UStaticMeshComponent*> PendingMesh;
	TQueue<FVector> PendingMovement;
};
