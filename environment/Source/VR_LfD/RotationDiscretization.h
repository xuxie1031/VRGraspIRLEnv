// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RotationDiscretization.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class VR_LFD_API URotationDiscretization : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	URotationDiscretization();

	UFUNCTION(BlueprintCallable, Category = "Discretization")
		float Discretize(float target, int scale);

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

		
	
};
