// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <random>

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "GaussionRandomGenerator.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class VR_LFD_API UGaussionRandomGenerator : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UGaussionRandomGenerator();

	UFUNCTION(BlueprintCallable, Category = "Random")
		void InitGaussionRandomGenerator(float mean, float stddev);

	UFUNCTION(BlueprintCallable, Category = "Random")
		float GaussionRandom();

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	std::default_random_engine generator;
	std::normal_distribution<float> distribution;
	
};
