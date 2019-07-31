// Fill out your copyright notice in the Description page of Project Settings.

#include "RandomVectorComponent.h"
#include "stdlib.h"
#include <cmath>


// Sets default values for this component's properties
URandomVectorComponent::URandomVectorComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void URandomVectorComponent::BeginPlay()
{
	Super::BeginPlay();

	// ...
	
}


// Called every frame
void URandomVectorComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// ...
}

FVector URandomVectorComponent::GetRandomVector(float max, float min)
{
	float x, y, z;

	x = (float)rand() / (RAND_MAX + 1) * (max - min) + min;
	x = 0;
	y = (float)rand() / (RAND_MAX + 1) * (max - min) + min;
	z = (float)rand() / (RAND_MAX + 1) * (max - min) + min;
	z = 0;

	if (y < 0) y = min;
	else y = max;

	FVector ret(x, y, z);
	return ret;
}

