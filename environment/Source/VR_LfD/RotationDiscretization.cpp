// Fill out your copyright notice in the Description page of Project Settings.

#include "RotationDiscretization.h"


// Sets default values for this component's properties
URotationDiscretization::URotationDiscretization()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void URotationDiscretization::BeginPlay()
{
	Super::BeginPlay();

	// ...
	
}


// Called every frame
void URotationDiscretization::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// ...
}

float URotationDiscretization::Discretize(float target, int scale)
{
	int targetToInt = (int)target;
	targetToInt = targetToInt / scale * scale;
	return (float)targetToInt;
}

