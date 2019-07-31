// Fill out your copyright notice in the Description page of Project Settings.

#include "GaussionRandomGenerator.h"

// Sets default values for this component's properties
UGaussionRandomGenerator::UGaussionRandomGenerator()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void UGaussionRandomGenerator::BeginPlay()
{
	Super::BeginPlay();

	// ...

}


// Called every frame
void UGaussionRandomGenerator::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// ...
}

void UGaussionRandomGenerator::InitGaussionRandomGenerator(float mean, float stddev)
{
	distribution = std::normal_distribution<float>(mean, stddev);
}

float UGaussionRandomGenerator::GaussionRandom()
{
	float ret = distribution(generator);
	return ret;
}


