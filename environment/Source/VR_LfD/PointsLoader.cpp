// Fill out your copyright notice in the Description page of Project Settings.

#include "PointsLoader.h"
#include <string>
#include <fstream>
#include <cstring>

using namespace std;

// Sets default values
APointsLoader::APointsLoader()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void APointsLoader::BeginPlay()
{
	Super::BeginPlay();

}

// Called every frame
void APointsLoader::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void APointsLoader::Init(FString path)
{
	char buffer[256];
	std::string std_path(TCHAR_TO_UTF8(*path));
	ifstream in(std_path);
	while (!in.eof()) {
		in.getline(buffer, 100);
		float x = atof(buffer);
		in.getline(buffer, 100);
		float y = atof(buffer);
		in.getline(buffer, 100);
		float z = atof(buffer);
		in.getline(buffer, 100);
		int value = atoi(buffer);

		FVector point(x, y, z);
		points.Push(point);
		values.Push(value);
	}
}

