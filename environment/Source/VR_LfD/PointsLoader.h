// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PointsLoader.generated.h"

UCLASS()
class VR_LFD_API APointsLoader : public AActor
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	APointsLoader();

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "point")
		TArray<FVector>points;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "point")
		TArray<int>values;

	UFUNCTION(BlueprintCallable, Category = "point")
		void Init(FString path);

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

private:
};
