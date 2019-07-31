// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "BaseRobot.h"
#include "BaxterRobot.generated.h"

/**
 * 
 */
UCLASS()
class VR_LFD_API ABaxterRobot : public ABaseRobot
{
	GENERATED_BODY()
	
public:
	// Sets default values for this actor's properties
	ABaxterRobot();

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "SM Assets")
		USceneComponent* SceneComponent;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "SM Assets")
		UStaticMeshComponent* SM_Base;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "SM Assets")
		UStaticMeshComponent* SM_Torso;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "SM Assets")
		UStaticMeshComponent* SM_Head;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "SM Assets")
		UStaticMeshComponent* SM_Screen;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "SM Assets")
		UStaticMeshComponent* SM_LArmMount;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "SM Assets")
		UStaticMeshComponent* SM_LUShoulder;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "SM Assets")
		UStaticMeshComponent* SM_LLShoulder;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "SM Assets")
		UStaticMeshComponent* SM_LUElbow;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "SM Assets")
		UStaticMeshComponent* SM_LLElbow;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "SM Assets")
		UStaticMeshComponent* SM_LUForearm;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "SM Assets")
		UStaticMeshComponent* SM_LLForearm;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "SM Assets")
		UStaticMeshComponent* SM_LWrist;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "SM Assets")
		UStaticMeshComponent* SM_RArmMount;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "SM Assets")
		UStaticMeshComponent* SM_RUShoulder;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "SM Assets")
		UStaticMeshComponent* SM_RLShoulder;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "SM Assets")
		UStaticMeshComponent* SM_RUElbow;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "SM Assets")
		UStaticMeshComponent* SM_RLElbow;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "SM Assets")
		UStaticMeshComponent* SM_RUForearm;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "SM Assets")
		UStaticMeshComponent* SM_RLForearm;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "SM Assets")
		UStaticMeshComponent* SM_RWrist;

	UFUNCTION(BlueprintCallable, Category = "SM Functions")
		void SetCompoments(UStaticMeshComponent* In_Base, 
			UStaticMeshComponent* In_Torso, 
			UStaticMeshComponent* In_Head, 
			UStaticMeshComponent* In_Screen, 
			UStaticMeshComponent* In_LArmMount, 
			UStaticMeshComponent* In_LUShoulder, 
			UStaticMeshComponent* In_LLShoulder, 
			UStaticMeshComponent* In_LUElbow, 
			UStaticMeshComponent* In_LLElbow, 
			UStaticMeshComponent* In_LUForearm, 
			UStaticMeshComponent* In_LLForearm, 
			UStaticMeshComponent* In_LWrist, 
			UStaticMeshComponent* In_RArmMount, 
			UStaticMeshComponent* In_RUShoulder, 
			UStaticMeshComponent* In_RLShoulder, 
			UStaticMeshComponent* In_RUElbow, 
			UStaticMeshComponent* In_RLElbow, 
			UStaticMeshComponent* In_RUForearm, 
			UStaticMeshComponent* In_RLForearm, 
			UStaticMeshComponent* In_RWrist);

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Called every frame
	virtual void Tick(float DeltaSeconds) override;

	virtual void LaunchAnimation() override;

	virtual void ParseRobotData() override;

	void BaxterAnimation(const Value& SegArr);

private:

	TMap<FString, UStaticMeshComponent*> NameToMeshMap;

	void InitializeTransMap(UStaticMeshComponent** c, const TCHAR* name);

};
