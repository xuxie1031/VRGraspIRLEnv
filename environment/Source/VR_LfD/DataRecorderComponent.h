// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Components/StaticMeshComponent.h"
#include "Engine/Engine.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <string>

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DataRecorderComponent.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class VR_LFD_API UDataRecorderComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UDataRecorderComponent();

	UFUNCTION(BlueprintCallable, Category = "Recorder")
		void InitJsonDoc(FVector inTargetOrigin);

	UFUNCTION(BlueprintCallable, Category = "Recorder")
		void PushJoints(UStaticMeshComponent* In_Base,
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
			UStaticMeshComponent* In_RWrist,
			UStaticMeshComponent* In_RGripperBase,
			UStaticMeshComponent* In_RGripperLFinger,
			UStaticMeshComponent* In_RGripperRFinger,
			UStaticMeshComponent* In_GrabTarget);

	UFUNCTION(BlueprintCallable, Category = "Recorder")
		void StoreCharacterInfo();

	UFUNCTION(BlueprintCallable, Category = "Recorder")
		void StoreTrajectory(FVector base, FVector lGripper, FVector rGripper, FVector base_next, FVector lGripper_next, FVector rGripper_next, float fingerMovement, bool terminate, int flag);

	UFUNCTION(BlueprintCallable, Category = "Recorder")
		void StoreObjInfo();

	UFUNCTION(BlueprintCallable, Category = "Recorder")
		void SetGrabbingState(bool inState);

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Recorder")
		FString NameOfFile;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Recorder")
		bool LogSignal;

protected:
	// Called when the game starts
	virtual void BeginPlay() override;
	//virtual void BeginDestroy() override;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	void WriteFile(int flag);

private:
	FVector LocRelativeToTargetOrigin(FVector inLoc);

	std::string pendingTra;

	rapidjson::Document logDoc;
	rapidjson::Document trajectoryDoc;
	TArray<UStaticMeshComponent*> allJoints;
	bool grabbing;
	
	FVector targetOrigin;
};
