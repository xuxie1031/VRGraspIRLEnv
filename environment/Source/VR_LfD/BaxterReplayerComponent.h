// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Components/StaticMeshComponent.h"
#include "Engine/Engine.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/istreamwrapper.h"

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "BaxterReplayerComponent.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class VR_LFD_API UBaxterReplayerComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UBaxterReplayerComponent();

	UFUNCTION(BlueprintCallable, Category = "Replayer")
		void ReadJsonDoc(FString fileName);

	UFUNCTION(BlueprintCallable, Category = "Replayer")
		void MapJoints(UStaticMeshComponent* In_Base,
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

	UFUNCTION(BlueprintCallable, Category = "Replayer")
		bool ReplayAtStamp(int timeStamp);

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	TMap<FString, UStaticMeshComponent*> NameToMeshMap;		
	rapidjson::Document jsonDoc;
};
