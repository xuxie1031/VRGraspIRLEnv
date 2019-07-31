// Fill out your copyright notice in the Description page of Project Settings.

#include "BaxterReplayerComponent.h"
#include "Misc/Paths.h"
#include <fstream>


// Sets default values for this component's properties
UBaxterReplayerComponent::UBaxterReplayerComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void UBaxterReplayerComponent::BeginPlay()
{
	Super::BeginPlay();

	// ...
}


// Called every frame
void UBaxterReplayerComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// ...
}

void UBaxterReplayerComponent::ReadJsonDoc(FString fileName)
{
	FString filePath = FPaths::GetProjectFilePath();

	filePath += "/../log/./" + fileName;

	std::string strPath = TCHAR_TO_UTF8(*filePath);
	std::ifstream ifs(strPath.c_str());
	rapidjson::IStreamWrapper isw(ifs);
	jsonDoc.ParseStream(isw);
}

void UBaxterReplayerComponent::MapJoints(UStaticMeshComponent* In_Base,
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
	UStaticMeshComponent* In_GrabTarget)
{
	NameToMeshMap.Emplace("SM_Base", In_Base);
	NameToMeshMap.Emplace("SM_Torso", In_Torso);
	NameToMeshMap.Emplace("SM_Head", In_Head);
	NameToMeshMap.Emplace("SM_Screen", In_Screen);
	NameToMeshMap.Emplace("SM_LArmMount", In_LArmMount);
	NameToMeshMap.Emplace("SM_LUShoulder", In_LUShoulder);
	NameToMeshMap.Emplace("SM_LLShoulder", In_LLShoulder);
	NameToMeshMap.Emplace("SM_LUElbow", In_LUElbow);
	NameToMeshMap.Emplace("SM_LLElbow", In_LLElbow);
	NameToMeshMap.Emplace("SM_LUForearm", In_LUForearm);
	NameToMeshMap.Emplace("SM_LLForearm", In_LLForearm);
	NameToMeshMap.Emplace("SM_LWrist", In_LWrist);
	NameToMeshMap.Emplace("SM_RArmMount", In_RArmMount);
	NameToMeshMap.Emplace("SM_RUShoulder", In_RUShoulder);
	NameToMeshMap.Emplace("SM_RLShoulder", In_RLShoulder);
	NameToMeshMap.Emplace("SM_RUElbow", In_RUElbow);
	NameToMeshMap.Emplace("SM_RLElbow", In_RLElbow);
	NameToMeshMap.Emplace("SM_RUForearm", In_RUForearm);
	NameToMeshMap.Emplace("SM_RLForearm", In_RLForearm);
	NameToMeshMap.Emplace("SM_RWrist", In_RWrist);
	NameToMeshMap.Emplace("Gripper_Base", In_RGripperBase);
	NameToMeshMap.Emplace("Gripper_LFinger", In_RGripperLFinger);
	NameToMeshMap.Emplace("Gripper_RFinger", In_RGripperRFinger);
	NameToMeshMap.Emplace("StaticMeshComponent0", In_GrabTarget);
}

bool UBaxterReplayerComponent::ReplayAtStamp(int timeStamp) 
{
	rapidjson::Value curStampObj;
	rapidjson::Value meshArray;
	bool grabbing = false;

	if ((unsigned)timeStamp < jsonDoc.Size()) {
		curStampObj = jsonDoc[timeStamp];
		grabbing = curStampObj["Grabbing"].GetBool();
		meshArray = curStampObj["Meshes"];

		for (rapidjson::SizeType i = 0; i < meshArray.Size(); i++) {
			rapidjson::Value thisMeshObj;

			thisMeshObj = meshArray[i];
			FString meshName(thisMeshObj["Name"].GetString());
			FVector thisLoc(thisMeshObj["Loc"]["X"].GetDouble(), thisMeshObj["Loc"]["Y"].GetDouble(), thisMeshObj["Loc"]["Z"].GetDouble());
			FRotator thisRot(thisMeshObj["Rot"]["P"].GetDouble(), thisMeshObj["Rot"]["Y"].GetDouble(), thisMeshObj["Rot"]["R"].GetDouble());

			UStaticMeshComponent* thisMesh = NameToMeshMap[meshName];
			thisMesh->SetWorldLocation(thisLoc);
			thisMesh->SetWorldRotation(thisRot);
		}
	}

	return grabbing;
}

