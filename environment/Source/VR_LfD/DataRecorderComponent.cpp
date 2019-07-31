// Fill out your copyright notice in the Description page of Project Settings.

#include "DataRecorderComponent.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/Paths.h"
#include "GenericPlatform/GenericPlatformFile.h"
#include "HAL/PlatformFilemanager.h"
#include <string>

using namespace std;

float Clamp(float x) {
	float ret = x;
	if (ret > 1.0f) ret = 1.0f;
	else if (ret < -1.0f) ret = -1.0f;
	
	return ret;
}

// Sets default values for this component's properties
UDataRecorderComponent::UDataRecorderComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void UDataRecorderComponent::BeginPlay()
{
	Super::BeginPlay();

	// ...
}

void UDataRecorderComponent::WriteFile(int flag)
{
	if (LogSignal) {
		rapidjson::StringBuffer logBuffer;
		rapidjson::Writer<rapidjson::StringBuffer> logWriter(logBuffer);
		logDoc.Accept(logWriter);

		FString LogFolder = "log";
		FDateTime UTC = FDateTime::Now();
		FString LogFileNameSuffix = UTC.ToString();

		// WorkingDir is the path to the project file
		FString WorkingDir = FPaths::GetProjectFilePath();

		// Create a new folder for LogFiles
		FString logDir = WorkingDir + "/../" + LogFolder;

		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();

		// Directory Exists?
		if (!PlatformFile.DirectoryExists(*logDir))
		{
			PlatformFile.CreateDirectory(*logDir);

			if (!PlatformFile.DirectoryExists(*logDir))
			{
				exit(1);

			}
		}

		FString logFile = (logDir + "/./" + NameOfFile + "_" + LogFileNameSuffix + ".json"); // file to be stored
		std::string logStrPath = TCHAR_TO_UTF8(*logFile);  //convert FString to String

		FILE* myLogFile = fopen(logStrPath.c_str(), "wb");
		if (myLogFile) {
			fputs(logBuffer.GetString(), myLogFile);
			fclose(myLogFile);
		}

		FString traDir;
		if (flag == 1) traDir = WorkingDir + "/../success";
		else traDir = WorkingDir + "/../failure";

		// Directory Exists?
		if (!PlatformFile.DirectoryExists(*traDir))
		{
			PlatformFile.CreateDirectory(*traDir);

			if (!PlatformFile.DirectoryExists(*traDir))
			{
				exit(1);

			}
		}

		FString traFile = (traDir + "/./" + NameOfFile + "_" + LogFileNameSuffix + ".txt"); // file to be stored
		string traFileStr(TCHAR_TO_UTF8(*traFile));

		FILE* myTraFile = fopen(traFileStr.c_str(), "wb");
		if (myTraFile) {
			fputs(pendingTra.c_str(), myTraFile);
			fclose(myTraFile);
		}
	}

	logDoc.Clear();
	pendingTra.clear();
}


// Called every frame
void UDataRecorderComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// ...
}

void UDataRecorderComponent::InitJsonDoc(FVector inTargetOrigin) 
{
	if (LogSignal) {
		logDoc.SetArray();
		trajectoryDoc.SetArray();
	}

	targetOrigin = inTargetOrigin;
}

void UDataRecorderComponent::PushJoints(UStaticMeshComponent* In_Base,
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
	allJoints.Push(In_Base);
	allJoints.Push(In_Torso);
	allJoints.Push(In_Head);
	allJoints.Push(In_Screen);

	allJoints.Push(In_LArmMount);
	allJoints.Push(In_LUShoulder);
	allJoints.Push(In_LLShoulder);
	allJoints.Push(In_LUElbow);
	allJoints.Push(In_LLElbow);
	allJoints.Push(In_LUForearm);
	allJoints.Push(In_LLForearm);
	allJoints.Push(In_LWrist);

	allJoints.Push(In_RArmMount);
	allJoints.Push(In_RUShoulder);
	allJoints.Push(In_RLShoulder);
	allJoints.Push(In_RUElbow);
	allJoints.Push(In_RLElbow);
	allJoints.Push(In_RUForearm);
	allJoints.Push(In_RLForearm);
	allJoints.Push(In_RWrist);

	allJoints.Push(In_RGripperBase);
	allJoints.Push(In_RGripperLFinger);
	allJoints.Push(In_RGripperRFinger);

	allJoints.Push(In_GrabTarget);
}

void UDataRecorderComponent::StoreCharacterInfo() {
	rapidjson::Value curStampObj(rapidjson::kObjectType);
	rapidjson::Value meshArray(rapidjson::kArrayType);
	rapidjson::Document::AllocatorType &allocator = logDoc.GetAllocator();

	//Add grabbing status
	curStampObj.AddMember("Grabbing", grabbing, allocator);

	for (int i = 0; i < allJoints.Num(); i++) {
		rapidjson::Value thisMeshObj(rapidjson::kObjectType);

		//Mesh name
		FString meshName = allJoints[i]->GetName();
		std::string stdStringName(TCHAR_TO_UTF8(*meshName));
		rapidjson::Value tmpMeshName(stdStringName.c_str(), allocator);

		//Mesh location
		rapidjson::Value locObj(rapidjson::kObjectType);
		FVector thisLoc = allJoints[i]->GetComponentLocation();
		locObj.AddMember("X", thisLoc.X, allocator);
		locObj.AddMember("Y", thisLoc.Y, allocator);
		locObj.AddMember("Z", thisLoc.Z, allocator);

		//Mesh rotation
		rapidjson::Value rotObj(rapidjson::kObjectType);
		FRotator thisRot = allJoints[i]->GetComponentRotation();
		rotObj.AddMember("P", thisRot.Pitch, allocator);
		rotObj.AddMember("Y", thisRot.Yaw, allocator);
		rotObj.AddMember("R", thisRot.Roll, allocator);

		thisMeshObj.AddMember("Name", tmpMeshName, allocator);
		thisMeshObj.AddMember("Loc", locObj, allocator);
		thisMeshObj.AddMember("Rot", rotObj, allocator);

		meshArray.PushBack(thisMeshObj, allocator);
	}
	curStampObj.AddMember("Meshes", meshArray, allocator);
	logDoc.PushBack(curStampObj, allocator);
}

void UDataRecorderComponent::StoreTrajectory(FVector base, FVector lGripper, FVector rGripper, FVector base_next, FVector lGripper_next, FVector rGripper_next, float fingerMovement, bool terminate, int flag) {
	string thisStep;
	thisStep = to_string(LocRelativeToTargetOrigin(base).X) + ", " + to_string(LocRelativeToTargetOrigin(base).Y) + ", " + to_string(LocRelativeToTargetOrigin(base).Z) + ", ";
	thisStep += to_string(LocRelativeToTargetOrigin(lGripper).X) + ", " + to_string(LocRelativeToTargetOrigin(lGripper).Y) + ", " + to_string(LocRelativeToTargetOrigin(lGripper).Z) + ", ";
	thisStep += to_string(LocRelativeToTargetOrigin(rGripper).X) + ", " + to_string(LocRelativeToTargetOrigin(rGripper).Y) + ", " + to_string(LocRelativeToTargetOrigin(rGripper).Z) + ", ";
	FVector baseMovement;

	baseMovement = base_next - base;

	thisStep += to_string(Clamp(baseMovement.X)) + ", " + to_string(Clamp(baseMovement.Y)) + ", " + to_string(Clamp(baseMovement.Z)) + ", ";
	thisStep += to_string(Clamp(fingerMovement * 100.0f)) + ",";
	thisStep += to_string(LocRelativeToTargetOrigin(base_next).X) + ", " + to_string(LocRelativeToTargetOrigin(base_next).Y) + ", " + to_string(LocRelativeToTargetOrigin(base_next).Z) + ", ";
	thisStep += to_string(LocRelativeToTargetOrigin(lGripper_next).X) + ", " + to_string(LocRelativeToTargetOrigin(lGripper_next).Y) + ", " + to_string(LocRelativeToTargetOrigin(lGripper_next).Z) + ", ";
	thisStep += to_string(LocRelativeToTargetOrigin(rGripper_next).X) + ", " + to_string(LocRelativeToTargetOrigin(rGripper_next).Y) + ", " + to_string(LocRelativeToTargetOrigin(rGripper_next).Z) + ", ";
	thisStep += to_string(terminate) + ", " + to_string(flag) + "\n";

	pendingTra += thisStep;

	if (terminate) WriteFile(flag);
}

void UDataRecorderComponent::StoreObjInfo() {
	
}

void UDataRecorderComponent::SetGrabbingState(bool inState)
{
	grabbing = inState;
}

FVector UDataRecorderComponent::LocRelativeToTargetOrigin(FVector inLoc)
{
	return inLoc - targetOrigin;
}
