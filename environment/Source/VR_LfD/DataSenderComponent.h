// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Sockets.h"
#include "SocketSubsystem.h"
#include "Engine/Engine.h"
#include "Components/StaticMeshComponent.h"

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DataSenderComponent.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class VR_LFD_API UDataSenderComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UDataSenderComponent();

	UFUNCTION(BlueprintCallable, Category = "Sender")
		bool Conn(int IP_A, int IP_B, int IP_C, int IP_D, int port);

	UFUNCTION(BlueprintCallable, Category = "Sender")
		bool Send(bool reset);

	UFUNCTION(BlueprintCallable, Category = "Sender")
		void SetPara(bool inState, USceneComponent* inPoint);

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	FSocket * Socket;
	USceneComponent* wayPoint;
	bool connectState;
	
};
