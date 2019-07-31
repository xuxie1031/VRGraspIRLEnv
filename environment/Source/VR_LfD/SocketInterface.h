// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Runtime/Core/Public/HAL/ThreadingBase.h"
#include "Sockets.h"
#include "Runtime/Networking/Public/Common/TcpListener.h"
#include "Networking.h"
#include "TCPReceiver.h"

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "SocketInterface.generated.h"

#define UB_TCP_BUFFER_SIZE 20000

// This class does not need to be modified.
UINTERFACE(MinimalAPI)
class USocketInterface : public UInterface
{
	GENERATED_BODY()
};

/**
 * 
 */
class VR_LFD_API ISocketInterface
{
	GENERATED_BODY()

	// Add interface functions to this class. This is the class that will be inherited to implement this interface.
public:
	FSocket* ListenerSocket;
    FSocket* ConnectionSocket;
    FTcpListener* TcpListener;
    TCPReceiver* Receiver;
    // Whether we are accepting
    bool Accepting;
	
	void SocketInit(int IP_A, int IP_B, int IP_C, int IP_D, int port);
	void SocketDestroy();
	
};
