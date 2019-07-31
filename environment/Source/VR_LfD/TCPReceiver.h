#pragma once
// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Runtime/Core/Public/HAL/ThreadingBase.h"
#include "Runtime/Sockets/Public/SocketSubsystem.h"
#include "Engine.h"
#include "Sockets.h"

/**
* Declares a delegate to be invoked when data has been received.
*
* The first parameter will hold the received data.
* The second parameter is the number of bytes received.
* Returns how many data the delegate consumes.
*/
FUNC_DECLARE_DELEGATE(FOnTcpReceive, int32, const uint8*, int32);


/**
*
*/
class VR_LFD_API TCPReceiver : public FRunnable
{
public:
	TCPReceiver(FSocket* Socket, int32 BufferSize = 1024, const FTimespan& InSleepTime = FTimespan::Zero())
		: Socket(Socket)
		, BufferSize(BufferSize)
		, BytesReceived(0)
		, Stopping(false)
		, SleepTime(InSleepTime) {
		Thread = FRunnableThread::Create(this, TEXT("TCPReceiver"), 8 * 1024, TPri_Normal);
		DataReceived = new uint8[BufferSize];
		UE_LOG(LogTemp, Warning, TEXT("BUFFER SIZE %d\n"), this->BufferSize);
	}
	~TCPReceiver() {
		if (Thread != nullptr)
		{
			Thread->Kill(true);
			delete Thread;
		}
		if (DataReceived != nullptr) {
			delete[] DataReceived;
		}
	}

	// FRunnable interface

	virtual bool Init() override
	{
		// Set to block mode.
		return (Socket != nullptr) && (Socket->SetNonBlocking(false));
	}

	virtual uint32 Run() override
	{
		while (!Stopping)
		{
			uint32 PendingSize = 0;
			if (Socket->HasPendingData(PendingSize)) {
				int32 NewReceived = 0;
				bool RecvFlag = Socket->Recv(DataReceived + BytesReceived, BufferSize - BytesReceived, NewReceived);

				if (RecvFlag == false) {
					auto Error = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->GetSocketError(ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->GetLastErrorCode());
					GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red, FString::Printf(TEXT("RECV RETURN ERROR! %s"), Error));
				}

				BytesReceived += NewReceived;

				if (ReceiveDelegate.IsBound() && NewReceived > 0) {
					int32 Consumed = ReceiveDelegate.Execute(DataReceived, BytesReceived);

					// Remove the prefix from the buffer.
					if (Consumed > 0) {
						FGenericPlatformMemory::Memmove(DataReceived, DataReceived + Consumed, BytesReceived - Consumed);
						BytesReceived -= Consumed;
					}
				}
			}
			FPlatformProcess::Sleep(SleepTime.GetSeconds());
		}

		return 0;
	}

	virtual void Stop() override
	{
		Stopping = true;
	}

	virtual void Exit() override { }

	/**
	* Gets a delegate to be invoked when received data.
	*/
	FOnTcpReceive& OnTcpReceive()
	{
		return ReceiveDelegate;
	}

private:

	/** Holds the server socket. */
	FSocket* Socket;

	/** Holds the size of the buffer. */
	int32 BufferSize;

	/** Holds the received data. */
	uint8* DataReceived;

	/** Holds the number of bytes received so far. */
	int32 BytesReceived;

	/** Holds a flag indicating that the thread is stopping. */
	bool Stopping;

	/** Holds the thread object. */
	FRunnableThread* Thread;

	/** Holds a delegate to be invoked when data has been received. */
	FOnTcpReceive ReceiveDelegate;

	const FTimespan SleepTime;
};

