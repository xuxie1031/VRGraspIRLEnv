// Fill out your copyright notice in the Description page of Project Settings.

#include "SocketInterface.h"
#include "Runtime/Networking/Public/Common/TcpSocketBuilder.h"
#include "IPv4/IPv4Endpoint.h"
#include "IPv4/IPv4Address.h"

// Add default functionality here for any ISocketInterface functions that are not pure virtual.
void ISocketInterface::SocketInit(int IP_A, int IP_B, int IP_C, int IP_D, int port)
{
	FIPv4Endpoint Endpoint(FIPv4Address(IP_A, IP_B, IP_C, IP_D), port);
	ListenerSocket = FTcpSocketBuilder(TEXT("TCPLISTENER"))
		.AsReusable()
		.BoundToEndpoint(Endpoint)
		.WithReceiveBufferSize(UB_TCP_BUFFER_SIZE)
		.WithSendBufferSize(UB_TCP_BUFFER_SIZE)
		.Listening(1);
	ListenerSocket->Connect(Endpoint.ToInternetAddr().Get());
}

void ISocketInterface::SocketDestroy()
{
	// Destroy the TCPListener.
	if (TcpListener != NULL) {
		delete TcpListener;
		TcpListener = NULL;
	}
	// Release the socket.
	if (ListenerSocket != NULL) {
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ListenerSocket);
		ListenerSocket = NULL;
	}
	// Destroy the Receiver.
	if (Receiver != NULL) {
		delete Receiver;
		Receiver = NULL;
	}
	// Release the socket of the connection.
	if (ConnectionSocket != NULL) {
		ConnectionSocket->Close();
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ConnectionSocket);
		ConnectionSocket = NULL;
	}
}