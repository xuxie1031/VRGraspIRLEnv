// Fill out your copyright notice in the Description page of Project Settings.

#include "DataReciverComponent.h"


// Sets default values for this component's properties
UDataReciverComponent::UDataReciverComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void UDataReciverComponent::BeginPlay()
{
	Super::BeginPlay();

	// ...
	socketInitialized = false;
	newWaypointSent = true;
	connected = false;
	LastROSExecResult = -1;
}

void UDataReciverComponent::BeginDestroy()
{
	Super::BeginDestroy();

	// ...
	if (socketInitialized) SocketDestroy();
}


// Called every frame
void UDataReciverComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// ...
}

void UDataReciverComponent::InitReciver(int IP_A, int IP_B, int IP_C, int IP_D, int port)
{
	SocketInit(IP_A, IP_B, IP_C, IP_D, port);
	socketInitialized = true;

	BuildListenerConnection();

	// Trace of Robots in level
	TArray<AActor*> FoundActors;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), ABaseRobot::StaticClass(), FoundActors);
	for (auto Actor : FoundActors)
	{
		ABaseRobot* BaseRobot = Cast<ABaseRobot>(Actor);
		Name2RobotMap.Emplace(BaseRobot->GetName(), BaseRobot);
	}
}

void UDataReciverComponent::BuildListenerConnection()
{
	if (ListenerSocket != NULL) {

		TcpListener = new FTcpListener(*ListenerSocket);
		TcpListener->OnConnectionAccepted().BindUObject(this, &UDataReciverComponent::OnTcpListenerConnectionAccepted);

		if (TcpListener->Init()) {
			GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red, FString::Printf(TEXT("Initialize listener!")));
			// We are now accepting clients.
			Accepting = true;
		}
	}
	else {
		GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red, FString::Printf(TEXT("Failed initializing listener!")));
	}
}

bool UDataReciverComponent::OnTcpListenerConnectionAccepted(FSocket* Socket, const FIPv4Endpoint& Endpoint)
{
	if (Accepting == true) {
		GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red, FString::Printf(TEXT("A new connection")));
		connected = true;

		auto State = Socket->GetConnectionState();

		// Start another thread listening on this connection.
		ConnectionSocket = Socket;
		Receiver = new TCPReceiver(Socket, UB_TCP_BUFFER_SIZE);
		Receiver->OnTcpReceive().BindUObject(this, &UDataReciverComponent::OnTcpReceived);
		Receiver->Init();
		return true;
	}
	// Reject the connection.
	return false;
}

int32 UDataReciverComponent::OnTcpReceived(const uint8* Data, int32 BytesReceived)
{
	// This is a static buffer for convert the data.
	// Notice this function should be called only on single thread.
	static uint8 buffer[UB_TCP_BUFFER_SIZE];

	// This is a static frame used for parsing.
	// Notice this function should be called only on single thread.

	int32 consumed = 0;
	int32 previous = 0;
	int32 count = 0;    // How many frames we have parsed?
						// Looking for line breaker of '\n'
	for (int i = 0; i < BytesReceived; ++i) {
		if (Data[i] == '\n') {
			// copy the data back.
			FGenericPlatformMemory::Memmove(reinterpret_cast<void*>(buffer), reinterpret_cast<const void*>(Data + previous), i - previous);
			// Add a zero.
			buffer[i - previous] = 0;

			//JSON String
			FString FJSONData(ANSI_TO_TCHAR(reinterpret_cast<const char*>(buffer)));
			std::string JSONData(TCHAR_TO_UTF8(*FJSONData));
			UE_LOG(LogTemp, Warning, TEXT("json data: %s"), *FString(JSONData.c_str()));

			Document doc;
			doc.Parse(JSONData.c_str());
			int ROSExecResult = doc["ExecResult"].GetInt();

			ABaseRobot* BaseRobot = Name2RobotMap[TCHAR_TO_UTF8(*baxterName)];
			if (BaseRobot) {
				if (ROSExecResult != -1) {
					LastROSExecResult = ROSExecResult;
				}
				else {
					BaseRobot->RecvRobotData(JSONData);
				}
			}

			// Consume everything including the '\n'.
			consumed = i + 1;

			// Reset the previous indexer.
			previous = i + 1;
		}
	}

	return consumed;
}

int UDataReciverComponent::GetLastROSExecResult() 
{
	return LastROSExecResult;
}

void UDataReciverComponent::ClearLastROSExecResult()
{
	LastROSExecResult = -1;
}