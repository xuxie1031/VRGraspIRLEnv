// Fill out your copyright notice in the Description page of Project Settings.

#include "GripperController.h"
#include "IPAddress.h"
#include "Runtime/Networking/Public/Interfaces/IPv4/IPv4Address.h"


// Sets default values
AGripperController::AGripperController()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AGripperController::BeginPlay()
{
	Super::BeginPlay();

	socketInitialized = false;
	senderConnectStatus = false;
	terminate = false;
	lFingerTouched = false;
	rFingerTouched = false;
	fingerFirstFlipped = false;
	waitForSmoothAction = 0;

	FTimerHandle ReceiverHandler;
	GetWorldTimerManager().SetTimer(ReceiverHandler, this, &AGripperController::CheckAnimQueue, 0.0001f, true);

}

void AGripperController::BeginDestroy()
{
	Super::BeginDestroy();

	SocketDestroy();

}

// Called every frame
void AGripperController::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void AGripperController::InitReciver(int IP_A, int IP_B, int IP_C, int IP_D, int port)
{
	SocketInit(IP_A, IP_B, IP_C, IP_D, port);
	socketInitialized = true;

	BuildListenerConnection();

}

void AGripperController::BuildListenerConnection()
{
	if (ListenerSocket != NULL) {

		TcpListener = new FTcpListener(*ListenerSocket);
		TcpListener->OnConnectionAccepted().BindUObject(this, &AGripperController::OnTcpListenerConnectionAccepted);

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

bool AGripperController::OnTcpListenerConnectionAccepted(FSocket* Socket, const FIPv4Endpoint& Endpoint)
{
	if (Accepting == true) {
		GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red, FString::Printf(TEXT("A new connection")));
		connected = true;

		auto State = Socket->GetConnectionState();

		// Start another thread listening on this connection.
		ConnectionSocket = Socket;
		Receiver = new TCPReceiver(Socket, UB_TCP_BUFFER_SIZE);
		Receiver->OnTcpReceive().BindUObject(this, &AGripperController::OnTcpReceived);
		Receiver->Init();
		return true;
	}
	// Reject the connection.
	return false;
}

int32 AGripperController::OnTcpReceived(const uint8* Data, int32 BytesReceived)
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

			Document incomingJson;
			incomingJson.Parse(JSONData.c_str());
			std::string cmd = incomingJson["cmd"].GetString();

			UE_LOG(LogTemp, Warning, TEXT("cmd: %s"), *FString(cmd.c_str()));

			if (cmd == "reset") {
				smoothActionExecutor->SetPushingState(true);
				smoothActionExecutor->PushAction(2);
				smoothActionExecutor->PushAction(0);
				smoothActionExecutor->PushTF(gripperBaseStartTF);
				smoothActionExecutor->PushTF(lFingerStartTF);
				smoothActionExecutor->PushTF(rFingerStartTF);
				smoothActionExecutor->PushTF(playDeskStartTF);
				smoothActionExecutor->PushTF(grabTargetStartTF);
				smoothActionExecutor->SetPushingState(false);

				waitForSmoothAction = 1;
			}
			else if (cmd == "step") {
				Value stateArray;
				stateArray = incomingJson["state"];

				int intI = 0;
				for (rapidjson::SizeType i = 0; i < stateArray.Size(); i++, intI++) {
					stateAsFloat[intI] = stateArray[i].GetFloat();
				}

				Value actionArray;
				actionArray = incomingJson["action"];

				intI = 0;
				for (rapidjson::SizeType i = 0; i < actionArray.Size(); i++, intI++) {
					actionAsFloat[intI] = actionArray[i].GetFloat();
				}

				int animSteps = 5;
				float baseSetpLen_X = actionAsFloat[0] / animSteps;
				float baseSetpLen_Y = actionAsFloat[1] / animSteps;
				float baseSetpLen_Z = actionAsFloat[2] / animSteps;

				if (lFinger->RelativeLocation.Z + actionAsFloat[3] * 0.01 > 0.05) actionAsFloat[3] = (0.05 - lFinger->RelativeLocation.Z) * 100;
				if (lFinger->RelativeLocation.Z + actionAsFloat[3] * 0.01 < 0.013) actionAsFloat[3] = (0.013 - lFinger->RelativeLocation.Z) * 100;

				float lFingerSetpLen = actionAsFloat[3] * 0.01 / animSteps;
				float rFingerSetpLen = actionAsFloat[3] * -0.01 / animSteps;

				smoothActionExecutor->SetPushingState(true);
				for (int i = 0; i < animSteps; i++) {
					smoothActionExecutor->PushAction(1);
					smoothActionExecutor->PushMovement(FVector(baseSetpLen_X, baseSetpLen_Y, baseSetpLen_Z));
					smoothActionExecutor->PushMovement(FVector(0, 0, lFingerSetpLen));
					smoothActionExecutor->PushMovement(FVector(0, 0, rFingerSetpLen));
				}
				smoothActionExecutor->SetPushingState(false);

				waitForSmoothAction = 2;
			}
			else if (cmd == "grasp_check") {
				smoothActionExecutor->HandleFingerMovementinRFinger(0.013 - lFinger->RelativeLocation.Z, 50);

				waitForSmoothAction = 3;
			}
			else if (cmd == "repeat") {
				Send("repeat");
			}

			// Consume everything including the '\n'.
			consumed = i + 1;

			// Reset the previous indexer.
			previous = i + 1;

		}
	}
	return consumed;
}

bool AGripperController::SenderConn(int IP_A, int IP_B, int IP_C, int IP_D, int port)
{
	Socket = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateSocket(NAME_Stream, TEXT("default"), false);

	FIPv4Address ip(IP_A, IP_B, IP_C, IP_D);
	TSharedRef<FInternetAddr> addr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
	addr->SetIp(ip.Value);
	addr->SetPort(port);

	bool success = Socket->Connect(*addr);
	if (success) GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red, FString::Printf(TEXT("Sender Connected!")));

	senderConnectStatus = success;
	return success;
}

void AGripperController::Send(std::string cmd)
{
	Document jsonDoc;
	Document::AllocatorType &allocator = jsonDoc.GetAllocator();
	jsonDoc.SetObject();

	Value tmpStr(cmd.c_str(), allocator);
	jsonDoc.AddMember("cmd", tmpStr, allocator);

	Value stateArray(rapidjson::kArrayType);


	if (cmd == "reset") {
		stateArray.PushBack(LocRelativeToTarget(0).X, allocator);
		stateArray.PushBack(LocRelativeToTarget(0).Y, allocator);
		stateArray.PushBack(LocRelativeToTarget(0).Z, allocator);
		stateArray.PushBack(LocRelativeToTarget(1).X, allocator);
		stateArray.PushBack(LocRelativeToTarget(1).Y, allocator);
		stateArray.PushBack(LocRelativeToTarget(2).Z, allocator);
		stateArray.PushBack(LocRelativeToTarget(2).X, allocator);
		stateArray.PushBack(LocRelativeToTarget(2).Y, allocator);
		stateArray.PushBack(LocRelativeToTarget(2).Z, allocator);
		jsonDoc.AddMember("state", stateArray, allocator);
	}
	else if (cmd == "step") {
		for (int i = 0; i < 9; i++) {
			stateArray.PushBack(stateAsFloat[i], allocator);
		}
		jsonDoc.AddMember("state", stateArray, allocator);

		Value actionArray(rapidjson::kArrayType);
		for (int i = 0; i < 4; i++) {
			actionArray.PushBack(actionAsFloat[i], allocator);
		}
		jsonDoc.AddMember("action", actionArray, allocator);

		Value nextStateArray(rapidjson::kArrayType);
		nextStateArray.PushBack(LocRelativeToTarget(0).X, allocator);
		nextStateArray.PushBack(LocRelativeToTarget(0).Y, allocator);
		nextStateArray.PushBack(LocRelativeToTarget(0).Z, allocator);
		nextStateArray.PushBack(LocRelativeToTarget(1).X, allocator);
		nextStateArray.PushBack(LocRelativeToTarget(1).Y, allocator);
		nextStateArray.PushBack(LocRelativeToTarget(1).Z, allocator);
		nextStateArray.PushBack(LocRelativeToTarget(2).X, allocator);
		nextStateArray.PushBack(LocRelativeToTarget(2).Y, allocator);
		nextStateArray.PushBack(LocRelativeToTarget(2).Z, allocator);
		jsonDoc.AddMember("next_state", nextStateArray, allocator);

		jsonDoc.AddMember("terminal", terminate, allocator);

		int flag = 0;
		if (fingerFirstFlipped) {
			flag = 1;
			fingerFirstFlipped = false;
		}
		jsonDoc.AddMember("flag", flag, allocator);
	}
	else if (cmd == "grasp_check") {
		int flag = 0;
		if (grabTarget->GetComponentLocation().Z - grabTargetStartTF.GetLocation().Z > 3) flag = 1;
		else flag = -1;
		jsonDoc.AddMember("flag", flag, allocator);
	}

	std::string strJson;

	if (cmd == "repeat") strJson = lastOutJsonStr;
	else {
		StringBuffer buffer;
		Writer<StringBuffer> writer(buffer);
		jsonDoc.Accept(writer);
		strJson = buffer.GetString();
		lastOutJsonStr = strJson;
	}

	FString content(strJson.c_str());
	content += '\n';
	TCHAR *serializedChar = content.GetCharArray().GetData();
	int32 size = FCString::Strlen(serializedChar);
	int32 sent = 0;

	bool result = Socket->Send((uint8*)TCHAR_TO_UTF8(serializedChar), size, sent);
}

void AGripperController::Reset()
{
	actionCnt = 0;
	terminate = false;
	lFingerTouched = false;
	rFingerTouched = false;
	fingerFirstFlipped = false;
}

void AGripperController::CheckAnimQueue()
{
	if (waitForSmoothAction) {
		if (terminate && waitForSmoothAction == 2) {
			smoothActionExecutor->ClearQueue();
		}
		else if (terminate && waitForSmoothAction == 3) {
			if (lFingerTouchedName == rFingerTouchedName && lFingerTouchedName != "")
				smoothActionExecutor->ClearQueue();
		}
		if (smoothActionExecutor->QueueIsEmpty()) {
			if (waitForSmoothAction == 1) {
				actionCnt = 0;
				terminate = false;
				lFingerTouched = false;
				rFingerTouched = false;
				fingerFirstFlipped = false;
				Send("reset");
			}
			else if (waitForSmoothAction == 2) {
				Send("step");
				actionCnt++;
			}
			else if (waitForSmoothAction == 3) {
				smoothActionExecutor->SetPushingState(true);
				for (int i = 0; i < 20; i++) {
					smoothActionExecutor->PushAction(1);
					smoothActionExecutor->PushMovement(FVector(0, 0, 0.25));
					smoothActionExecutor->PushMovement(FVector(0, 0, 0));
					smoothActionExecutor->PushMovement(FVector(0, 0, 0));
				}
				smoothActionExecutor->SetPushingState(false);

				waitForSmoothAction = 4;
				return;
			}
			else if (waitForSmoothAction == 4) {
				Send("grasp_check");
			}
			waitForSmoothAction = 0;
		}
	}
}

void AGripperController::Init(UStaticMeshComponent* inPlayDesk,
	UStaticMeshComponent* inGrabTarget,
	UStaticMeshComponent* inGripperBase,
	UStaticMeshComponent* inLFinger,
	UStaticMeshComponent* inRFinger,
	ASmoothActionExecuteActor* inExecutor)
{
	playDesk = inPlayDesk;
	grabTarget = inGrabTarget;
	gripperBase = inGripperBase;
	lFinger = inLFinger;
	rFinger = inRFinger;

	playDeskStartTF = playDesk->GetComponentTransform();
	grabTargetStartTF = grabTarget->GetComponentTransform();

	gripperBaseStartTF = gripperBase->GetComponentTransform();
	lFingerStartTF = lFinger->GetComponentTransform();
	rFingerStartTF = rFinger->GetComponentTransform();

	smoothActionExecutor = inExecutor;
}

FVector AGripperController::LocRelativeToTarget(int meshNo)
{
	UStaticMeshComponent* thisMesh = gripperBase;
	if (meshNo == 1) thisMesh = lFinger;
	else if (meshNo == 2) thisMesh = rFinger;

	FVector ret = thisMesh->GetComponentLocation() - grabTargetStartTF.GetLocation();
	return ret;
}
