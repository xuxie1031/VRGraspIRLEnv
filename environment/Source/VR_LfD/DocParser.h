// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <string>
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "DocParser.generated.h"

using namespace rapidjson;

// This class does not need to be modified.
UINTERFACE(MinimalAPI)
class UDocParser : public UInterface
{
	GENERATED_BODY()
};

/**
 * 
 */
class VR_LFD_API IDocParser
{
	GENERATED_BODY()

	// Add interface functions to this class. This is the class that will be inherited to implement this interface.
public:
	Document ParsedDoc(const Value &obj);
	FQuat QuatMaker(const Value &quat);
	FVector VectorMaker(const Value &vector);
	
};
