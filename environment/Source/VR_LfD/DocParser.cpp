// Fill out your copyright notice in the Description page of Project Settings.

#include "DocParser.h"


// Add default functionality here for any IDocParser functions that are not pure virtual.
Document IDocParser::ParsedDoc(const Value &obj)
{
	Document d;
	StringBuffer buf;
	Writer<StringBuffer> writer(buf);

	obj.Accept(writer);
	std::string objStr = buf.GetString();
	d.Parse(objStr.c_str());

	return d;
}

FQuat IDocParser::QuatMaker(const Value &quat)
{
	Document d = ParsedDoc(quat);
	double x = d["X"].GetDouble();
	double y = d["Y"].GetDouble();
	double z = d["Z"].GetDouble();
	double w = d["W"].GetDouble();

	return FQuat(x, y, z, w);
}

FVector IDocParser::VectorMaker(const Value &vector)
{
	Document d = ParsedDoc(vector);
	double x = d["X"].GetDouble();
	double y = d["Y"].GetDouble();
	double z = d["Z"].GetDouble();

	return FVector(x, y, z);
}