// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "AffineLie.generated.h"

/**
 * 
 */
UCLASS()
class VERTICALSLICE_API UAffineLie : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

	UFUNCTION(BlueprintCallable, Category = "AffineLie")
	static void AffineExponentialMap(const FVector translation, const FRotator rotation, const FVector scale, FMatrix& affine);

	UFUNCTION(BlueprintCallable, Category = "AffineLie")
	static void AffineToTRS(const FMatrix affine, FVector& translation, FRotator& rotation, FVector& scale);

	UFUNCTION(BlueprintCallable, Category = "AffineLie")
	static void AlgebraToAffine(TArray<float> alg, FMatrix& affine);

	UFUNCTION(BlueprintCallable, Category = "AffineLie")
	static void TRSToAlgebra(const FVector translation, const FRotator rotation, const FVector scale, TArray<float>& alg);

	UFUNCTION(BlueprintCallable, Category = "AffineLie")
	static void InterpolateAlgebra(const TArray<float> algA, const TArray<float> algB, const float u, TArray<float>& algOut);

	
};
