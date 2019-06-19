// Fill out your copyright notice in the Description page of Project Settings.
//
// Help:
//    http://api.unrealengine.com/INT/API/Runtime/Core/Math/FVector/
//    http://api.unrealengine.com/INT/API/Runtime/Core/Math/FRotator/
//    https://docs.unrealengine.com/en-US/Programming/UnrealArchitecture/TArrays/
//

#include "AffineLie.h"

#include <sstream>


#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

typedef Eigen::Matrix<double, 3, 3> Matrix33d;
typedef Eigen::Matrix<double, 4, 4> Matrix44d;

Matrix44d algebraTranslationMatrix(double x, double y, double z) {
	Matrix44d m;
	m(0, 0) =  1; m(0, 1) =  0; m(0, 2) =  0; m(0, 3) =  x;
	m(1, 0) =  0; m(1, 1) =  1; m(1, 2) =  0; m(1, 3) =  y;
	m(2, 0) =  0; m(2, 1) =  0; m(2, 2) =  1; m(2, 3) =  z;
	m(3, 0) =  0; m(3, 1) =  0; m(3, 2) =  0; m(3, 3) =  1;
	return m;
}

Matrix33d algebraRotationMatrix(double x, double y, double z) {
	Matrix33d m;
	m(0, 0) =  0; m(0, 1) =  z; m(0, 2) =  y;
	m(1, 0) = -z; m(1, 1) =  0; m(1, 2) =  x;
	m(2, 0) = -y; m(2, 1) = -x; m(2, 2) =  0;
	return m;
}

Matrix33d algebraScaleMatrix(double x, double y, double z) {
	Matrix33d m;
	m(0, 0) =  x; m(0, 1) =  0; m(0, 2) =  0;
	m(1, 0) =  0; m(1, 1) =  y; m(1, 2) =  0;
	m(2, 0) =  0; m(2, 1) =  0; m(2, 2) =  z;
	return m;
}

Matrix44d asHomogenous4x4(Matrix33d m3x3) {
	Matrix44d m4x4;
	int r, c;

	// Copy in values from 3x3
	for (r = 0; r < 3; r++) {
		for (c = 0; c < 3; c++) {
			m4x4(r, c) = m3x3(r, c);
		}
	}

	// Set zeros and homogenous coordinate
	for (r = 0; r < 3; r++) { m4x4(r, 3) = 0; }
	for (c = 0; c < 3; c++) { m4x4(3, c) = 0; }
	m4x4(3, 3) = 1;
	return m4x4;
}

Matrix33d asLinear3x3(Matrix44d m4x4) {
	Matrix33d m3x3;
	int r, c;

	// Copy in values from 3x3
	for (r = 0; r < 3; r++) {
		for (c = 0; c < 3; c++) {
			m3x3(r, c) = m4x4(r, c);
		}
	}

	return m3x3;
}

Matrix44d affExp(Matrix44d L, Matrix33d X, Matrix33d Y) {
	return L * asHomogenous4x4(X.exp() * Y.exp());
}

void affLog(Matrix44d aff, Matrix44d &L, Matrix33d &X, Matrix33d &Y) {
	Matrix33d A = asLinear3x3(aff);
	Matrix33d S = (A * A).sqrt();
	L = algebraTranslationMatrix(aff(0, 3), aff(1, 3), aff(2, 3));
	X = (A * S.inverse()).log();
	Y = S.log();
}

void UAffineLie::AffineExponentialMap(const FVector translation, const FRotator rotation, const FVector scale, FMatrix& affine) {
	Matrix44d L = algebraTranslationMatrix(translation.X, translation.Y, translation.Z);
	Matrix33d X = algebraRotationMatrix(rotation.Roll, rotation.Pitch, rotation.Yaw);
	Matrix33d Y = algebraScaleMatrix(scale.X, scale.Y, scale.Z);
	Matrix44d aff = affExp(L, X, Y);

	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			affine.M[r][c] = aff(r, c);
		}
	}
} 

void UAffineLie::AffineToTRS(const FMatrix affine, FVector& translation, FRotator& rotation, FVector& scale) {
	FMatrix m = affine.GetTransposed();
	translation = m.GetOrigin();
	rotation = m.Rotator();
	scale = m.GetScaleVector();
}

void UAffineLie::AlgebraToAffine(TArray<float> alg, FMatrix& affine) {
	Matrix44d L = algebraTranslationMatrix(alg[0], alg[1], alg[2]);
	Matrix33d X = algebraRotationMatrix(alg[3], alg[4], alg[5]);
	Matrix33d Y = algebraScaleMatrix(alg[6], alg[7], alg[8]);
	Matrix44d aff = affExp(L, X, Y);

	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			affine.M[r][c] = aff(r, c);
		}
	}
}

void UAffineLie::TRSToAlgebra(const FVector translation, const FRotator rotation, const FVector scale, TArray<float>& alg) {
	alg.Empty();
	alg.Add(translation.X);
	alg.Add(translation.Y);
	alg.Add(translation.Z);
	alg.Add(rotation.Roll);
	alg.Add(rotation.Pitch);
	alg.Add(rotation.Yaw);
	alg.Add(scale.X);
	alg.Add(scale.Y);
	alg.Add(scale.Z);
}

void UAffineLie::InterpolateAlgebra(const TArray<float> algA, const TArray<float> algB, const float u, TArray<float>& algOut) {
	algOut.Empty();
	for (int i = 0; i < 9; i++) {
		algOut.Add(algA[i] + (algB[i] - algA[i]) * u);
	}
}