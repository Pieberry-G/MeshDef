#pragma once

#include "EditMesh.h"

#include <Eigen/Dense>
#include <Eigen/SparseLU>
#include <Eigen/SparseCholesky>
#include <Eigen/IterativeLinearSolvers>

//#include <tbb/tbb.h>

#include <ctime>
#include <set>
#include <array>

namespace MeshDef {

enum LinSolverType
{
	LS_LU,
	LS_CHOL
};

class MeshDeformer3D
{
protected: // data
	EditMesh& mesh;
	std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> oldVertCoord;
	std::vector<std::set<size_t>> vvNeighbor;
	std::vector<double> HECot;
	std::vector<double> HETan;
	std::map<std::pair<size_t, size_t>, size_t> endPoint2HE;
	std::vector<std::array<size_t, 3>> triVertInd;

	double constraintCoef;
	LinSolverType linSolverType;
	Eigen::SparseLU<Eigen::SparseMatrix<double>> LUSolver;
	Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> cholSolver;
	std::vector<Eigen::Matrix3d> validTrans;
	Eigen::VectorXd RHS[3];

	Eigen::SparseMatrix<double> W;

public: // data access
	const Eigen::Vector3d& getOldVertCoord(size_t i) const { assert(i < oldVertCoord.size()); return oldVertCoord[i]; }
	const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& getOldVertCoord(void) const { return oldVertCoord; }

protected: // protected data access
	double getHECot(size_t v0I, size_t v1I) const 
	{ 
		auto HEFinder = endPoint2HE.find(std::pair<size_t, size_t>(v0I, v1I));
		return HEFinder != endPoint2HE.end() ? HECot[HEFinder->second] : 0.0;
	}

	double getHETan(size_t v0I, size_t v1I) const 
	{ 
		auto HEFinder = endPoint2HE.find(std::pair<size_t, size_t>(v0I, v1I));
		return HEFinder != endPoint2HE.end() ? HETan[HEFinder->second] : 0.0;
	}

	Eigen::Matrix3d getValidTrans(size_t v0I, size_t v1I) const
	{
		auto HEFinder = endPoint2HE.find(std::pair<size_t, size_t>(v0I, v1I));
		return HEFinder != endPoint2HE.end() ? validTrans[mesh.m_heData[HEFinder->second].face] : Eigen::Matrix3d::Identity();
	}

public: // constructor
	MeshDeformer3D(EditMesh& p_mesh, LinSolverType p_linSolverType = LS_CHOL, double p_constraintCoef = 128.0) :
		mesh(p_mesh), linSolverType(p_linSolverType), constraintCoef(p_constraintCoef), oldVertCoord(mesh.m_vertices)
	{
		updateMeshTopo();
	}

	void updateMeshTopo(void)
	{
		HECot.resize(mesh.m_heData.size());
		HETan.resize(mesh.m_heData.size());
		endPoint2HE.clear();
		vvNeighbor.resize(0);
		vvNeighbor.resize(mesh.m_vertData.size());
		triVertInd.resize(mesh.m_faceData.size());
		size_t triI = 0;
		for (auto triHEIter = mesh.m_faceData.begin(); triHEIter != mesh.m_faceData.end(); triHEIter++, triI++)
		{
			size_t he0I = *triHEIter;
			const half_edge* HEPtr = &mesh.m_heData[he0I];
			size_t v0I = HEPtr->vert;
			size_t he1I = HEPtr->next;
			HEPtr = &mesh.m_heData[he1I];
			size_t v1I = HEPtr->vert;
			size_t he2I = HEPtr->next;
			HEPtr = &mesh.m_heData[he2I];
			size_t v2I = HEPtr->vert;
			assert(HEPtr->next == he0I);

			triVertInd[triI][0] = v0I;
			triVertInd[triI][1] = v1I;
			triVertInd[triI][2] = v2I;

			vvNeighbor[v0I].insert(v1I);
			vvNeighbor[v0I].insert(v2I);
			vvNeighbor[v1I].insert(v0I);
			vvNeighbor[v1I].insert(v2I);
			vvNeighbor[v2I].insert(v0I);
			vvNeighbor[v2I].insert(v1I);

			endPoint2HE[std::pair<size_t, size_t>(v0I, v1I)] = he0I;
			endPoint2HE[std::pair<size_t, size_t>(v1I, v2I)] = he1I;
			endPoint2HE[std::pair<size_t, size_t>(v2I, v0I)] = he2I;

			Eigen::Vector3d e01 = oldVertCoord[v1I] - oldVertCoord[v0I];
			Eigen::Vector3d e12 = oldVertCoord[v2I] - oldVertCoord[v1I];
			Eigen::Vector3d e20 = oldVertCoord[v0I] - oldVertCoord[v2I];
			double e01sql = e01.squaredNorm();
			double e12sql = e12.squaredNorm();
			double e20sql = e20.squaredNorm();
			double dot0102 = -e01.dot(e20);
			double dot1210 = -e12.dot(e01);
			double dot2021 = -e20.dot(e12);
			HECot[he0I] = dot2021 / sqrt(e20sql*e12sql - dot2021*dot2021);
			HECot[he1I] = dot0102 / sqrt(e01sql*e20sql - dot0102*dot0102);
			HECot[he2I] = dot1210 / sqrt(e12sql*e01sql - dot1210*dot1210);

			double cos0102 = dot0102/(e01.norm()*e20.norm());
			double cos1210 = dot1210/(e01.norm()*e12.norm());
			double cos2021 = dot2021/(e12.norm()*e20.norm());
			HETan[he0I] = sqrt(1.0 - cos0102*cos0102) / (1.0 + cos0102);
			HETan[he1I] = sqrt(1.0 - cos1210*cos1210) / (1.0 + cos1210);
			HETan[he2I] = sqrt(1.0 - cos2021*cos2021) / (1.0 + cos2021);
		}
	}

	void makeCurrentAsInitial(const std::map<size_t, Eigen::Vector3d>& constraint)
	{
		oldVertCoord = mesh.m_vertices;
		updateMeshTopo();
		buildLinsysAndDecompose(constraint);
	}

public: // operations
	void localGlobalSolve_e(const std::map<size_t, Eigen::Vector3d>& constraint,
		double targetResidual = 1e-6, int maxIter = 100)
	{
		clock_t start = clock();
		if (validTrans.empty()) { validTrans.resize(oldVertCoord.size(), Eigen::Matrix3d::Identity()); }
		double residual;
		double lastRes = 0.0;
		std::vector<double> residuals;
		int i = 0;
		for( ; i < maxIter; i++)
		{
			residual = getRHSAndSolve(constraint);
			if((residual < targetResidual) ||
				(abs(lastRes-residual)/residual < targetResidual)) { break; }

			lastRes = residual;
			residuals.emplace_back(residual);
		}
		printf("Iterated %d times for %.3f seconds, convergence residual = %.6le\n", 
			i, float(clock()-start)/CLOCKS_PER_SEC, residual);
	}

	void buildLinsysAndDecompose(const std::map<size_t, Eigen::Vector3d>& constraint)
	{
		Eigen::SparseMatrix<double> coefMtr(oldVertCoord.size(), oldVertCoord.size());
		Eigen::VectorXi reserveSizes(oldVertCoord.size());
		for(size_t rowI = 0; rowI < reserveSizes.size(); rowI++)
		{
			reserveSizes[rowI] = vvNeighbor[rowI].size() + 1;
			assert(reserveSizes[rowI] > 2);
		}
		coefMtr.reserve(reserveSizes);
		for(size_t rowI = 0; rowI < coefMtr.outerSize(); rowI++)
		{
			size_t vertI = rowI;
			double diagVal = 0.0;
			for(auto nbIter = vvNeighbor[vertI].begin(); nbIter != vvNeighbor[vertI].end(); nbIter++)
			{
				double weight = getHETan(vertI, *nbIter) + getHETan(*nbIter, vertI);
				diagVal += weight;
				coefMtr.insert(rowI, *nbIter) = -weight;
			}
			coefMtr.insert(rowI, rowI) = diagVal;
		}
		for (auto consIter = constraint.begin(); consIter != constraint.end(); consIter++)
		{
			coefMtr.coeffRef(consIter->first, consIter->first) += constraintCoef;
		}
		coefMtr.makeCompressed();

		switch(linSolverType)
		{
		case LS_LU:
			LUSolver.analyzePattern(coefMtr);
			LUSolver.factorize(coefMtr);
			break;

		case LS_CHOL:
			cholSolver.analyzePattern(coefMtr);
			cholSolver.factorize(coefMtr);
			break;
		}
	}

	void initValidTrans(void)
	{
		validTrans.resize(0);
		validTrans.resize(oldVertCoord.size(), Eigen::Matrix3d::Identity());
	}

	// local solve
	void updateValidTrans(void)
	{
		//tbb::parallel_for(0, (int)oldVertCoord.size(), 1, [&](int rowI)
		for(int rowI = 0; rowI < (int)oldVertCoord.size(); rowI++)
		{
			Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
			size_t vertI = rowI;
			for(auto nbIter = vvNeighbor[vertI].begin(); nbIter != vvNeighbor[vertI].end(); nbIter++)
			{
				Eigen::Vector3d u(mesh.m_vertices[*nbIter] - mesh.m_vertices[vertI]);
				Eigen::Vector3d v(oldVertCoord[*nbIter] - oldVertCoord[vertI]);
				J += (getHETan(vertI, *nbIter) + getHETan(*nbIter, vertI)) * u * v.transpose();
			}

			Eigen::JacobiSVD<Eigen::Matrix3d> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
			validTrans[rowI] = svd.matrixU() * svd.matrixV().transpose();
		}
	}

	// global solve
	double getRHSAndSolve(const std::map<size_t, Eigen::Vector3d>& constraint)
	{
		RHS[0].resize(oldVertCoord.size());
		RHS[1].resize(oldVertCoord.size());
		RHS[2].resize(oldVertCoord.size());
		//tbb::parallel_for(0, (int)oldVertCoord.size(), 1, [&](int rowI)
		for (int rowI = 0; rowI < (int)oldVertCoord.size(); rowI++)
		{
			size_t vertI = rowI;
			RHS[0][rowI] = RHS[1][rowI] = RHS[2][rowI] = 0.0;
			for(auto nbIter = vvNeighbor[vertI].begin(); nbIter != vvNeighbor[vertI].end(); nbIter++)
			{
				const Eigen::Vector3d edgeVec(oldVertCoord[vertI]-oldVertCoord[*nbIter]);
				const Eigen::Vector3d rHSVec = (getHETan(vertI, *nbIter) + getHETan(*nbIter, vertI)) / 2.0 * 
					(validTrans[vertI] + validTrans[*nbIter]) * edgeVec;
				RHS[0][rowI] += rHSVec[0];
				RHS[1][rowI] += rHSVec[1];
				RHS[2][rowI] += rHSVec[2];
			}
		}
		for (auto consIter = constraint.begin(); consIter != constraint.end(); consIter++)
		{
			RHS[0][consIter->first] += constraintCoef * consIter->second[0];
			RHS[1][consIter->first] += constraintCoef * consIter->second[1];
			RHS[2][consIter->first] += constraintCoef * consIter->second[2];
		}

		clock_t start = clock();
		Eigen::VectorXd newRHS[3];

		// solve
		start = clock();
		Eigen::VectorXd solved[3];
		switch(linSolverType)
		{
		case LS_LU:
			solved[0] = LUSolver.solve(RHS[0]);
			solved[1] = LUSolver.solve(RHS[1]);
			solved[2] = LUSolver.solve(RHS[2]);
			break;

		case LS_CHOL:
			solved[0] = cholSolver.solve(RHS[0]);
			solved[1] = cholSolver.solve(RHS[1]);
			solved[2] = cholSolver.solve(RHS[2]);
			break;
		}

		double change = 0.0;
		for(int rowI = 0; rowI < oldVertCoord.size(); rowI++)
		{
			size_t vertI = rowI;
			change += abs(mesh.m_vertices[vertI][0] - solved[0][rowI]);
			change += abs(mesh.m_vertices[vertI][1] - solved[1][rowI]);
			change += abs(mesh.m_vertices[vertI][2] - solved[2][rowI]);
			mesh.m_vertices[vertI][0] = solved[0][rowI];
			mesh.m_vertices[vertI][1] = solved[1][rowI];
			mesh.m_vertices[vertI][2] = solved[2][rowI];
		}

		updateValidTrans();

		return change / oldVertCoord.size();
	}


	////////////////////////////////////////////////////////////////////////
	// Nonlinear Mesh Deformation Solver

	Eigen::VectorXd gradPsi;
	double vhat, g;
	Eigen::SparseMatrix<double> L;
	Eigen::SparseMatrix<double> Jdeltahat;
	Eigen::VectorXd delta, deltahat;
	std::vector<double> gammahat;
	std::vector<std::vector<size_t>> incTris;
	std::vector<Eigen::VectorXd> mu;
	Eigen::SparseMatrix<double> Phi;
	Eigen::VectorXd posVhat;
	Eigen::VectorXd X, f;
	Eigen::SparseMatrix<double> Jf;
	int iterNum;
	FILE *objValOut;

	void getCrossMtr(const Eigen::Vector3d& v, Eigen::Matrix3d& mtr)
	{
		mtr << 0.0, -v[2], v[1],
			v[2], 0.0, -v[0],
			-v[1], v[0], 0.0;
	}

	void computeGradPsi(void)
	{
		gradPsi.resize(mesh.m_vertices.size() * 3);
		gradPsi.setZero();
		for (auto triIter = triVertInd.begin(); triIter != triVertInd.end(); triIter++)
		{
			const Eigen::Vector3d& xi = mesh.m_vertices[(*triIter)[0]];
			const Eigen::Vector3d& xj = mesh.m_vertices[(*triIter)[1]];
			const Eigen::Vector3d& xk = mesh.m_vertices[(*triIter)[2]];

			Eigen::Matrix3d crossMtr;
			getCrossMtr(-xj, crossMtr);
			gradPsi.block((*triIter)[0] * 3, 0, 3, 1) += (xk.transpose() * crossMtr).transpose();

			getCrossMtr(xi, crossMtr);
			gradPsi.block((*triIter)[1] * 3, 0, 3, 1) += (xk.transpose() * crossMtr).transpose();

			gradPsi.block((*triIter)[2] * 3, 0, 3, 1) += xi.cross(xj);
		}
		gradPsi /= 6.0;
	}

	double computePsi(void)
	{
		double result = 0.0;
		for (auto triIter = triVertInd.begin(); triIter != triVertInd.end(); triIter++)
		{
			const Eigen::Vector3d& xi = mesh.m_vertices[(*triIter)[0]];
			const Eigen::Vector3d& xj = mesh.m_vertices[(*triIter)[1]];
			const Eigen::Vector3d& xk = mesh.m_vertices[(*triIter)[2]];

			result += (xi.cross(xj)).dot(xk);
		}
		result /= 6.0;

		return result;
	}

	void computeGradPsi_finiteDiff(void)
	{
		gradPsi.resize(mesh.m_vertices.size() * 3);

		double psi0 = computePsi();
		double h = 1e-5;
		for (size_t rowI = 0; rowI < gradPsi.size(); rowI++)
		{
			size_t vertI = rowI / 3;
			int dimI = rowI % 3;
			mesh.m_vertices[vertI][dimI] += h;
		
			gradPsi[rowI] = (computePsi() - psi0) / h;

			mesh.m_vertices[vertI][dimI] -= h;
		}
	}

	void computeL(void)
	{
		L = Eigen::SparseMatrix<double>(3 * mesh.m_vertices.size(), 3 * mesh.m_vertices.size());
		size_t reserveSize = 0;
		for (size_t vertI = 0; vertI < mesh.m_vertices.size(); vertI++)
		{
			reserveSize += vvNeighbor[vertI].size() + 1;
		}
		L.reserve(reserveSize * 3);
		for (size_t vertI = 0; vertI < mesh.m_vertices.size(); vertI++)
		{
			size_t rowIStart = 3 * vertI;
			double weightSum = 0.0;
			for (auto nbIter = vvNeighbor[vertI].begin(); nbIter != vvNeighbor[vertI].end(); nbIter++)
			{
				double weight = getHETan(vertI, *nbIter) + getHETan(*nbIter, vertI);
				weightSum += weight;

				size_t colIStart = 3 * *nbIter;
				L.insert(rowIStart, colIStart) = -weight;
				L.insert(rowIStart + 1, colIStart + 1) = -weight;
				L.insert(rowIStart + 2, colIStart + 2) = -weight;
			}

			// normalize weights
			for (auto nbIter = vvNeighbor[vertI].begin(); nbIter != vvNeighbor[vertI].end(); nbIter++)
			{
				size_t colIStart = 3 * *nbIter;
				L.coeffRef(rowIStart, colIStart) /= weightSum;
				L.coeffRef(rowIStart + 1, colIStart + 1) /= weightSum;
				L.coeffRef(rowIStart + 2, colIStart + 2) /= weightSum;
			}

			L.insert(rowIStart, rowIStart) = 1.0;
			L.insert(rowIStart + 1, rowIStart + 1) = 1.0;
			L.insert(rowIStart + 2, rowIStart + 2) = 1.0;
		}
		L.makeCompressed();
	}

	void computeIncTris(void)
	{
		incTris.resize(0);
		incTris.resize(mesh.m_vertices.size());
		for (size_t vertI = 0; vertI < mesh.m_vertData.size(); vertI++)
		{
			const half_edge *hePtr = &mesh.m_heData[mesh.m_vertData[vertI]];

			incTris[vertI].emplace_back(hePtr->face);
			hePtr = &mesh.m_heData[mesh.m_heData[mesh.m_heData[hePtr->next].next].twin];
			while (hePtr != &mesh.m_heData[mesh.m_vertData[vertI]])
			{
				incTris[vertI].emplace_back(hePtr->face);
				hePtr = &mesh.m_heData[mesh.m_heData[mesh.m_heData[hePtr->next].next].twin];
			}
		}
	}

	void computeMu(void)
	{
		assert(incTris.size() == mesh.m_vertices.size());
		assert(L.rows() == mesh.m_vertices.size() * 3);

		mu.resize(mesh.m_vertices.size());
		for (size_t vertI = 0; vertI < mesh.m_vertices.size(); vertI++)
		{
			// compute normals of incident triangles
			Eigen::MatrixXd normals(3, incTris[vertI].size());
			for (int incTriI = 0; incTriI < incTris[vertI].size(); incTriI++)
			{
				const auto& vInd = triVertInd[incTris[vertI][incTriI]];
				const Eigen::Vector3d& v0 = mesh.m_vertices[vInd[0]];
				const Eigen::Vector3d& v1 = mesh.m_vertices[vInd[1]];
				const Eigen::Vector3d& v2 = mesh.m_vertices[vInd[2]];
				
				const Eigen::Vector3d e01 = v1 - v0;
				const Eigen::Vector3d e02 = v2 - v0;

				normals.block(0, incTriI, 3, 1) = e01.cross(e02);
			}

			// compute laplacian
			Eigen::Vector3d lapVal = mesh.m_vertices[vertI];
			size_t rowI = vertI * 3;
			for (auto nbIter = vvNeighbor[vertI].begin(); nbIter != vvNeighbor[vertI].end(); nbIter++)
			{
				size_t colI = 3 * *nbIter;
				lapVal += L.coeffRef(rowI, colI) * mesh.m_vertices[*nbIter];
			}

			mu[vertI] = normals.fullPivHouseholderQr().solve(lapVal);
		}
	}

	void computeDelta(bool original)
	{
		assert(mu.size() == mesh.m_vertices.size());

		delta.resize(3 * mesh.m_vertices.size());
		for (size_t vertI = 0; vertI < mesh.m_vertices.size(); vertI++)
		{
			// compute normals of incident triangles
			Eigen::Vector3d lapVec = Eigen::Vector3d::Zero();
			for (int incTriI = 0; incTriI < incTris[vertI].size(); incTriI++)
			{
				const auto& vInd = triVertInd[incTris[vertI][incTriI]];
				const Eigen::Vector3d& v0 = mesh.m_vertices[vInd[0]];
				const Eigen::Vector3d& v1 = mesh.m_vertices[vInd[1]];
				const Eigen::Vector3d& v2 = mesh.m_vertices[vInd[2]];

				const Eigen::Vector3d e01 = v1 - v0;
				const Eigen::Vector3d e02 = v2 - v0;

				lapVec += mu[vertI][incTriI] * e01.cross(e02);
			}
			delta.block(vertI * 3, 0, 3, 1) = lapVec;
		}

		if (original)
		{
			gammahat.resize(mesh.m_vertices.size());
			for (size_t vertI = 0; vertI < mesh.m_vertices.size(); vertI++)
			{
				gammahat[vertI] = delta.block(vertI * 3, 0, 3, 1).norm();
			}
		}
		else
		{
			assert(gammahat.size() == mesh.m_vertices.size());

			deltahat.resize(mesh.m_vertices.size() * 3);
			for (size_t vertI = 0; vertI < mesh.m_vertices.size(); vertI++)
			{
				const Eigen::Vector3d &deltaI = delta.block(vertI * 3, 0, 3, 1);
				deltahat.block(vertI * 3, 0, 3, 1) = deltaI.normalized() * gammahat[vertI];
			}
		}
	}

	void computeJdeltahat(void)
	{
		assert(incTris.size() == mesh.m_vertices.size());

		Eigen::SparseMatrix<double> JdX(mesh.m_vertices.size() * 3, mesh.m_vertices.size() * 3);
		JdX.reserve(7 * 9 * mesh.m_vertices.size());
		for (size_t vertI = 0; vertI < mesh.m_vertices.size(); vertI++)
		{
			for (int incTriI = 0; incTriI < incTris[vertI].size(); incTriI++)
			{
				const auto& vInd = triVertInd[incTris[vertI][incTriI]];
				Eigen::Matrix3d v0Cross; getCrossMtr(mesh.m_vertices[vInd[0]], v0Cross);
				Eigen::Matrix3d v1Cross; getCrossMtr(mesh.m_vertices[vInd[1]], v1Cross);
				Eigen::Matrix3d v2Cross; getCrossMtr(mesh.m_vertices[vInd[2]], v2Cross);
				Eigen::Matrix3d derivs[3] = {
					mu[vertI][incTriI] * (v2Cross - v1Cross),
					mu[vertI][incTriI] * (v0Cross - v2Cross),
					mu[vertI][incTriI] * (v1Cross - v0Cross)
				};
				size_t rowIStart = vertI * 3;
				for (int dimI = 0; dimI < 3; dimI++)
				{
					size_t colIStart = vInd[dimI] * 3;
					for (int localRowI = 0; localRowI < 3; localRowI++)
					{
						for (int localColI = 0; localColI < 3; localColI++)
						{
							JdX.coeffRef(rowIStart + localRowI, colIStart + localColI) += 
								derivs[dimI](localRowI, localColI);
						}
					}
				}
			}
		}
		JdX.makeCompressed();

		computeDelta(false);
		Eigen::SparseMatrix<double> JdeltahatD(mesh.m_vertices.size() * 3, mesh.m_vertices.size() * 3);
		JdeltahatD.reserve(9 * mesh.m_vertices.size());
		for (size_t vertI = 0; vertI < mesh.m_vertices.size(); vertI++)
		{
			size_t rowIStart = vertI * 3;
			double x = delta[rowIStart];
			double y = delta[rowIStart + 1];
			double z = delta[rowIStart + 2];

			double x2 = x * x, y2 = y * y, z2 = z * z;
			double scale = gammahat[vertI] / std::pow(x2 + y2 + z2, 1.5);

			JdeltahatD.insert(rowIStart, rowIStart) = scale * (y2 + z2);
			JdeltahatD.insert(rowIStart, rowIStart + 1) = 
				JdeltahatD.insert(rowIStart + 1, rowIStart) = scale * (-x * y);
			JdeltahatD.insert(rowIStart, rowIStart + 2) = 
				JdeltahatD.insert(rowIStart + 2, rowIStart) = scale * (-x * z);
			JdeltahatD.insert(rowIStart + 1, rowIStart + 1) = scale * (x2 + z2);
			JdeltahatD.insert(rowIStart + 1, rowIStart + 2) =
				JdeltahatD.insert(rowIStart + 2, rowIStart + 1) = scale * (-y * z);
			JdeltahatD.insert(rowIStart + 2, rowIStart + 2) = scale * (x2 + y2);
		}
		JdeltahatD.makeCompressed();

		Jdeltahat = JdeltahatD * JdX;
	}

	void computeJdeltahat_finiteDiff(void)
	{
		computeDelta(true);
		const Eigen::VectorXd deltahat0 = deltahat;

		Jdeltahat = Eigen::SparseMatrix<double>(mesh.m_vertices.size() * 3, mesh.m_vertices.size() * 3);
		Jdeltahat.reserve(7 * 9 * mesh.m_vertices.size());
		double h = 1e-5;
		double epsilon = 1e-10;
		for (size_t colI = 0; colI < Jdeltahat.cols(); colI++)
		{
			size_t vertI = colI / 3;
			int dimI = colI % 3;
			mesh.m_vertices[vertI][dimI] += h;

			computeDelta(false);
			const Eigen::VectorXd colIVec = (deltahat - deltahat0) / h;
			for (size_t rowI = 0; rowI < colIVec.size(); rowI++)
			{
				if (abs(colIVec[rowI]) > epsilon)
				{
					Jdeltahat.insert(rowI, colI) = colIVec[rowI];
				}
			}

			mesh.m_vertices[vertI][dimI] -= h;
		}
	}

	void computePosConstraint(const std::map<size_t, Eigen::Vector3d>& constraint, double coef = 1.0)
	{
		Phi = Eigen::SparseMatrix<double>(constraint.size() * 3, mesh.m_vertices.size() * 3);
		Phi.reserve(constraint.size() * 3);
		posVhat.resize(constraint.size() * 3);
		int consI = 0;
		for (auto consIter = constraint.begin(); consIter != constraint.end(); consIter++, consI++)
		{
			int rowIStart = consI * 3;
			size_t colIStart = consIter->first * 3;

			Phi.insert(rowIStart, colIStart) = coef;
			Phi.insert(rowIStart + 1, colIStart + 1) = coef;
			Phi.insert(rowIStart + 2, colIStart + 2) = coef;

			posVhat[rowIStart] = consIter->second[0] * coef;
			posVhat[rowIStart + 1] = consIter->second[1] * coef;
			posVhat[rowIStart + 2] = consIter->second[2] * coef;
		}
	}

	void computeF(void)
	{
		assert(X.size() == mesh.m_vertices.size() * 3);
		assert(L.cols() == mesh.m_vertices.size() * 3);
		assert(Phi.cols() == mesh.m_vertices.size() * 3);

		f.resize(L.rows() + Phi.rows());
		computeDelta(false);
		f.block(0, 0, L.rows(), 1) = L * X - deltahat;
		f.block(L.rows(), 0, Phi.rows(), 1) = Phi * X - posVhat;
	}

	////////////////////////////////////////////////////////////////////////////////
	// API
	void initNonlinearDeformer(const std::map<size_t, Eigen::Vector3d>& constraint)
	{
		computeL();
		computeIncTris();
		computeMu();
		computeDelta(true);
		computePosConstraint(constraint);
		vhat = computePsi();
		X.resize(mesh.m_vertices.size() * 3);
		for (size_t vertI = 0; vertI < mesh.m_vertices.size(); vertI++)
		{
			size_t rowIStart = vertI * 3;
			X[rowIStart] = mesh.m_vertices[vertI][0];
			X[rowIStart + 1] = mesh.m_vertices[vertI][1];
			X[rowIStart + 2] = mesh.m_vertices[vertI][2];
		}
		iterNum = 0;
		objValOut = fopen("objVals", "w");
		assert(objValOut);
	}

	bool gaussNewtonIter(bool lineSearch = true)
	{
		///////////////////////////////////////////////////////
		// Evaluate variables: Jg, g, f, Jf
		// note that the order of the function calls matter
		printf("\n#%d Gauss-Newton iteration:\n", iterNum);
		printf("\tEvaluating Jg...");
		computeGradPsi(); // Jg

		printf(", g...");
		g = computePsi() - vhat; // g

		// f
		printf(", f...");
		computeJdeltahat();
		f.resize(L.rows() + Phi.rows());
		f.block(0, 0, L.rows(), 1) = L * X - deltahat;
		f.block(L.rows(), 0, Phi.rows(), 1) = Phi * X - posVhat;

		// Jf
		printf(", Jf...");
		Eigen::SparseMatrix<double> Ldiff = L - Jdeltahat;
		Jf.resize(Ldiff.rows() + Phi.rows(), Ldiff.cols());
		Jf.reserve(Ldiff.nonZeros() + Phi.nonZeros());
		for (int k = 0; k<Ldiff.outerSize(); ++k)
		{
			for (Eigen::SparseMatrix<double>::InnerIterator it(Ldiff, k); it; ++it)
			{
				Jf.insert(it.row(), it.col()) = it.value();
			}
		}
		for (int k = 0; k<Phi.outerSize(); ++k)
		{
			for (Eigen::SparseMatrix<double>::InnerIterator it(Phi, k); it; ++it)
			{
				Jf.insert(it.row() + Ldiff.rows(), it.col()) = it.value();
			}
		}
		Jf.makeCompressed();
		printf("done\n");

		////////////////////////////
		// Compute lambda and h
		printf("\tSolving for lamda and h...");
		const Eigen::SparseMatrix<double, Eigen::ColMajor> JfT = Jf.transpose();
		const Eigen::SparseMatrix<double> JfTJf = JfT * Jf;
		Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> JfTJfCholFac;
		JfTJfCholFac.analyzePattern(JfTJf);
		JfTJfCholFac.factorize(JfTJf);
		const Eigen::VectorXd JfTf = JfT * f;
		const double lamda = -(g - gradPsi.transpose() * JfTJfCholFac.solve(JfTf)) /
			(gradPsi.transpose() * JfTJfCholFac.solve(gradPsi));
		const Eigen::VectorXd h = -JfTJfCholFac.solve(JfTf + lamda * gradPsi);
		printf("done\n");

		/////////////////////////////////////////////////////////
		// Backtracing line search with Armijo condition
		printf("\tBacktracking line search...");
		const double c1 = 0.5, c2 = 0.5;
		double alpha = h.norm();
		const Eigen::VectorXd p = h / alpha;
		const double cm = c1 * (JfTf + lamda * gradPsi).dot(p);
		Eigen::VectorXd XSearch;
		double fxapSqrNormHalf;
		const double fSqrNormHalf = f.squaredNorm() * 0.5;
		const double lamdaGX = lamda * (computePsi() - vhat);
		mesh.updateBBox();
		const double XScale = (mesh.bboxMax - mesh.bboxMin).minCoeff();
		assert(XScale > 0.0);
		alpha /= c2;
		do
		{
			alpha *= c2;
			const Eigen::VectorXd dX = alpha * p;
			const double dXAbsMax = max(dX.maxCoeff(), abs(dX.minCoeff()));
			bool converged = false;
			if (dXAbsMax / XScale < 1.0e-4)
			{
				converged = true;
			}

			XSearch = X + dX;
			for (size_t vertI = 0; vertI < mesh.m_vertices.size(); vertI++)
			{
				size_t rowIStart = vertI * 3;
				mesh.m_vertices[vertI][0] = XSearch[rowIStart];
				mesh.m_vertices[vertI][1] = XSearch[rowIStart + 1];
				mesh.m_vertices[vertI][2] = XSearch[rowIStart + 2];
			}
			computeDelta(false);
			fxapSqrNormHalf = (L * XSearch - deltahat).squaredNorm();
			fxapSqrNormHalf += (Phi * XSearch - posVhat).squaredNorm();
			fxapSqrNormHalf *= 0.5;

			if ((fxapSqrNormHalf < 1.0e-10) || converged)
			{
				printf("done\n#%d Gauss-Newton iteration completed, objective = %.3le\n", iterNum++, fxapSqrNormHalf);
				printf("Minimization objective tiny enough, converged.\n");
				X = XSearch;
				fprintf(objValOut, "%le %le\n", fxapSqrNormHalf, abs(computePsi() - vhat) / vhat);
				fclose(objValOut);
				return true;
			}

			if(!lineSearch) { break; }

		} while (fxapSqrNormHalf + lamda * (computePsi() - vhat) > fSqrNormHalf + lamdaGX + alpha * cm);
		X = XSearch;
		printf("done\n");

		printf("#%d Gauss-Newton iteration completed, objective = %.3le\n", iterNum++, fxapSqrNormHalf);
		fprintf(objValOut, "%le %le\n", fxapSqrNormHalf, abs(computePsi() - vhat) / vhat);

		return false;
	}

	bool gaussNewtonIterSub(const Eigen::SparseMatrix<double>& W, bool lineSearch = true)
	{
		///////////////////////////////////////////////////////
		// Evaluate variables: Jg, g, f, Jf
		// note that the order of the function calls matter
		printf("\n#%d Gauss-Newton iteration:\n", iterNum);
		printf("\tEvaluating Jg...");
		computeGradPsi(); // Jg
		gradPsi = W.transpose() * gradPsi; // subspace mapping

		printf(", g...");
		g = computePsi() - vhat; // g

		// f
		printf(", f...");
		computeJdeltahat();
		printf("...");
		f.resize(L.rows() + Phi.rows());
		f.block(0, 0, L.rows(), 1) = L * X - deltahat;
		f.block(L.rows(), 0, Phi.rows(), 1) = Phi * X - posVhat;

		// Jf and subspace projection
		printf(", Jf...");
		const Eigen::SparseMatrix<double> Ldiff = L - Jdeltahat;
		Eigen::MatrixXd JfSubDense(Ldiff.rows() + Phi.rows(), W.cols());
		Eigen::SparseMatrix<double> tempSPM = Ldiff * W;
		JfSubDense.block(0, 0, Ldiff.rows(), W.cols()) = tempSPM;
		tempSPM = Phi * W;
		JfSubDense.block(Ldiff.rows(), 0, Phi.rows(), W.cols()) = tempSPM;
		printf("done\n");

		////////////////////////////
		// Compute lambda and h
		printf("\tSolving for lamda and h...");
		const Eigen::MatrixXd JfSubDenseT = JfSubDense.transpose();
		const Eigen::MatrixXd JfSubDenseTJfSubDense = JfSubDenseT * JfSubDense;
		Eigen::LDLT<Eigen::MatrixXd> denseCholSolver = JfSubDenseTJfSubDense.ldlt();
		const Eigen::VectorXd JfSubDenseTf = JfSubDenseT * f;
		const double lamdaP = -(g - gradPsi.transpose() * denseCholSolver.solve(JfSubDenseTf)) /
			(gradPsi.transpose() * denseCholSolver.solve(gradPsi));
		const Eigen::VectorXd hP = -denseCholSolver.solve(JfSubDenseTf + lamdaP * gradPsi);
		printf("done\n");

		/////////////////////////////////////////////////////////
		// Backtracing line search with Armijo condition
		printf("\tBacktracking line search...");
		const double c1 = 0.5, c2 = 0.5;
		double alpha = hP.norm();
		const Eigen::VectorXd pP = hP / alpha;
		const double cm = c1 * (JfSubDenseTf + lamdaP * gradPsi).dot(pP);
		const Eigen::VectorXd p = W * pP;
		Eigen::VectorXd XSearch;
		double fxapSqrNormHalf;
		const double fSqrNormHalf = f.squaredNorm() * 0.5;
		const double lamdaGWP = lamdaP * (computePsi() - vhat);
		mesh.updateBBox();
		const double XScale = (mesh.bboxMax - mesh.bboxMin).minCoeff();
		assert(XScale > 0.0);
		alpha /= c2;
		do
		{
			alpha *= c2;
			const Eigen::VectorXd dX = alpha * p;
			const double dXAbsMax = max(dX.maxCoeff(), abs(dX.minCoeff()));
			bool converged = false;
			if (dXAbsMax / XScale < 1.0e-4)
			{
				converged = true;
			}

			XSearch = X + dX;
			for (size_t vertI = 0; vertI < mesh.m_vertices.size(); vertI++)
			{
				size_t rowIStart = vertI * 3;
				mesh.m_vertices[vertI][0] = XSearch[rowIStart];
				mesh.m_vertices[vertI][1] = XSearch[rowIStart + 1];
				mesh.m_vertices[vertI][2] = XSearch[rowIStart + 2];
			}
			computeDelta(false);
			fxapSqrNormHalf = (L * XSearch - deltahat).squaredNorm();
			fxapSqrNormHalf += (Phi * XSearch - posVhat).squaredNorm();
			fxapSqrNormHalf *= 0.5;

			if ((fxapSqrNormHalf < 1.0e-10) || converged)
			{
				printf("done\n#%d Gauss-Newton iteration completed, objective = %.3le\n", iterNum++, fxapSqrNormHalf);
				printf("Minimization objective tiny enough, converged.\n");
				X = XSearch;
				const Eigen::VectorXd dP = alpha * pP;
				for (int cvI = 0; cvI < mesh.ctrlMeshPtr->m_vertices.size(); cvI++)
				{
					int indStart = cvI * 3;
					mesh.ctrlMeshPtr->m_vertices[cvI][0] += dP[indStart];
					mesh.ctrlMeshPtr->m_vertices[cvI][1] += dP[indStart + 1];
					mesh.ctrlMeshPtr->m_vertices[cvI][2] += dP[indStart + 2];
				}
				fprintf(objValOut, "%le %le\n", fxapSqrNormHalf, abs(computePsi() - vhat) / vhat);
				fclose(objValOut);
				return true;
			}

			if(!lineSearch) { break; }

		} while (fxapSqrNormHalf + lamdaP * (computePsi() - vhat) > fSqrNormHalf + lamdaGWP + alpha * cm);
		X = XSearch;
		const Eigen::VectorXd dP = alpha * pP;
		for (int cvI = 0; cvI < mesh.ctrlMeshPtr->m_vertices.size(); cvI++)
		{
			int indStart = cvI * 3;
			mesh.ctrlMeshPtr->m_vertices[cvI][0] += dP[indStart];
			mesh.ctrlMeshPtr->m_vertices[cvI][1] += dP[indStart + 1];
			mesh.ctrlMeshPtr->m_vertices[cvI][2] += dP[indStart + 2];
		}
		printf("done\n");

		printf("#%d Gauss-Newton iteration completed, objective = %.3le\n", iterNum++, fxapSqrNormHalf);
		fprintf(objValOut, "%le %le\n", fxapSqrNormHalf, abs(computePsi() - vhat) / vhat);

		return false;
	}

	void saveObjValFile(void)
	{
		assert(objValOut);
		fclose(objValOut);
	}

private: // prevent copy constructor
	MeshDeformer3D(const MeshDeformer3D&) : mesh(EditMesh()) {}
};

} // namespace MeshDef