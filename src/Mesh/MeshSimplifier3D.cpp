#include "Mesh/MeshSimplifier3D.h"

#include "Mesh/SelfIntersections.h"

#include <OsiClpSolverInterface.hpp>
#include <CoinPackedVector.hpp>
#include <CoinPackedMatrix.hpp>

namespace MeshDef {

namespace detail {
	void delete_face1( std::vector<std::size_t>& faceData, std::vector<half_edge>& heData, std::size_t f )
	{
		assert( f < faceData.size() );

		// In order to delete the face properly, we need to move a face from the end of the list to overwrite 'f'. Then we need to update the 
		// indices stored in the moved face's half-edges.
		faceData[f] = faceData.back();
		faceData.pop_back();

		if (f != faceData.size())
		{
			//std::clog << "Reindexed face " << faceData.size() << " to " << f << std::endl;

			std::size_t he = faceData[f];
			do {
				assert( heData[he].face == faceData.size() );

				heData[he].face = f;
				he = heData[he].next;
			} while( he != faceData[f] );
		}
	}

	template <int N>
	void delete_faces1(std::vector<std::size_t>& faceData, std::vector<half_edge>& heData, std::size_t (&fToDelete)[N])
	{
		// Sort the faces by decreasing index so that we can safely delete them all without causing any of them to be accidentally re-indexed (which
		// cause 'fToDelete' to contain invalid indices). This also chooses the optimal deletion order to minimize re-indexing.
		std::sort(fToDelete, fToDelete + N, std::greater<std::size_t>());
		for( std::size_t i = 0; i < N; ++i )
		{
			detail::delete_face1(faceData, heData, fToDelete[i]);
		}
	}
} // namespace detail

MeshSimplifier3D::MeshSimplifier3D(EditMesh& pMesh) :
	mesh(pMesh), editCount_QSlim(-1) {}

void MeshSimplifier3D::SimplifyQSlim(double threshold, int decreaseTris)
{
	// decide how many triangles to leave
	int leftTris;
	if (decreaseTris < 0)
	{
		if (threshold > 1.0)
		{
			leftTris = threshold;
			threshold /= mesh.m_faceData.size();
		}
		else
		{
			leftTris = threshold * mesh.m_faceData.size();
		}
	}
	else
	{
		leftTris = mesh.m_faceData.size() - decreaseTris;
		threshold = double(leftTris) / mesh.m_faceData.size();
	}

	int collapsedEdges = 0;
	std::vector<size_t> affectedHE; // affected half edges after collapsing (needed for updating heap)
	std::vector<size_t> deletedHE; // deleted half edges after collapsing (needed for updating heap)
	while ((!edgesToCollapse.empty()) && (mesh.m_faceData.size() > leftTris))
	{
		// get the edge introducing min error from the heap
		size_t e = edgesToCollapse.top()->edge_index;
		edgesToCollapse.popTop();
		twinInside[e] = twinInside[mesh.m_heData[e].twin] = -2;

		assert(e < mesh.m_heData.size());

		// only collapse collapsable edges
		// Note that although we didn't insert incollapsible edges into the heap initially,
		// collapsing the edges will make some originally collapsable edges incollapsible!
		if (Collapsable(e))
		{
			// m_heData[e].vert will be left, and the other vertex will be lazy deleted 
			// half edges are also lazy deleted (only triangles are really deleted)
			const size_t endPoints[2] = { mesh.m_heData[e].vert, mesh.m_heData[mesh.m_heData[e].twin].vert };

			CollapseEdge(e, affectedHE, deletedHE);

			// incremente count for checking stop condition
			collapsedEdges++;

			//update vertex quadrics and importances to approximate global error
			vertQuadrics[endPoints[0]] += vertQuadrics[endPoints[1]];

			// delete deleted edges from the edge heap
			for (int i = 0; i < 6; i++)
			{
				if ((i == 0) || (i == 3))
				{
					// these 2 are just the collapsed edge, 
					// which has been deleted from the heap already
					continue;
				}
				else
				{
					switch (twinInside[deletedHE[i]])
					{
					case -1:
						// self inside heap
						edgesToCollapse.remove(deletedHE[i]);
						twinInside[deletedHE[i]] = twinInside[mesh.m_heData[deletedHE[i]].twin] = -2;
						break;

					case -2:
						// both are not inside heap
						assert(twinInside[mesh.m_heData[deletedHE[i]].twin] == -2);
						break;

					default:
						// twin inside heap
						edgesToCollapse.remove(twinInside[deletedHE[i]]);
						twinInside[deletedHE[i]] = twinInside[mesh.m_heData[deletedHE[i]].twin] = -2;
						break;
					}
				}
			}

			// update min error in heap and best position for affected edges
			double minErr;
			bool feasible;
			for (auto HEIter = affectedHE.begin(); HEIter != affectedHE.end(); HEIter++)
			{
				switch (twinInside[*HEIter])
				{
				case -1:
					// self inside heap
					feasible = GetMinErr(*HEIter, minErr);
					if (feasible)
					{
						edgesToCollapse.update(*HEIter, -minErr);
					}
					else
					{
						edgesToCollapse.remove(*HEIter);
					}
					break;

				case -2:
					// both not inside heap
					assert(twinInside[mesh.m_heData[*HEIter].twin] == -2);
					if (Collapsable(*HEIter))
					{
						feasible = GetMinErr(*HEIter, minErr);
						if (feasible)
						{
							edgesToCollapse.insert(*HEIter, -minErr);
							twinInside[*HEIter] = -1;
							twinInside[mesh.m_heData[*HEIter].twin] = *HEIter;
						}
					}
					break;

				default:
					// twin inside heap
					feasible = GetMinErr(*HEIter, minErr);
					if (feasible)
					{
						edgesToCollapse.update(twinInside[*HEIter], -minErr);
					}
					else
					{
						edgesToCollapse.remove(twinInside[*HEIter]);
					}
					break;
				}
			}
		}
	}

	// pop out incollapsible edges for correctly visualizing the next candidate
	while (!edgesToCollapse.empty() && !Collapsable(edgesToCollapse.top()->edge_index)) 
	{ 
		edgesToCollapse.popTop(); 
	}

	// mark the QSlim data structure state and also the mesh state
	editCount_QSlim++;
	mesh.flag_edited();
}

void MeshSimplifier3D::InitQSlim()
{
	// compute triangle plane equation (quadric)
	std::vector<Quadric> triQuadrics;
	triQuadrics.resize(mesh.m_faceData.size());
	for (size_t triI = 0; triI < mesh.m_faceData.size(); triI++)
	{
		ComputeTriQ(triI, triQuadrics[triI]);
	}

	// compute vertex quadrics by summing over quadrics of incident triangles
	vertQuadrics.resize(0);
	vertQuadrics.reserve(mesh.m_vertices.size());
	for (size_t vertI = 0; vertI < mesh.m_vertData.size(); vertI++)
	{
		const half_edge* hePtr = &mesh.m_heData[mesh.m_vertData[vertI]];

		Quadric Q(triQuadrics[hePtr->face]);
		hePtr = &mesh.m_heData[mesh.m_heData[mesh.m_heData[hePtr->next].next].twin];
		while (hePtr != &mesh.m_heData[mesh.m_vertData[vertI]])
		{
			Q += triQuadrics[hePtr->face];
			hePtr = &mesh.m_heData[mesh.m_heData[mesh.m_heData[hePtr->next].next].twin];
		}
		vertQuadrics.emplace_back(Q);
	}

	// Init edge heap by calculating the min error of collapsing each edge
	// and use it as the key for sorting the edges in a binary heap.
	// We will also save the best positions now to avoid recalculating.
	edgesToCollapse.clear();
	edgeBestPoses.resize(mesh.m_heData.size());
	twinInside.resize(0);
	twinInside.resize(mesh.m_heData.size(), -1);
	for (size_t edgeI = 0; edgeI < mesh.m_heData.size(); edgeI++)
	{
		// twinInSide is a recorder to record which edges are inside the heap
		// because we don't want to add both twin half edges into the heap.
		// This recorder will also make us easier to update or reinsert edges
		// into heap after each edge collapse.
		if (twinInside[edgeI] == -1)
		{
			// only insert collapsable edges
			// Collapsable is defined a not introducing non-manifold edges after collapsing.
			if (Collapsable(edgeI))
			{
				twinInside[mesh.m_heData[edgeI].twin] = edgeI;
				double minErr;
				bool feasible = GetMinErr(edgeI, minErr);
				if (feasible)
				{
					edgesToCollapse.insert(edgeI, -minErr);
				}
			}
			else
			{
				twinInside[edgeI] = -2;
				twinInside[mesh.m_heData[edgeI].twin] = -2;
			}
		}
	}

	// mark the state of edge heap to be the same as the mesh
	editCount_QSlim = mesh.edit_count;
}

void MeshSimplifier3D::ComputeTriQ(size_t triI, Quadric& Q)
{
	const half_edge *hePtr = &mesh.m_heData[mesh.m_faceData[triI]];
	const Eigen::Vector3d v0 = mesh.m_vertices[hePtr->vert];
	hePtr = &mesh.m_heData[hePtr->next];
	const Eigen::Vector3d v1 = mesh.m_vertices[hePtr->vert];
	hePtr = &mesh.m_heData[hePtr->next];
	const Eigen::Vector3d v2 = mesh.m_vertices[hePtr->vert];
	assert(hePtr->next == mesh.m_faceData[triI]);

	Eigen::Vector3d triNormalVec = (v1 - v0).cross(v2 - v0);
	Eigen::Vector3d triNormal = triNormalVec.normalized();
	Q.init(triNormal, -(triNormal.dot(v0)), triNormalVec.norm() / 2.0, v0.dot(v1.cross(v2)));
	Q *= Q.getArea();
}

// 2.1 Edge Collapse (QSlim) 
// solve the linear system to get the best position after collapsing and
// the minimum error, return the minimum error, save the best position into edgeBestPoses
// for later use
bool MeshSimplifier3D::GetMinErr(size_t edgeI, double& minErr)
{
	size_t v0I = mesh.m_heData[edgeI].vert;
	size_t v1I = mesh.m_heData[mesh.m_heData[edgeI].twin].vert;
	const Eigen::Vector3d& v0 = mesh.m_vertices[v0I];
	const Eigen::Vector3d& v1 = mesh.m_vertices[v1I];

	Quadric Q = vertQuadrics[v0I];
	Q += vertQuadrics[v1I];

	minErr = Q.solve(edgeBestPoses[edgeI]);
	if (minErr < 0.0)
	{
		edgeBestPoses[edgeI] = (v0 + v1) / 2;
		minErr = Q.value(edgeBestPoses[edgeI]);
	}
	return true;
}


void MeshSimplifier3D::SimplifyProgressiveHull(double threshold, int decreaseTris, bool outerHull)
{
	// decide how many triangles to leave
	int leftTris;
	if (decreaseTris < 0)
	{
		if (threshold > 1.0)
		{
			leftTris = threshold;
			threshold /= mesh.m_faceData.size();
		}
		else
		{
			leftTris = threshold * mesh.m_faceData.size();
		}
	}
	else
	{
		leftTris = mesh.m_faceData.size() - decreaseTris;
		threshold = double(leftTris) / mesh.m_faceData.size();
	}

	int collapsedEdges = 0;
	std::vector<size_t> affectedHE; // affected half edges after collapsing (needed for updating heap)
	std::vector<size_t> deletedHE; // deleted half edges after collapsing (needed for updating heap)
	while ((!edgesToCollapse.empty()) && (mesh.m_faceData.size() > leftTris))
	{
		// get the edge introducing min error from the heap
		size_t e = edgesToCollapse.top()->edge_index;
		edgesToCollapse.popTop();
		twinInside[e] = twinInside[mesh.m_heData[e].twin] = -2;

		assert(e < mesh.m_heData.size());

		// only collapse collapsable edges
		// Note that although we didn't insert incollapsible edges into the heap initially,
		// collapsing the edges will make some originally collapsable edges incollapsible!
		if (Collapsable(e))
		{
			CollapseEdge(e, affectedHE, deletedHE);

			// incremente count for checking stop condition
			collapsedEdges++;

			// delete deleted edges from the edge heap
			for (int i = 0; i < 6; i++)
			{
				if ((i == 0) || (i == 3))
				{
					// these 2 are just the collapsed edge, 
					// which has been deleted from the heap already
					continue;
				}
				else
				{
					switch (twinInside[deletedHE[i]])
					{
					case -1:
						// self inside heap
						edgesToCollapse.remove(deletedHE[i]);
						twinInside[deletedHE[i]] = twinInside[mesh.m_heData[deletedHE[i]].twin] = -2;
						break;

					case -2:
						// both are not inside heap
						assert(twinInside[mesh.m_heData[deletedHE[i]].twin] == -2);
						break;

					default:
						// twin inside heap
						edgesToCollapse.remove(twinInside[deletedHE[i]]);
						twinInside[deletedHE[i]] = twinInside[mesh.m_heData[deletedHE[i]].twin] = -2;
						break;
					}
				}
			}

			// update min error in heap and best position for affected edges
			double minVolume;
			bool feasible;
			for (auto HEIter = affectedHE.begin(); HEIter != affectedHE.end(); HEIter++)
			{
				switch (twinInside[*HEIter])
				{
				case -1:
					// self inside heap
					feasible = GetMinVolume(*HEIter, minVolume, outerHull);
					if (feasible)
					{
						edgesToCollapse.update(*HEIter, -minVolume);
					}
					else
					{
						edgesToCollapse.remove(*HEIter);
					}
					break;

				case -2:
					// both not inside heap
					assert(twinInside[mesh.m_heData[*HEIter].twin] == -2);
					if (Collapsable(*HEIter))
					{
						feasible = GetMinVolume(*HEIter, minVolume, outerHull);
						if (feasible)
						{
							edgesToCollapse.insert(*HEIter, -minVolume);
							twinInside[*HEIter] = -1;
							twinInside[mesh.m_heData[*HEIter].twin] = *HEIter;
						}
					}
					break;

				default:
					// twin inside heap
					feasible = GetMinVolume(*HEIter, minVolume, outerHull);
					if (feasible)
					{
						edgesToCollapse.update(twinInside[*HEIter], -minVolume);
					}
					else
					{
						edgesToCollapse.remove(twinInside[*HEIter]);
					}
					break;
				}
			}
		}
	}

	// pop out incollapsible edges for correctly visualizing the next candidate
	while (!edgesToCollapse.empty() && !Collapsable(edgesToCollapse.top()->edge_index)) 
	{ 
		edgesToCollapse.popTop(); 
	}

	// mark the QSlim data structure state and also the mesh state
	editCount_QSlim++;
	mesh.flag_edited();
}

void MeshSimplifier3D::InitProgressiveHull(bool outerHull)
{
	// Init edge heap by calculating the min error of collapsing each edge
	// and use it as the key for sorting the edges in a binary heap.
	// We will also save the best positions now to avoid recalculating.
	edgesToCollapse.clear();
	edgeBestPoses.resize(mesh.m_heData.size());
	twinInside.resize(0);
	twinInside.resize(mesh.m_heData.size(), -1);
	for (size_t edgeI = 0; edgeI < mesh.m_heData.size(); edgeI++)
	{
		// twinInSide is a recorder to record which edges are inside the heap
		// because we don't want to add both twin half edges into the heap.
		// This recorder will also make us easier to update or reinsert edges
		// into heap after each edge collapse.
		if (twinInside[edgeI] == -1)
		{
			// only insert collapsable edges
			// Collapsable is defined a not introducing non-manifold edges after collapsing.
			if (Collapsable(edgeI))
			{
				twinInside[mesh.m_heData[edgeI].twin] = edgeI;
				double minVolume;
				bool feasible = GetMinVolume(edgeI, minVolume, outerHull);
				if (feasible)
				{
					edgesToCollapse.insert(edgeI, -minVolume);
				}
			}
			else
			{
				twinInside[edgeI] = -2;
				twinInside[mesh.m_heData[edgeI].twin] = -2;
			}
		}
	}

	// mark the state of edge heap to be the same as the mesh
	editCount_QSlim = mesh.edit_count;
}

// 2.1 Edge Collapse with Volume Preservation
// Use linear programming to find position that minimizes volume while enclosing original volume
// Return the volume increase cost, save the best position into edgeBestPoses

bool MeshSimplifier3D::GetMinVolume(size_t edgeI, double& minVolume, bool outerHull)
{
	// Get the ring of vertices adjacent to the edge endpoints
	std::vector<Eigen::Vector3d> adjacentTrisData;
	GetAdjacentTrisData(edgeI, adjacentTrisData, true);
	if (DetectSelfIntersections(adjacentTrisData) > 0)
	{
		return false;
	}

	// Use linear programming to find position that minimizes volume
	// while satisfying constraints to enclose original volume
	if (!SolveLinearProgrammingForVertex(adjacentTrisData, edgeBestPoses[edgeI], minVolume, outerHull))
	{
		return false;
	}

	std::vector<Eigen::Vector3d> newTrisData;
	GetAdjacentTrisData(edgeI, newTrisData, false);
	for (size_t i = 0; i < newTrisData.size(); i += 3)
	{
		newTrisData[i] = edgeBestPoses[edgeI];
	}
	for (size_t i = 0; i < newTrisData.size(); i ++)
	{
		adjacentTrisData.push_back(newTrisData[i]);
	}

	if (DetectSelfIntersections(adjacentTrisData) > 0)
	{
		return false;
	}

	return true;
}

// Get triangles adjacent to the edge
void MeshSimplifier3D::GetAdjacentTrisData(size_t edgeI, std::vector<Eigen::Vector3d>& adjacentTrisData, bool includeDirectFaces)
{
	adjacentTrisData.clear();

	const half_edge& heBase = mesh.m_heData[edgeI];
	const half_edge& heTwin = mesh.m_heData[heBase.twin];

	const half_edge* HEPtr = &mesh.m_heData[heBase.next];
	if (!includeDirectFaces)
	{
		HEPtr = &mesh.m_heData[mesh.m_heData[HEPtr->twin].next];
	}
	while (HEPtr != &heTwin)
	{
		assert(HEPtr->vert == heTwin.vert);
		const half_edge *hePtr = HEPtr;
		for (int j = 0; j < 3; j++)
		{
			adjacentTrisData.push_back(mesh.m_vertices[hePtr->vert]);
			hePtr = &mesh.m_heData[hePtr->next];
		}
		HEPtr = &mesh.m_heData[mesh.m_heData[HEPtr->twin].next];
	}

	HEPtr = &mesh.m_heData[heTwin.next];
	if (!includeDirectFaces)
	{
		HEPtr = &mesh.m_heData[mesh.m_heData[HEPtr->twin].next];
	}
	while (HEPtr != &heBase)
	{
		assert(HEPtr->vert == heBase.vert);
		const half_edge *hePtr = HEPtr;
		for (int j = 0; j < 3; j++)
		{
			adjacentTrisData.push_back(mesh.m_vertices[hePtr->vert]);
			hePtr = &mesh.m_heData[hePtr->next];
		}
		HEPtr = &mesh.m_heData[mesh.m_heData[HEPtr->twin].next];
	}
}

// Solve linear programming problem to find optimal vertex position
bool MeshSimplifier3D::SolveLinearProgrammingForVertex(const std::vector<Eigen::Vector3d>& adjacentTrisData, Eigen::Vector3d& optimalPos, double& minVolume, bool outerHull)
{
	// Implementation of linear programming algorithm
	// Constraints: new mesh must enclose original volume
	// Objective: minimize new mesh volume

	int nAdjacentTris = adjacentTrisData.size() / 3;

	// 目标函数系数
	Eigen::Vector3d objCoeff = Eigen::Vector3d::Zero();
	// 约束矩阵
	Eigen::MatrixXd constraintMat(nAdjacentTris, 3);
	// 约束下界
	Eigen::VectorXd lowerBounds(nAdjacentTris);
	Eigen::VectorXd upperBounds(nAdjacentTris);

	for (size_t i = 0; i < nAdjacentTris; i++)
	{
		const Eigen::Vector3d v0 = adjacentTrisData[i * 3 + 0];
		const Eigen::Vector3d v1 = adjacentTrisData[i * 3 + 1];
		const Eigen::Vector3d v2 = adjacentTrisData[i * 3 + 2];
	
		Eigen::Vector3d triNormalVec = (v1 - v0).cross(v2 - v0);
		Eigen::Vector3d triNormal = triNormalVec.normalized();
		constraintMat.row(i) = triNormal;

		if (outerHull)
		{
			lowerBounds(i) = triNormal.dot(v0) + 1e-6;
			upperBounds(i) = std::numeric_limits<double>::max();
			objCoeff += (v1 - v0).cross(v2 - v0) / 6.0;
		}
		else
		{
			lowerBounds(i) = std::numeric_limits<double>::min();
			upperBounds(i) = triNormal.dot(v0) - 1e-6;
			objCoeff -= (v1 - v0).cross(v2 - v0) / 6.0;
		}
	}

	// 创建求解器
	OsiClpSolverInterface solver;
	solver.messageHandler()->setLogLevel(0);

	// 设置问题维度
	int numCols = 3;  // x, y, z
	int numRows = nAdjacentTris;  // 约束数量

	// 变量下界 (>= 0)
	double* colLower = new double[numCols];
	double* colUpper = new double[numCols];
	for (int i = 0; i < numCols; i++)
	{
		colLower[i] = -10.0;
		colUpper[i] = 10.0;
	}

	// 约束边界
	double* rowLower = new double[numRows];
	double* rowUpper = new double[numRows];
	for (int i = 0; i < numRows; i++)
	{
		rowLower[i] = lowerBounds(i);
		rowUpper[i] = upperBounds(i);
	}

	// 约束矩阵（压缩列格式）
	CoinPackedMatrix matrix;
	matrix.setDimensions(0, numCols);

	for (int i = 0; i < numRows; i++)
	{
		CoinPackedVector row;
		for (int j = 0; j < numCols; j++)
		{
			if (constraintMat(i, j) != 0)
			{
				row.insert(j, constraintMat(i, j));
			}
		}
		matrix.appendRow(row);
	}

	// 加载问题
	solver.loadProblem(matrix, colLower, colUpper, objCoeff.data(), rowLower, rowUpper);

	// 求解
	solver.initialSolve();

	// 输出结果
	if (solver.isProvenOptimal())
	{
		const double* solution = solver.getColSolution();
		for (int i = 0; i < numCols; i++)
		{
			optimalPos(i) = solution[i];
		}
		minVolume = 0.0;
		for (size_t i = 0; i < nAdjacentTris; i++)
		{
			const Eigen::Vector3d v0 = adjacentTrisData[i * 3 + 0];
			const Eigen::Vector3d v1 = adjacentTrisData[i * 3 + 1];
			const Eigen::Vector3d v2 = adjacentTrisData[i * 3 + 2];
			minVolume += (outerHull ? 1 : -1) * (optimalPos - v0).dot((v1 - v0).cross(v2 - v0)) / 6.0;
		}
	}

	// 清理
	delete[] colLower;
	delete[] colUpper;
	delete[] rowLower;
	delete[] rowUpper;

	return solver.isProvenOptimal();
}


// changed the original function to lazy delete half edges for keeping the index the same so that it's more convenient
// also added the function of giving back deleted and affected half edge information for updating the edge heap
std::size_t MeshSimplifier3D::CollapseEdge(std::size_t he,
	std::vector<size_t>& affectedHE, std::vector<size_t>& deletedHE,
	bool optimizePos)
{
	assert(he < mesh.m_heData.size());
	assert(mesh.m_heData[he].face != HOLE_INDEX && mesh.m_heData[mesh.m_heData[he].twin].face != HOLE_INDEX && "Cannot collapse a boundary edge");

	const half_edge& heBase = mesh.m_heData[he];
	const half_edge& heTwin = mesh.m_heData[heBase.twin];

	// We are going to delete the faces on either side of the chosen edge, 
	// so we need to delete 3 half_edges and patch up the twin links on the 4 bordering edges.
	std::size_t heBorder[4];
	heBorder[0] = mesh.m_heData[heBase.next].twin;
	heBorder[1] = mesh.m_heData[mesh.m_heData[heBase.next].next].twin;
	heBorder[2] = mesh.m_heData[mesh.m_heData[heTwin.next].next].twin;
	heBorder[3] = mesh.m_heData[heTwin.next].twin;

	// TODO: Relax this assertion. We should be able to collapse a spike jutting into a hole.
	assert((mesh.m_heData[heBorder[0]].face != HOLE_INDEX || mesh.m_heData[heBorder[1]].face != HOLE_INDEX) && "Cannot collapse an edge on a face with holes on either side.");
	assert((mesh.m_heData[heBorder[2]].face != HOLE_INDEX || mesh.m_heData[heBorder[3]].face != HOLE_INDEX) && "Cannot collapse an edge on a face with holes on either side.");

	//// Check if we can actually collapse. This checks for a degree 3 vertex at the vertices not on the edge we are collapsing.
	//if (mesh.m_heData[mesh.m_heData[mesh.m_heData[heBorder[1]].next].twin].next == heBorder[0])
	//	return HOLE_INDEX;
	//if (mesh.m_heData[mesh.m_heData[mesh.m_heData[heBorder[2]].next].twin].next == heBorder[3])
	//	return HOLE_INDEX;

	// Capture the indices of things (2 faces & 6 half-edges) we want to delete.
	std::size_t fToDelete[] = { heBase.face, heTwin.face };
	std::size_t heToDelete[] = { he, heBase.next, mesh.m_heData[heBase.next].next, heBase.twin, heTwin.next, mesh.m_heData[heTwin.next].next };

	// We may also need to fix the vertex->half_edge link for the verts using these faces. There are technically 4, but we only update the 3 that are not going to be deleted.
	std::size_t verts[] = { mesh.prev(heBase).vert, heBase.vert, mesh.prev(heTwin).vert };

	// Move the base vertex (arbitrarily) to the middle of the edge. Could leave it where it is, or do something fancier too.
	if (optimizePos) { mesh.m_vertices[heBase.vert] = edgeBestPoses[he]; }

	// Adjust all the twin's 1-ring to link to the vertex we are not going to delete.
	std::size_t heIt = mesh.twin(mesh.next(heBase)).next;
	std::size_t heEnd = heBase.twin;
	for (; heIt != heEnd; heIt = mesh.twin(mesh.m_heData[heIt]).next){
		assert(mesh.m_heData[heIt].vert == heTwin.vert);

		// Associate to the other vertex now, so we can delete this one.
		mesh.m_heData[heIt].vert = heBase.vert;
	}

	// Fix the vert associations if required, picking a non-hole face.
	if (mesh.m_vertData[verts[0]] == mesh.m_heData[heBorder[1]].twin)
		mesh.m_vertData[verts[0]] = (mesh.m_heData[heBorder[0]].face != HOLE_INDEX) ? heBorder[0] : mesh.m_heData[heBorder[1]].next;
	if (mesh.m_vertData[verts[1]] == he || mesh.m_vertData[verts[1]] == heTwin.next)
		mesh.m_vertData[verts[1]] = (mesh.m_heData[heBorder[1]].face != HOLE_INDEX) ? heBorder[1] : heBorder[2];
	if (mesh.m_vertData[verts[2]] == mesh.m_heData[heBorder[2]].twin)
		mesh.m_vertData[verts[2]] = (mesh.m_heData[heBorder[3]].face != HOLE_INDEX) ? heBorder[3] : mesh.m_heData[heBorder[2]].next;

	// "Delete" the other vertex
	mesh.m_vertData[heTwin.vert] = HOLE_INDEX;

	// Collapse the two triangles bordering our chosen half-edge by connecting the opposite edges together.
	mesh.m_heData[heBorder[0]].twin = heBorder[1];
	mesh.m_heData[heBorder[1]].twin = heBorder[0];
	mesh.m_heData[heBorder[2]].twin = heBorder[3];
	mesh.m_heData[heBorder[3]].twin = heBorder[2];

	// Have to delete the faces in the proper order.
	if (fToDelete[0] < fToDelete[1])
		std::swap(fToDelete[0], fToDelete[1]);

	//Changed to lazy delete half edges to keep the index the same for convenience:
	//this->delete_half_edges_impl(heToDelete);
	detail::delete_face1(mesh.m_faceData, mesh.m_heData, fToDelete[0]);
	detail::delete_face1(mesh.m_faceData, mesh.m_heData, fToDelete[1]);

	// collect information about deleted and affected half edges for
	// updating the heap
	deletedHE.resize(6);
	std::memcpy(deletedHE.data(), heToDelete, sizeof(heToDelete));
	affectedHE.resize(0);
	affectedHE.emplace_back(mesh.m_vertData[verts[1]]);
	size_t HEInd = mesh.m_heData[mesh.m_heData[mesh.m_heData[affectedHE.front()].next].next].twin;
	while (HEInd != affectedHE.front())
	{
		affectedHE.emplace_back(HEInd);
		HEInd = mesh.m_heData[mesh.m_heData[mesh.m_heData[HEInd].next].next].twin;
	}
	return verts[1];
}

// ensure that the 2 vertices of the collapsed edge are not directly connected by
// a 4th vertex, so it won't introduce non-manifold edges after collapsing
// If the 2 vertices are only directly connected by 3 vertices, it is also viewed as incollapsable.
bool MeshSimplifier3D::Collapsable(size_t edgeI)
{
	const half_edge& heBase = mesh.m_heData[edgeI];
	const half_edge& heTwin = mesh.m_heData[heBase.twin];

	std::size_t heBorder[4];
	heBorder[0] = mesh.m_heData[heBase.next].twin;
	heBorder[1] = mesh.m_heData[mesh.m_heData[heBase.next].next].twin;
	heBorder[2] = mesh.m_heData[mesh.m_heData[heTwin.next].next].twin;
	heBorder[3] = mesh.m_heData[heTwin.next].twin;

	std::set<size_t> neighbor;
	const half_edge* HEPtr = &mesh.m_heData[mesh.m_heData[heBorder[0]].next];
	while (HEPtr != &mesh.m_heData[heBorder[2]])
	{
		assert(HEPtr->vert == heTwin.vert);

		neighbor.insert(mesh.m_heData[HEPtr->next].vert);

		HEPtr = &mesh.m_heData[mesh.m_heData[HEPtr->twin].next];
	}

	int n = 0;
	HEPtr = &mesh.m_heData[mesh.m_heData[heBorder[3]].next];
	while (HEPtr != &mesh.m_heData[heBorder[1]])
	{
		assert(HEPtr->vert == heBase.vert);

		n++;
		if (neighbor.find(mesh.m_heData[HEPtr->next].vert) != neighbor.end())
		{
			return false;
		}

		HEPtr = &mesh.m_heData[mesh.m_heData[HEPtr->twin].next];
	}

	if (neighbor.empty() && (n == 0))
	{
		return false;
	}

	return true;
}

void MeshSimplifier3D::CompactMesh()
{
    // 标记正在使用的顶点和半边
    std::vector<bool> vertexUsed(mesh.m_vertices.size(), false);
    std::vector<bool> halfEdgeUsed(mesh.m_heData.size(), false);
    
    // 第一步：标记所有正在使用的顶点和半边
	for (size_t triI = 0; triI < mesh.m_faceData.size(); triI++)
	{
        size_t startHe = mesh.m_faceData[triI];
        size_t currentHe = startHe;
        
        for (int j = 0; j < 3; j++)
        {
            halfEdgeUsed[currentHe] = true;
            vertexUsed[mesh.m_heData[currentHe].vert] = true;
            currentHe = mesh.m_heData[currentHe].next;
        }
    }
    
    // 第二步：重新索引顶点
    std::vector<size_t> vertexMap(mesh.m_vertices.size(), HOLE_INDEX);
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> newVertices;
    std::vector<size_t> newVertData;
    
    size_t newVertexIndex = 0;
    for (size_t i = 0; i < mesh.m_vertices.size(); ++i)
    {
        if (vertexUsed[i])
        {
            vertexMap[i] = newVertexIndex;
            newVertices.push_back(mesh.m_vertices[i]);
            newVertData.push_back(HOLE_INDEX); // 暂时设为HOLE_INDEX，后面会更新
            newVertexIndex++;
        }
    }
    
    // 第三步：重新索引半边
    std::vector<size_t> halfEdgeMap(mesh.m_heData.size(), HOLE_INDEX);
    std::vector<half_edge> newHalfEdges;
    
    size_t newHalfEdgeIndex = 0;
    for (size_t i = 0; i < mesh.m_heData.size(); ++i)
    {
        if (halfEdgeUsed[i]) {
            halfEdgeMap[i] = newHalfEdgeIndex;
            
            // 创建新的半边，但先不设置twin和next（因为目标索引可能还没创建）
            half_edge newHe = mesh.m_heData[i];
            newHe.vert = vertexMap[newHe.vert]; // 更新顶点索引
            newHalfEdges.push_back(newHe);
            newHalfEdgeIndex++;
        }
    }
    
    // 第四步：更新面数据中的起始半边索引
    std::vector<size_t> newFaceData;
    for (size_t faceIdx = 0; faceIdx < mesh.m_faceData.size(); ++faceIdx)
    {
        newFaceData.push_back(halfEdgeMap[mesh.m_faceData[faceIdx]]);
    }
    
    // 第五步：更新半边之间的连接关系（next和twin）
    for (size_t i = 0; i < mesh.m_heData.size(); ++i)
    {
        if (halfEdgeUsed[i])
        {
            size_t newIndex = halfEdgeMap[i];
            
            // 更新next指针
            if (halfEdgeUsed[mesh.m_heData[i].next])
            {
                newHalfEdges[newIndex].next = halfEdgeMap[mesh.m_heData[i].next];
            }
            
            // 更新twin指针
            if (mesh.m_heData[i].twin != HOLE_INDEX && halfEdgeUsed[mesh.m_heData[i].twin])
            {
                newHalfEdges[newIndex].twin = halfEdgeMap[mesh.m_heData[i].twin];
            }
            
            // 更新face指针（应该保持不变，因为面索引没有改变）
            newHalfEdges[newIndex].face = mesh.m_heData[i].face;
        }
    }
    
    // 第六步：更新顶点数据中的起始半边引用
    for (size_t oldVertIdx = 0; oldVertIdx < mesh.m_vertData.size(); ++oldVertIdx)
    {
        if (vertexUsed[oldVertIdx] && halfEdgeUsed[mesh.m_vertData[oldVertIdx]])
        {
            size_t newVertIdx = vertexMap[oldVertIdx];
            newVertData[newVertIdx] = halfEdgeMap[mesh.m_vertData[oldVertIdx]];
        }
    }
    
    // 第七步：替换原始数据
    mesh.m_vertices = std::move(newVertices);
    mesh.m_vertData = std::move(newVertData);
    mesh.m_heData = std::move(newHalfEdges);
    mesh.m_faceData = std::move(newFaceData);
    
    // // 第八步：更新相关的简化数据结构
    // UpdateSimplificationData(vertexMap, halfEdgeMap);
}

// void MeshSimplifier3D::UpdateSimplificationData(const std::vector<size_t>& vertexMap, 
//                                                const std::vector<size_t>& halfEdgeMap) {
//     // 更新顶点二次误差度量数据
//     std::vector<Quadric> newVertQuadrics;
//     for (size_t i = 0; i < vertexMap.size(); ++i) {
//         if (vertexMap[i] != HOLE_INDEX) {
//             if (newVertQuadrics.size() <= vertexMap[i]) {
//                 newVertQuadrics.resize(vertexMap[i] + 1);
//             }
//             newVertQuadrics[vertexMap[i]] = std::move(vertQuadrics[i]);
//         }
//     }
//     vertQuadrics = std::move(newVertQuadrics);
//     
//     // 更新最佳位置数据
//     std::vector<Eigen::Vector3d> newEdgeBestPoses;
//     for (size_t i = 0; i < halfEdgeMap.size(); ++i) {
//         if (halfEdgeMap[i] != HOLE_INDEX) {
//             if (newEdgeBestPoses.size() <= halfEdgeMap[i]) {
//                 newEdgeBestPoses.resize(halfEdgeMap[i] + 1);
//             }
//             newEdgeBestPoses[halfEdgeMap[i]] = std::move(edgeBestPoses[i]);
//         }
//     }
//     edgeBestPoses = std::move(newEdgeBestPoses);
//     
//     // 重置twinInside和边堆，因为它们需要基于新的半边索引重新构建
//     twinInside.clear();
//     edgesToCollapse.clear();
//     editCount_QSlim = mesh.edit_count - 1; // 强制下次使用时重新初始化
// }

} // namespace MeshDef