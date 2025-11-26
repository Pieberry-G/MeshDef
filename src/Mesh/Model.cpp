#include "Mesh/Model.h"

namespace MeshDef {

    Model::Model(const std::string& name)
        : last_drawn(-1), m_EditMesh(NULL), m_Name(name)
    {
        bboxMin.setZero();
        bboxMax.setZero();
    }

    Model::Model(EditMeshPtr editMesh, const std::string& name)
        : last_drawn(-1), m_EditMesh(editMesh), m_Name(name)
    {
        bboxMin.setZero();
        bboxMax.setZero();
    }

    Model::~Model()
    {
        // Shared pointers don't need to be deleted?
    }

    void Model::init()
    {
        if (!m_EditMesh)
        {
            m_EditMesh = EditMeshPtr(new EditMesh());
        }
    }

    Eigen::Vector3d Model::info_vertex(std::size_t i)
    {
	    return m_EditMesh->get_vertex(i);
    }

    void Model::info_bbox(Eigen::Vector3d &bboxMin, Eigen::Vector3d &bboxMax)
    {
        bboxMin = m_EditMesh->bboxMin;
        bboxMax = m_EditMesh->bboxMax;
    }

    void Model::getIndicesForFace(size_t tri_index, size_t indicesForFace[3])
    {
	    m_EditMesh->getIndicesForFace(tri_index, indicesForFace);
    }

    void Model::DrawMeshToPolyscope()
    {
        if (m_PsMesh) { m_PsMesh->remove(); }
        
        Eigen::MatrixXd vertices= m_EditMesh->get_vertices();
        Eigen::MatrixXi faces= m_EditMesh->get_faces();
        
        m_PsMesh = polyscope::registerSurfaceMesh(m_Name, vertices, faces);
        m_PsMesh->setTransform(m_Transform);
    }

    void Model::DrawSelectedVertice()
    {
        std::vector<glm::vec3> colors(m_EditMesh->get_vert_size());
        for (int i = 0; i < m_EditMesh->get_vert_size(); i++)
        {
            int vertFlag = m_EditMesh->getVertFlag(i);
            if (vertFlag == 1) { colors[i] = glm::vec3(1.0f, 0.0f, 0.0f); }
            else if (vertFlag == 2) { colors[i] = glm::vec3(0.0f, 0.0f, 1.0f); }
            else { colors[i] = m_PsMesh->getSurfaceColor(); }
        }
        polyscope::SurfaceVertexColorQuantity* quantity = m_PsMesh->addVertexColorQuantity("Selected Vertices", colors);
        quantity->setEnabled(true);
    }

    void Model::RemoveMeshFromPolyscope()
    {
        if (!m_PsMesh) { return; }

        m_PsMesh->remove();
        m_PsMesh = nullptr;
    }

} // namespace MeshDef