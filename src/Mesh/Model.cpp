#include "Mesh/Model.h"

namespace MeshDef {

Model::Model() : last_drawn(-1), m_EditMesh(NULL)
{
    bboxMin.setZero();
    bboxMax.setZero();
}

Model::Model(EditMeshPtr editMesh) : last_drawn(-1), m_EditMesh(editMesh)
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

void Model::UpdateDrawMesh()
{
    int i_size = 3 * m_EditMesh->get_face_size();
    int v_size = 3 * i_size;//m_em->get_vert_size();

    float *v_data = new float[v_size*2];// vertex and normal data
    int   *i_data = new int[i_size];	// face indexes
    int   *s_data = new int[i_size];	// selection data
	float *c_data = new float[v_size];	// vertex colors

    m_EditMesh->get_draw_data(v_data, i_data);
    m_EditMesh->get_draw_normals(&v_data[v_size]);
    m_EditMesh->get_draw_selection(s_data);
	m_EditMesh->get_draw_colors(c_data);

    m_Vertices.clear();
    m_Faces.clear();

    for (int i = 0; i < v_size; i += 3)
    {
        m_Vertices.push_back(glm::vec3(v_data[i], v_data[i + 1], v_data[i + 2]));
    }
    for (int i = 0; i < i_size; i += 3)
    {
        std::vector<size_t> face;
        for (int j = 0; j < 3; j++)
        {
            face.push_back(static_cast<size_t>(i_data[i + j]));
        }
        m_Faces.push_back(face);
    }

    delete [] v_data;
    delete [] i_data;
    delete [] s_data;
	delete [] c_data;

    //last_drawn = m_EditMesh->get_edit_count();
}

void Model::DrawMeshToPolyscope()
{
    UpdateDrawMesh();

    if (m_PsMesh)
    {
        m_PsMesh->remove();
    }

    m_PsMesh = polyscope::registerSurfaceMesh(m_Name, m_Vertices, m_Faces);
    m_PsMesh->setTransform(m_Transform);
}

void Model::RemoveMeshFromPolyscope()
{
    if (!m_PsMesh)
    {
        return;
    }

    m_PsMesh->remove();
    m_PsMesh = nullptr;
}

} // namespace MeshDef