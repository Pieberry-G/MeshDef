#pragma once

#include "Mesh/DrawMesh.h"
#include "Mesh/EditMesh.h"
#include <memory>

namespace MeshDef {

typedef std::shared_ptr<EditMesh> EditMesh_ptr;

class Model
{
public:
    Model();
    Model(EditMesh_ptr em);
    ~Model();

    void init();

    void DrawMesh();

    /***********************************************************
     * the reflection interface to get mesh information
     ***********************************************************/
    std::size_t  info_sizev() {return m_em->get_vert_size();}
    std::size_t  info_sizef() {return m_em->get_face_size();}
	Eigen::Vector3d info_vertex(std::size_t i);

    void info_bbox(Eigen::Vector3d &bboxMin, Eigen::Vector3d &bboxMax);
	void getIndicesForFace(size_t tri_index, size_t indicesForFace[3]);

    /***********************************************
     * mesh modification algorithms
     ***********************************************/
    // expose your editmesh functions here

    // selection functions
    void select_vert( size_t index )   {m_em->select_vert(index);}
    void deselect_vert( size_t index ) {m_em->deselect_vert(index);}
    void deselect_verts()              {m_em->deselect_allVerts();}
    bool isSelected( size_t index )    {return m_em->isSelected(index);}

	const EditMesh_ptr get_editMesh() const;
    void               set_editMesh(EditMesh_ptr em);

protected:
    void UpdateDrawMesh();
    void AddToPolyscope();
    void RemoveFromPolyscope();

    /***********************************************
     * Variables for this Instance of the EditMesh
     ***********************************************/
    // the bounding box for this instance of the geometry
    Eigen::Vector3f bboxMin;
    Eigen::Vector3f bboxMax;

    //indicates the last drawn edit mesh, if this does not match
    // the value in m_em then the draw mesh needs to be updated
    int last_drawn;

    EditMesh_ptr m_em;

    std::string m_Name = "Default Mesh";
    std::vector<glm::vec3> m_Vertices;
    std::vector<std::vector<size_t>> m_Faces;
    glm::mat4 m_Transform = glm::mat4(1.0f);  // object's model transformations
    polyscope::SurfaceMesh* m_PsMesh = nullptr;
};

inline const EditMesh_ptr Model::get_editMesh() const {
	return m_em;
}

inline void Model::set_editMesh(EditMesh_ptr em) {
    m_em = em;
}

} // namespace MeshDef