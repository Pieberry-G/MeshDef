#pragma once

#include "Mesh/EditMesh.h"

#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>

namespace MeshDef {

    typedef std::shared_ptr<EditMesh> EditMeshPtr;

    class Model
    {
    public:
        Model(const std::string& name = "Default Mesh");
        Model(EditMeshPtr em, const std::string& name = "Default Mesh");
        ~Model();

        void init();

        void DrawMeshToPolyscope();
        void DrawSelectedVertice();
        void ShowVertices(const std::vector<size_t>& vertices);
        void RemoveMeshFromPolyscope();

        /***********************************************************
         * the reflection interface to get mesh information
         ***********************************************************/
        std::size_t  info_sizev() { return m_EditMesh->get_vert_size(); }
        std::size_t  info_sizef() { return m_EditMesh->get_face_size(); }
	    Eigen::Vector3d info_vertex(std::size_t i);

        void info_bbox(Eigen::Vector3d &bboxMin, Eigen::Vector3d &bboxMax);
	    void getIndicesForFace(size_t tri_index, size_t indicesForFace[3]);

        /***********************************************
         * mesh modification algorithms
         ***********************************************/
        // expose your editmesh functions here

        // selection functions
        void select_vert( size_t index, int vertFlag )   { m_EditMesh->select_vert(index, vertFlag); }
        void deselect_vert( size_t index ) { m_EditMesh->deselect_vert(index); }
        void deselect_verts()              { m_EditMesh->deselect_allVerts(); }
        int getVertFlag( size_t index )    { return m_EditMesh->getVertFlag(index); }

        const std::string& GetName() const { return m_Name; }
        const EditMeshPtr GetEditMesh() const { return m_EditMesh; }
        void SetEditMesh(EditMeshPtr editMesh) { m_EditMesh = editMesh; }

        polyscope::SurfaceMesh* GetPsMesh() { return m_PsMesh; }

    protected:
        /***********************************************
         * Variables for this Instance of the EditMesh
         ***********************************************/
        // the bounding box for this instance of the geometry
        Eigen::Vector3f bboxMin;
        Eigen::Vector3f bboxMax;

        //indicates the last drawn edit mesh, if this does not match
        // the value in m_em then the draw mesh needs to be updated
        int last_drawn;

        std::string m_Name;
        EditMeshPtr m_EditMesh;
        glm::mat4 m_Transform = glm::mat4(1.0f);  // object's model transformations
        polyscope::SurfaceMesh* m_PsMesh = nullptr;
    };

} // namespace MeshDef