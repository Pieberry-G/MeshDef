#include "Mesh/Model.h"

#include "Mesh/MultiViewCameras.h"

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

    void Model::ShowCandidateHandles(const std::vector<size_t>& candidateHandles)
    {
        double vertexRadius = 0.015;
        std::vector<glm::vec3> colors = {
            { 1.0f, 0.0f, 0.0f },   // 红色 - Red
            { 0.0f, 1.0f, 0.0f },   // 绿色 - Green
            { 0.0f, 0.0f, 1.0f },   // 蓝色 - Blue
            { 1.0f, 1.0f, 0.0f },   // 黄色 - Yellow
            { 1.0f, 0.0f, 1.0f },   // 品红 - Magenta
            { 0.0f, 1.0f, 1.0f },   // 青色 - Cyan
            { 1.0f, 0.5f, 0.0f },   // 橙色 - Orange
            { 0.5f, 0.0f, 1.0f },   // 紫色 - Purple
            { 0.0f, 1.0f, 0.5f },   // 青绿 - Teal
            { 1.0f, 0.5f, 0.5f },   // 粉红 - Pink
            { 0.5f, 1.0f, 0.0f },   // 黄绿 - Lime
            { 0.0f, 0.5f, 1.0f },   // 天蓝 - Sky Blue
            { 1.0f, 0.0f, 0.5f },   // 玫红 - Rose
            { 0.5f, 0.5f, 1.0f },   // 淡紫 - Lavender
            { 1.0f, 1.0f, 0.5f },   // 浅黄 - Light Yellow
            { 0.5f, 1.0f, 1.0f },   // 淡青 - Light Cyan
        };

        MD_CORE_ASSERT(candidateHandles.size() <= 16);
        for (int i = 0; i < candidateHandles.size(); i++)
        {
            // Show selected vertices.
            std::vector<glm::vec3> vertPos = { m_PsMesh->vertices[candidateHandles[i]]};
            std::vector<std::array<size_t, 2>> vertInd;
            polyscope::SurfaceGraphQuantity* showVerts = m_PsMesh->addSurfaceGraphQuantity("Candidate Handles (" + std::to_string(i) + ")", vertPos, vertInd);
            showVerts->setEnabled(true);
            showVerts->setRadius(vertexRadius);
            showVerts->setColor(colors[i]);
        }
    }

    void Model::RemoveAllQuantities()
    {
        m_PsMesh->removeAllQuantities();
    }

    void Model::ShowSelectedHandles(const std::vector<size_t>& candidateHandles, const std::vector<size_t>& selectedHandlesIndices)
    {
        double vertexRadius = 0.015;
        std::vector<glm::vec3> colors = {
            { 1.0f, 0.0f, 0.0f },   // 红色 - Red
            { 0.0f, 1.0f, 0.0f },   // 绿色 - Green
            { 0.0f, 0.0f, 1.0f },   // 蓝色 - Blue
            { 1.0f, 1.0f, 0.0f },   // 黄色 - Yellow
            { 1.0f, 0.0f, 1.0f },   // 品红 - Magenta
            { 0.0f, 1.0f, 1.0f },   // 青色 - Cyan
            { 1.0f, 0.5f, 0.0f },   // 橙色 - Orange
            { 0.5f, 0.0f, 1.0f },   // 紫色 - Purple
            { 0.0f, 1.0f, 0.5f },   // 青绿 - Teal
            { 1.0f, 0.5f, 0.5f },   // 粉红 - Pink
            { 0.5f, 1.0f, 0.0f },   // 黄绿 - Lime
            { 0.0f, 0.5f, 1.0f },   // 天蓝 - Sky Blue
            { 1.0f, 0.0f, 0.5f },   // 玫红 - Rose
            { 0.5f, 0.5f, 1.0f },   // 淡紫 - Lavender
            { 1.0f, 1.0f, 0.5f },   // 浅黄 - Light Yellow
            { 0.5f, 1.0f, 1.0f },   // 淡青 - Light Cyan
        };

        MD_CORE_ASSERT(candidateHandles.size() <= 16);
        for (size_t i : selectedHandlesIndices)
        {
            // Show selected vertices.
            std::vector<glm::vec3> vertPos = { m_PsMesh->vertices[candidateHandles[i]]};
            std::vector<std::array<size_t, 2>> vertInd;
            polyscope::SurfaceGraphQuantity* showVerts = m_PsMesh->addSurfaceGraphQuantity("Selected Handles (" + std::to_string(i) + ")", vertPos, vertInd);
            showVerts->setEnabled(true);
            showVerts->setRadius(vertexRadius);
            showVerts->setColor(colors[i]);
        }
    }

    void Model::RemoveMeshFromPolyscope()
    {
        if (!m_PsMesh) { return; }

        m_PsMesh->remove();
        m_PsMesh = nullptr;
    }

    void Model::RenderMultiViewImages(glm::vec2 imageSize, const MultiViewCameras& cameras, const std::string& outputDir)
    {
        std::filesystem::create_directories(outputDir);
        for (int i = 0; i < cameras.viewMatrices.size(); i++)
        {
            std::vector<glm::vec4> image = polyscope::renderMeshImage(m_PsMesh, cameras.viewMatrices[i], cameras.projMatrix, imageSize);
            unsigned char* buffer = new unsigned char[imageSize.x * imageSize.y * 4];
            for (size_t i = 0; i < image.size(); ++i) {
                buffer[i * 4 + 0] = static_cast<unsigned char>(glm::clamp(image[i].r, 0.0f, 1.0f) * 255.0f); // R
                buffer[i * 4 + 1] = static_cast<unsigned char>(glm::clamp(image[i].g, 0.0f, 1.0f) * 255.0f); // G
                buffer[i * 4 + 2] = static_cast<unsigned char>(glm::clamp(image[i].b, 0.0f, 1.0f) * 255.0f); // B
                buffer[i * 4 + 3] = static_cast<unsigned char>(glm::clamp(image[i].a, 0.0f, 1.0f) * 255.0f); // A
            }
            polyscope::saveImage(outputDir + std::to_string(i) + ".png", buffer, imageSize.x, imageSize.y, 4);
            delete[] buffer;
        }
    }

} // namespace MeshDef