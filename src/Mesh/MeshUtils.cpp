#include "Mesh/MeshUtils.h"

#include "Mesh/Model.h"
#include "Mesh/EditMesh.h"
#include "Mesh/OBJLoader.h"

#include "Eigen/Dense"

#include "glad/glad.h"
 
namespace MeshDef {

    glm::mat4 Orthographic(float right, float left, float top, float bottom, float zNear, float zFar)
    {
        if (left == 0.0f) { left = -right; }
        if (top == 0.0f) { top = right; }
        if (bottom == 0.0f) { bottom = -top; }
        
        return glm::ortho(left, right, bottom, top, zNear, zFar);
    }

    glm::mat4 Perspective(float fovy, float aspect, float zNear, float zFar)
    {
        return glm::perspective(fovy, aspect, zNear, zFar);
    }

    std::vector<glm::mat4> Views(float distance, float right, std::vector<int> angles)
    {
        if (right == 0.0f) { right = 1.0f / distance; }

        if (angles.empty())
        {
            angles = { 0, -45, -90, 180, 90, 45 };
        }

        std::vector<glm::mat4> views(angles.size(), glm::mat4(1.0f));

        for (int i = 0; i < views.size(); i++)
        {
            glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), glm::radians((float)angles[i]), glm::vec3(0.0f, 1.0f, 0.0f));
            glm::mat4 translation = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -distance));
            views[i] = translation * rotation;
        }
        
        return views;
    }


    void MakeCameras(std::vector<glm::mat4>& viewMatrices, glm::mat4& projMatrix, const std::string& camType, const std::vector<int>& angles)
    {
        if (camType == "ortho")
        {
            float distance = 10.0f;
            float right = 1.0f;
            viewMatrices = Views(distance, right, angles);
            projMatrix = Orthographic(1.0f);
        }
        else
        {
            float distance = 1 / glm::tan(glm::radians(49.13f) / 2.0);
            float right = 1.0f;
            viewMatrices = Views(distance, right, angles);
            projMatrix = Perspective();
        }
    }
    

    bool VertInsideSelectBox (const Eigen::Vector3d &botLeftOrigin,
                                        const Eigen::Vector3d &pointOnBotLeftRay,
                                        const Eigen::Vector3d &botRightOrigin,
                                        const Eigen::Vector3d &pointOnBotRightRay,
                                        const Eigen::Vector3d &topRightOrigin,
                                        const Eigen::Vector3d &pointOnTopRightRay,
                                        const Eigen::Vector3d &topLeftOrigin,
                                        const Eigen::Vector3d &pointOnTopLeftRay,
                                        Eigen::Vector3d vertex)
    {
        Eigen::Vector3d ray, tmp, normalLeft, normalRight, normalBot, normalTop;
        ray  = pointOnBotLeftRay - botLeftOrigin;
        tmp = topLeftOrigin - botLeftOrigin;
        normalLeft  = ray.cross(tmp);

        ray = pointOnBotRightRay - botRightOrigin;
        tmp = botLeftOrigin - botRightOrigin;
        normalBot   = ray.cross(tmp);

        ray = pointOnTopRightRay - topRightOrigin;
        tmp = botRightOrigin - topRightOrigin;
        normalRight = ray.cross(tmp);

        ray = pointOnTopLeftRay - topLeftOrigin;
        tmp = topRightOrigin - topLeftOrigin;
        normalTop   = ray.cross(tmp);

        Eigen::Vector3d bot_left_check  = vertex - botLeftOrigin;
        Eigen::Vector3d top_right_check = vertex - topRightOrigin;

        return (bot_left_check.dot(normalLeft)   > 0 &&
                bot_left_check.dot(normalBot)    > 0 &&
                top_right_check.dot(normalRight) > 0 &&
                top_right_check.dot(normalTop)   > 0);
    }

    // load an Edit Mesh object from an OBJ file.
    EditMesh* LoadEditMeshFromOBJFile(const std::string& filepath)
    {
        EditMesh *m = NULL;

        // parse mesh
        ObjLoader obj_parser(filepath);
        //obj_parser.generateNormals(); // don't use normals atm so no point in doing this

        // gather data from the parser to construct mesh
        GLubyte *data = NULL;
        int *indices;
        int data_size, num_indices, num_attr;
        obj_attrib_info *parsed_attr_info;

        // export parser data
        // NOTE: data is owned by the obj_parser
        obj_parser.objExportGLSeparate(data_size, data, num_indices, indices, num_attr, parsed_attr_info);

        if (parsed_attr_info == NULL)
        {
            printf("Mesh Not Found: Failed to load\n");
            return NULL;
        }

        std::vector<double> xyzPositions;
        std::vector<std::size_t> triangleIndices;

        // populate indices
        for (int i = 0; i < num_indices; i++)
            triangleIndices.push_back(indices[i]);
        
        // populate vertices from byte array (so lots of pointer pushing)
        int attr_end = data_size;
        int v_stride = parsed_attr_info[0].data_stride;
        int v_offset = parsed_attr_info[0].data_offset;

        if (v_stride == 0)
        for (int i = 0; i < num_attr; i++)
        {
            int off = parsed_attr_info[i].data_offset;
            if (off < attr_end && off > v_offset)
                attr_end = off;
        }

        int      attrib_size = parsed_attr_info[0].attrib_size;
        int      elem_size = attrib_size / parsed_attr_info[0].num_comp;

        GLubyte *pAttrib = data;
        GLubyte *pEnd    = data + attr_end;

        // TODO: safety check on number of elements per attribute
        // (there should never be less than 3 but who knows)
        // if (sizeof(float) != elem_size)
        //    unhandled as of right now

        // not particularily safe...
        for (; pAttrib < pEnd; pAttrib += elem_size)
        {
            double tmp = (double)*(float*)pAttrib;
            xyzPositions.push_back(tmp);
        }

        m = new EditMesh();
        m->Init(xyzPositions, triangleIndices);

        return m;
    }

    std::unique_ptr<Model> LoadModelFromFile(const std::string& filepath)
    {
        std::filesystem::path fsPath(filepath);
        std::string meshName = fsPath.stem().string();
        EditMesh* mesh = LoadEditMeshFromOBJFile(filepath);
        return std::make_unique<Model>(EditMeshPtr(mesh), meshName);
    }

} // namespace MeshDef