#include "Mesh/MeshUtils.h"

#include "Mesh/Model.h"
#include "Mesh/EditMesh.h"
#include "Mesh/OBJLoader.h"

#include "PlatformUtils/PythonInterpreter.h"

#include <glad/glad.h>

#include <Eigen/Dense>

#include <ConeOptimization.h>
#include <MeshDefinition.h>
 
namespace MeshDef {

    static void saveCones(const std::vector<double>& conesK, std::string conesPath, Mesh& mesh, double eps = 1e-9)
    {
        std::ofstream conesFile(conesPath);
        if (conesFile.fail())
        {
            std::cout << "Open " << conesPath << "failed\n";
            exit(EXIT_FAILURE);
        }

        for (int i = 0; i < conesK.size(); ++i)
        {
            if ((conesK[i] > -eps && conesK[i] < eps)||mesh.is_boundary(mesh.vertex_handle(i))) continue;
            conesFile << i + 1 << " " << conesK[i] << std::endl;
        }
        conesFile.close();
    }

    std::vector<size_t> CalculateCandidateHandles(const std::vector<std::array<double, 3>>& vertices, const std::vector<std::array<int, 3>>& faces)
    {
        Mesh mesh;
        MeshTools::CreateMesh(mesh, vertices, faces);

        for (float distortion = 0.1f; distortion < 1.0f; distortion += 0.1f)
        {
            ConeOptimization ConeOpt;
            ConeOpt.Initialization(mesh, distortion);
            ConeOpt.Optimization();
            // saveCones(ConeOpt.kc, "-cones.txt", mesh);

            double eps = 1e-9;
            std::vector<size_t> candidateHandles;
            for (int i = 0; i < ConeOpt.kc.size(); ++i)
            {
                if ((ConeOpt.kc[i] > -eps && ConeOpt.kc[i] < eps) || mesh.is_boundary(mesh.vertex_handle(i))) continue;
                candidateHandles.push_back(i);
            }
            
            if (candidateHandles.size() <= 16) return candidateHandles;
        }
    }

    std::vector<size_t> SelectHandlesByVLM(const std::string& imagesDir)
    {
        pybind11::object result = pybind11::module_::import("handle_selection").attr("select_handles")(imagesDir);
        
        std::vector<size_t> selectedHandlesIndex;
        std::istringstream iss(result.cast<std::string>());
        std::string token;
    
        while (std::getline(iss, token, ','))
        {
            selectedHandlesIndex.push_back(std::stoull(token));
        }
        return selectedHandlesIndex;
    }

    bool VertInsideSelectBox(const Eigen::Vector3d &botLeftOrigin,
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