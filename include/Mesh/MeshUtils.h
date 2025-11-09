/* Copyright (c) Russell Gillette
 * December 2013
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
 * to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/* == MeshUtils.h ==
 * 
 * Utility and helper functions that operate on meshes or mesh data without a
 * specified Mesh instance.
 */

#pragma once

#include "Mesh/Model.h"
#include "Mesh/EditMesh.h"
#include "Mesh/OBJLoader.h"

#include "Eigen/Dense"

#include "glad/glad.h"
 
namespace MeshDef {

    inline bool VertInsideSelectBox (const Eigen::Vector3d &botLeftOrigin,
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
    EditMesh *loadEditMeshFromFile(std::string file_name)
    {
        EditMesh *m = NULL;

        // parse mesh
        ObjLoader obj_parser(file_name);
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
        m->init(xyzPositions, triangleIndices);

        return m;
    }

    std::unique_ptr<Model> loadModelFromFile(std::string file_name)
    {
        EditMesh *em = loadEditMeshFromFile(file_name);
        return std::make_unique<Model>(EditMeshPtr(em));
    }

} // namespace MeshDef