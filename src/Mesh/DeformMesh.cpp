#include "Mesh/DeformMesh.h"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/embed.h>

namespace MeshDef {

    PYBIND11_EMBEDDED_MODULE(mesh_module, m)
    {
        pybind11::class_<DeformMesh>(m, "Mesh")
            .def(pybind11::init<>())
            .def("get_vertices", &DeformMesh::getVertices)
            .def("get_faces", &DeformMesh::getFaces)
            .def("get_fixed_vert_indices", &DeformMesh::getFixedVertIndices)
            .def("get_moving_vert_indices", &DeformMesh::getMovingVertIndices)
            .def("get_target_positions", &DeformMesh::getTargetPositions);
    }

} // namespace MeshDef