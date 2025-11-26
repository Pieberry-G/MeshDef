#include "Mesh/SimpleMesh.h"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/embed.h>

namespace MeshDef {

    SimpleMesh::SimpleMesh(const Eigen::MatrixXd& _vertices, const Eigen::MatrixXi& _faces)
        : vertices(_vertices), faces(_faces) {}

    PYBIND11_EMBEDDED_MODULE(mesh_module, m)
    {
        pybind11::class_<SimpleMesh>(m, "Mesh")
            .def(pybind11::init<>())
            .def("get_vertices", &SimpleMesh::getVertices)
            .def("set_vertices", &SimpleMesh::setVertices)
            .def("get_faces", &SimpleMesh::getFaces);
    }

} // namespace MeshDef