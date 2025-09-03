#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "api_shim.hpp"

namespace py = pybind11;

PYBIND11_MODULE(avbd2d, m) {
    py::class_<AvbdWorldConfig>(m, "WorldConfig")
        .def(py::init<>())
        .def_readwrite("gx", &AvbdWorldConfig::gx)
        .def_readwrite("gy", &AvbdWorldConfig::gy)
        .def_readwrite("iterations", &AvbdWorldConfig::iterations);

    py::class_<AvbdBodyState>(m, "BodyState")
        .def_readonly("x", &AvbdBodyState::x)
        .def_readonly("y", &AvbdBodyState::y)
        .def_readonly("theta", &AvbdBodyState::theta)
        .def_readonly("vx", &AvbdBodyState::vx)
        .def_readonly("vy", &AvbdBodyState::vy)
        .def_readonly("omega", &AvbdBodyState::omega);

    py::class_<AvbdWorld>(m, "World")
        .def(py::init<const AvbdWorldConfig&>(), py::arg("config"))
        .def("add_box", &AvbdWorld::add_box,
             py::arg("cx"), py::arg("cy"), py::arg("w"), py::arg("h"),
             py::arg("density")=1.0, py::arg("fixed")=false,
             py::arg("friction")=0.6)
        .def("step", &AvbdWorld::step, py::arg("dt"))
        .def("get_states", &AvbdWorld::get_states);
}