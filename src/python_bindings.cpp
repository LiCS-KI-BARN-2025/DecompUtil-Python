#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_util/seed_decomp.h>
#include <decomp_util/iterative_decomp.h>

namespace py = pybind11;

// Vector conversion for arrays
template <int Dim>
Vecf<Dim> vector_from_python(py::array_t<double> array) {
    py::buffer_info buf = array.request();
    if (buf.ndim != 1 || buf.shape[0] != Dim)
        throw std::runtime_error("Input array must be 1-dimensional with correct size");
    double* ptr = static_cast<double*>(buf.ptr);
    return Eigen::Map<Vecf<Dim>>(ptr, Dim);
}

// Matrix conversion for arrays
template <int Dim>
Matf<Dim,Dim> matrix_from_python(py::array_t<double> array) {
    py::buffer_info buf = array.request();
    if (buf.ndim != 2 || buf.shape[0] != Dim || buf.shape[1] != Dim)
        throw std::runtime_error("Input array must be 2-dimensional with correct size");
    double* ptr = static_cast<double*>(buf.ptr);
    return Eigen::Map<Matf<Dim,Dim>>(ptr, Dim, Dim);
}

template <int Dim>
void declare_hyperplane(py::module &m, const std::string &typestr) {
    using Class = Hyperplane<Dim>;
    py::class_<Class>(m, (std::string("Hyperplane") + typestr).c_str())
        .def(py::init<>())
        .def_readwrite("p_", &Class::p_)
        .def_readwrite("n_", &Class::n_);
}

template <int Dim>
void declare_polyhedron(py::module &m, const std::string &typestr) {
    using Class = Polyhedron<Dim>;
    py::class_<Class>(m, (std::string("Polyhedron") + typestr).c_str())
        .def(py::init<>())
        .def("add", &Class::add)
        .def("hyperplanes", &Class::hyperplanes)
        .def("vertices", [](const Class& poly) {
            return cal_vertices(poly);
        })
        .def("inside", &Class::inside);
}

template <int Dim>
void declare_ellipsoid(py::module &m, const std::string &typestr) {
    using Class = Ellipsoid<Dim>;
    auto cls = py::class_<Class, std::shared_ptr<Class>>(m, (std::string("Ellipsoid") + typestr).c_str())
        .def(py::init<>())
        .def(py::init([](py::array_t<double> C, py::array_t<double> d) {
            return std::make_unique<Class>(
                matrix_from_python<Dim>(C),
                vector_from_python<Dim>(d));
        }))
        .def_property_readonly("C_", [](const Class& e) { return e.C_; })
        .def_property_readonly("d_", [](const Class& e) { return e.d_; })
        .def("dist", &Class::dist)
        .def("points_inside", &Class::points_inside)
        .def("closest_point", &Class::closest_point);

    // Only add sample() method for 2D
    if constexpr (Dim == 2) {
        cls.def("sample", [](const Class& e, int num) {
            return e.sample(num);
        });
    }
}

template <int Dim>
void declare_decomp_base(py::module &m, const std::string &typestr) {
    py::class_<DecompBase<Dim>, std::shared_ptr<DecompBase<Dim>>>(m, (std::string("DecompBase") + typestr).c_str())
        .def("set_obs", &DecompBase<Dim>::set_obs)
        .def("get_obs", &DecompBase<Dim>::get_obs)
        .def("get_ellipsoid", &DecompBase<Dim>::get_ellipsoid)
        .def("get_polyhedron", &DecompBase<Dim>::get_polyhedron)
        .def("set_local_bbox", &DecompBase<Dim>::set_local_bbox);
}

template <int Dim>
void declare_line_segment(py::module &m, const std::string &typestr) {
    using Class = LineSegment<Dim>;
    py::class_<Class, DecompBase<Dim>, std::shared_ptr<Class>>(m, (std::string("LineSegment") + typestr).c_str())
        .def(py::init<>())
        .def(py::init([](py::array_t<double> p1, py::array_t<double> p2) {
            py::buffer_info buf1 = p1.request();
            py::buffer_info buf2 = p2.request();
            
            if (buf1.ndim != 1 || buf1.shape[0] != Dim || 
                buf2.ndim != 1 || buf2.shape[0] != Dim)
                throw std::runtime_error("Input arrays must be 1-dimensional with size " + std::to_string(Dim));
            
            Vecf<Dim> v1, v2;
            double* ptr1 = static_cast<double*>(buf1.ptr);
            double* ptr2 = static_cast<double*>(buf2.ptr);
            
            for(int i = 0; i < Dim; i++) {
                v1[i] = ptr1[i];
                v2[i] = ptr2[i];
            }
            
            return std::make_shared<Class>(v1, v2);
        }))
        .def("dilate", &Class::dilate)
        .def("get_line_segment", &Class::get_line_segment)
        .def("set_obs", [](Class& self, py::array_t<double> obs) {
            py::buffer_info buf = obs.request();
            if (buf.ndim != 2 || buf.shape[1] != Dim)
                throw std::runtime_error("Obstacles must be Nx" + std::to_string(Dim) + " array");
            
            vec_Vecf<Dim> obstacles;
            double* ptr = static_cast<double*>(buf.ptr);
            
            for(size_t i = 0; i < buf.shape[0]; i++) {
                Vecf<Dim> point;
                for(int j = 0; j < Dim; j++) {
                    point[j] = ptr[i * Dim + j];
                }
                obstacles.push_back(point);
            }
            self.set_obs(obstacles);
        })
        .def("get_obs", &Class::get_obs)
        .def("get_ellipsoid", &Class::get_ellipsoid)
        .def("get_polyhedron", &Class::get_polyhedron)
        .def("set_local_bbox", [](Class& self, py::array_t<double> bbox) {
            py::buffer_info buf = bbox.request();
            if (buf.ndim != 1 || buf.shape[0] != Dim)
                throw std::runtime_error("Bbox must be " + std::to_string(Dim) + "-dimensional array");
            
            Vecf<Dim> bbox_vec;
            double* ptr = static_cast<double*>(buf.ptr);
            for(int i = 0; i < Dim; i++) {
                bbox_vec[i] = ptr[i];
            }
            self.set_local_bbox(bbox_vec);
        });
}

template <int Dim>
void declare_ellipsoid_decomp(py::module &m, const std::string &typestr) {
    using Class = EllipsoidDecomp<Dim>;
    py::class_<Class, std::shared_ptr<Class>>(m, (std::string("EllipsoidDecomp") + typestr).c_str())
        .def(py::init<>())
        .def(py::init([](py::array_t<double> origin, py::array_t<double> dim) {
            return std::make_shared<Class>(
                vector_from_python<Dim>(origin),
                vector_from_python<Dim>(dim));
        }))
        .def("set_obs", [](Class& self, py::array_t<double> obs) {
            py::buffer_info buf = obs.request();
            if (buf.ndim != 2 || buf.shape[1] != Dim)
                throw std::runtime_error("Obstacles must be Nx" + std::to_string(Dim) + " array");
            
            vec_Vecf<Dim> obstacles;
            double* ptr = static_cast<double*>(buf.ptr);
            for(size_t i = 0; i < buf.shape[0]; i++) {
                Vecf<Dim> point;
                for(int j = 0; j < Dim; j++) {
                    point[j] = ptr[i * Dim + j];
                }
                obstacles.push_back(point);
            }
            self.set_obs(obstacles);
        })
        .def("dilate", [](Class& self, py::array_t<double> path, double offset_x = 0.0) {
            py::buffer_info buf = path.request();
            if (buf.ndim != 2 || buf.shape[1] != Dim)
                throw std::runtime_error("Path must be Nx" + std::to_string(Dim) + " array");
            
            vec_Vecf<Dim> path_vec;
            double* ptr = static_cast<double*>(buf.ptr);
            for(size_t i = 0; i < buf.shape[0]; i++) {
                Vecf<Dim> point;
                for(int j = 0; j < Dim; j++) {
                    point[j] = ptr[i * Dim + j];
                }
                path_vec.push_back(point);
            }
            self.dilate(path_vec, offset_x);
        })
        .def("set_local_bbox", [](Class& self, py::array_t<double> bbox) {
            self.set_local_bbox(vector_from_python<Dim>(bbox));
        })
        .def("get_path", &Class::get_path)
        .def("get_ellipsoids", &Class::get_ellipsoids)
        .def("get_polyhedrons", &Class::get_polyhedrons);
}

template <int Dim>
void declare_iterative_decomp(py::module &m, const std::string &typestr) {
    using Class = IterativeDecomp<Dim>;
    py::class_<Class, EllipsoidDecomp<Dim>, std::shared_ptr<Class>>(m, (std::string("IterativeDecomp") + typestr).c_str())
        .def(py::init<>())
        .def(py::init([](py::array_t<double> origin, py::array_t<double> dim) {
            return std::make_shared<Class>(
                vector_from_python<Dim>(origin),
                vector_from_python<Dim>(dim));
        }))
        .def("dilate_iter", [](Class& self, py::array_t<double> path, int iter_num, double res, double offset_x) {
            py::buffer_info buf = path.request();
            if (buf.ndim != 2 || buf.shape[1] != Dim)
                throw std::runtime_error("Path must be Nx" + std::to_string(Dim) + " array");
            
            vec_Vecf<Dim> path_vec;
            double* ptr = static_cast<double*>(buf.ptr);
            for(size_t i = 0; i < buf.shape[0]; i++) {
                Vecf<Dim> point;
                for(int j = 0; j < Dim; j++) {
                    point[j] = ptr[i * Dim + j];
                }
                path_vec.push_back(point);
            }
            self.dilate_iter(path_vec, iter_num, res, offset_x);
        });
}

template <int Dim>
void declare_seed_decomp(py::module &m, const std::string &typestr) {
    using Class = SeedDecomp<Dim>;
    py::class_<Class, DecompBase<Dim>, std::shared_ptr<Class>>(m, (std::string("SeedDecomp") + typestr).c_str())
        .def(py::init<>())
        .def(py::init([](py::array_t<double> p) {
            return std::make_shared<Class>(vector_from_python<Dim>(p));
        }))
        .def("set_obs", [](Class& self, py::array_t<double> obs) {
            py::buffer_info buf = obs.request();
            if (buf.ndim != 2 || buf.shape[1] != Dim)
                throw std::runtime_error("Obstacles must be Nx" + std::to_string(Dim) + " array");
            
            vec_Vecf<Dim> obstacles;
            double* ptr = static_cast<double*>(buf.ptr);
            for(size_t i = 0; i < buf.shape[0]; i++) {
                Vecf<Dim> point;
                for(int j = 0; j < Dim; j++) {
                    point[j] = ptr[i * Dim + j];
                }
                obstacles.push_back(point);
            }
            self.set_obs(obstacles);
        })
        .def("dilate", &SeedDecomp<Dim>::dilate)
        .def("get_seed", &SeedDecomp<Dim>::get_seed)
        .def("set_local_bbox", [](Class& self, py::array_t<double> bbox) {
            self.set_local_bbox(vector_from_python<Dim>(bbox));
        });
}

PYBIND11_MODULE(pydecomp_util, m) {
    m.doc() = "Python bindings for DecompUtil library";

    // Bind 2D variants
    declare_hyperplane<2>(m, "2D");
    declare_polyhedron<2>(m, "2D");
    declare_ellipsoid<2>(m, "2D");
    declare_decomp_base<2>(m, "2D");
    declare_line_segment<2>(m, "2D");
    declare_ellipsoid_decomp<2>(m, "2D");
    declare_iterative_decomp<2>(m, "2D");
    declare_seed_decomp<2>(m, "2D");

    // Bind 3D variants
    declare_hyperplane<3>(m, "3D");
    declare_polyhedron<3>(m, "3D");
    declare_ellipsoid<3>(m, "3D");
    declare_decomp_base<3>(m, "3D");
    declare_line_segment<3>(m, "3D");
    declare_ellipsoid_decomp<3>(m, "3D");
    declare_iterative_decomp<3>(m, "3D");
    declare_seed_decomp<3>(m, "3D");
}