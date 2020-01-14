#include "igl/opengl/glfw/renderer.h"

#include <GLFW/glfw3.h>
#include <igl/unproject_onto_mesh.h>
#include "igl/look_at.h"
#include <Eigen/Dense>
#include <stdlib.h>

Renderer::Renderer() : selected_core_index(0),
                       next_core_id(2) {
    core_list.emplace_back(igl::opengl::ViewerCore());
    core_list.front().id = 1;
    // C-style callbacks
    callback_init = nullptr;
    callback_pre_draw = nullptr;
    callback_post_draw = nullptr;
    callback_mouse_down = nullptr;
    callback_mouse_up = nullptr;
    callback_mouse_move = nullptr;
    callback_mouse_scroll = nullptr;
    callback_key_down = nullptr;
    callback_key_up = nullptr;

    callback_init_data = nullptr;
    callback_pre_draw_data = nullptr;
    callback_post_draw_data = nullptr;
    callback_mouse_down_data = nullptr;
    callback_mouse_up_data = nullptr;
    callback_mouse_move_data = nullptr;
    callback_mouse_scroll_data = nullptr;
    callback_key_down_data = nullptr;
    callback_key_up_data = nullptr;
    highdpi = 1;

    xold = 0;
    yold = 0;

}

IGL_INLINE void Renderer::draw(GLFWwindow *window) {
    using namespace std;
    using namespace Eigen;

    int width, height;
    glfwGetFramebufferSize(window, &width, &height);

    int width_window, height_window;
    glfwGetWindowSize(window, &width_window, &height_window);

    auto highdpi_tmp = (width_window == 0 || width == 0) ? highdpi : (width / width_window);

    if (fabs(highdpi_tmp - highdpi) > 1e-8) {
        post_resize(window, width, height);
        highdpi = highdpi_tmp;
    }

    for (auto &core : core_list) {
        core.clear_framebuffers();
    }

    for (auto &core : core_list) {
        for (auto &mesh : scn->data_list) {
            if (mesh.is_visible & core.id) {
                core.draw(scn->MakeTrans(), mesh);
            }
        }
    }

}

void Renderer::SetScene(igl::opengl::glfw::Viewer *viewer) {
    scn = viewer;
}

IGL_INLINE void Renderer::init(igl::opengl::glfw::Viewer *viewer) {
    scn = viewer;
    core().init();

    core().align_camera_center(scn->data().V, scn->data().F);
    for (auto &mesh: scn->data_list)
        DrawBoxAndPoints(mesh);
    Reset();
    trees.resize(scn->data_list.size());
    for(auto &obj: scn->data_list){
        igl::AABB<Eigen::MatrixXd, 3> tree;
        tree.init(obj.V, obj.F);
        trees[scn->mesh_index(obj.id)] = tree;
    }
}

void Renderer::UpdatePosition(double xpos, double ypos) {
    xrel = xold - xpos;
    yrel = yold - ypos;
    xold = xpos;
    yold = ypos;
}

void Renderer::MouseProcessing(int button) {

    if (button == 1) {
        if (scn->selected_data_index == -1) {
            scn->MyTranslate(Eigen::Vector3f(-xrel / 2000.0f, 0, 0));
            scn->MyTranslate(Eigen::Vector3f(0, yrel / 2000.0f, 0));
        } else {
            scn->data().MyTranslate(Eigen::Vector3f(-xrel / 2000.0f, 0, 0));
            scn->data().MyTranslate(Eigen::Vector3f(0, yrel / 2000.0f, 0));
        }
    } else {
        if (scn->selected_data_index == -1) {
            scn->MyRotate(Eigen::Vector3f(1, 0, 0), xrel / 180.0f);
            scn->MyRotate(Eigen::Vector3f(0, 0, 1), yrel / 180.0f);
        } else {
            scn->data().MyRotate(Eigen::Vector3f(1, 0, 0), xrel / 180.0f);
            scn->data().MyRotate(Eigen::Vector3f(0, 0, 1), yrel / 180.0f);
        }
    }

}

Renderer::~Renderer() {
    //if (scn)
    //	delete scn;
}

int Renderer::Picking(double newx, double newy) {
    float objects_distance[scn->data_list.size()];
    int object_pressed[scn->data_list.size()];

    for (int i = 0; i < scn->data_list.size(); i++) {
        int fid;
        Eigen::Vector3f bc; // alphas - a1, a2, a3
        double x = newx;
        double y = core().viewport(3) - newy;

        object_pressed[i] = -1;
        objects_distance[i] = 100000001;

        Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
        igl::look_at(core().camera_eye, core().camera_center, core().camera_up, view);
        view = view * (core().trackball_angle * Eigen::Scaling(core().camera_zoom * core().camera_base_zoom)
                       * Eigen::Translation3f(core().camera_translation + core().camera_base_translation)).matrix() *
               scn->MakeTrans() * scn->data_list[i].MakeTrans();

        if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), view,
                                     core().proj, core().viewport, scn->data_list[i].V, scn->data_list[i].F, fid, bc)) {
            object_pressed[i] = 1;

            // Find triangle's vertexes coordinate - p1, p2, p3
            Eigen::Vector3f p0;
            p0[0] = scn->data_list[i].V.row(scn->data_list[i].F.row(fid)[0])[0];
            p0[1] = scn->data_list[i].V.row(scn->data_list[i].F.row(fid)[0])[1];
            p0[2] = scn->data_list[i].V.row(scn->data_list[i].F.row(fid)[0])[2];

            Eigen::Vector3f p1;
            p1[0] = scn->data_list[i].V.row(scn->data_list[i].F.row(fid)[1])[0];
            p1[1] = scn->data_list[i].V.row(scn->data_list[i].F.row(fid)[1])[1];
            p1[2] = scn->data_list[i].V.row(scn->data_list[i].F.row(fid)[1])[2];

            Eigen::Vector3f p2;
            p2[0] = scn->data_list[i].V.row(scn->data_list[i].F.row(fid)[2])[0];
            p2[1] = scn->data_list[i].V.row(scn->data_list[i].F.row(fid)[2])[1];
            p2[2] = scn->data_list[i].V.row(scn->data_list[i].F.row(fid)[2])[2];


            Eigen::Vector3f p = p0 * bc[0] + p1 * bc[1] + p2 * bc[2]; // Create p using barycentric coordinate

            Eigen::Vector4f p4; // Transform p from vector of size 3  to vector of size 4
            p4[0] = p[0];
            p4[1] = p[1];
            p4[2] = p[2];
            p4[3] = 1;

            Eigen::Vector4f p_final = view * p4;
            float distance = std::sqrt(p_final[0] * p_final[0] + p_final[1] * p_final[1] + p_final[2] * p_final[2]);
            objects_distance[i] = distance;
        }
    }

    int min_distance_index = -1;
    float min_distance = 100000000;

    // Find the closest object within the mouse coordinates
    for (int i = 0; i < scn->data_list.size(); i++) {
        if (object_pressed[i] != -1 && objects_distance[i] < min_distance) {
            min_distance = objects_distance[i];
            min_distance_index = i;
        }
    }
    return min_distance_index;
}

IGL_INLINE void Renderer::resize(GLFWwindow *window, int w, int h) {
    if (window) {
        glfwSetWindowSize(window, w / highdpi, h / highdpi);
    }
    post_resize(window, w, h);
}

IGL_INLINE void Renderer::post_resize(GLFWwindow *window, int w, int h) {
    if (core_list.size() == 1) {
        core().viewport = Eigen::Vector4f(0, 0, w, h);
    } else {
        // It is up to the user to define the behavior of the post_resize() function
        // when there are multiple viewports (through the `callback_post_resize` callback)
    }
    //for (unsigned int i = 0; i < plugins.size(); ++i)
    //{
    //	plugins[i]->post_resize(w, h);
    //}
    if (callback_post_resize) {
        callback_post_resize(window, w, h);
    }
}

IGL_INLINE igl::opengl::ViewerCore &Renderer::core(unsigned core_id /*= 0*/) {
    assert(!core_list.empty() && "core_list should never be empty");
    int core_index;
    if (core_id == 0)
        core_index = selected_core_index;
    else
        core_index = this->core_index(core_id);
    assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
    return core_list[core_index];
}

IGL_INLINE const igl::opengl::ViewerCore &Renderer::core(unsigned core_id /*= 0*/) const {
    assert(!core_list.empty() && "core_list should never be empty");
    int core_index;
    if (core_id == 0)
        core_index = selected_core_index;
    else
        core_index = this->core_index(core_id);
    assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
    return core_list[core_index];
}

IGL_INLINE bool Renderer::erase_core(const size_t index) {
    assert((index >= 0 && index < core_list.size()) && "index should be in bounds");
    //assert(data_list.size() >= 1);
    if (core_list.size() == 1) {
        // Cannot remove last viewport
        return false;
    }
    core_list[index].shut(); // does nothing
    core_list.erase(core_list.begin() + index);
    if (selected_core_index >= index && selected_core_index > 0) {
        selected_core_index--;
    }
    return true;
}

IGL_INLINE size_t Renderer::core_index(const int id) const {
    for (size_t i = 0; i < core_list.size(); ++i) {
        if (core_list[i].id == id)
            return i;
    }
    return 0;
}

IGL_INLINE int Renderer::append_core(Eigen::Vector4f viewport, bool append_empty /*= false*/) {
    core_list.push_back(core()); // copies the previous active core and only changes the viewport
    core_list.back().viewport = viewport;
    core_list.back().id = next_core_id;
    next_core_id <<= 1;
    if (!append_empty) {
        for (auto &data : scn->data_list) {
            data.set_visible(true, core_list.back().id);
            //data.copy_options(core(), core_list.back());
        }
    }
    selected_core_index = core_list.size() - 1;
    return core_list.back().id;
}

//IGL_INLINE void Viewer::select_hovered_core()
//{
//	int width_window, height_window = 800;
//   glfwGetFramebufferSize(window, &width_window, &height_window);
//	for (int i = 0; i < core_list.size(); i++)
//	{
//		Eigen::Vector4f viewport = core_list[i].viewport;

//		if ((current_mouse_x > viewport[0]) &&
//			(current_mouse_x < viewport[0] + viewport[2]) &&
//			((height_window - current_mouse_y) > viewport[1]) &&
//			((height_window - current_mouse_y) < viewport[1] + viewport[3]))
//		{
//			selected_core_index = i;
//			break;
//		}
//	}
//}

void Renderer::Reset() {
//	Scale
    if(scn->MakeTrans().row(0)(0) == 1)
        scn->MyScale(Eigen::Vector3f(0.29, 0.29, 0.29));
//	Translate
    scn->data_list[0].MyTranslate(Eigen::Vector3f(-0.50, 0.20, 0));
    scn->data_list[1].MyTranslate(Eigen::Vector3f(0.50, 0.20, 0));
//	Color selected mesh
    Eigen::MatrixXd color(1, 3);
    color << 1, 0, 0;
    if (scn->selected_data_index != -1)
        scn->data().set_colors(color);
}

void Renderer::Animate() {
    if (core().is_animating && !IsBoxesColide(scn->data_list[0], scn->data_list[1], trees[0], trees[1])) {
        if (scn->selected_data_index == 0) {
            scn->data().MyTranslate(Eigen::Vector3f(0.015, 0, 0));
        } else if (scn->selected_data_index == 1) {

            scn->data().MyTranslate(Eigen::Vector3f(-0.015, 0, 0));
        }
    }
}

void Renderer::DrawBox(igl::opengl::ViewerData &obj,
                       Eigen::MatrixXd top_points,
                       Eigen::MatrixXd bottom_points,
                       Eigen::MatrixXd color) {
    // Corners of the bounding box
    Eigen::MatrixXd V_box(8, 3);
    V_box << top_points, bottom_points;
    // Edges of the bounding box
    Eigen::MatrixXi E_box(12, 2);
    E_box <<0, 1, 1, 2,
            2, 3, 3, 0,
            4, 5, 5, 6,
            6, 7, 7, 4,
            0, 4, 1, 5,
            2, 6, 7, 3;
    for (unsigned i = 0; i < E_box.rows(); ++i){
        obj.add_edges
                (V_box.row(E_box(i, 0)),
                 V_box.row(E_box(i, 1)),
                 color
                );
    }
}

void Renderer::DrawBoxAndPoints(igl::opengl::ViewerData &obj) {
    obj.line_width = 2;
    obj.show_overlay_depth = 0;
    obj.set_face_based(!obj.face_based);
    core().toggle(obj.show_lines);
    obj.point_size = 10;

    Eigen::Vector3d m = obj.V.colwise().minCoeff();
    Eigen::Vector3d M = obj.V.colwise().maxCoeff();

    Eigen::MatrixXd top(4, 3);
    top << m(0), m(1), m(2),
            M(0), m(1), m(2),
            M(0), M(1), m(2),
            m(0), M(1), m(2);

    Eigen::MatrixXd bottom(4, 3);
    bottom << m(0), m(1), M(2),
            M(0), m(1), M(2),
            M(0), M(1), M(2),
            m(0), M(1), M(2);
    DrawBox(obj, top, bottom, Eigen::RowVector3d(0, 1, 0));

    // Plot the center of the bounding box, and center-of-rotation of the mesh
    Eigen::MatrixXd center(1, 3);
    Eigen::MatrixXd center_of_rotation(1, 3);
    center << (M(0) + m(0)) * 0.5, (M(1) + m(1)) * 0.5, (M(2) + m(2)) * 0.5;
    center_of_rotation << 0, 0, 0;
    obj.add_points(center, Eigen::RowVector3d(0, 0, 0));
    obj.add_points(center_of_rotation, Eigen::RowVector3d(0, 0, 1));
}

void Renderer::DrawSmallBox(igl::opengl::ViewerData &obj, Eigen::AlignedBox<double, 3> box){
    Eigen::MatrixXd top(4, 3);
    top << box.corner(box.TopLeftCeil)(0),box.corner(box.TopLeftCeil)(1),box.corner(box.TopLeftCeil)(2),
            box.corner(box.TopRightCeil)(0),box.corner(box.TopRightCeil)(1),box.corner(box.TopRightCeil)(2),
            box.corner(box.TopRight)(0),box.corner(box.TopRight)(1),box.corner(box.TopRight)(2),
            box.corner(box.TopLeft)(0),box.corner(box.TopLeft)(1),box.corner(box.TopLeft)(2);
    Eigen::MatrixXd bottom(4, 3);
    bottom << box.corner(box.BottomLeftCeil)(0),box.corner(box.BottomLeftCeil)(1),box.corner(box.BottomLeftCeil)(2),
            box.corner(box.BottomRightCeil)(0),box.corner(box.BottomRightCeil)(1),box.corner(box.BottomRightCeil)(2),
            box.corner(box.BottomRight)(0),box.corner(box.BottomRight)(1),box.corner(box.BottomRight)(2),
            box.corner(box.BottomLeft)(0),box.corner(box.BottomLeft)(1),box.corner(box.BottomLeft)(2);
    DrawBox(obj, top, bottom, Eigen::RowVector3d(1, 0, 1));
}


bool Renderer::IsBoxesColide(igl::opengl::ViewerData &obj1, igl::opengl::ViewerData &obj2,
                             igl::AABB<Eigen::MatrixXd, 3> tree1, igl::AABB<Eigen::MatrixXd, 3> tree2) {
    Eigen::Matrix3f A_matrix = obj1.GetRotationMatrix(); Eigen::Matrix3f B_matrix = obj2.GetRotationMatrix();

    Eigen::Vector3f A0 = A_matrix * Eigen::Vector3f(1, 0, 0); Eigen::Vector3f A1 = A_matrix * Eigen::Vector3f(0, 1, 0); Eigen::Vector3f A2 = A_matrix * Eigen::Vector3f(0, 0, 1);

    Eigen::Vector3f B0 = B_matrix * Eigen::Vector3f(1, 0, 0); Eigen::Vector3f B1 = B_matrix * Eigen::Vector3f(0, 1, 0);Eigen::Vector3f B2 = B_matrix * Eigen::Vector3f(0, 0, 1);

    Eigen::Matrix3f C = A_matrix.transpose() * B_matrix;

    float a0 = tree1.m_box.sizes()[0] / 2; float a1 = tree1.m_box.sizes()[1] / 2; float a2 = tree1.m_box.sizes()[2] / 2;
    float b0 = tree2.m_box.sizes()[0] / 2; float b1 = tree2.m_box.sizes()[1] / 2; float b2 = tree2.m_box.sizes()[2] / 2;

    float c00 = C.row(0)(0); float c01 = C.row(0)(1); float c02 = C.row(0)(2);
    float c10 = C.row(1)(0); float c11 = C.row(1)(1); float c12 = C.row(1)(2);
    float c20 = C.row(2)(0); float c21 = C.row(2)(1); float c22 = C.row(2)(2);

    Eigen::Vector4f C1; C1 << tree1.m_box.center()[0], tree1.m_box.center()[1], tree1.m_box.center()[2], 1;
    Eigen::Vector4f C2; C2 << tree2.m_box.center()[0], tree2.m_box.center()[1], tree2.m_box.center()[2], 1;

    Eigen::Vector3f C1_scn = (obj1.MakeTrans() * C1).head(3);
    Eigen::Vector3f C2_scn = (obj2.MakeTrans() * C2).head(3);

    Eigen::Vector3f D = C2_scn - C1_scn;

    if(!OBBCheckSat({A0,A1,A2,B0,B1,B2,a0,a1,a2,b0,b1,b2,c00,c01,c02,c10,c11,c12,c20,c21,c22,D})){
        if(tree1.is_leaf()) {
            if (tree2.is_leaf()) {
                    DrawSmallBox(obj1, tree1.m_box);
                    DrawSmallBox(obj2, tree2.m_box);
                    return true;
            } else {
                return IsBoxesColide(obj1, obj2, tree1, *tree2.m_left) || IsBoxesColide(obj1, obj2, tree1, *tree2.m_right);
            }
        }
        else if(tree2.is_leaf()) {
            return IsBoxesColide(obj1, obj2, *tree1.m_left, tree2) || IsBoxesColide(obj1, obj2, *tree1.m_right, tree2);
        }
        else {
            return  IsBoxesColide(obj1, obj2, *tree1.m_left, *tree2.m_left) ||
                    IsBoxesColide(obj1, obj2, *tree1.m_right, *tree2.m_left)||
                    IsBoxesColide(obj1, obj2, *tree1.m_left, *tree2.m_right)||
                    IsBoxesColide(obj1, obj2, *tree1.m_right, *tree2.m_right);
        }
    }
    return false;
}

bool Renderer::OBBCheckSat(OBBSatVars vars){
    bool test1  = vars.a0 + (vars.b0 * abs(vars.c00) + vars.b1 * abs(vars.c01) + vars.b2 * abs(vars.c02)) < abs(vars.A0.dot(vars.D));
    bool test2  = vars.a1 + (vars.b0 * abs(vars.c10) + vars.b1 * abs(vars.c11) + vars.b2 * abs(vars.c12)) < abs(vars.A1.dot(vars.D));
    bool test3  = vars.a2 + (vars.b0 * abs(vars.c20) + vars.b1 * abs(vars.c21) + vars.b2 * abs(vars.c22)) < abs(vars.A2.dot(vars.D));

    bool test4  = (vars.a0 * abs(vars.c00) + vars.a1 * abs(vars.c10) + vars.a2 * abs(vars.c20)) + vars.b0 < abs(vars.B0.dot(vars.D));
    bool test5  = (vars.a0 * abs(vars.c01) + vars.a1 * abs(vars.c11) + vars.a2 * abs(vars.c21)) + vars.b1 < abs(vars.B1.dot(vars.D));
    bool test6  = (vars.a0 * abs(vars.c02) + vars.a1 * abs(vars.c12) + vars.a2 * abs(vars.c22)) + vars.b2 < abs(vars.B2.dot(vars.D));

    bool test7  = (vars.a1 * abs(vars.c20) + vars.a2 * abs(vars.c10)) + (vars.b1 * abs(vars.c02) + vars.b2 * abs(vars.c01)) < abs(vars.c10 * vars.A2.dot(vars.D) - vars.c20 * vars.A1.dot(vars.D));
    bool test8  = (vars.a1 * abs(vars.c21) + vars.a2 * abs(vars.c11)) + (vars.b0 * abs(vars.c02) + vars.b2 * abs(vars.c00)) < abs(vars.c11 * vars.A2.dot(vars.D) - vars.c21 * vars.A1.dot(vars.D));
    bool test9  = (vars.a1 * abs(vars.c22) + vars.a2 * abs(vars.c12)) + (vars.b0 * abs(vars.c01) + vars.b1 * abs(vars.c00)) < abs(vars.c12 * vars.A2.dot(vars.D) - vars.c22 * vars.A1.dot(vars.D));

    bool test10 = (vars.a0 * abs(vars.c20) + vars.a2 * abs(vars.c00)) + (vars.b1 * abs(vars.c12) + vars.b2 * abs(vars.c11)) < abs(vars.c20 * vars.A0.dot(vars.D) - vars.c00 * vars.A2.dot(vars.D));
    bool test11 = (vars.a0 * abs(vars.c21) + vars.a2 * abs(vars.c01)) + (vars.b0 * abs(vars.c12) + vars.b2 * abs(vars.c10)) < abs(vars.c21 * vars.A0.dot(vars.D) - vars.c01 * vars.A2.dot(vars.D));
    bool test12 = (vars.a0 * abs(vars.c22) + vars.a2 * abs(vars.c02)) + (vars.b0 * abs(vars.c11) + vars.b1 * abs(vars.c10)) < abs(vars.c22 * vars.A0.dot(vars.D) - vars.c02 * vars.A2.dot(vars.D));

    bool test13 = (vars.a0 * abs(vars.c10) + vars.a1 * abs(vars.c00)) + (vars.b1 * abs(vars.c22) + vars.b2 * abs(vars.c21)) <  abs(vars.c00 * vars.A1.dot(vars.D) - vars.c10 * vars.A0.dot(vars.D));
    bool test14 = (vars.a0 * abs(vars.c11) + vars.a1 * abs(vars.c01)) + (vars.b0 * abs(vars.c22) + vars.b2 * abs(vars.c20)) <  abs(vars.c01 * vars.A1.dot(vars.D) - vars.c11 * vars.A0.dot(vars.D));
    bool test15 = (vars.a0 * abs(vars.c12) + vars.a1 * abs(vars.c02)) + (vars.b0 * abs(vars.c21) + vars.b1 * abs(vars.c20)) <  abs(vars.c02 * vars.A1.dot(vars.D) - vars.c12 * vars.A0.dot(vars.D));

    return test1 || test2 || test3 || test4 || test5 ||
           test6 || test7 || test8 || test9 ||test10 ||
           test11|| test12||test13 || test14||test15;
}



