#include "igl/opengl/glfw/renderer.h"

#include <GLFW/glfw3.h>
#include <igl/unproject_onto_mesh.h>
#include "igl/look_at.h"
#include <Eigen/Dense>

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
                Eigen::Matrix4f scn_trans = scn->MakeTrans() * GetAncestorTransIfNeeded(mesh.id);
                core.draw(scn_trans, mesh, static_cast<int>(scn->data_list.size() - 1));
            }
        }
    }

}

Eigen::Matrix4f Renderer::GetAncestorTrans(int link_index) {
    Eigen::Matrix4f links = Eigen::Matrix4f::Identity();
    for(int i = 0; i < link_index; i++){
        links = links * scn->data_list[i].MakeConnectedTrans();
    }
    return links;
}

Eigen::Matrix3f Renderer::GetAncestorInverse(int link_index) {
    Eigen::Matrix3f links = scn->data_list[link_index - 1].GetRotationMatrix().inverse();
    for(int i = link_index - 2; i > 0; i--){
        links = links * scn->data_list[i].GetRotationMatrix().inverse() * links;
    }
    return links;
}


void Renderer::init_system() {
    int i = 0;
    scn->links.resize(scn->data_list.size() - 1);
    scn->parents_axis.resize(scn->data_list.size() - 1);

    for (; i < scn->data_list.size() - 1; i++) {
        scn->data_list[i].line_width = 3;
        scn->data_list[i].show_overlay_depth = 0;
        scn->data_list[i].point_size = 10;
        scn->links_number += 1;
        scn->data_list[i].set_face_based(!scn->data_list[i].face_based);
        core().toggle(scn->data_list[i].show_lines);


        Eigen::Vector3d m = scn->data_list[i].V.colwise().minCoeff();
        Eigen::Vector3d M = scn->data_list[i].V.colwise().maxCoeff();
        double link_height = M(1) - m(1);
        scn->data_list[i].SetCenterOfRotation(Eigen::Vector3f(scn->data_list[i].V.colwise().mean()[0], m[1],
                                                              scn->data_list[i].V.colwise().mean()[0]));

        if (i == 0)
            scn->data_list[i].MyPreTranslate(Eigen::Vector3f(0, 0, 0));
        else
            scn->data_list[i].MyPreTranslate(Eigen::Vector3f(0, link_height, 0));
    }
    scn->data_list[i].MyPreTranslate(Eigen::Vector3f(5, 0, 0));
    scn->data_list[0].MyPreTranslate(Eigen::Vector3f(0, -1, 0));

    scn->MyScale(Eigen::Vector3f(0.2, 0.2, 0.2));
}

IGL_INLINE void Renderer::init(igl::opengl::glfw::Viewer *viewer) {
    scn = viewer;

    init_system();

    core().init();
    core().align_camera_center(scn->data().V, scn->data().F);


}


void Renderer::resize_by_scrolling(double x, double y) {
    igl::opengl::glfw::Viewer *scn = GetScene();
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    igl::look_at(core().camera_eye, core().camera_center, core().camera_up, view);
    view = view * (core().trackball_angle * Eigen::Scaling(core().camera_zoom * core().camera_base_zoom)
                   * Eigen::Translation3f(core().camera_translation + core().camera_base_translation)).matrix()
           * GetScene()->MakeTrans();


    if (scn->selected_data_index == -1) {
        GetScene()->MyTranslate((view * Eigen::Vector4f(0, 0, 1, 1)).head(3) * y * 0.01, prerotation);
    } else {
        if (scn->selected_data_index == scn->data_list.size() - 1) {
            GetScene()->data().MyTranslate((view * Eigen::Vector4f(0, 0, 1, 1)).head(3) * y * 0.01, prerotation, scn);
        } else {
            GetScene()->data_list[0].MyTranslate((view * Eigen::Vector4f(0, 0, 1, 1)).head(3) * y * 0.01 , prerotation, scn);
        }
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
        if (scn->selected_data_index == -1) { // System Translate
            scn->TranslateInSystem(scn->MakeTrans(), Eigen::Vector3f(-xrel / 200.0f, 0, 0));
            scn->TranslateInSystem(scn->MakeTrans(), Eigen::Vector3f(0, yrel / 200.0f, 0));

        } else {                              // Object Translate
            if (scn->selected_data_index == scn->data_list.size() - 1) {
                scn->data().MyTranslate(Eigen::Vector3f(-xrel / 200.0f, 0, 0), prerotation);
                scn->data().MyTranslate(Eigen::Vector3f(0, yrel / 200.0f, 0), prerotation);
            } else {
                scn->data_list[0].MyTranslate(Eigen::Vector3f(-xrel / 200.0f, 0, 0), prerotation);
                scn->data_list[0].MyTranslate(Eigen::Vector3f(0, yrel / 200.0f, 0), prerotation);
            }
        }
    } else {
        if (scn->selected_data_index == -1) { // System Rotate
            prerotation = false;
            scn->RotateInSystem(Eigen::Vector3f(1, 0, 0), xrel / 180.0f);
            scn->RotateInSystem(Eigen::Vector3f(0, 0, 1), yrel / 180.0f);
        } else {                              // Object Rotate
            scn->data().RotateInSystem(Eigen::Vector3f(1, 0, 0), xrel / 180.0f);
            scn->data().RotateInSystem(Eigen::Vector3f(0, 0, 1), yrel / 180.0f);
        }
    }
}

Renderer::~Renderer() {
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
        objects_distance[i] = INFINITY;

        Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
        igl::look_at(core().camera_eye, core().camera_center, core().camera_up, view);
        view = view
               * (core().trackball_angle * Eigen::Scaling(core().camera_zoom * core().camera_base_zoom)
               * Eigen::Translation3f(core().camera_translation + core().camera_base_translation)).matrix()
               * scn->MakeTrans()
               * GetAncestorTransIfNeeded(i)
               * scn->data_list[i].GetConnectedTransIfNeeded(i, static_cast<int>(scn->data_list.size() - 1));

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
    float min_distance = INFINITY;

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


IGL_INLINE size_t Renderer::core_index(const int id) const {
    for (size_t i = 0; i < core_list.size(); ++i) {
        if (core_list[i].id == id)
            return i;
    }
    return 0;
}


void Renderer::IK_Solver() {
    Eigen::Vector4f first_link = GetAncestorTrans(1) * Eigen::Vector4f(scn->data_list[0].V.colwise().mean()[0],
                                                                       scn->data_list[0].V.colwise().minCoeff()[1],
                                                                       scn->data_list[0].V.colwise().mean()[2],
                                                                       1);
    Eigen::Vector3f first_link_B = first_link.head(3);

    int last_link_index = static_cast<int>(scn->data_list.size() - 2);
    Eigen::Vector4f last_link =  Eigen::Vector4f(scn->data_list[last_link_index].V.colwise().mean()[0],
                                                 scn->data_list[last_link_index].V.colwise().maxCoeff()[1],
                                                 scn->data_list[last_link_index].V.colwise().mean()[2],
                                                 1);
    Eigen::Vector3f curr_last_link_E = (GetAncestorTrans(last_link_index + 1) * last_link).head(3);

    Eigen::Vector3f object_D = scn->data_list[scn->data_list.size() - 1].MakeTrans().col(3).head(3);
    double distance_D_E = (object_D - curr_last_link_E).norm();
    double distance_D_Base = (object_D - first_link_B).norm();
    double max_arm_length = 1.6 * scn->links_number;
    if (distance_D_Base > max_arm_length) {
        std::cout << "cannot reach" << std::endl;
        should_animate = false;
        return;
    }
    if (distance_D_E < 0.1) {
        should_animate = false;
        return;
    }


    for (int i = last_link_index; i > -1; i--) {

        Eigen::Vector4f curr_link = GetAncestorTransIfNeeded(i + 1) * Eigen::Vector4f(scn->data_list[i].V.colwise().mean()[0],
                                                                              scn->data_list[i].V.colwise().minCoeff()[1],
                                                                              scn->data_list[i].V.colwise().mean()[2],
                                                                              1);
        Eigen::Vector3f curr_link_R = curr_link.head(3);

        Eigen::Vector3f RD = (object_D - curr_link_R).normalized();
        Eigen::Vector3f RE = (curr_last_link_E - curr_link_R).normalized();
        float pre_angle = RE.dot(RD);
        if (pre_angle > 1) {
            pre_angle = 1;
        } else if (pre_angle < -1)
            pre_angle = -1;

        float angle = acosf(pre_angle);

        Eigen::Vector3f cross = RE.cross(RD).normalized();
        Eigen::Vector3f crossInverse = GetAncestorInverseIfNeeded(last_link_index) * cross;
        if (distance_D_E > 0.5)
            angle = angle / 10;
        scn->data_list[i].MyRotate(crossInverse, angle);

        curr_last_link_E = (GetAncestorTrans(last_link_index + 1) * last_link).head(3);
        distance_D_E = (object_D - curr_last_link_E).norm();
    }
}

IGL_INLINE int Renderer::append_core(Eigen::Vector4f viewport, bool append_empty /*= false*/)
{
    core_list.push_back(core()); // copies the previous active core and only changes the viewport
    core_list.back().viewport = viewport;
    core_list.back().id = next_core_id;
    next_core_id <<= 1;
    if (!append_empty)
    {
        for (auto& data : scn->data_list)
        {
//            core_list.back().
            data.set_visible(true, core_list.back().id);
            data.copy_options(core(), core_list.back());
        }
    }
    selected_core_index = core_list.size() - 1;
    return core_list.back().id;
}