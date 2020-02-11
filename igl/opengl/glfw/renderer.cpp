#include "igl/opengl/glfw/renderer.h"

#include <GLFW/glfw3.h>
#include <igl/unproject_onto_mesh.h>
#include "igl/look_at.h"
#include <Eigen/Dense>
#include <cstdlib>

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

    //TODO START - CHANGE POINT OF VIEW TO BE FROM THE SNAKE EYES.
    int last_link_index = (scn->links_number - 1);
    Eigen::Vector4f last_link =  Eigen::Vector4f(scn->data_list[last_link_index].V.colwise().mean()[0],
                                                 scn->data_list[last_link_index].V.colwise().maxCoeff()[1],
                                                 scn->data_list[last_link_index].V.colwise().mean()[2],
                                                 1);
    Eigen::Vector3f curr_last_link_E = (GetAncestorTrans(last_link_index + 1) * last_link).head(3);
    core(1).camera_eye =  curr_last_link_E.normalized();
    core(1).camera_translation = last_link.head(3).normalized();
    core(1).camera_up = Eigen::Vector3f(0, scn->data_list[last_link_index].V.colwise().maxCoeff()[1], 0).normalized();
    //TODO END

    for (auto &core : core_list) {
        for (auto &mesh : scn->data_list) {
            if (mesh.is_visible & core.id) {
                Eigen::Matrix4f scn_trans = scn->MakeTrans() * GetAncestorTransIfNeeded(scn->mesh_index(mesh.id));
                if(scn->mesh_index(mesh.id) > scn->links_number - 1) {
                    if(std::abs(mesh.MakeTrans().col(3)(0)) > 5 && std::abs(mesh.MakeTrans().col(3)(1)) > 5 && std::abs(mesh.MakeTrans().col(3)(2)) > 5){
                        scn->erase_mesh(scn->mesh_index(mesh.id));
                    }else{
                        int i = scn->mesh_index(mesh.id);
                        if(i < scn->links_number + 4) {
                            mesh.MyTranslate(Eigen::Vector3f(0.003 * level + 0.003,
                                                             0.002 * level + 0.002,
                                                             0.003 * level + 0.003));
                        }else if(i < scn->links_number + 8) {
                            mesh.MyTranslate(Eigen::Vector3f(-0.003 * level - 0.003,
                                                             0.003 * level + 0.002,
                                                             0.003 * level + 0.003));
                        }else if(i < scn->links_number + 12){
                            mesh.MyTranslate(Eigen::Vector3f(0.003 * level + 0.003,
                                                             -0.002 * level - 0.002,
                                                             0.003 * level + 0.003));
                        }
                        else{
                            mesh.MyTranslate(Eigen::Vector3f(-0.003 * level - 0.003,
                                                             0.002 * level + 0.002,
                                                             -0.003 * level - 0.003));
                        }

                    }
                }
                core.draw(scn_trans, mesh, scn->links_number);
            }
        }
    }

}


Eigen::Matrix4f Renderer::GetAncestorTrans(int link_index) {
    Eigen::Matrix4f links = Eigen::Matrix4f::Identity();
//    links.row(1)(1) = scn->snake_scale_factor;
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
//    scn->data_list[0].MyScale(Eigen::Vector3f(1,scn->snake_scale_factor,1));
    for (; i < scn->links_number; i++) {
        scn->data_list[i].line_width = 3;
        scn->data_list[i].show_overlay_depth = 0;
        scn->data_list[i].point_size = 10;
        scn->data_list[i].set_face_based(!scn->data_list[i].face_based);
        core().toggle(scn->data_list[i].show_lines);

        Eigen::Vector3d m = scn->data_list[i].V.colwise().minCoeff();
        Eigen::Vector3d M = scn->data_list[i].V.colwise().maxCoeff();
        double link_height = M(1) - m(1);
        scn->data_list[i].SetCenterOfRotation(Eigen::Vector3f(scn->data_list[i].V.colwise().mean()[0],
                                                              m[1],
                                                              scn->data_list[i].V.colwise().mean()[0]));
        if (i == 0)
            scn->data_list[i].MyPreTranslate(Eigen::Vector3f(0, 0, 0));
        else
            scn->data_list[i].MyPreTranslate(Eigen::Vector3f(0, link_height , 0));
    }

    for(; i < scn->data_list.size(); i++) {
        scn->data_list[i].MyPreTranslate(Eigen::Vector3f(-5 + ( rand() % ( 5 + 5 + 1 ) ),
                -5 + ( rand() % ( 5 + 5 + 1 ) ),
                -5 + ( rand() % ( 5 + 5 + 1 ) )));
    }
    scn->MyScale(Eigen::Vector3f(0.15, 0.15, 0.15));
}

IGL_INLINE void Renderer::init(igl::opengl::glfw::Viewer *viewer) {
    scn = viewer;

    init_system();

    core().init();
    core().align_camera_center(scn->data().V, scn->data().F);

    trees.resize(scn->data_list.size());
    for(auto &obj: scn->data_list){
        igl::AABB<Eigen::MatrixXd, 3> tree;
        tree.init(obj.V, obj.F);
        trees[scn->mesh_index(obj.id)] = tree;
    }
    scn->selected_data_index = -1;
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
    if(scn->selected_data_index > scn->links_number - 1) {
        Eigen::Vector4f tail_4f = GetAncestorTrans(1) * Eigen::Vector4f(scn->data_list[0].V.colwise().mean()[0],
                                                                           scn->data_list[0].V.colwise().minCoeff()[1],
                                                                           scn->data_list[0].V.colwise().mean()[2],
                                                                           1);
        Eigen::Vector3f tail_3f = tail_4f.head(3);

        Eigen::Vector3f selected_object_3f = scn->data().MakeTrans().col(3).head(3);
        double tail_obj_dist = (selected_object_3f - tail_3f).norm();

        if (tail_obj_dist > scn->snake_length) {
            object_picked = false;
            return;
        }

        int last_link_index = scn->links_number - 1;

        if (IsBoxesColide(scn->data_list[last_link_index], scn->data(),
                          trees[last_link_index], trees[scn->mesh_index(scn->data().id)])) {
            object_picked = false;
            return;
        }

        Eigen::Vector4f head_4f = Eigen::Vector4f(scn->data_list[last_link_index].V.colwise().mean()[0],
                                                  scn->data_list[last_link_index].V.colwise().maxCoeff()[1],
                                                  scn->data_list[last_link_index].V.colwise().mean()[2],
                                                  1);
        Eigen::Vector3f head_3f = (GetAncestorTrans(last_link_index + 1) * head_4f).head(3);

        double head_obj_dist = (selected_object_3f - head_3f).norm();

        for (int i = last_link_index; i > -1; i--) {

            Eigen::Vector4f curr_link_4f =
                    GetAncestorTransIfNeeded(i + 1) * Eigen::Vector4f(scn->data_list[i].V.colwise().mean()[0],
                                                                      scn->data_list[i].V.colwise().minCoeff()[1],
                                                                      scn->data_list[i].V.colwise().mean()[2],
                                                                      1);
            Eigen::Vector3f curr_link_3f = curr_link_4f.head(3);

            Eigen::Vector3f RD = (selected_object_3f - curr_link_3f).normalized();
            Eigen::Vector3f RE = (head_3f - curr_link_3f).normalized();
            float pre_angle = RE.dot(RD);
            if (pre_angle > 1) {
                pre_angle = 1;
            } else if (pre_angle < -1)
                pre_angle = -1;

            float angle = acosf(pre_angle);

            Eigen::Vector3f cross = RE.cross(RD).normalized();
            Eigen::Vector3f crossInverse = GetAncestorInverseIfNeeded(last_link_index) * cross;
            if (head_obj_dist > 0.5)
                angle = angle / 10;
            scn->data_list[i].MyRotate(crossInverse, angle);
            head_3f = (GetAncestorTrans(last_link_index + 1) * head_4f).head(3);
            head_obj_dist = (selected_object_3f - head_3f).norm();

        }
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
            data.set_visible(true, core_list.back().id);
            data.copy_options(core(), core_list.back());
        }
    }
    selected_core_index = core_list.size() - 1;
    return core_list.back().id;
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

    Eigen::Vector3f C1_scn = (GetAncestorTrans(scn->links_number) * C1).head(3);
    Eigen::Vector3f C2_scn = (obj2.MakeTrans() * C2).head(3);

    Eigen::Vector3f D = C2_scn - C1_scn;

    if(!OBBCheckSat({A0,A1,A2,B0,B1,B2,a0,a1,a2,b0,b1,b2,c00,c01,c02,c10,c11,c12,c20,c21,c22,D})){
        if(tree1.is_leaf()) {
            if (tree2.is_leaf()) {
                if(obj2.score_group == 0){
                    score += 5;
                }else if(obj2.score_group == 1){
                    score += 10;
                }else{
                    score += 15;
                }
                scn->erase_mesh(scn->mesh_index(obj2.id));
                std::cout << "Score: " << score << "\t\r" << std::flush;
                std::fflush(stdout);
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

