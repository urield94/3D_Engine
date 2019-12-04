#include "ViewerInh.h"
#include <igl/circulation.h>
#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/adjacency_list.h>
#include <igl/read_triangle_mesh.h>
#include <Eigen/Core>
#include <set>
#include <math.h>
#include <algorithm>

void ViewerInh::init_ds_for_data() {
    for (auto &obj : data_list) {
        ViewerInh::edges ds = reset(obj);;
        data_edges.push_back(ds);
    }
}

void ViewerInh::my_collapse(const int e,
                              const Eigen::RowVectorXd & p,
                              Eigen::MatrixXd & V,
                              Eigen::MatrixXi & F,
                              Eigen::MatrixXi & E,
                              Eigen::VectorXi & EMAP,
                              Eigen::MatrixXi & EF,
                              Eigen::MatrixXi & EI){

}

ViewerInh::edges ViewerInh::reset(igl::opengl::ViewerData obj){
    ViewerInh::edges ds;
    Eigen::MatrixXd V = obj.V;
    Eigen::MatrixXi F = obj.F;
    igl::edge_flaps(F,ds.E,ds.EMAP,ds.EF,ds.EI);

    ds.C.clear();
    ds.C.resize(ds.E.rows());
    while(!ds.Q.empty()){
        ds.Q.pop();
    };

    std::vector<std::vector<int> > vertex_to_vertices;
    std::vector<std::vector<int> > vertex_to_faces_index;
    igl::adjacency_list(F, vertex_to_vertices);
    igl::vertex_triangle_adjacency(V,F,vertex_to_vertices, vertex_to_faces_index);

/** Find Q - vector of vertex error**/
    std::vector<Eigen::Matrix4d> Q;

    for(int v = 0; v < V.rows(); v++) {
        std::vector<int> faces_index_of_vector = vertex_to_vertices[v];
        Eigen::Vector4d vertex(V.row(v)(0), V.row(v)(1), V.row(v)(2), 1);
        Eigen::Matrix4d sum_of_Kp;
        for (int f = 0; f < faces_index_of_vector.size(); f++) {
            Eigen::RowVectorXd p = obj.F_normals.row(
                    faces_index_of_vector[f]).normalized(); // p = (a, b, c) -> a^2 + b^2 + c^2 = 1
            double d = vertex(0) * p(0) + vertex(1) * p(1) + vertex(2) * p(2); //ax + by + cz = d
            Eigen::Vector4d p_with_d(p(0), p(1), p(2), d); // p_with_d = (a, b, c, d)
            Eigen::Matrix4d Kp = p_with_d * p_with_d.transpose(); //Kp
            sum_of_Kp += Kp;
        }
        Q.push_back(sum_of_Kp);
    }

/** Calculate cost for each edge**/

    for(int e = 0;e<ds.E.rows();e++) {
        int v1 = ds.E.row(e)(0);
        int v2 = ds.E.row(e)(1);
        Eigen::Matrix4d Q1 = Q[v1];
        Eigen::Matrix4d Q2 = Q[v2];

        Eigen::Vector4d vertex1(V.row(v1)(0), V.row(v1)(1), V.row(v1)(2), 1);
        Eigen::Vector4d vertex2(V.row(v2)(0), V.row(v2)(1), V.row(v2)(2), 1);

//        double t = sqrt(pow(vertex1(0) - vertex2(0), 2) + pow(vertex1(1) - vertex2(1), 2) + pow(vertex1(2) - vertex2(2), 2));
        Eigen::Vector4d midpoint = (vertex1 + vertex2) * 0.5;

        double cost1 =  vertex1.transpose() * (Q1 + Q2) * vertex1;
        double cost2 =  vertex2.transpose() * (Q1 + Q2) * vertex2;
        double cost3 = midpoint.transpose() * (Q1 + Q2) * midpoint;

        double lower_cost = std::min(cost1, cost2);
        lower_cost = std::min(lower_cost, cost3);

        ds.Q.push(std::make_pair(lower_cost, e));

        Eigen::RowVectorXd p(1,3);
        if (lower_cost == cost1)
            p << vertex1(0), vertex1(1), vertex1(2);
        else if(lower_cost == cost2)
            p << vertex2(0), vertex2(1), vertex2(2);
        else
            p << midpoint(0), midpoint(1), midpoint(2);

        ds.C[e] = p;
    }

    obj.clear();
    obj.set_mesh(V, F);
    obj.set_face_based(true);
    return ds;
}


void ViewerInh::simplify(int num_to_collapse){
    edges ds = data_edges[selected_data_index];
    Eigen::MatrixXd V = data_list[selected_data_index].V;
    Eigen::MatrixXi F = data_list[selected_data_index].F;
    bool something_collapsed = false;
    int collapsed = 0;
    while(collapsed < num_to_collapse)
    {
        const int e = ds.Q.top().second;
        const Eigen::Vector3d p = ds.C[e];
        ds.Q.pop();
//
//        ds.E(e,0) = 0;
//        ds.E(e,1) = 0;
//        ds.EF(e,0) = 0;
//        ds.EF(e,1) = 0;
//        ds.EI(e,0) = 0;
//        ds.EI(e,1) = 0;
//
//        for(int side = 0;side<2;side++) {
//            const int f = ds.EF(e, side);
//            F(f,0) = 0;
//            F(f,1) = 0;
//            F(f,2) = 0;
//        }
//
//        V.row(ds.E(e,0)) = p;
//        V.row(ds.E(e,1)) = p;
        if(!igl::collapse_edge(e,p, V, F,ds.E,ds.EMAP,ds.EF,ds.EI)){
            continue;
        }

        std::cout << "edge: "<< e << ", cost: " << ds.Q.top().first << ", p: " << p << "\n" << std::endl;
        ds.num_collapsed++;
        collapsed++;
        something_collapsed = true;

    }

        if(something_collapsed){
            data().clear();
            data().set_mesh(V, F);
            data().set_face_based(true);
    }
}


void ViewerInh::simplification(){
    data_edges[selected_data_index] = reset(data());
    long num_of_edges = data_edges[selected_data_index].E.rows();
    if (num_of_edges < 8)
        return;
    const int max_iter = std::ceil(num_of_edges * 0.05);
    simplify(max_iter);
}



