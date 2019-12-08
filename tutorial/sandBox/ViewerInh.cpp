#include "ViewerInh.h"
#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/adjacency_list.h>
#include <Eigen/Core>
#include <set>
#include <math.h>
#include "igl/circulation.h"


void ViewerInh::init_ds_for_data() {
    int num_of_meshes = static_cast<int>(data_list.size());
    data_edges.resize(static_cast<unsigned long>(num_of_meshes));
    for (int i = 0; i < num_of_meshes; i++) {
        auto ds = new edges;                                            // Create new data structure
        data_edges[i] = reset(data_list[i], ds);
    }
}


edges *ViewerInh::reset(igl::opengl::ViewerData obj, edges* ds){
    Eigen::MatrixXd V = obj.V;                                                         // Get mesh's vertexes
    Eigen::MatrixXi F = obj.F;                                                         // Get mesh's faces
    igl::edge_flaps(F,ds->E,ds->EMAP,ds->EF,ds->EI);                                   // Init the edge related field in the data structure
    ds->Qit.resize(static_cast<unsigned long>(ds->E.rows()));
    ds->C.resize(ds->E.rows(),V.cols());
    ds->Q.clear();
    ds->EQ = get_Q(V,F,obj.F_normals);

    for(int e = 0; e < ds->E.rows(); e++){
        double cost;
        Eigen::RowVectorXd p(1,3);
        calc_edges_cost(e,V,F,ds->E,ds->EMAP,ds->EF, ds->EI, cost, p, ds->EQ);         // Calculate cost for each edge
        ds->Qit[e] = ds->Q.insert(std::pair<double,int>(cost,e)).first;                // Save the edge cost in Q and in Q's iterator- Qit
        ds->C.row(e)   = p;                                                            // Save the optimal vertex p in C at index e
    }
    ds->num_collapsed = 0;
    return ds;
}

 std::vector<Eigen::Matrix4d> ViewerInh::get_Q(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXd &F_normals){
    std::vector<std::vector<int> > vertex_to_vertices;
    std::vector<std::vector<int> > vertex_to_faces_index;
    igl::adjacency_list(F, vertex_to_vertices);
    igl::vertex_triangle_adjacency(V,F,vertex_to_vertices, vertex_to_faces_index);     // Get the faces connected to each vertex

    std::vector<Eigen::Matrix4d> Q;                                                    // Create vector Q of error quadrics
    Q.resize(V.rows());                                                                // Set the size of Q to be as the numbers of vertecies

    for(int v = 0; v < V.rows(); v++) {                                                // For each vertex v in V
        std::vector<int> faces_index_of_vector = vertex_to_vertices[v];                // Get the faces connected to v
        Eigen::Vector4d vertex(V.row(v)(0), V.row(v)(1), V.row(v)(2), 1);              // Get v in vector of size 4
        Eigen::Matrix4d sum_of_Kp;
        sum_of_Kp << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;                                  // Init 4X4 matrix to insert to Q

        for(int f: faces_index_of_vector) {                                            // For each face connected to v
            Eigen::RowVectorXd p = F_normals.row(f).normalized();                      // p = (a, b, c) -> a^2 + b^2 + c^2 = 1
            double d = (-vertex(0) * p(0))+ (-vertex(1) * p(1)) + (-vertex(2) * p(2)); // d = -ax - by - cz
            Eigen::Vector4d p_with_d(p(0), p(1), p(2), d);                             // p_with_d = (a, b, c, d)
            Eigen::Matrix4d Kp = p_with_d * p_with_d.transpose();                      // Kp
            sum_of_Kp += Kp;                                                           // Add Kp to sum_of_KP
        }
        Q[v] = sum_of_Kp;                                                              // Push sum_of_KP to Q in index v
    }
     return Q;
}

 void ViewerInh::calc_edges_cost(const int e,
                                 const Eigen::MatrixXd & V,
                                 const Eigen::MatrixXi & F,
                                 const Eigen::MatrixXi & E,
                                 const Eigen::VectorXi & EMAP,
                                 const Eigen::MatrixXi & EF,
                                 const Eigen::MatrixXi & EI,
                                 double & cost,
                                 Eigen::RowVectorXd & p,
                                 std::vector<Eigen::Matrix4d> & Q){
        Eigen::RowVectorXd p_(1,3);
        int v1 = E(e,0);                    // Index (in V) of the first vertex in edge e
        int v2 = E(e,1);                    // Index (in V) of the second vertex in edge e
        Eigen::Matrix4d Q1 = Q[v1];         // Error-quadric of v1
        Eigen::Matrix4d Q2 = Q[v2];         // Error-quadric of v2
        Eigen::Matrix4d Q_tag = (Q1 + Q2);  // Error-quadric Q

        Eigen::Vector4d vertex1(V.row(v1)(0), V.row(v1)(1), V.row(v1)(2), 1); // V.row(v1) as vector of size 4
        Eigen::Vector4d vertex2(V.row(v2)(0), V.row(v2)(1), V.row(v2)(2), 1); // V.row(v2) as vector of size 4

        double lowest_cost;          // Init the lowest cost of the edge

        // Create matrix of Q(1,1)-Q(3,4) and (0,0,0,1) at the bottom
        Eigen::Matrix4d v_tag_1;
        v_tag_1 <<
                Q_tag(0,0),Q_tag(0,1),Q_tag(0,2),Q_tag(0,3),
                Q_tag(1,0),Q_tag(1,1),Q_tag(1,2),Q_tag(1,3),
                Q_tag(2,0),Q_tag(2,1),Q_tag(2,2),Q_tag(2,3),
                0,           0,           0,        1;


        if(v_tag_1.determinant() != 0) {                            // Check if v_tag_1 is inversable
            Eigen::Vector4d v_tag_2(0, 0, 0, 1);
            Eigen::Vector4d v_tag = v_tag_1.inverse() * v_tag_2;    // Get new vertex v_tag to collapse e into
            lowest_cost = v_tag.transpose() * Q_tag * v_tag;        // Get the cost of e
            p_ << v_tag(0), v_tag(1), v_tag(2);                      // Set the optimal vertex p to be v_tag
        }

        else{                                                       // If v_tag_1 is not inverse
            Eigen::Vector4d midpoint = (vertex1 + vertex2) * 0.5;   // Calculate mitdpoint of e
            double cost1 = vertex1.transpose()  * Q_tag * vertex1;  // Calculate cost of v1
            double cost2 = vertex2.transpose()  * Q_tag * vertex2;  // Calculate cost of v2
            double cost3 = midpoint.transpose() * Q_tag * midpoint; // Calculate cost of mitdpoint


            lowest_cost = std::min(cost1, cost2);                   // Set cost of e as min(cost1,cost2,cost3)
            lowest_cost = std::min(lowest_cost, cost3);             //

            if (lowest_cost == cost1)                               //
                p_ << vertex1(0), vertex1(1), vertex1(2);           //
            else if (lowest_cost == cost2)                          // Set the optimal vertex p to the vertex with lowest cost
                p_ << vertex2(0), vertex2(1), vertex2(2);           //
            else                                                    //
                p_ << midpoint(0), midpoint(1), midpoint(2);        //
        }
     p = p_;
     cost = lowest_cost;
//     std::cout << e << ") cost " << cost << " at point " << p(0) << ", "<< p(1) << ", "<< p(2) << "\n"<< std::endl;
}

bool ViewerInh::my_collapse(Eigen::MatrixXd & V,
                             Eigen::MatrixXi & F,
                             Eigen::MatrixXd &F_normals,
                             edges *ds){
    //a. Takes out the lowest cost edge from queue.
    if(ds->Q.empty())
    {
        return false;
    }
    std::pair<double,int> q = *(ds->Q.begin());
    ds->Q.erase(ds->Q.begin());
    int e = q.second;
    double cost = q.first;
    ds->Qit[e] = ds->Q.end();
    std::vector<int> N  = igl::circulation(e, true,ds->EMAP,ds->EF,ds->EI);
    std::vector<int> Nd = igl::circulation(e,false,ds->EMAP,ds->EF,ds->EI);
    N.insert(N.begin(),Nd.begin(),Nd.end());

    //b. Deletes edge
    const auto & kill_edge = [&ds](const int e)
    {
        ds->E(e,0) = 0;
        ds->E(e,1) = 0;
        ds->EF(e,0) = 0;
        ds->EF(e,1) = 0;
        ds->EI(e,0) = 0;
        ds->EI(e,1) = 0;
    };

    const int eflip = ds->E(e,0)>ds->E(e,1);
    // source and destination
    const int s = eflip?ds->E(e,1):ds->E(e,0);
    const int d = eflip?ds->E(e,0):ds->E(e,1);
    const std::vector<int> nV2Fd = igl::circulation(e,!eflip,ds->EMAP,ds->EF,ds->EI);
    int e1,f1,e2,f2;

    const int m = F.rows();
    for(int side = 0;side<2;side++)
    {
        const int f = ds->EF(e,side);
        const int v = ds->EI(e,side);
        const int sign = (eflip==0?1:-1)*(1-2*side);
        // next edge emanating from d
        const int e1_ = ds->EMAP(f+m*((v+sign*1+3)%3));
        // prev edge pointing to s
        const int e2_ = ds->EMAP(f+m*((v+sign*2+3)%3));
        if(! (ds->E(e1_,0) == d || ds->E(e1_,1) == d))return false;
        if(! (ds->E(e2_,0) == s || ds->E(e2_,1) == s))return false;
        // face adjacent to f on e1, also incident on d
        const bool flip1 = ds->EF(e1_,1)==f;
        const int f1_ = flip1 ? ds->EF(e1_,0) : ds->EF(e1_,1);
        if(! (f1_!=f))return false;
        if(! (F(f1_,0)==d || F(f1_,1)==d || F(f1_,2) == d))return false;
        // across from which vertex of f1_ does e1 appear?
        const int v1 = flip1 ? ds->EI(e1_,0) : ds->EI(e1_,1);
        // Kill e1_
        kill_edge(e1_);
        // Kill f
        F(f,0) = 0;
        F(f,1) = 0;
        F(f,2) = 0;
        // map f1_'s edge on e1 to e2_
        if(! (ds->EMAP(f1_+m*v1) == e1_))return false;
        ds->EMAP(f1_+m*v1) = e2_;
        // side opposite f2, the face adjacent to f on e2_, also incident on s
        const int opp2 = (ds->EF(e2_,0)==f?0:1);
        if(! (ds->EF(e2_,opp2) == f))return false;
        ds->EF(e2_,opp2) =  f1_;
        ds->EI(e2_,opp2) = v1;
        // remap e2 from d to s
        ds->E(e2_,0) = ds->E(e2_,0)==d ? s : ds->E(e2_,0);
        ds->E(e2_,1) = ds->E(e2_,1)==d ? s : ds->E(e2_,1);
        if(side==0)
        {
            e1 = e1_;
            f1 = f;
        }else
        {
            e2 = e1_;
            f2 = f;
        }
    }

    //c. Deletes faces
    for(auto f : nV2Fd)
    {
        for(int v = 0;v<3;v++)
        {
            if(F(f,v) == d)
            {
                const int flip1 = (ds->EF(ds->EMAP(f+m*((v+1)%3)),0)==f)?1:0;
                const int flip2 = (ds->EF(ds->EMAP(f+m*((v+2)%3)),0)==f)?0:1;
                if(! (ds->E(ds->EMAP(f+m*((v+1)%3)),flip1) == d ||
                                ds->E(ds->EMAP(f+m*((v+1)%3)),flip1) == s)) return false;
                ds->E(ds->EMAP(f+m*((v+1)%3)),flip1) = s;
                if(! (ds->E(ds->EMAP(f+m*((v+2)%3)),flip2) == d ||
                                ds->E(ds->EMAP(f+m*((v+2)%3)),flip2) == s)) return false;
                ds->E(ds->EMAP(f+m*((v+2)%3)),flip2) = s;
                F(f,v) = s;
                break;
            }
        }
    }
    // Finally, "remove" this edge and its information
    kill_edge(e);

    //d. Merges vertices to a new vertex v̅
    Eigen::RowVectorXd v = ds->C.row(e);
    V.row(s) = v;
    V.row(d) = v;
//    ds->EQ = get_Q(V,F,F_normals);

    // Erase the two, other collapsed edges
    ds->Q.erase(ds->Qit[e1]);
    ds->Qit[e1] = ds->Q.end();
    ds->Q.erase(ds->Qit[e2]);
    ds->Qit[e2] = ds->Q.end();
    // update local neighbors
    // loop over original face neighbors
    for(auto n : N)
    {
        if(F(n,0) != 0 ||
           F(n,1) != 0 ||
           F(n,2) != 0)
        {
            for(int v = 0;v<3;v++)
            {
                // get edge id
                const int ei = ds->EMAP(v*F.rows()+n);
                // erase old entry
                ds->Q.erase(ds->Qit[ei]);
                // compute cost and potential placement
                double cost;
                Eigen::RowVectorXd place;
                calc_edges_cost(ei,V,F,ds->E,ds->EMAP,ds->EF,ds->EI,cost,place,ds->EQ);
                // Replace in queue
                ds->Qit[ei] = ds->Q.insert(std::pair<double,int>(cost,ei)).first;
                ds->C.row(ei) = place;
            }
        }
    }

    std::cout << "edge " << e << ", cost = " << cost << "  new v position (" << v(0) << ","<< v(1) << ","<< v(2) << ")\n"<< std::endl;
return true;
}

void ViewerInh::simplify(int num_to_collapse){
    edges *ds = data_edges[selected_data_index];
    Eigen::MatrixXd V = data_list[selected_data_index].V;
    Eigen::MatrixXi F = data_list[selected_data_index].F;
    Eigen::MatrixXd F_normals = data_list[selected_data_index].F_normals;
    int curr_collapsed = 0;

    const auto & collapse = [&ds, &V,  &F, &F_normals](){
            return my_collapse(V, F, F_normals,ds);
    };

    bool something_collapsed = false;
    while(curr_collapsed < num_to_collapse)
    {
        if(collapse()){
            ds->num_collapsed++;
            curr_collapsed++;
            something_collapsed = true;
        }else{
           break;
        }
    }

    if(something_collapsed){
        data_list[selected_data_index].clear();
        data_list[selected_data_index].set_mesh(V, F);
        data_list[selected_data_index].set_face_based(true);
    }

    data_edges[selected_data_index] = ds;
}


void ViewerInh::simplification(){
    long num_of_edges = data_edges[selected_data_index]->E.rows() - data_edges[selected_data_index]->num_collapsed;
    if (num_of_edges < 8)
        return;
    const int max_iter = std::ceil(num_of_edges * 0.05);
    simplify(max_iter);
}



