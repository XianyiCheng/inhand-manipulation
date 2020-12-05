#include "contact_kinematics.h"
#include "tree.h"
#include "mex.h"

#define PI 3.141592654
#define NUM_FINGERS 4
#define NUM_DISCRETIZATION 16
#define OUT_OF_INDEX INT_MAX

/* Input Arguments */
#define	POSRANGE_IN         prhs[0]
#define	WORKSPACE_IN	    prhs[1]
#define	SURFACE_IN          prhs[2]
#define	OBJECT_START_IN     prhs[3]
#define	FINGER_START_IN     prhs[4]
#define	OBJECT_GOAL_IN      prhs[5]
#define PLANNERIN_IN        prhs[6]
#define MAX_SAMPLES         prhs[7]
#define RRTSTAR_RADIUS      prhs[8]
#define EXTEND_MAX_ADDS     prhs[9]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1

/* Output Arguments */
#define	OBJECTPATH_OUT	plhs[0]
#define	FINGERPATH_OUT	plhs[1]
#define	PLANLENGTH_OUT	plhs[2]
#define	TREESIZE_OUT	plhs[3]
#define FRICTION_COEFF 0.8

int num_surface_points = 64;
// tunable parameters
double goal_thr = PI*1/180;
double goal_biased_prob = 0.7;
double primitive1_prob = 0.5;
double epsilon_translation = 1;
double epsilon_angle = PI*45/180;
int interpolation_steps = 10;
double interpolation_length = 1/double(interpolation_steps);
int max_samples = 1000;
int max_extends = 100;
// TODO: Can remove this once we have better costs.
double finger_cost_coef = 1 / (2 * NUM_FINGERS);


void to_config(double config[7], Vector3d p, Quaterniond q){
    config[0] = p[0];
    config[1] = p[1];
    config[2] = p[2];
    config[3] = q.w();
    config[4] = q.x();
    config[5] = q.y();
    config[6] = q.z();
}


bool force_closure(Vector3d p, Quaterniond q, int finger_locations[NUM_FINGERS], double* object_surface_discretization){

    std::vector<int> contact_index;
    int num_of_contacts = 0;
    // assume when the num in "finger_locations[NUM_FINGERS]" is OUT_OF_INDEX, it means the finger is out of contact.
    // finger workspace constraints first
    for(int i=0; i<NUM_FINGERS; i++){
        if(finger_locations[i] != OUT_OF_INDEX){
            contact_index.push_back(i);
            num_of_contacts ++;
        }
    }

    // define the friction cone in contact frame and in object frame
    MatrixXd w_c(6, 4*num_of_contacts);
    for(int j=0; j<num_of_contacts; j++){
        // define polyhedral friction cone
        VectorXd f1_c(6,1);
        f1_c << 0, -FRICTION_COEFF, 1, 0, 0, 0;
        VectorXd f2_c(6,1);
        f2_c << 0, FRICTION_COEFF, 1, 0, 0, 0;
        VectorXd f3_c(6,1);
        f3_c << -FRICTION_COEFF, 0, 1, 0, 0, 0;
        VectorXd f4_c(6,1);
        f4_c << FRICTION_COEFF, 0, 1, 0, 0, 0;

        // find the finger position and norm from surface
        int finger_location_idx = contact_index[j];  // e.g. contact_index = [0, 1, 3]
        Vector3d fp_o;
        Vector3d fn_o;
        for(int k=0; k<3; k++){
            fp_o[k] = object_surface_discretization[finger_locations[finger_location_idx]*6 + k];
            fn_o[k] = object_surface_discretization[finger_locations[finger_location_idx]*6 + k + 3];
        }

        Matrix3d R_wo = q.toRotationMatrix();
        Vector3d fp_w = R_wo*fp_o + p;

        // get contact kinematics and transfer force in contact frame to object frame
        // std::cout << "finger index: "<< j << std::endl;
        // std::cout << "fp_o: "<< fp_o << std::endl;
        // std::cout << "fn_o: "<< fn_o << std::endl;
        // std::cout << "fp_w: "<< fp_w << std::endl;

        MatrixXd adgco = contact_jacobian(fp_o, fn_o);
        // std::cout << "adgco: "<< adgco << std::endl;
        VectorXd f1_o = adgco.transpose()*f1_c;
        // std::cout << "f1_c: "<< f1_c << std::endl;
        // std::cout << "f1_o: "<< f1_o << std::endl;

        VectorXd f2_o = adgco.transpose()*f2_c;
        VectorXd f3_o = adgco.transpose()*f3_c;
        VectorXd f4_o = adgco.transpose()*f4_c;

        w_c.col(j*4) = f1_o;
        w_c.col(j*4 + 1) = f2_o;
        w_c.col(j*4 + 2) = f3_o;
        w_c.col(j*4 + 3) = f4_o;
    }

    // std::cout << "w_c: "<< w_c << std::endl;

    // solve it by LP
    VectorXd sum_2_w_c = w_c.rowwise().sum();   // sum(w_c, 2)
    VectorXd avg_w_c = sum_2_w_c/w_c.cols(); // get average
    VectorXd T_0 = -avg_w_c;
    MatrixXd T(6, 4*num_of_contacts);
    for(int i=0; i<w_c.cols(); i++){
        T.col(i) = w_c.col(i) - avg_w_c;
    }
    
    // set up LP
    VectorXd C = -T_0;
    // std::cout << "C: " << C <<std::endl;
    MatrixXd A = T.transpose();
    // std::cout << "C: " << C <<std::endl;
    VectorXd b(w_c.cols());
    b.fill(1);

    MatrixXd Ae(w_c.cols(), 6);
    VectorXd be(w_c.cols());
    VectorXd xl(6);
    xl << std::numeric_limits<double>::min(), std::numeric_limits<double>::min(), std::numeric_limits<double>::min(), std::numeric_limits<double>::min(), std::numeric_limits<double>::min(), std::numeric_limits<double>::min();
    VectorXd xu(6);
    xu << std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max();
    VectorXd xs(6); 
    double optimal_cost;

    bool result = lp(C, A, b, Ae, be, xl, xu, &xs, &optimal_cost);
    
    return result;
}


bool in_fingertip_workspace(Vector3d p, Quaterniond q, int finger_location, int finger_index, double finger_workspace[NUM_FINGERS*6], double* object_surface_discretization){

    // p: object positions
    // q: object orientations
    // finger_location: finger location index in object_surface discretization 
    // finger_index: index of fingers corresponding to fingertip workspace

    // get finger vector in object frame, I guess only first 3 is needed
    Vector3d fp_o;
    for(int i=0; i<3; i++){
        fp_o[i] = object_surface_discretization[finger_location*6 + i];
    }
    // std::cout << "finger position on object: " << fp_o << std::endl;

    // finger location in object frame, need to tranfer to world frame
    Matrix3d R_wo = q.toRotationMatrix();
    Vector3d fp_w = R_wo*fp_o + p;

    // std::cout << "finger position on world: " << fp_w << std::endl;
    
    // check if the finger is within the workspace
    double current_workspace[6]; 
    for(int j = 0; j<6; j++){                                        // get the current finger workspace
        current_workspace[j] = finger_workspace[finger_index*6 + j];
    }
    // std::cout << "current workspace: " << current_workspace[0] << ", " << current_workspace[1] << ", " << current_workspace[2] << ", " << current_workspace[3] << ", " << current_workspace[4] << ", " << current_workspace[5] << std::endl;
    for(int i=0; i<3; i++){
        if(fp_w[i] < current_workspace[i] || fp_w[i] > current_workspace[i+3]){ // if out of range, invalid
            return false;
        }
    }
    return true;
}


bool IsValidConfiguration(double config[7], int finger_locations[NUM_FINGERS], double finger_workspace[NUM_FINGERS*6], double* object_surface_discretization){
    //TODO: IsValidConfiguration: check constraints
    // finger workspace constraints: kinematics included
    // force closure constraints: friction included

    // get object config
    Vector3d obj_p(config[0], config[1], config[2]); 
    Quaterniond obj_q(config[3], config[4], config[5], config[6]); 

    // assume when the num in "finger_locations[NUM_FINGERS]" is 100, it means the finger is out of contact.
    //finger workspace constraints first
    for(int i=0; i<NUM_FINGERS; i++){
        if(finger_locations[i] != OUT_OF_INDEX){
            bool workspace_validity = in_fingertip_workspace(obj_p, obj_q, finger_locations[i], i, finger_workspace, object_surface_discretization);
            // std::cout << "workspace validity: " << workspace_validity << std::endl;
            if(!workspace_validity){
                return false;
            }
        }
    }
    //printf("workspace valid \n");
    
    // force closure constraints
    bool force_validity = force_closure(obj_p, obj_q, finger_locations, object_surface_discretization);
    if(!force_validity){
        // printf("force closure not valid \n");
        return false;
    }

    return true;
}

bool is_free_to_connect(int parent_idx, int child_idx, int steps, Tree* T, double finger_workspace[NUM_FINGERS*6], double* object_surface_discretization)
{
    std::vector<int> f_idx;
    int finger_change_count = 0;
    int finger_loc[NUM_FINGERS];

    for (int i = 0; i < NUM_FINGERS; i++){
        finger_loc[i] = T->nodes[parent_idx].finger_locations[i];

        if (T->nodes[parent_idx].finger_locations[i]!=T->nodes[child_idx].finger_locations[i])
        {
            f_idx.push_back(i);
        }
    }

    double* config_near = T->nodes[parent_idx].config;
    Vector3d x_near(config_near[0], config_near[1], config_near[2]); 
    Quaterniond q_near(config_near[3], config_near[4], config_near[5], config_near[6]); 

    double* config_rand = T->nodes[child_idx].config;
    Vector3d x_rand(config_rand[0], config_rand[1], config_rand[2]); 
    Quaterniond q_rand(config_rand[3], config_rand[4], config_rand[5], config_rand[6]); 

    int status = false; // extend status: 0: trapped, 1:advanced, 2:reached

    double t = 1/double(steps);

    for(int i = 0; i < steps; i++){
        Vector3d x_check = x_near + 0.1*(i+1)*(x_rand - x_near);
        Quaterniond q_check = q_near.slerp(0.1*(i+1), q_rand);
        double config_check[7];
        to_config(config_check, x_check, q_check);

        // try to change finger contact
        for (int j = 0; j < f_idx.size(); j++){
            int k = f_idx[j];
            int pre_loc = finger_loc[k]; 
            finger_loc[k] = OUT_OF_INDEX;
            // check is in workspace && is force closure
            int loc = T->nodes[child_idx].finger_locations[k];
            if (in_fingertip_workspace(x_check, q_check, loc, k, finger_workspace, object_surface_discretization)
                 && force_closure(x_check, q_check, finger_loc, object_surface_discretization)) {
                finger_loc[k] = loc;
                f_idx.erase(f_idx.begin() + j);
            } else {
                finger_loc[k] = pre_loc;
            }
        }

        if(IsValidConfiguration(config_check, finger_loc, finger_workspace, object_surface_discretization)) // TODO
        {
            status = true;
        } else { 
            break; }
    }

    status = status & (f_idx.size() == 0);

    return status;
}

double get_edge_cost(Tree* T, int node1, int node2) {
    // TODO: Update this function when we have a different cost function.
    double cost = dist(T->nodes[node1].config, T->nodes[node2].config);
    for (int f_idx = 0; f_idx < NUM_FINGERS; f_idx++) {
        if (T->nodes[node1].finger_locations[f_idx] != T->nodes[node2].finger_locations[f_idx]) {
            cost += finger_cost_coef;
        }
    }
    return cost;
}

void update_node_cost(
        Tree* T,
        int nodeIdx,
        double newCost) {
    double edgeCost;
    T->nodes[nodeIdx].cost = newCost;
    for (int child : T->nodes[nodeIdx].children) {
        edgeCost = get_edge_cost(T, nodeIdx, child);
        update_node_cost(T, child, newCost + edgeCost);
    }
    return;
}

void rewire_neighborhood(
        int nodeIdx,
        Tree* T,
        double radius,
        int steps,
        double finger_workspace[NUM_FINGERS*6],
        double* object_surface_discretization) {
    double nodeCost = T->nodes[nodeIdx].cost;
    // Get the neeighbors around the node in question.
    std::vector<int> neighbors;
    T->neighborhood(nodeIdx, radius, &neighbors);
    // For each of the neighbors...
    mexPrintf("Num Neighbors: %d\n", neighbors.size());
    for (int nidx : neighbors) {
        //  Check to see if having this as a parent would be a better cost.
        double newCost = nodeCost + get_edge_cost(T, nodeIdx, nidx);
        if (T->nodes[nidx].cost <= newCost) {
            continue;
        }
        // Check to make sure we wouldn't be making any cycles in the graph.
        // for (int nchild : T->nodes[nidx].children) {
        //     if (nchild == nodeIdx) { 
        //         continue;
        //     }
        // }
        //  Check to see if these two nodes can be connected.
        if (!is_free_to_connect(
                    nodeIdx,
                    nidx,
                    steps,
                    T,
                    finger_workspace,
                    object_surface_discretization)) {
            continue;
        }
        //  Rewire the node.
        T->remove_parent(nidx);
        T->set_parent(nodeIdx, nidx);
        update_node_cost(T, nidx, newCost); 
    }
    return;
}

int extend(
        int near_idx,
        double config_rand[7], 
        double epsilon_translation,
        double epsilon_angle,
        int steps,
        Tree* T,
        double finger_workspace[NUM_FINGERS*6],
        double* object_surface_discretization,
        double rstar_radius=0)
{
    // TODO: extend function 
    double* config_near = T->nodes[near_idx].config;
    Vector3d x_near(config_near[0], config_near[1], config_near[2]); 
    Quaterniond q_near(config_near[3], config_near[4], config_near[5], config_near[6]); 

    Vector3d x_rand(config_rand[0], config_rand[1], config_rand[2]); 
    Quaterniond q_rand(config_rand[3], config_rand[4], config_rand[5], config_rand[6]); 

    // steer by epsilon
    x_rand = steer_position(x_near, x_rand, epsilon_translation);
    q_rand = steer_quaternion(q_near, q_rand, epsilon_angle);

    int status = 0; // extend status: 0: trapped, 1:advanced, 2:reached

    double config_new[7] = {0,0,0,0,0,0,0};
    double t = 1/double(steps);

    for(int i = 0; i < steps; i++){
        Vector3d x_check = x_near + 0.1*(i+1)*(x_rand - x_near);
        Quaterniond q_check = q_near.slerp(0.1*(i+1), q_rand);
        double config_check[7];
        to_config(config_check, x_check, q_check);

        if(IsValidConfiguration(config_check, T->nodes[near_idx].finger_locations, finger_workspace, object_surface_discretization))
        {
            status = 1;//advanced
            to_config(config_new, x_check, q_check);
            if (dist(config_new, config_rand)<0.001) { status = 2; break; }//reached
            
        } else { 
            break; }
    }

    if (status!=0){
      Node node_new(config_new, T->nodes[near_idx].finger_locations);
      T->add_node(&node_new, near_idx);    
      T->nodes[T->nodes.size() - 1].cost = T->nodes[near_idx].cost
                      + get_edge_cost(T, near_idx, T->nodes.size() - 1);
      if (rstar_radius > 0) {
          mexPrintf("Rewiring\n");
          rewire_neighborhood(
                  T->nodes.size() - 1,
                  T,
                  rstar_radius,
                  steps,
                  finger_workspace,
                  object_surface_discretization);
      }
    }
    return status;
}

int connect(int* near_idx, double config[7], double epsilon_translation,
            double epsilon_angle, int steps, Tree* T,
            double finger_workspace[NUM_FINGERS*6],
            double* object_surface_discretization) 
{
    int code = 1;
    while (code == 1) {
        code = extend(*near_idx, config, epsilon_translation, epsilon_angle,
                      steps, T, finger_workspace,
                      object_surface_discretization);
        if (code == 1) {
            *near_idx = T->nodes.size() - 1;
        }
    }
    return code;
}


int primitiveOne(float goToGoalRand, Tree* T, Vector3d pos_lb,
                 Vector3d pos_ub, Vector3d goal_position,
                 Quaterniond goal_orientation,
                 double finger_workspace[NUM_FINGERS*6],
                 double* object_surface_discretization,
                 double rstar_radius=0)
{
        // bias sample toward the goal
        Vector3d x_rand;
        Quaterniond q_rand;
        if (goToGoalRand < goal_biased_prob){
            x_rand = sample_position(pos_ub, pos_lb);
            q_rand = generate_unit_quaternion();
        } else {
            x_rand = goal_position;
            q_rand = goal_orientation;
        }

        double config_rand[7] = {x_rand[0], x_rand[1], x_rand[2],
                                 q_rand.w(), q_rand.x(), q_rand.y(),
                                 q_rand.z()};
        // get nearest neighbors (extend all nearest neighbor with the
        // same object config but different finger locations)
        std::vector<int> nears;
        T->nearest_neighbors(config_rand, &nears);
        
        //extend every node_near, do: linear interpolate
        // and check constraints
        int numAdded = 0;
        int code;
        int total_extends = nears.size();
        // if (total_extends > max_extends) { 
        //     total_extends = max_extends;
        // }
        for (int i = 0; i < total_extends; i++) {
            // extend function
            code = extend(nears[i], config_rand, epsilon_translation,
                          epsilon_angle, interpolation_steps,T,
                          finger_workspace, object_surface_discretization,
                          rstar_radius);
            if (code != 0) {
                numAdded++;
            }
        }
        return numAdded;
}


void primitiveTwo(int node_idx, Tree* T, double finger_workspace[NUM_FINGERS*6], double* object_surface_discretization) {
    
    Vector3d p(T->nodes[node_idx].config[0],T->nodes[node_idx].config[1],T->nodes[node_idx].config[2]);
    Quaterniond q(T->nodes[node_idx].config[3],T->nodes[node_idx].config[4],T->nodes[node_idx].config[5],T->nodes[node_idx].config[6]);
    std::vector<int> finger_to_relocate;

    for (int k = 0; k < NUM_FINGERS; k++){
        int fingers_left[NUM_FINGERS];
        for (int i = 0; i < NUM_FINGERS; i++){
            if (i==k){ 
                fingers_left[i] = OUT_OF_INDEX;
            } else {
                fingers_left[i] = T->nodes[node_idx].finger_locations[i];
            }

        }
        if (force_closure(p, q, fingers_left, object_surface_discretization)){
            finger_to_relocate.push_back(k);
        } 
    }
    if (finger_to_relocate.size() > 0){

        // randomly choose a finger to relocate
        int idx = int(randd()*finger_to_relocate.size());
        int finger_idx = finger_to_relocate[idx];
        // find all valid location to put this finger
        std::vector<int> candidate_locations;
        for (int i = 0; i < num_surface_points; i++){
            if(in_fingertip_workspace(p, q, i, finger_idx, finger_workspace, object_surface_discretization)){
                if (i!=T->nodes[node_idx].finger_locations[finger_idx])
                {
                    candidate_locations.push_back(i);
                }
            }
        }
        //randomly choose a valid position to put the finger
        if (candidate_locations.size() > 0){
            int finger_location = candidate_locations[int(candidate_locations.size()*randd())];
            Node new_node(T->nodes[node_idx].config, T->nodes[node_idx].finger_locations);
            new_node.finger_locations[finger_idx] = finger_location;
            T->add_node(&new_node, node_idx);
            T->nodes[T->nodes.size() - 1].cost = T->nodes[node_idx].cost 
                    + get_edge_cost(T, node_idx, T->nodes.size() - 1);
        }

    }
    return;
}

static void plannerRRT(
        double object_position_range[6],
        double finger_workspace[NUM_FINGERS*6],
        double* object_surface_discretization, 
        double start_object_config[7],
        int start_finger_locations[NUM_FINGERS],
        double goal_object_config[7], 
        double*** object_path,
        int*** finger_path,
        int* planlength,
        int* treesize,
        double rstar_radius=0){
    
    //no plan by default
	*object_path = NULL;
    *finger_path = NULL;
	*planlength = 0;
    mexPrintf("RRT STAR RADIUS: %f\n", rstar_radius);
    

    Vector3d pos_lb(object_position_range[0], object_position_range[1], object_position_range[2]);
    Vector3d pos_ub(object_position_range[3], object_position_range[4], object_position_range[5]);

    Vector3d start_position(start_object_config[0],start_object_config[1], start_object_config[2]);
    Quaterniond start_orientation(start_object_config[3], start_object_config[4], start_object_config[5], start_object_config[6]);

    Vector3d goal_position(goal_object_config[0],goal_object_config[1], goal_object_config[2]);
    Quaterniond goal_orientation(goal_object_config[3], goal_object_config[4], goal_object_config[5], goal_object_config[6]);

    set_rand_seed(); //set random seed by cuurent time
    
    // initialize the tree, create start node
    Node start_node(start_object_config, start_finger_locations);
    Tree T;
    T.initial_node(&start_node); 
    int goal_idx = -1;

    //search
    for (int kk = 0; kk < max_samples; kk++){

        // random sample which primitive to choose
        if (randd() < primitive1_prob){
            primitiveOne(randd(), &T, pos_lb, pos_ub, goal_position,
                         goal_orientation, finger_workspace,
                         object_surface_discretization, rstar_radius);
            // if dist to goal is smaller than some threshold, goal is found
            int near_idx = T.nearest_neighbor(goal_object_config);
            double dd = dist(T.nodes[near_idx].config, goal_object_config);
            if (goal_idx == -1 && dd <= goal_thr)
            {
                printf("Found goal node in %d samples.\n", kk + 1);
                goal_idx = near_idx;
                if (rstar_radius <= 0) {
                    break;
                }
            }

        } else {
            //TODO primitive 2: randomly choose a node to relocate 1 randomly choosed finger
            // randomly choose a node
            // randomly choose a finger without which other fingers can still maintain force closure
            // randomly relocate the finger to an unoccupied discretized object surface point
            double primitive2_near_prob = 0.8;
            int node_idx;
            if (randd() < primitive2_near_prob)
            {
                std::vector<int> nears;
                T.nearest_neighbors(goal_object_config, &nears);
                node_idx = nears[int(nears.size()*randd())];
            } else {
                node_idx = int(T.nodes.size()*randd());
            }
            primitiveTwo(node_idx, &T, finger_workspace, object_surface_discretization);
        }
        if (kk % 100 == 0) {
            std::cout << "Iteration: " << kk << " Nodes Expaneded: " << T.nodes.size() << std::endl;
        }
    }

    if (goal_idx!=-1){
        printf("GOAL REACHED! \n");
    } else {
        printf("GOAL NOT REACHED. \n");
        goal_idx =  T.nearest_neighbor(goal_object_config);
    }

    std::vector<int> node_path;
    T.backtrack(goal_idx, &node_path);
    int l = node_path.size();
    *planlength = l;
    *object_path = (double**) malloc(l*sizeof(double*));
    *finger_path = (int**) malloc(l*sizeof(int*));
    for (int i=0; i< l; i++){
        (*object_path)[i] = T.nodes[node_path[l-i-1]].config;
        (*finger_path)[i] = T.nodes[node_path[l-i-1]].finger_locations;
    }  
    *treesize = T.nodes.size();
    std::cout << "Plan cost: " << T.nodes[goal_idx].cost << std::endl;
    return;
} 


void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
     
{ 
    
    /* Check for proper number of arguments */    
    double rrtstar_radius = 0;
    if (nrhs < 7 || nrhs > 9) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Incorrect number of input arguments (7 or 8 or 9)."); 
    } 
    if (nrhs >= 8){
        max_samples = (int)*mxGetPr(MAX_SAMPLES);
    }
    if (nrhs >= 9) {
        rrtstar_radius = (double)*mxGetPr(RRTSTAR_RADIUS);
    }

    double* object_range = mxGetPr(POSRANGE_IN);      
    double* finger_workspace = mxGetPr(WORKSPACE_IN);
    double* surface = mxGetPr(SURFACE_IN);
    double* start_object_config = mxGetPr(OBJECT_START_IN);
    double* start_finger_d = mxGetPr(FINGER_START_IN);
    double*	goal_object_config = mxGetPr(OBJECT_GOAL_IN);
    int planner_id = (int)*mxGetPr(PLANNERIN_IN);

    int start_finger[NUM_FINGERS];
    for (int i = 0; i<NUM_FINGERS; i++){
        start_finger[i] = int(start_finger_d[i]) - 1;
    }

    //call the planner
    double** object_path = NULL;
    int** finger_path = NULL;
    int planlength = 0;
    int treesize = 0;
    
    //you can may be call the corresponding planner function here
    if (planner_id == RRT) {
      plannerRRT(object_range, finger_workspace, surface, start_object_config, start_finger, goal_object_config, 
        &object_path, &finger_path, &planlength, &treesize, rrtstar_radius);
    } else {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidID",
        "Planner ID can only be 0."); 
    }
    
    /* Create return values */
    if(planlength > 0)
    {
        OBJECTPATH_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)7, mxDOUBLE_CLASS, mxREAL); 
        FINGERPATH_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)NUM_FINGERS, mxDOUBLE_CLASS, mxREAL);
        double* object_out = mxGetPr(OBJECTPATH_OUT);   
        double* finger_out = mxGetPr(FINGERPATH_OUT);      
        //copy the values
        for(int i = 0; i < planlength; i++)
        {
            for (int j = 0; j < 7; j++)
            {
                object_out[j*planlength + i] = object_path[i][j];
            }
            for (int j = 0; j < NUM_FINGERS; j++)
            {
                finger_out[j*planlength + i] = double(finger_path[i][j])+1;
            }
        }
    }
    else
    {
        OBJECTPATH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)7, mxDOUBLE_CLASS, mxREAL); 
        FINGERPATH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)NUM_FINGERS, mxDOUBLE_CLASS, mxREAL);
        double* object_out = mxGetPr(OBJECTPATH_OUT);   
        double* finger_out = mxGetPr(FINGERPATH_OUT);      
        //copy the values

        for (int j = 0; j < 7; j++)
        {
            object_out[j] = start_object_config[j];
        }
        for (int j = 0; j < NUM_FINGERS; j++)
        {
            finger_out[j] = double(start_finger[j])+1;
        }
        
    }
    PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL); 
    TREESIZE_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL); 
    int* planlength_out = (int*) mxGetPr(PLANLENGTH_OUT);
    *planlength_out = planlength;
    int* treesize_out = (int*) mxGetPr(TREESIZE_OUT);
    *treesize_out = treesize;
    
    return;
    
}
