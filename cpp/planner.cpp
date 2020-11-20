#include "contact_kinematics.h"
#include "tree.h"
#include "mex.h"

#define PI 3.141592654
#define NUM_FINGERS 4

/* Input Arguments */
#define	POSRANGE_IN      prhs[0]
#define	WORKSPACE_IN	prhs[1]
#define	SURFACE_IN     prhs[2]
#define	OBJECT_START_IN     prhs[3]
#define	FINGER_START_IN     prhs[4]
#define	OBJECT_GOAL_IN     prhs[5]
#define PLANNERIN_IN     prhs[6]

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

void to_config(double config[7], Vector3d p, Quaterniond q){
    config[0] = p[0];
    config[1] = p[1];
    config[2] = p[2];
    config[3] = q.w();
    config[4] = q.x();
    config[5] = q.y();
    config[6] = q.z();
}


bool force_closure(Vector3d p, Quaterniond q, int* finger_locations, double* object_surface_discretization, int num_of_contacts){
    return true;
}

bool in_fingertip_workspace(Vector3d p, Quaterniond q, int finger_idx, int finger_location, double finger_workspace[NUM_FINGERS*6], double* object_surface_discretization){
    return true;
}

bool IsValidConfiguration(double config[7], int finger_locations[NUM_FINGERS], double finger_workspace[NUM_FINGERS*6], double* object_surface_discretization){
    //TODO: IsValidConfiguration: check constraints
    // finger workspace constraints: kinematics included
    // force closure constraints: friction included
    return true;
}


int extend(int near_idx, double config_rand[7], 
double epsilon_translation, double epsilon_angle, int steps, Tree* T, double finger_workspace[NUM_FINGERS*6], double* object_surface_discretization)
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

        if(IsValidConfiguration(config_check, T->nodes[near_idx].finger_locations, finger_workspace, object_surface_discretization)) // TODO
        {
            status = 1;//advanced
            to_config(config_new, x_check, q_check);
            if (dist(config_new, config_rand)<0.001) { status = 2; break; }//reached
            
        } else { break; }
    }

    if (status!=0){
      Node node_new(config_new, T->nodes[near_idx].finger_locations);
      node_new.cost = T->nodes[near_idx].cost + dist(config_new, config_near); // TODO: cost functions
      T->add_node(&node_new, near_idx);    
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
                 double* object_surface_discretization)
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
        for (int i = 0; i < nears.size(); i++) {
            // extend function
            code = extend(nears[i], config_rand, epsilon_translation,
                          epsilon_angle, interpolation_steps,T,
                          finger_workspace, object_surface_discretization);
            if (code != 0) {
                numAdded++;
            }
        }
        return numAdded;
}


void primitiveTwo(Tree* T, double finger_workspace[NUM_FINGERS*6], double* object_surface_discretization){
    int node_idx = int(T->nodes.size()*randd());
    Vector3d p(T->nodes[node_idx].config[0],T->nodes[node_idx].config[1],T->nodes[node_idx].config[2]);
    Quaterniond q(T->nodes[node_idx].config[3],T->nodes[node_idx].config[4],T->nodes[node_idx].config[5],T->nodes[node_idx].config[6]);
    std::vector<int> finger_to_relocate;
    for (int k = 0; k < NUM_FINGERS; k++){
        int fingers_left[NUM_FINGERS - 1];
        int counter = 0;
        for (int i = 0; i < NUM_FINGERS; i++){
            if (i==k) { continue; }
            fingers_left[counter] = T->nodes[node_idx].finger_locations[i];
            counter++;
        }
        if (force_closure(p, q, fingers_left, object_surface_discretization, NUM_FINGERS-1)){
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
            if(in_fingertip_workspace(p, q, finger_idx, i, finger_workspace, object_surface_discretization)){
                candidate_locations.push_back(i);
            }
        }
        //randomly choose a valid position to put the finger
        int finger_location = candidate_locations[int(candidate_locations.size()*randd())];
        Node new_node(T->nodes[node_idx].config, T->nodes[node_idx].finger_locations);
        new_node.finger_locations[finger_idx] = finger_location;
        T->add_node(&new_node, node_idx);
    }
    return;
}

static void plannerRRT(double object_position_range[6], double finger_workspace[NUM_FINGERS*6], double* object_surface_discretization, 
    double start_object_config[7], int start_finger_locations[NUM_FINGERS], double goal_object_config[7], 
    double*** object_path, int*** finger_path, int* planlength, int* treesize){
    
    //no plan by default
	*object_path = NULL;
    *finger_path = NULL;
	*planlength = 0;
    

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
                         object_surface_discretization);
            // if dist to goal is smaller than some threshold, goal is found
            int near_idx = T.nearest_neighbor(goal_object_config);
            double dd = dist(T.nodes[near_idx].config, goal_object_config);
            if (dd <= goal_thr)
            {
                printf("goal reached \n");
                goal_idx = near_idx;
                break;
            }

        } else {
            //TODO primitive 2: randomly choose a node to relocate 1 randomly choosed finger
            // randomly choose a node
            // randomly choose a finger
            // check if other fingers can still maintain force closure
            // if yes: randomly relocate the finger to an unoccupied discretized object surface point
            // if no: check other fingers until all fingers are checked
            continue;
        }
    }

    if (goal_idx!=-1){
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
        return;
    }

} 

static void plannerRRTCONNECT(double object_position_range[6], double finger_workspace[NUM_FINGERS*6], double* object_surface_discretization, 
    double start_object_config[7], int start_finger_locations[NUM_FINGERS], double goal_object_config[7], 
    double*** object_path, int*** finger_path, int* planlength, int* treesize){
    
    //no plan by default
	*object_path = NULL;
    *finger_path = NULL;
	*planlength = 0;
    

    Vector3d pos_lb(object_position_range[0], object_position_range[1], object_position_range[2]);
    Vector3d pos_ub(object_position_range[3], object_position_range[4], object_position_range[5]);

    Vector3d start_position(start_object_config[0],start_object_config[1], start_object_config[2]);
    Quaterniond start_orientation(start_object_config[3], start_object_config[4], start_object_config[5], start_object_config[6]);

    Vector3d goal_position(goal_object_config[0],goal_object_config[1], goal_object_config[2]);
    Quaterniond goal_orientation(goal_object_config[3], goal_object_config[4], goal_object_config[5], goal_object_config[6]);

    set_rand_seed(); //set random seed by cuurent time
    
    // initialize the tree, create start node
    Node start_node(start_object_config, start_finger_locations);
    // TODO: Right now set goal for fingers to be same as start location,
    // but we should think about how to handle this better in the future.
    Node goal_node(goal_object_config, start_finger_locations);
    Tree startTree, goalTree, T1, T2;
    startTree.initial_node(&start_node); 
    goalTree.initial_node(&goal_node);
    bool extendStartTree = 1;
    int numAdded, code, nearestIdx, nidx;
    double* config;


    //search
    for (int kk = 0; kk < max_samples; kk++){
        if (extendStartTree) {
            T1 = startTree;
            T2 = goalTree;
        } else {
            T1 = goalTree;
            T2 = startTree;
        }
        // random sample which primitive to choose
        if (randd() < primitive1_prob) {
            numAdded = primitiveOne(randd(), &T1, pos_lb, pos_ub,
                                    goal_position, goal_orientation,
                                    finger_workspace,
                                    object_surface_discretization);
            // Since we may have added multiple multiple nodes, try to
            // connect all of them.
            if (numAdded > 0) {
                for (nidx = T1.nodes.size() - numAdded;
                        nidx < T1.nodes.size(); nidx++) {
                    config = T1.nodes[nidx].config;
                    nearestIdx = T2.nearest_neighbor(config) ;
                    code = connect(&nearestIdx, config, epsilon_translation,
                                   epsilon_angle, interpolation_steps, &T2, 
                                   finger_workspace,
                                   object_surface_discretization);
                    if (code == 2) {
                        // Able to connect the trees, build the plan.
                        std::vector<int> startPath, goalPath;
                        if (extendStartTree) {
                            T1.backtrack(nidx, &startPath);
                            T2.backtrack(nearestIdx, &goalPath);
                        } else {
                            T1.backtrack(nidx, &goalPath);
                            T2.backtrack(nearestIdx, &startPath);
                        }
                        int l = startPath.size() + goalPath.size();
                        *planlength = l;
                        *object_path = (double**) malloc(l*sizeof(double*));
                        *finger_path = (int**) malloc(l*sizeof(int*));
                        for (int i = 0; i < startPath.size(); i++) {
                            int idx = startPath.size() - i - 1;
                            (*object_path)[i] = 
                                startTree.nodes[startPath[idx]].config;
                            (*finger_path)[i] =
                                startTree.nodes[startPath[idx]].finger_locations;
                        }
                        for (int i = startPath.size(); i < l; i++) {
                            int idx = i - startPath.size();
                            (*object_path)[i] = 
                                goalTree.nodes[goalPath[idx]].config;
                            (*finger_path)[i] =
                                goalTree.nodes[goalPath[idx]].finger_locations;
                        }
                        *treesize = startTree.nodes.size()
                                    + goalTree.nodes.size();
                        return;
                    }
                }
            }
            extendStartTree = ~extendStartTree;
        } else {
            //TODO PRIMITIVE 2
            continue;
        }
    }
    return;
} 



void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
     
{ 
    
    /* Check for proper number of arguments */    
    if (nrhs != 7) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Seven input arguments required."); 
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
    if (planner_id == RRT)
    {
      plannerRRT(object_range, finger_workspace, surface, start_object_config, start_finger, goal_object_config, 
        &object_path, &finger_path, &planlength, &treesize);
    } else if (planner_id == RRTCONNECT) {
        plannerRRTCONNECT(object_range, finger_workspace, surface,
                          start_object_config, start_finger,
                          goal_object_config, &object_path, &finger_path,
                          &planlength, &treesize);
    }
    else
    {
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
