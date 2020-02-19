/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <mex.h>
#include "planner.h"

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8


static std::vector<std::pair<int,int>> action ;


double heuristics(int goalposeX,int goalposeY, int X,int Y){
    int deltaX = X-goalposeX;
    int deltaY = Y-goalposeY;
    return std::max(std::abs(deltaX),std::abs(deltaY))+ 0.4*std::min(std::abs(deltaX),std::abs(deltaY));//sqrt(deltaX*deltaX + deltaY*deltaY);
}

 

const int weight =10000;
static std::vector<int> Binary_vec;
static bool Optimal_path_found ;
static bool last_few_steps;
double cal_f_val(int g_val,int h_val){
    return g_val + weight * h_val ;
}
bool If_robot_near_target(int robotposeX,int robotposeY,int targetposeX,int targetposeY,int action_size){
    int deltaX = (robotposeX-targetposeX);
    int deltaY = (robotposeY-targetposeY);
    double dist = sqrt((deltaX*deltaX) + (deltaY*deltaY));
    std::cout<<"diff"<<deltaX <<" "<<deltaY<<" "; 
    std::cout<<"distance now is "<<dist<<"action size now is "<<action_size<<"\n";
    if(dist< 2 || action_size < 2){
        return true;
    }      
    return false;
}
bool If_Reachable_by_robot(int action_size,int mid_index,int curr_time){
    if(action.size() < (mid_index)){ //mid_index-curr_time
        return true;
    }
    return false;
}


static void planner(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{
    // 8-connected grid
    std::cout<< "curr_time:  "<<curr_time << "data" << std::endl;
    std::cout<<"Robotpose "<<robotposeX<<" "<<robotposeY<<" "<<"Target pose "<<targetposeX<<" "<<targetposeY<<std::endl;
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    int start_index,end_index,mid_index;
    int goalposeX,goalposeY;
    HashTable <std::pair<int,int>,double> CLOSED_table;
    std::vector<Object> OPEN;

    if(curr_time == 0){
        action.clear();
        start_index = 0;
        end_index =  target_steps;
        mid_index = round(target_steps/2);
        Optimal_path_found = false;
        last_few_steps = false;
    }else{
        start_index = Binary_vec[0];
        end_index = Binary_vec[1];
        mid_index = Binary_vec[2];
        std::cout<<" Binary_search indices are"<<Binary_vec[0]<<" "<<Binary_vec[1]<<"and mid index is "<<Binary_vec[2]<<"\n";
    }

    if(Optimal_path_found){
        std::cout<<"Discovered the action sequence with complete solution and now " ;
        std::cout<<" action.size is "<<action.size()<<std::endl;
        action_ptr[0] = action[action.size()-1].first;
        action_ptr[1] = action[action.size()-1].second;
        action.pop_back();
        if(If_robot_near_target(robotposeX,robotposeY,targetposeX,targetposeY,action.size())){
            std::cout<<"now back to A star for closer target";
            Optimal_path_found = false;
            last_few_steps = true;
        }
        return;

    }


    
    

    std::cout<<"closed table # elem at start "<< CLOSED_table.GetN();


    if(last_few_steps){
        goalposeX = target_traj[curr_time];
        goalposeY = target_traj[curr_time+target_steps];
        std::cout<<"now the goal is "<<goalposeX<<" "<<goalposeY<<"\n";

    }else{
        goalposeX = target_traj[mid_index];
        goalposeY = target_traj[mid_index+target_steps];

    }

    
    for(int i=0;i<7;i++){
        if(robotposeX == target_traj[curr_time+i] && robotposeY == target_traj[curr_time+target_steps+i]){
            if(i>1){
                action_ptr[0] = target_traj[curr_time+i-1];
                action_ptr[1] = target_traj[curr_time+i-1+target_steps];

            }else{
                action_ptr[0] = robotposeX;
                action_ptr[1] = robotposeY;
            }
            std::cout<< "retained the action"<<std::endl;
            return;  
        } 
    }
    
    
    //if(;;){
    //if(curr_time == 0 || curr_time % 400 == 399){
    double g_val =0;
    double h_val = heuristics(goalposeX,goalposeY, robotposeX,robotposeY);
    double f_val = cal_f_val(g_val,h_val);
    
    OPEN.push_back(Object(g_val,h_val,f_val,Make_Node(robotposeX,robotposeY)));
    std::make_heap(OPEN.begin(),OPEN.end(),myComparator());
    auto goal = Make_Node(goalposeX,goalposeY);
    //std::vector<object2> CLOSED;


    // compute the path 
    while (OPEN.size() != 0){
        std::push_heap(OPEN.begin(),OPEN.end(),myComparator());// maintaing the order of heap
        auto min_node = OPEN.front().Node; // returns the smallest Node by soring the f value
        auto min_g_val = OPEN.front().g;
        std::cout<<"min node "<< min_node.first<<" "<<min_node.second<<"\n";
         
        std::pop_heap(OPEN.begin(),OPEN.end(),myComparator());
        OPEN.pop_back(); // remove this node from OPEN
        // if goal reached stop the loop
        if (min_node == goal){
            break;
        }
        //CLOSED.push_back(object2(min_g_val,min_node)); 
        CLOSED_table.Update(min_node,min_g_val); // inserting the smallest node and g_val to closed table as Hashtable - goal will not be pushed back into the Closed

        //std::cout<<"passed beyond break \n"; 
        for(int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int newx = min_node.first + dX[dir];
            int newy = min_node.second + dY[dir];

            if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
            {
                if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
                {
                    //std::cout<<"checked that this node is availible...check if g_val current is greater than new g_val\n";
                    //if (g_val > (min_g_val+ (int)map[GETMAPINDEX(newx,newy,x_size,y_size)])) { // checking if g(s') > g(s)+c(s,s')
                            
                    g_val = (min_g_val+ (int)map[GETMAPINDEX(newx,newy,x_size,y_size)]); // update g and f values in array
                    h_val = heuristics(goalposeX,goalposeY,newx,newy);
                    f_val = cal_f_val(g_val,h_val);
                    //std::cout<<"f values at "<<newx<<" "<<newy<<" is "<<f[newx][newy]<<std::endl;
                    //std::cout<<"updated f values";
                    // now check if the node is in closed and open";
                    int count2 = 0;
                    if(CLOSED_table.IsIncluded(Make_Node(newx,newy))==true){
                        count2++;// not updating this node in closed as in simple A star we won't get g values revised after getting in closed.
                        //std::cout<<"in closed loop and g value got updated.... make this function";
                    }

                    int count = 0;
                    for(int i = 0;i < OPEN.size();i++){
                        if(Make_Node(newx,newy)==OPEN[i].Node){
                            //std::cout<<"node found in OPEN loop "<<newx << " "<<newy<<"\n" ;
                            if(OPEN[i].g > g_val){   
                                OPEN[i].g = g_val;
                                OPEN[i].f = f_val;
                            }
                            count++;
                            break;
                        }
                    }
                    if(count == 0 && count2 == 0){// if not in open and closed
                        OPEN.push_back(Object(g_val,h_val,f_val,Make_Node(newx,newy)));
                        std::cout<<"pushing in the following "<<newx<<" "<<newy<<" "<<g_val<<" "<<f_val<<"\n";
                    } 

                }

            }
            std::cout<<"going for next";
        }  

    } 
    // duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    // std::cout<<"Duration after open loop "<< duration <<'\n';
    //std::cout<<"planning is done now tracking back\n";
    
    std::pair<int,int> best;

    auto track = goal;
    //auto action2 = action;
    action.clear();

    std::cout<<"closed table # elem after open loop "<< CLOSED_table.GetN()<<"\n";

    while(  track != Make_Node(robotposeX,robotposeY) && CLOSED_table.GetN()>= 0){
        int check = std::numeric_limits<int>::max(); // some max value
        action.push_back(track);
        //int count =0;
        //std::cout<<"inside closed loop \n";
        for(int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int newx = track.first + dX[dir];
            int newy = track.second + dY[dir];
            auto back_track_node = Make_Node(newx,newy);
            //for(int i = 0;i < CLOSED.size();i++){
            if(CLOSED_table.IsIncluded((back_track_node))){
                //std::cout<<"node found in OPEN loop \n";
                auto cost = *CLOSED_table[(back_track_node)]+(int)map[GETMAPINDEX(newx,newy,x_size,y_size)] ;  
                if(check >= cost){ 
                    check = cost ; // if in closed loop and g+c is best then go to that s
                    best = back_track_node;

                    //count++;
                    //std::cout<<"count inside the CLOSED loop \n";
                }
            }
            
        }
        // duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        // std::cout<<"Duration after open loop "<< duration <<'\n';

        track = best;
        //std::cout<< " pushing back " <<bestX <<" "<<bestY<<"\n";
        //CLOSED.erase(CLOSED.begin()+best); // deletes the ith element;
        CLOSED_table.Delete((best));
        
    }
   

    std::cout<<"closed table # elem after closed loop "<< CLOSED_table.GetN()<<"\n";
    
    action_ptr[0] = action[action.size()-1].first;
    action_ptr[1] = action[action.size()-1].second;
    std::cout<<"action size "<<action.size()<<" ";
    std::cout<<"action "<< action_ptr[0] <<" "<< action_ptr[1]<<"goal "<<goalposeX<<" "<<goalposeY<< "\n";
    std::cout<<"cost of the goals "<<(int)map[GETMAPINDEX(goalposeX,goalposeY,x_size,y_size)]<<"\n";
    action.pop_back();
    
    //Binary_search bin(robotposeX,robotposeY,target_traj,target_steps,start_index,end_index,action.size(),curr_time);
    //Binary_search(robotposeX,robotposeY,target_traj,target_steps,start_index,end_index,action.size(),curr_time).Get_Mid_target_index();
    
    if(!last_few_steps){
        Binary_vec.clear();
        
        if(!If_Reachable_by_robot(action.size(),mid_index,curr_time)){ // if not reachabel by robot
            Binary_vec.push_back(mid_index);// pushing back next start_node
            Binary_vec.push_back(end_index);
            int new_mid = (mid_index+end_index)/2;
            Binary_vec.push_back(new_mid);
        }else{
            Binary_vec.push_back(start_index);// pushing back next start_node
            Binary_vec.push_back(mid_index);
            int new_mid = (mid_index+start_index)/2;
            Binary_vec.push_back(new_mid);
        }
        int diff = abs(Binary_vec[0]-Binary_vec[2]);
        std::cout<<"next start index will be "<<Binary_vec[0]<<" "<<Binary_vec[1]<<" "<<Binary_vec[2]<<"\n";
        if(diff < 3 && If_Reachable_by_robot(action.size(),mid_index,curr_time)){
            std::cout<<"reached optimal position .... just execute steps\n";
            std::cout<<"so goal pose would be "<<target_traj[mid_index]<<" "<<target_traj[mid_index+target_steps]<<"\n";
            Optimal_path_found = true;
        }
    }

    //action_ptr[0] = 0;
    //action_ptr[1] = 0;

    return;
}


// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;   
}


/*

    // for now greedily move towards the final target position,
    // but this is where you can put your planner

    int goalposeX = (int) target_traj[target_steps-1];
    int goalposeY = (int) target_traj[target_steps-1+target_steps];
    // printf("robot: %d %d;\n", robotposeX, robotposeY);
    // printf("goal: %d %d;\n", goalposeX, goalposeY);

    int bestX = 0, bestY = 0; // robot will not move if greedy action leads to collision
    double olddisttotarget = (double)sqrt(((robotposeX-goalposeX)*(robotposeX-goalposeX) + (robotposeY-goalposeY)*(robotposeY-goalposeY)));
    double disttotarget;
    for(int dir = 0; dir < NUMOFDIRS; dir++)
    {
        int newx = robotposeX + dX[dir];
        int newy = robotposeY + dY[dir];

        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
        {
            if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
            {
                disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
                if(disttotarget < olddisttotarget)
                {
                    olddisttotarget = disttotarget;
                    bestX = dX[dir];
                    bestY = dY[dir];
                }
            }
        }
    }
	robotposeX = robotposeX + bestX;
    robotposeY = robotposeY + bestY;
    
}   */
