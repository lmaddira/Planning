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

static std::vector<std::pair<int,int>> action_astar ;

static std::vector<int> Binary_vec;
static bool Optimal_path_found ;
static bool last_few_steps;
static int n_end,n_mid,n_start;
static int d;


bool If_robot_near_target(int robotposeX,int robotposeY,int targetposeX,int targetposeY,int action_size){
    int deltaX = abs(robotposeX-targetposeX);
    int deltaY = abs(robotposeY-targetposeY);
    double dist = sqrt((deltaX*deltaX) + (deltaY*deltaY));
    std::cout<<"diff"<<deltaX <<" "<<deltaY<<" "; 
    std::cout<<"distance now is "<<dist<<"action size now is "<<action_size<<"\n";
    if(dist< 7 || action_size < 10){
        return true;
    }      
    return false;
}

bool If_Reachable_by_robot(int action_size,int mid_index,int curr_time){
    if(action_size < (mid_index)){ //mid_index-curr_time
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
    int goalposeX,goalposeY;
    int start_index,mid_index,end_index;
    std::vector<int> low_cost_goal_index;
    
    
    for(int i = 0;i<target_steps;i++){
        int x = target_traj[i];
        int y = target_traj[i+target_steps];
        if((int)map[GETMAPINDEX(x,y,x_size,y_size)]<10){
            low_cost_goal_index.push_back(i);
        }
    }
    std::cout<<"number of low cost indices found are "<< low_cost_goal_index.size()<<" total target steps are "<<target_steps<<"\n";

    if(curr_time == 0){
        n_start = 0;
        n_end = low_cost_goal_index.size()-1;
        n_mid = round(n_end/4);
        action_astar.clear();
        Binary_vec.clear();
        start_index = low_cost_goal_index[0]; // ith position of the traj which is in low cost
        end_index =  low_cost_goal_index[n_end-1];
        mid_index = low_cost_goal_index[n_mid] ;
        Optimal_path_found = false;
        last_few_steps = false;
        d = 4;
        std::cout<<"nend is "<<n_end;
        
    }else{
        start_index = Binary_vec[0];
        end_index = Binary_vec[1];
        mid_index = Binary_vec[2];
        std::cout<<" Binary_search indices are"<<Binary_vec[0]<<" "<<Binary_vec[1]<<"and mid index is "<<Binary_vec[2]<<"\n";
    }
    

    // if(!Optimal_path_found && action_astar.size()>2300){
    //     std::cout<<"Discovered the action sequence with complete solution and now " ;
    //     std::cout<<" action.size is "<<action_astar.size()<<std::endl;
    //     action_ptr[0] = action_astar[action_astar.size()-1].first;
    //     action_ptr[1] = action_astar[action_astar.size()-1].second;
    //     action_astar.pop_back();Binary_vec[0];
    //     Binary_vec[0] = curr_time;
    //     Binary_vec[1] = end_index;
    //     Binary_vec[2] = (curr_time+end_index)/2;
    //     std::cout<<" Binary_search indices are"<<Binary_vec[0]<<" "<<Binary_vec[1]<<"and mid index is "<<Binary_vec[2]<<"\n";

    //     return;
    // }




    if(Optimal_path_found){
        std::cout<<"Discovered the action sequence with complete solution and now " ;
        std::cout<<" action.size is "<<action_astar.size()<<std::endl;
        if(robotposeX == goalposeX && robotposeY==goalposeY){
            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
            return;
        }
        for(int i=0;i<4;i++){
            if(robotposeX == target_traj[curr_time+i] && robotposeY == target_traj[curr_time+target_steps+i]){
                // if(i>1){
                //     action_ptr[0] = target_traj[curr_time+i-1];
                //     action_ptr[1] = target_traj[curr_time+i-1+target_steps];

                // }else{
                action_ptr[0] = robotposeX;
                action_ptr[1] = robotposeY;
                //}
                std::cout<< "retained the action"<<std::endl;
                return;  
            } 
        }
        action_ptr[0] = action_astar[action_astar.size()-1].first;
        action_ptr[1] = action_astar[action_astar.size()-1].second;
        action_astar.pop_back();
        if(If_robot_near_target(robotposeX,robotposeY,targetposeX,targetposeY,action_astar.size())){
            std::cout<<"now back to A star for closer target";
            Optimal_path_found = false;
            last_few_steps = true;
        }
        return;

    }
    if(last_few_steps){
        goalposeX = target_traj[curr_time+2];
        goalposeY = target_traj[curr_time+2+target_steps];
        std::cout<<"now the goal in last few steps is "<<goalposeX<<" "<<goalposeY<<"\n";

    }else{
        goalposeX = target_traj[mid_index];
        goalposeY = target_traj[mid_index+target_steps];

    }
    
    if(robotposeX == goalposeX && robotposeY==goalposeY){
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        return;
    }

    
    for(int i=0;i<7;i++){
        if(robotposeX == target_traj[curr_time+i] && robotposeY == target_traj[curr_time+target_steps+i]){
            // if(i>1){
            //     action_ptr[0] = target_traj[curr_time+i-1];
            //     action_ptr[1] = target_traj[curr_time+i-1+target_steps];

            // }else{
            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
            //}
            std::cout<< "retained the action"<<std::endl;
            return;  
        } 
    }
    


    if(!Optimal_path_found){// if optimal path found is false i.e both initially and last few steps
        if(!last_few_steps && curr_time != 0){
            int diff = abs(n_end-n_mid);
            std::cout<< "difference is "<<diff;
            std::cout<<"next start index will be "<<Binary_vec[0]<<" "<<Binary_vec[1]<<" "<<Binary_vec[2]<<"\n";
            if(diff < 2 && If_Reachable_by_robot(action_astar.size(),Binary_vec[2],curr_time)){
                std::cout<<"reached optimal position .... just execute steps\n";
                std::cout<<"difference btw start and mid is "<<diff<<"so goal pose would be "<<target_traj[Binary_vec[2]]<<" "<<target_traj[Binary_vec[2]+target_steps]<<"\n";
                Optimal_path_found = true;
            }
        }
        action_astar = do_Astar(map,collision_thresh,x_size,y_size,robotposeX,robotposeY,target_steps,target_traj,targetposeX,targetposeY,curr_time,goalposeX,goalposeY);
// this will automatically update the action sequence
    }
    


    action_ptr[0] = action_astar[action_astar.size()-1].first;
    action_ptr[1] = action_astar[action_astar.size()-1].second;
    std::cout<<"action size "<<action_astar.size()<<" ";
    std::cout<<"action "<< action_ptr[0] <<" "<< action_ptr[1]<<"goal "<<goalposeX<<" "<<goalposeY<< "\n";
    std::cout<<"cost of the goals "<<(int)map[GETMAPINDEX(goalposeX,goalposeY,x_size,y_size)]<<"\n";
    action_astar.pop_back();
    
    //Binary_search bin(robotposeX,robotposeY,target_traj,target_steps,start_index,end_index,action.size(),curr_time);
    //Binary_search(robotposeX,robotposeY,target_traj,target_steps,start_index,end_index,action.size(),curr_time).Get_Mid_target_index();
    
    if(!last_few_steps){
        Binary_vec.clear();
        
        if(!If_Reachable_by_robot(action_astar.size(),mid_index,curr_time)){ // if not reachabel by robot
            n_start = n_mid;
            n_mid = (n_start+n_end)/d;
            if(d==2 && (n_start == n_mid)){
                n_start = n_end;
                n_mid = n_end+1;
                n_end = n_end+2;
                Binary_vec.push_back(low_cost_goal_index[n_start]);// pushing back next start_node
                Binary_vec.push_back(low_cost_goal_index[n_end]);
                Binary_vec.push_back(low_cost_goal_index[n_mid]);
            }
            if(n_start == n_mid){
                d = 2;
            }
            Binary_vec.push_back(low_cost_goal_index[n_start]);// pushing back next start_node
            Binary_vec.push_back(low_cost_goal_index[n_end]);
            Binary_vec.push_back(low_cost_goal_index[n_mid]);
            std::cout<<" its not reachable checking next goals between"<<n_start<<" and start "<<Binary_vec[0]<< " end  "<<n_end<<" and end "<<Binary_vec[1]<<" "<<n_mid<<" midd "<<Binary_vec[2]<<"\n";
        }else{
            n_end = n_mid;
            n_mid = (n_start+n_end)/4;
            Binary_vec.push_back(low_cost_goal_index[n_start]);// pushing back next start_node
            Binary_vec.push_back(low_cost_goal_index[n_end]);
            Binary_vec.push_back(low_cost_goal_index[n_mid]);
            std::cout<<" its reachable checking next goals between"<<Binary_vec[0]<< " "<<Binary_vec[1]<<" "<<Binary_vec[2]<<"\n";
        }
        
        
    }

    // action_ptr[0] = 0;
    // action_ptr[1] = 0;

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
