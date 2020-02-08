/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <mex.h>
#include <vector>
#include <map>
#include <limits>
#include <iostream>
#include <algorithm>

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

/*double heuristics(int x,int y, int goalposeX,int goalposeY){
	double h = sqrt(((x-goalposeX)*(x-goalposeX) + (y-goalposeY)*(y-goalposeY)));//euclidean distance
}*/






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
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    double g[x_size][y_size]; 
    double h[x_size][y_size];
    double f[x_size][y_size];
    std::vector <std::pair <int,int> > OPEN;
    std::vector <std::pair <int,int> > CLOSED;
    int goalposeX;
    int goalposeY;
    if (sqrt((robotposeX-targetposeX)*(robotposeX-targetposeX) + (robotposeY-targetposeY)*(robotposeY-targetposeY)) > 10){
    	goalposeX = target_traj[curr_time+5];
    	goalposeY = target_traj[target_steps+curr_time+5];

    }else{
    	goalposeX = targetposeX;
    	goalposeY = targetposeY;
    }
    
    
    OPEN.push_back(std::make_pair(robotposeX,robotposeY));
	// initiate g and f as infinity
    for (int i=0;i<x_size;i++){
    	for(int j=0;j<y_size;j++){
    		g[i][j]= std::numeric_limits<int>::max() ;
    	}
    }
    for (int i=0;i<x_size;i++){
    	for(int j=0;j<y_size;j++){
    		f[i][j]=  std::numeric_limits<int>::max();
    	}
    }

    // create all h values of map as its a constant values for each cell
    for (int i=0;i<x_size;i++){
    	for(int j=0;j<y_size;j++){
    		h[i][j]= abs(i- goalposeX)+abs(j-goalposeY);//sqrt(((i- goalposeX)*(i-goalposeX) + (j-goalposeY)*(j-goalposeY)));
    	}
    }
    g[robotposeX][robotposeY] = 0;
    f[robotposeX][robotposeY] = g[robotposeX][robotposeY] + h[robotposeX][robotposeY];
    //std::cout<<"f(robopose) "<< f[robotposeX][robotposeY] << std::endl;

    while(OPEN.size()!= 0 ){
    	int check= std::numeric_limits<int>::max();
    	int s_x,s_y;
    	// only if g changes then f changes so update the value of f along with g always.
    	// get new s from f(s) values from open loop
    	for (int i=0;i<OPEN.size();i++){
    	 	int a = OPEN[i].first;
    	 	int b = OPEN[i].second;
    		if(check > f[a][b]){
    			check = f[a][b];
    			//std::cout<< "check "<<check<<std::endl;
    			s_x = a;
    			s_y = b;
    		}
	    }
	    // insert this new s in closed
	    CLOSED.push_back(std::make_pair(s_x,s_y));
	    if(s_x == goalposeX && s_y == goalposeY)
	    	break;
	    // remove s from OPEN
	    //auto remove = std::find(OPEN.begin(), OPEN.end(), std::make_pair(s_x,s_y));
   		OPEN.erase(std::find(OPEN.begin(), OPEN.end(), std::make_pair(s_x,s_y)));
	    //for every successor of s update g and f
	    for(int dir = 0; dir < NUMOFDIRS; dir++)
		{
		    int newx = s_x + dX[dir];
		    int newy = s_y + dY[dir];

		    if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) // ensuring inside the map limits
		    {
		        if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
		        {
		            if (g[newx][newy] > (g[s_x][s_y]+ (int)map[GETMAPINDEX(newx,newy,x_size,y_size)])) { // checking if g(s') > g(s)+c(s,s')
		            	
		            	g[newx][newy] = (g[s_x][s_y]+ (int)map[GETMAPINDEX(newx,newy,x_size,y_size)]); // update g and f values in array
		            	f[newx][newy] = g[newx][newy] + 10 * h[newx][newy];
		            	//std::cout<<"f values at "<<newx<<" "<<newy<<" is "<<f[newx][newy]<<std::endl;
		            	if (std::find(OPEN.begin(),OPEN.end(),std::make_pair(newx,newy)) == OPEN.end()){
		            		OPEN.push_back(std::make_pair(newx,newy));// insert s' in OPEN 
		            	}
		            	
		            }
		            
		        }
		    }
		}

	}
	/*for (int i=0;i<CLOSED.size();i++){
		std::cout<<"CLOSED set contains "<<CLOSED[i].first<<" "<<CLOSED[i].second<<std::endl;
	}*/

	// once goal is reached lets back track the route and make a array of path
	int s_x = goalposeX;
 	int s_y = goalposeY;
 	int bestX ;
	int bestY ;
 	std::vector<std::pair<int,int>> action;
	while( (s_x!= robotposeX || s_y != robotposeY ) && CLOSED.size()!= 0 ){
		
		int check =  std::numeric_limits<int>::max();
    	 	
		for(int dir = 0; dir < NUMOFDIRS; dir++)
		{
		    int newx = s_x + dX[dir];
		    int newy = s_y + dY[dir];
		    if(std::find(CLOSED.begin(),CLOSED.end(),std::make_pair(newx,newy))!=CLOSED.end()){
		    	if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) // ensuring inside the map limits
			    {
			    	if(check > (g[newx][newy]+(int)map[GETMAPINDEX(newx,newy,x_size,y_size)]) ){ 
			    		check = (g[newx][newy]+(int)map[GETMAPINDEX(newx,newy,x_size,y_size)]); // if in closed loop and g+c is best then go to that s
			    		bestX = newx;
			    		bestY = newy;
			    	}
			       
			    }

		    } 

		    
		}
		// it returns bestX, bestY in cosed loop
		auto remove = std::find(CLOSED.begin(), CLOSED.end(), std::make_pair(s_x,s_y));
   		CLOSED.erase(remove);
		action.push_back(std::make_pair(s_x,s_y));
		s_x = bestX;
		s_y = bestY;
		
	}/*
	for (int i=0;i<action.size();i++){
		std::cout<<"action set contains "<<action[i].first<<" "<<action[i].second<<std::endl;
	}
	std::cout<<"action size "<<action.size()<<std::endl;
    */
    action_ptr[0] = action[action.size()-1].first;
    action_ptr[1] = action[action.size()-1].second;
    
    std::cout<<"Robotpose "<<robotposeX<<" "<<robotposeY<<std::endl;
    std::cout<<"action of robot "<<action_ptr[0]<<" "<<action_ptr[1]<<std::endl;
    //std::cout<<"map dimensions "<<x_size<<" "<<y_size<<std::endl;
    std::cout<<"goalpose "<<goalposeX<<" "<<goalposeY<<std::endl;
 
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
