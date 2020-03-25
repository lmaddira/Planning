#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <queue>
#include <map>
#include <unordered_map>
#include <cstdio>
#include <ctime>
#include "mex.h"

using namespace std;

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654
#define MINSTEP (PI/60)

//the length of each link in the arm (should be the same as the one used in runtest.m)
#define LINKLENGTH_CELLS 10

typedef struct {
  int X1, Y1;
  int X2, Y2;
  int Increment;
  int UsingYIndex;
  int DeltaX, DeltaY;
  int DTerm;
  int IncrE, IncrNE;
  int XIndex, YIndex;
  int Flipped;
} bresenham_param_t;


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size)
{
    double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params)
{
  params->UsingYIndex = 0;

  if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
    (params->UsingYIndex)++;

  if (params->UsingYIndex)
    {
      params->Y1=p1x;
      params->X1=p1y;
      params->Y2=p2x;
      params->X2=p2y;
    }
  else
    {
      params->X1=p1x;
      params->Y1=p1y;
      params->X2=p2x;
      params->Y2=p2y;
    }

   if ((p2x - p1x) * (p2y - p1y) < 0)
    {
      params->Flipped = 1;
      params->Y1 = -params->Y1;
      params->Y2 = -params->Y2;
    }
  else
    params->Flipped = 0;

  if (params->X2 > params->X1)
    params->Increment = 1;
  else
    params->Increment = -1;

  params->DeltaX=params->X2-params->X1;
  params->DeltaY=params->Y2-params->Y1;

  params->IncrE=2*params->DeltaY*params->Increment;
  params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
  params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

  params->XIndex = params->X1;
  params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y)
{
  if (params->UsingYIndex)
    {
      *y = params->XIndex;
      *x = params->YIndex;
      if (params->Flipped)
        *x = -*x;
    }
  else
    {
      *x = params->XIndex;
      *y = params->YIndex;
      if (params->Flipped)
        *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params)
{
  if (params->XIndex == params->X2)
    {
      return 0;
    }
  params->XIndex += params->Increment;
  if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
    params->DTerm += params->IncrE;
  else
    {
      params->DTerm += params->IncrNE;
      params->YIndex += params->Increment;
    }
  return 1;
}



int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
		   int x_size,
 		   int y_size)

{
	bresenham_param_t params;
	int nX, nY; 
    short unsigned int nX0, nY0, nX1, nY1;

    //printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
    
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

    //printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
            return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
		   int x_size, int y_size)
{
    double x0,y0,x1,y1;
    int i;
    
 	//iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
    y1 = 0;
	for(i = 0; i < numofDOFs; i++)
	{
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
				return 0;
	}    
    return 1;
}

////////////////////////
// my code
///////////////////

struct Node{
  vector<double> angles;
  double ID; // it's own id
  double parentID; // it's parent id equivalent to tree id in PRM
  vector<int> neighbours; // index of neighbours used in PRM 
  double f;
  double g;
};

struct Compare { 
    bool operator()(Node const& N1, Node const& N2) 
    { 
        // return "true" if "p1" is ordered  
        // before "p2", for example: 
        return N1.f > N2.f; 
    } 
};

// general planner functions

class searchBasedPlanner{
public:
  vector<Node> points;
  vector<vector<double>> backtrackplan;
  double*  map;
  int x_size;
  int y_size;
  double* armstart_anglesV_rad;
  double* armgoal_anglesV_rad;
  int numofDOFs;
  double*** plan;
  int* planlength;

  searchBasedPlanner(double*  map,int x_size,  int y_size,  double* armstart_anglesV_rad,  double* armgoal_anglesV_rad,  int numofDOFs,  double*** plan,  int* planlength){
    this->map = map;
    this->x_size = x_size;
    this->y_size = y_size;
    this->armstart_anglesV_rad = armstart_anglesV_rad;
    this->armgoal_anglesV_rad = armgoal_anglesV_rad;
    this->numofDOFs = numofDOFs;
    this->plan = plan;
    this->planlength = planlength;
  }

  // add new node to the planner graph
  int add_vertex(vector<double> &qnew){
    int size = points.size(); // take the size of the current node to add next node 
    points.push_back(Node());
    points[size].angles = qnew;
    print_angles(qnew);
    points[size].ID = size;
    return size;
  }

  // connect edge and retain its parent index
  void add_edge(int child,int parent){
    points[child].parentID = parent;
    return ;
  }
  

  double distance(int index, vector<double> &qrand){
    double dist=0;
    double temp;
    for(int i=0;i<numofDOFs;i++){
      temp  = points[index] .angles[i] - qrand[i];
      dist+=temp*temp;
    }
    return sqrt(dist); // euclidean distance returned 
  }
  double distance(int index1,int index2){
    return distance(index1,points[index2].angles);
  }
  // brute force search for nearest neighbour
  int nearestNeighbour(vector<double> &qrand){
    double min_dist = INT8_MAX; // max number
    int index;
    double dist;     
    for(int i=0;i<points.size();i++){
      dist = distance(i,qrand);
      if(min_dist>dist){
        min_dist = dist;
        index = i;
      } 
    }
    return index;
  }

  // checks the validity of the edge and returns the place where edge stops being valid
  bool valid_edge(int index, vector<double> & qrand) {
    double dist = 0;
    cout<<" checking validity of edge ";
    for (int i = 0; i < numofDOFs; i++) {
      if (dist < abs(points[index].angles[i] - qrand[i])) {
        dist = abs(points[index].angles[i] - qrand[i]);
      }
    }
    int numofsamples = (int)(dist / MINSTEP);
    cout<<" num of samples "<<numofsamples;
    vector<double> intermediate(numofDOFs);

    for (int i = 0; i < numofsamples; i++) {
      for (int j = 0; j < numofDOFs; j++) {
        intermediate[j] = points[index].angles[j] + ((double)(i) / (numofsamples - 1))*(qrand[j] - points[index].angles[j]);
      }

      if (!IsValidArmConfiguration(intermediate.data(), numofDOFs, map, x_size, y_size)){
        cout<<" invalid arm configuration for sample "<<numofsamples<<"\n";
        return false;
      } 
    }
    cout<<" edge is valid \n";
    return true;
  }

  bool valid_edge(int index1,int index2){
    cout<<" checking validity of edge between "<<index1<<" "<<index2<<"\n";
  	return valid_edge(index1,points[index2].angles);
  }
  bool ReachedGoal(int index){
    for(int i=0;i<numofDOFs;i++){
      if(armgoal_anglesV_rad[i] != points[index].angles[i]){
        return false;
      }
    }
    return true;
  }

  void print_angles(vector<double> &angles){
  	for(int i=0;i<numofDOFs;i++){
  		std::cout<<" Q"<<i<<" "<<angles[i];
  	}
  	std::cout<<"\n";
  }

  void backtrack(){
    int n = points.size()-1; 
    int index = n;// getting the goal pose
    std::cout<<"index in backtrack "<<index<<"\n";
    backtrackplan.push_back(points[index].angles);
    print_angles(points[index].angles);
    for(int i=0;i<n;i++){
    	std::cout<<" in the loop of backtrack \n";
    	double distance = 0;
    	print_angles(backtrackplan.back());
	   	for (int j = 0; j < numofDOFs; j++){
	        if(distance < fabs(points[index].angles[j] - backtrackplan.back()[j])){
	            distance = fabs(points[index].angles[j] - backtrackplan.back()[j]);
	          	std::cout<<" distance "<<distance<<std::endl;
            }
	    }
	    std::cout<<"distance "<<distance<<" ";
	    int numofsamples = (int)(distance/(PI/40));
	    printf("num of samples in backtrack %d \n",numofsamples);
	    vector<double> intermediate(numofDOFs);
	    if(numofsamples>2){
	    	for (int k = 0; k < numofsamples; k++) {
	      		for (int j = 0; j < numofDOFs; j++) {
	        		intermediate[j] = backtrackplan.back()[j] + ((double)(k) / (numofsamples - 1))*(points[index].angles[j]-backtrackplan.back()[j]);
	      		}

	      		std::cout<<" pushingback in backtrack intermidiate \n";
	      		backtrackplan.push_back(intermediate);
	      	}

	    }else{
	    	std::cout<<" pushing back the index angles \n";
	    	backtrackplan.push_back(points[index].angles);
	    }
	    

	    if(index == 0) break;
	    std::cout<<"going to next index\n";
	    index = points[index].parentID;
    }
    backtrackplan.push_back(points[index].angles);
    return;
  }
  void get_backtrack(vector<vector<double>> &newplan){
    int n = backtrackplan.size();
    for(int i=0;i<n;i++){
      newplan.push_back(backtrackplan[i]);
    }
  }
  void return_plan() {
    *plan = NULL;
    *planlength = 0;

    int total = backtrackplan.size();
    *plan = (double**)malloc(total * sizeof(double*));
    for (int i = 0; i < total; i++) {
      (*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
      for (int j = 0; j < numofDOFs; j++) {
        (*plan)[i][j] = backtrackplan[total-1-i][j];
        std::cout<<" plan "<< backtrackplan[total-1-i][j];
      }
      std::cout<<"\n";
    }
    *planlength = total;
    cout<<"plan length "<<total;
    return;
  }
  int getLastIndex(){
    return points.size()-1;
  }
  void getLastAngle(vector<double> &qnew){
    int n = points.size()-1;
    for(int i=0;i<numofDOFs;i++){
      qnew[i]=points[n].angles[i];
    }
    return;
  }


};
enum RRT_result{
  advanced,reached,trapped
};

// RRT planner related functions 
class RRTplanner : public searchBasedPlanner{
public:
  RRTplanner(double*  map,int x_size,  int y_size,  double* armstart_anglesV_rad,  double* armgoal_anglesV_rad,  int numofDOFs,  
    double*** plan,  int* planlength): searchBasedPlanner(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad,numofDOFs, plan, planlength){}

  RRT_result newConfig(vector<double> & qrand, int qnearID, vector<double> & qnew, double E){
    RRT_result result = trapped;
    double dist = 0;

    for (int i = 0; i < numofDOFs; i++) {
      if (dist < abs(points[qnearID].angles[i] - qrand[i])) {
        dist = abs(points[qnearID].angles[i] - qrand[i]);
      }
    }
    int numofsamples = (int)(dist / MINSTEP);
    vector<double> intermediate(numofDOFs);
    for (int i = 0; i < numofsamples; i++) {
      for (int j = 0; j < numofDOFs; j++) {
        intermediate[j] = points[qnearID].angles[j] + ((double)(i) / (numofsamples - 1))*(qrand[j] - points[qnearID].angles[j]);
      }
      
      if (!IsValidArmConfiguration(intermediate.data(), numofDOFs, map, x_size, y_size)){
        if(i==0){
          return result;
        }else{
          cout<<" invalid config terminating with newq\n";
          result = advanced;
          return result;
        }
      }
      if(distance(qnearID,intermediate) >E){
        cout<<" distance is more terminating with qnew\n";
        result = advanced;
        return result;
      }
      for (int j = 0; j < numofDOFs; j++) qnew[j]=intermediate[j];
    }
    cout<<" valid config \n";
    result = reached; 
    return result;
  }

  void extend(vector<double> &qrand,double E){
    vector<double> qnew(numofDOFs);
    int qnearID = nearestNeighbour(qrand);
    cout<<"nearest neighbour "<<qnearID<<"\n";
    RRT_result result = newConfig(qrand,qnearID,qnew,E);
    cout<<"result "<<result<<"\n";
    if(result!=trapped){
    	int childID = add_vertex(qnew);
    	std::cout<<"adding vertex  as id"<<childID<< "\n";
      add_edge(childID,qnearID);
    }
    return;
  }

};
class RRTconnectplanner : public RRTplanner{
public:
  RRTconnectplanner(double*  map,int x_size,  int y_size,  double* armstart_anglesV_rad,  double* armgoal_anglesV_rad,  int numofDOFs,  
    double*** plan,  int* planlength): RRTplanner(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad,numofDOFs, plan, planlength){}

  
  bool connect(vector<double> &qrand){ // returns if it met the point qrand (same as extend)
    vector<double> qnew(numofDOFs);
    int qnearID = nearestNeighbour(qrand);
    cout<<"nearest neighbour "<<qnearID<<"\n";
    RRT_result result = newConfig(qrand,qnearID,qnew,INT8_MAX);
    cout<<"result "<<result<<"\n";
    if(result!=trapped){
      int childID = add_vertex(qnew);
      add_edge(childID,qnearID);
      if(result==reached){ 
        return true;
      }
    }
    return false;
  }
  void returnConnect_plan(vector<vector<double>> &startplan,vector<vector<double>> &goalplan) {
    *plan = NULL;
    *planlength = 0;

    int start_total = startplan.size();
    int goal_total = goalplan.size();
    cout<<"start plan length "<<start_total<<" goal plan length "<<goal_total<<"\n";
    int total = start_total+goal_total;
    *plan = (double**)malloc(total * sizeof(double*));
    for (int i = 0; i < start_total; i++) {
      cout<<"\nplan "<<i<<" ";
      (*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
      for (int j = 0; j < numofDOFs; j++) {
        (*plan)[i][j] = startplan[start_total-1-i][j];
        cout<<" Q"<<j<<" "<<startplan[start_total-1-i][j];
      }
    }
    cout<<"pushing the goal tree now\n";
    for (int i = start_total; i < total; i++) {
      cout<<"\nplan "<<i<<" ";
      (*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
      for (int j = 0; j < numofDOFs; j++) {
        (*plan)[i][j] = goalplan[i-start_total][j];
        cout<<" Q"<<j<<" "<<goalplan[i-start_total][j];
      }
    }
    *planlength = total;
    cout<<"plan length "<<total;
    return;
  }  
};


class RRTstarplanner : public RRTplanner{
public:
	RRTstarplanner(double*  map,int x_size,  int y_size,  double* armstart_anglesV_rad,  double* armgoal_anglesV_rad,  int numofDOFs,  
    double*** plan,  int* planlength): RRTplanner(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad,numofDOFs, plan, planlength){}

	// adds all nearest neighbours in neighbours vector // we are doing brute force for the second time//check if more efficient way is possible
    void neighboursInRadius(int qnewID, double radius){ 
    	double min_dist = INT8_MAX; // max number
	    double dist;     
	    for(int i=0;i<points.size();i++){
	    	if(i==qnewID) continue;
        dist = distance(i,qnewID);
        if(dist<radius){
          if(valid_edge(i,qnewID)) // check if the edge to the new point from this neighbour is valid and then add it in neighbours
          {
            points[qnewID].neighbours.push_back(i);
          }
        }
	    }
    }

    double cost(int fromID,int toID){ // cost fromID + cost of edge fromID to toID
    	return points[fromID].g + distance(fromID,toID);
    }

    // add edge from qnew to parent(with mincost in neighbours) and updated cost of qnew
    void add_minCost_edge(int qnewID){
    	int min_index;
    	double mincost = INT8_MAX;
    	for(int i=0;i<points[qnewID].neighbours.size();i++){
    		if(mincost > cost(i,qnewID)){
    			min_index = i;
    			mincost = cost(i,qnewID);	
          std::cout<<"min cost now "<<mincost <<" with ID"<<i<<"\n";
    		} 
    	}
    	add_edge(qnewID,min_index); 
    	points[qnewID].g = mincost;
      std::cout<<"cost of vertex "<<qnewID<<" is "<<points[qnewID].g<<"\n";
    	return ;
    }
    void update_edge(int qnewID,int qnearID){
    	add_edge(qnearID,qnewID);// parent gets updated so no need to exclusivley remove the edge	
    	points[qnearID].g = cost(qnewID,qnearID); // update cost of edge
    }
    void rewire(int qnewID){
    	for(int i=0;i<points[qnewID].neighbours.size();i++){
    		if(points[i].g > cost(qnewID,i)){ // if current cost of neighbour > cost of qnew + edge from qnew to neighbour
    			// remove prev edge and add new edge from qnew and update new cost
    			update_edge(qnewID,i); 
          std::cout<<"checking the parent to see if it's actually updated"<<points[i].parentID<<"\n";
          std::cout<<"updated the edge/ rewired  of "<<i<<" to "<<qnewID<<"\n"; 			
    		} 
    	}
    }

    double radius(double E, double gamma){
      auto v = points.size();
      double volume[] = {1,2,3.142,4.189,4.935,5.264,5.168,4.725,4.059,3.299,2.55,1.884,1.335};
      double delta;
      if(numofDOFs <12){
        delta = volume[numofDOFs];
      }else{
        delta = 1;
      }
      double temp = gamma*log(v)/v/delta;
      double d = 1.0/numofDOFs;
      std::cout<<"temp "<<temp<<" delta "<<delta<<" logv/v "<<log(v)/v<<"temp pow 1/dof "<<(double)pow(temp,d) <<" 1/dof "<<d<<"\n";
      double rad = (double)MIN((double)pow(temp,d),E);
      std::cout<<"radius is "<<rad<< "and E val "<<E<<"\n";
      return rad;    
    }

    // extend for RRT star is little different 
    void extend_rewire(vector<double> &qrand,double E,double gamma){
	    vector<double> qnew(numofDOFs);
	    int qnearID = nearestNeighbour(qrand);
	    cout<<"nearest neighbour "<<qnearID<<"\n";
	    RRT_result result = newConfig(qrand,qnearID,qnew,E); 
	    cout<<"result "<<result<<"\n";
	    if(result!=trapped){
	      int qnewID = add_vertex(qnew); // we don't add edge here in this step in RRT star
	      neighboursInRadius(qnewID,radius(E,gamma)); // now we have all vertex in radius which can form valid edge
	      printf(" number of vertices in neighours %d \n",points[qnewID].neighbours.size());
        std::cout<<"neighbours are ";
        for(auto i:points[qnewID].neighbours){
          std::cout<<" "<<i<<"\n";
        }
        if(points[qnewID].neighbours.size()==0){
          std::cout<<"Error with radius seection..... ";
          return;
        } 
	      add_minCost_edge(qnewID);
	      rewire(qnewID);
	    }
	    return;
    }
    


};

class PRMplanner : public searchBasedPlanner{
public:
  PRMplanner(double*  map,int x_size,  int y_size,  double* armstart_anglesV_rad,  double* armgoal_anglesV_rad,  int numofDOFs,  
    double*** plan,  int* planlength): searchBasedPlanner(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad,numofDOFs, plan, planlength){}

  int add_vertex(vector<double> &qnew){
    int size = points.size(); // take the size of the current node to add next node 
    points.push_back(Node());
    points[size].angles = qnew;
    points[size].ID = size;
    points[size].g = INT8_MAX;
    points[size].parentID = size;
    cout<<" parent being initialised in add_vertex "<< points[size].parentID<<"\n";
    return size;
  }
  // gives the tree_root parent index 
  int tree_root(int index){
    if(points[index].parentID!=index){
      points[index].parentID = tree_root(points[index].parentID);
    }
    return points[index].parentID;
  }
  // will return if the parents are same that is if they are in same tree.
  bool same_tree(int index1,int index2){
    int a1 = tree_root(index1);
    int a2 = tree_root(index2);
    cout<<" tree of old point "<<index1<< " is "<<a1<<" and "<<index2<<" "<<a2<<"\n";
    int check =0;
    if(a1==a2){
      check++;
    }
    if(check==0){
      return false;
    }
    return true;
  }
  void add_all_edges(int child, int parent){
    points[child].neighbours.push_back(parent);
    points[parent].neighbours.push_back(child);
    points[tree_root(parent)].parentID = tree_root(child);
    cout<<" tree is "<<tree_root(parent)<<"\n";
    return;

  }
  void adjacentNeighbours(int newpointID, int radius){
    int index;
    double dist;   
    for(int i=0;i<points.size();i++){
      if(i==newpointID) continue;
      dist = distance(i,newpointID); // if a point is found in the radius then add the point edge both sides
      if(dist<radius){
        cout<<" found distance with in radius at index "<<i<<" \n";
        if(!same_tree(i,newpointID)|| points[newpointID].neighbours.size()< 8){ // max number of same tree connections can be 8 
          if(valid_edge(i, points[newpointID].angles)){
            cout<<"valid edge \n";
            add_all_edges(i,newpointID);
            cout<<"added edge \n";
          }
        }
      } 
    }
    return;
  }
  

  void find_path(int startindex,int goalindex){
    //Do backward Astar search with the graph build by random samples 
    vector<Node> OPEN;
    unordered_map<int, double> CLOSED;
    int weight = 10; // or u can get this from main function
    // intiate gval, h and f val for start
    cout<<"started backward A star planning.....\n";
    OPEN.clear();
    cout<<"OPEN size at start "<<OPEN.size()<<"\n";
    points[startindex].g = 0;
    points[startindex].f = points[startindex].g + 10*distance(startindex,goalindex); // f = g+h;
    OPEN.push_back(points[startindex]); // pushing back a node
    cout<<" pushed back goal point and its index is "<<startindex<<"\n";

    int n=0;
    cout<<"checking if start has any neighbours "<<points[goalindex].neighbours.size()<<"\n";
    while(OPEN.size()>0)
    {
      n++;
      make_heap(OPEN.begin(),OPEN.end(),Compare());
      push_heap(OPEN.begin(),OPEN.end(),Compare());
      // cout << "all nodes in OPEN now ";
      // for(int i=0;i<OPEN.size();i++){
      //   cout<<OPEN[i].ID<<" and f value is "<<OPEN[i].f<<" \n";
      // }
      cout<<"\n";
      Node min_node = OPEN.front();
      pop_heap(OPEN.begin(),OPEN.end(),Compare());
      OPEN.pop_back();
      // cout << "all nodes in OPEN now after pop back ";
      // for(int i=0;i<OPEN.size();i++){
      //   cout<<OPEN[i].ID<<" ";
      // }
      double min_gval = min_node.g;
      int min_index = min_node.ID; 
      cout<<" min_index..."<<min_index<<"\n"; 
      cout<<"no of neighbours for minnode is "<<min_node.neighbours.size()<<"\n";
      if(min_index == goalindex){
        CLOSED.insert(make_pair(min_index,min_gval));
        cout<<" goal point expanded by A star algorithm\n";
        break;
      }
      CLOSED.insert(make_pair(min_index,min_gval));
      for(int i=0;i< min_node.neighbours.size();i++){ // for all the neighbours find the min f val
        int index = min_node.neighbours[i];
        //cout<<"neighbour index is "<<index<<"\n";
        if(CLOSED.find(index)==CLOSED.end()) // if not found in closed loop
        {
          //cout<<"not in closed loop \n";
          double g_val = min_gval + distance(min_index,index); // g(s') = g(s)+c(s,s');
          double h_val = distance(goalindex,index);
          double f_val = g_val+ weight*h_val; 
          int count =0; 
          for(int i=0;i<OPEN.size();i++){
            //cout<<" ID in OPEN now is "<<OPEN[i].ID<<" index is "<<index<<"\n";
            if(OPEN[i].ID == index){
              if(OPEN[i].g > g_val){
                OPEN[i].g = g_val;
                OPEN[i].f = f_val;
                cout<<"updating f_val\n";
              }
              count++;
              break;
            }
          }
          if(count == 0){
            points[index].g = g_val;
            points[index].f = f_val;
            OPEN.push_back(points[index]);
            cout<<"adding new point to OPEN \n";
          }
        }    
      }
    }
    auto track = goalindex;
    int best_index;
    cout<<" OPEN loop expanded goal and size is "<<OPEN.size()<<" \n";
    cout<<" closed loop size before backtrack  "<<CLOSED.size()<<" \n";
    cout<<"all points in closed loop ";
    for(auto &i:CLOSED){
      cout<<"index "<<i.first<<" g value "<<i.second<<" \n";
    }
    if(CLOSED.find(goalindex)==CLOSED.end()){
      cout<<" goal not found in the given sample set.... planning failed\n";
      return;
    }

    while(track!=startindex && CLOSED.size()>0){
      double check = INT8_MAX;
      CLOSED.erase(track);
      backtrackplan.push_back(points[track].angles);
      for(auto index : points[track].neighbours){
        cout<<"checking current neighbour "<<index<<" ";
        if(CLOSED.find(index)!=CLOSED.end()){
          cout<<"\n found this neighbour in closed "<<index<<"\n";
          auto cost = CLOSED[index]+ distance(index,track);
          if(check > cost){
            check = cost;
            best_index = index;
          }
        }
      }
      track = best_index;
      cout<<"best index "<<track<<"\n";
    }
    backtrackplan.push_back(points[track].angles);
    cout<<"planning done\n";
    return;
  }


}; 

void randomSample(vector<double> & qrand){
  //srand((int)time(nullptr));
  for (int i = 0; i<qrand.size();i++){
    // if(i == 0){
    //   qrand[i] = fmod(rand(),PI/2);
    //   std::cout<< " Q0"<<qrand[i]<< " ";
    // }
    // else
    // {
    qrand[i] = fmod(rand(),(2*PI)); 
    std::cout<< " Q"<<i<<" "<<qrand[i]<< " ";
    // }
  }
  //std::cout<<"\n";
  return;
}
void goalSample(vector<double> &qrand,double* armgoal_anglesV_rad){
  // goal as sample
  for(int i = 0; i<qrand.size();i++){
    qrand[i] = armgoal_anglesV_rad[i]; 
    std::cout<< "goal  Q"<<i<<" "<<qrand[i]<< " ";
  }
  
  return;
}

// double delta_val(int numofDOFs){
//   switch (numofDOFs)
//   {
//     case 1:
//       return 1;
//     case 2:
//       return 3.42;
//     case 3:
//       return 4.189;
//     case 4:
//       return 4.935;
//     case 5:
//       return 5.264;
//     case 6:
//       return 5.168;
//     case 7:
//       return 4.725;
//     default:
//       return 1;
//   }
    
// }
int rand50() 
{ // rand() function will generate odd or even number with equal probability. If rand() generates odd number, the function will return 1 else it will return 0. 
    return rand() & 1; 
}  
bool rand75() 
{ // Random Function to that returns 1 with 75% probability and 0 with 25% probability using Bitwise OR 
    return rand50() | rand50(); 
} 

bool rand90()
{
  return (rand75() | rand50());
}
void goalarea_Sample(vector<double> &qrand,double* armgoal_anglesV_rad){
  // goal as sample

  for(int i = 0; i<qrand.size();i++){
    if(rand50()){
      qrand[i] = armgoal_anglesV_rad[i] + (fmod(rand(),PI/60)); 
    }else{
      qrand[i] = max(0.0, armgoal_anglesV_rad[i] - (fmod(rand(),PI/60)));
    }
    std::cout<< "goal  Q"<<i<<" "<<qrand[i]<< " ";
  }
  
  return;
}

