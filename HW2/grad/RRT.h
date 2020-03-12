#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <queue>
#include <set>
#include <map>
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
#define MINSTEP (PI/30)

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
  double parentID; // it's parent id
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
      temp  = points[index].angles[i] - qrand[i];
      dist+=temp*temp;
    }
    return sqrt(dist); // euclidean distance returned 
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

    for (int i = 0; i < numofDOFs; i++) {
      if (dist < abs(points[index].angles[i] - qrand[i])) {
        dist = abs(points[index].angles[i] - qrand[i]);
      }
    }
    int numofsamples = (int)(dist / MINSTEP);
    vector<double> intermediate(numofDOFs);

    for (int i = 0; i < numofsamples; i++) {
      for (int j = 0; j < numofDOFs; j++) {
        intermediate[j] = points[index].angles[j] + ((double)(i) / (numofsamples - 1))*(qrand[j] - points[index].angles[j]);
      }

      if (!IsValidArmConfiguration(intermediate.data(), numofDOFs, map, x_size, y_size)) return false;
    }
    return true;
  }
  bool ReachedGoal(int index){
    for(int i=0;i<numofDOFs;i++){
      if(armgoal_anglesV_rad[i] != points[index].angles[i]){
        return false;
      }
    }
    return true;
  }

  void backtrack(){
    int n = points.size() -1; 
    int index = n;// getting the goal pose
    for(int i=0;i<n;i++){
      backtrackplan.push_back(points[index].angles);
      if(index == 0) break;
      index = points[index].parentID;
    }
    return;
  }

  void return_plan() {

    *plan = NULL;
    *planlength = 0;

    int total = backtrackplan.size();
    *plan = (double**)malloc(total * sizeof(double*));
    for (int i = 0; i < total; i++) {
      (*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
      for (int j = 0; j < numofDOFs; j++) {
        (*plan)[i][j] = backtrackplan[i][j];
      }
    }
    *planlength = total;
    cout<<"plan length "<<total;
    return;
  }
  int getLastIndex(){
    return points.size()-1;
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
      if(distance(qnearID,intermediate) >E){
        cout<<" distance is more terminating with qnew\n";
        result = advanced;
        return result;
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
      for (int j = 0; j < numofDOFs; j++) qnew[j]=intermediate[j];
    }
    cout<<" valid config \n";
    result = reached; 
    return result;
  }

  void extend(vector<double> &qrand,int E){
    vector<double> qnew(numofDOFs);
    int qnearID = nearestNeighbour(qrand);
    cout<<"nearest neighbour "<<qnearID<<"\n";
    RRT_result result = newConfig(qrand,qnearID,qnew,E);
    cout<<"result "<<result<<"\n";
    if(result!=trapped){
      int childID = add_vertex(qnew);
      add_edge(childID,qnearID);
    }
    return;
  }

};

void randomSample(vector<double> & qrand){
  //srand((int)time(nullptr));
  for (int i = 0; i<qrand.size();i++){
    if(i == 0){
      qrand[i] = fmod(rand(),PI/2);
      std::cout<< " Q0"<<qrand[i]<< " ";
    }
    else
    {
      qrand[i] = fmod(rand(),(2*PI)); 
      std::cout<< " Q"<<i<<" "<<qrand[i]<< " ";
    }
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
// void startSample(vector<double> qrand,double* armstart_anglesV_rad){
//     // goal as sample
//   for(int i = 0; i<qrand.size();i++){
//     qrand[i] = armstart_anglesV_rad[i]; 
//     std::cout<< "start  Q"<<i<<" "<<Q[i]<< " ";
//   }
//   return;
// }
