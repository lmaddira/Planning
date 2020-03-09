#include <iostream>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <stdlib.h>
#include <time.h>
#include <chrono>
#include <unordered_map>
#include <typeinfo> 
#include "kd_tree.h"
#include "hashset.h"
#include "hashbase.h"
#include "hashtable.h"

#define PI 3.141592654
static const int E = 3; // look and change accordingly 
//the length of each link in the arm (should be the same as the one used in runtest.m)

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define LINKLENGTH_CELLS 10

int rand50() 
{ 
    // rand() function will generate odd or even 
    // number with equal probability. If rand() 
    // generates odd number, the function will 
    // return 1 else it will return 0. 
    return rand() & 1; 
} 
  
// Random Function to that returns 1 with 75% 
// probability and 0 with 25% probability using 
// Bitwise OR 
bool rand75() 
{ 
    return rand50() | rand50(); 
} 

bool rand90()
{
	return (rand75() | rand75());
}


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
    //printf(" x1 %d and angle %d at i %d ",x1,angles[i],i);
    	std::cout<< " x1 "<<x1<<" angle "<< angles[i]<< " at i "<<i<<" ";
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);
    //printf(" y1 %d \n",y1);
    	std::cout<<" y1 "<<y1<<std::endl;
		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
				return 0;
	}    
    return 1;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void randomSample(int numofDOFs,double * Q){
	//double Q[numofDOFs];
	//srand((int)time(nullptr));
	for (int i = 0; i<numofDOFs;i++){
		if(i == 0){
			Q[i] = fmod(rand(),PI);
			//std::cout<< " Q0"<<Q[i]<< " ";
		}
		else
		{
			Q[i] = fmod(rand(),(2*PI)); 
			//std::cout<< " Q"<<i<<" "<<Q[i]<< " ";
		}
	}
	std::cout<<std::endl;
	return;
}
void startSample(int numofDOFs,double * Q,double* armstart_anglesV_rad){
		// goal as sample
	for(int j = 0; j<numofDOFs;j++){
		Q[j] = armstart_anglesV_rad[j];	
		std::cout<< "start  Q"<<j<<" "<<Q[j]<< " ";
	}
	return;
}

void goalSample(int numofDOFs,double * Q,double* armgoal_anglesV_rad){
	// goal as sample
	for(int j = 0; j<numofDOFs;j++){
		Q[j] = armgoal_anglesV_rad[j];	
		std::cout<< "goal  Q"<<j<<" "<<Q[j]<< " ";
	}
	return;
}

struct object{
	std::array<double,k> point;
	// object(){
	// 	this->point[]=NULL;
	// }
	object(std::array<double,k> point){
		for(int i=0;i<k;i++){
			this->point[i] = point[i];
		}
	}
	bool operator==(const object& p) const
    { 
    	for(int i=0;i<k;i++){
    		if(this->point[i] != p.point[i])
    			return false;
    	}
        return true; 
    } 
};
template <>
unsigned long long int HashBase< std::array<double,k>>::HashCode(const std::array<double,k> &key) const
{

    unsigned long long int hash = 0;
    for(int i=0;i<k;i++){
    	double m = key[i];
    	hash+=(int)m ;
    }
    //hash = (key[0] *31) + (key[1] *23 +key[2]*17)  ;

    return hash;

}
//HashTable <object,object> umap;
HashTable <std::array<double,k>,std::array<double,k>> umap;
// class MyHashFunction { 
// public: 
  
//     // Use sum of lengths of first and last names 
//     // as hash function. 
//     size_t hash = 0;
//     size_t operator()(const object& p) const
//     { 
//     	for(int i=0;i<k;i++){
//     		hash+=(size_t)p.point[i]; 
//     	}
       
//         return hash;
//     } 
    
// }; 
// unordered_map<object,object> umap;

void Qnew(double* Q_new,double* Q_near,double* Q_rand,int m,int n){ // Qnear and Qrand to be bisected in m:n ratio.  
	std::cout<<" new Q in function\n";
	for(int i=0;i<k;i++){
		//std::cout<<"Qnear point"<<Q_near[i]<<" Q_rand "<<Q_rand[i]<<" m "<<m<<" n "<<n <<"\n";
		Q_new[i] = (((m*Q_rand[i]) + (n*Q_near[i]))/(m+n)) ;
		//Q_new[i] = 0;//Q_near[i] ;
		//std::cout<<" Q"<<i<<" "<<Q_new[i]<<"\n";
	}
	return;
}
double* Isvalidedge(double* parent,double* child,int numofDOFs,double* map,int x_size,int y_size){
	double Q_new[k];
	std::cout<<"Im here";
	auto Q_prev = parent;
	for(int i =1; i<= E ;i++){
		Qnew(Q_new,parent,child,i,((2*E)-i));
		std::cout<<"before valid arm";

		if(!IsValidArmConfiguration(Q_new, numofDOFs, map,x_size,y_size)){
			return Q_prev;
		}
		Q_prev = Q_new;
		std::cout<<"it is valid config at iteration "<<i <<std::endl;

	}
	return child;
}

bool ReachedGoal(double* Q_new,double* Q_near,double* Q_goal){
	double slope = (Q_goal[0]-Q_near[0])/(Q_new[0]-Q_near[0]);
	for(int i=1;i<k;i++){
		if(slope != (Q_goal[i]-Q_near[i])/(Q_new[i]-Q_near[i])){
			return false;
		}
	}
	return true;
}

int Extend(Node *root,double* Q_rand,int min_dist,int numofDOFs,double* map,int x_size,int y_size,double* armgoal_anglesV_rad){	
	auto Qnear = nearestNode(root,Q_rand);
	std::array<double,k> parent;
	std::array<double,k> child;
	std::cout<<" nearest neighbour \n";
	for(int i =0;i<k;i++){
		std::cout<<" Q"<<i<<" "<< Qnear->point[i]<<"\n";
	}
	int advanced = 1;

	if(distance(Qnear->point,Q_rand) > min_dist){
		double Q_new[numofDOFs];
		double dist = distance(Qnear->point,Q_rand);
	 	std::cout<<"distance is "<<dist <<"\n";
		Qnew(Q_new,Qnear->point,Q_rand,(int)E,ceil(dist-E));
		for(int i =0;i<k;i++){
			Q_rand[i] = Q_new[i];
		}
	}

	auto valid = Isvalidedge(Qnear->point,Q_rand,numofDOFs,map,x_size,y_size);
	for(int i =0;i<k;i++){
		// if(valid[i]!=Q_rand[i]){
		// 	std::cout<< "couldn't reach child ";
		// 	advanced = 1;
		// }
		Q_rand[i] = valid[i]; // update Q_rand with valid point that can be reachable. 
		if(valid[i]==Qnear->point[i]){
			std::cout<<"point trapped";
			advanced +=1;
		}
	}
	if(advanced==k){
		std::cout<<"point trapped so returning \n";
		advanced = 0;
		return advanced; // if basically got trapped then return 
	}else{
		advanced = 1;
	}
	if(ReachedGoal(Q_rand,Qnear->point,armgoal_anglesV_rad)){ // if reached goal then insert goal in tree and return 
		std::cout<<"reached goal \n";
		for(int i=0;i<k;i++){
			child[i] = armgoal_anglesV_rad[i];
			parent[i] = Qnear->point[i];
			std::cout<<"parent "<<i<<" "<<parent[i]<<" child in goal "<<child[i]<<"\n";
		}
		insert(root,armgoal_anglesV_rad);
		advanced =2;
		umap.Update((child),(parent));
		return advanced;
	}else{
		std::cout<<"didn't reach goal yet \n"; // else update the tree with new node
		for(int i=0;i<k;i++){
			child[i] = Q_rand[i];
			parent[i] = Qnear->point[i];
		}
		insert(root,Q_rand);
		umap.Update((child),(parent));
		return advanced;
	}
	
	


	// std::cout<<"if it is a valid edge ";
	// auto valid = Isvalidedge(Qnear->point,Q_rand,numofDOFs,map,x_size,y_size);
	// for(int i =0;i<k;i++){
	// 	std::cout<<" valid return "<<i<<" "<< valid[i]<<"\n";
	// }
	// if(distance(Qnear->point,Q_rand) <= min_dist && IsValidArmConfiguration(Q_rand, numofDOFs, map,x_size,y_size)){// case 1 : distance < e and is valid config
	// 	std::cout<<" distance < epsilon and valid config \n";
	// 	if(ReachedGoal(Q_rand,Qnear->point,armgoal_anglesV_rad)){
	// 		std::cout<<" reached goal in here \n";
	// 		insert(root,armgoal_anglesV_rad);
	// 		advanced =2;
	// 		for(int i=0;i<k;i++){
	// 			child[i] = armgoal_anglesV_rad[i];
	// 			parent[i] = Qnear->point[i];
	// 		}
	// 		umap.Update((child),(parent)); // Q_rand - is the child and it's value is parent - this will be used to backtrack once we reach goal.

	// 		return advanced;
	// 	}else{
	// 		std::cout<<" didn't reached goal in here inserting new point\n";
	// 		insert(root,Q_rand);
	// 		advanced = 1;
	// 		for(int i=0;i<k;i++){
	// 			child[i] = Q_rand[i];
	// 			parent[i] = Qnear->point[i];
	// 		}

	// 		umap.Update((child),(parent)); // Q_rand - is the child and it's value is parent - this will be used to backtrack once we reach goal.
	// 		return advanced;
	// 	}
			
	// }else if(distance(Qnear->point,Q_rand) > min_dist){
	// 	double dist = distance(Qnear->point,Q_rand);
	// 	std::cout<<"distance is "<<dist <<"\n";
	// 	double Q_new[numofDOFs];
	// 	Qnew(Q_new,Qnear->point,Q_rand,(int)E,ceil(dist-E));
	// 	std::cout<<"distance is greater than epsilon \n";
	// 	std::cout<<" new Q here \n";
	// 	for(int i =0;i<k;i++){
	// 		std::cout<<" Q"<<i<<" "<<Q_new[i]<<"\n";
	// 	}
	// 	if(IsValidArmConfiguration(Q_new, numofDOFs, map,x_size,y_size)){
	// 		std::cout<<"valid with new Q \n";
	// 		if(ReachedGoal(Q_new,Qnear->point,armgoal_anglesV_rad)){
	// 			std::cout<<" reached goal in here \n";
	// 			advanced = 2;
	// 			insert(root,armgoal_anglesV_rad);
	// 			for(int i=0;i<k;i++){
	// 				child[i] = armgoal_anglesV_rad[i];
	// 				parent[i] = Qnear->point[i];
	// 			} 				
	// 			umap.Update((child),(parent));
	// 			return advanced;
	// 		}else{
	// 			std::cout<<" didn't reached goal in here  inserting new point\n";
	// 			advanced = 1;
	// 			insert(root,Q_new);
	// 			for(int i=0;i<k;i++){
	// 				child[i] = Q_new[i];
	// 				parent[i] = Qnear->point[i];
	// 			} 				
	// 			umap.Update((child),(parent));
	// 			return advanced;
	// 		}
	// 		return advanced;
	// 	}else{
	// 		std::cout<<"not valid config with new Q\n";
	// 		//while(!advanced && distance(Q_new,Qnear->point)> 2){
	// 		double Q_new[numofDOFs];
	// 		Qnew(Q_new,Qnear->point,Q_rand,1,1);
	// 		std::cout<<" new Q here \n";
	// 		for(int i =0;i<k;i++){
	// 			std::cout<<" Q"<<i<<" "<<Q_new[i]<<"\n";
	// 		}
	// 		if(IsValidArmConfiguration(Q_new, numofDOFs, map,x_size,y_size)){
				
	// 			if(ReachedGoal(Q_new,Qnear->point,armgoal_anglesV_rad)){
	// 				std::cout<<" reached goal in here \n";
	// 				advanced = 2;
	// 				insert(root,armgoal_anglesV_rad);
	// 				for(int i=0;i<k;i++){
	// 					child[i] = armgoal_anglesV_rad[i];
	// 					parent[i] = Qnear->point[i];
	// 				} 				
	// 				umap.Update((child),(parent));
	// 				return advanced;
	// 			}else{
	// 				std::cout<<" didn't reached goal in here inserting new point\n";
	// 				advanced = 1;
	// 				insert(root,Q_new);
	// 				for(int i=0;i<k;i++){
	// 					child[i] = Q_new[i];
	// 					parent[i] = Qnear->point[i];
	// 				} 				
	// 				umap.Update((child),(parent));
	// 				return advanced;
	// 			}
	// 		}
	// 		//}
	// 		return advanced;
	// 	}

	// }else{// if(distance(Qnear->point,Q_rand) <= min_dist && !IsValidArmConfiguration(Q_rand, numofDOFs, map,x_size,y_size))
	// 	std::cout<<"distance is < but not valid config \n";	
	// 	auto  Q_new = Q_rand;
	// 	//while(!advanced && distance(Q_new,Qnear->point)<2){
	// 	Qnew(Q_new,Qnear->point,Q_rand,1,1);
	// 	// std::cout<<"new point config \n";
	// 	// for(int i =0;i<k;i++){
	// 	// 	std::cout<<" Q"<<i<<" "<<Q_new[i]<<"\n";
	// 	// }
	// 	if(IsValidArmConfiguration(Q_new, numofDOFs, map,x_size,y_size)){
	// 		if(ReachedGoal(Q_new,Qnear->point,armgoal_anglesV_rad)){
	// 			std::cout<<" reached goal in here \n";
	// 			advanced = 2;
	// 			insert(root,armgoal_anglesV_rad);
	// 			for(int i=0;i<k;i++){
	// 				child[i] = armgoal_anglesV_rad[i];
	// 				parent[i] = Qnear->point[i];
	// 			} 				
	// 			umap.Update((child),(parent));
	// 			return advanced;
	// 		}else{
	// 			std::cout<<" didn't reached goal in here iserting a node \n";
	// 			advanced = 1;
	// 			insert(root,Q_new);
	// 			for(int i=0;i<k;i++){
	// 				child[i] = Q_new[i];
	// 				parent[i] = Qnear->point[i];
	// 			} 				
	// 			umap.Update((child),(parent));
	// 			return advanced;
	// 		}
	// 	}
	// 	//}
	// }
	
}




std::vector< std::array<double,k>> RRTplanner(double* map,
int x_size,
int y_size,
double* armstart_anglesV_rad,
double* armgoal_anglesV_rad,
int numofDOFs,
double*** plan,
int* planlength)
{
	// max number of samples
	int samples = 100000;
	struct Node *root = NULL;
	// configuration space for the arms pockets
	// Q1 - [0 PI] and rest Q [0 2*PI]
	double Q_rand[numofDOFs]; 
	int advanced;

	for(int s =0;s<samples;s++)
	{
		// choosing sample randomly if  not the final plan
		if(s==0){
			startSample(numofDOFs,Q_rand,armstart_anglesV_rad); // inserting start node in tree
			root = insert(root,Q_rand);
		}
		else if(rand90() && (s+1)!=samples){
			randomSample(numofDOFs,Q_rand);	
		}else{
			goalSample(numofDOFs,Q_rand,armgoal_anglesV_rad);
		}
		std::cout<<"next random sample is \n";
		// for(int i =0;i<k;i++){
		// 	std::cout<<" Q"<<i<<" "<<Q_rand[i]<<"\n";
		// }
		advanced = Extend(root,Q_rand,E,numofDOFs,map,x_size,y_size,armgoal_anglesV_rad);
		if(advanced == 2){
			std::cout<<"reached goal\n";
			break;
		}else{
			std::cout<<"going for next point\n";
		}
	}
	// backtrack
	std::array<double,k> search,start;
	for(int i =0;i<k;i++){
		start[i]= armstart_anglesV_rad[i];
		search[i]=armgoal_anglesV_rad[i];
	}
	std::vector< std::array<double,k> > backtrack;

	int n = umap.GetN();
	std::cout<<" no of nodes in hash "<<n<<"\n";
	//search = *umap[search];
	// std::cout<<typeid(get).name()<<std::endl;
	// backtrack.push_back(get);

	//std::cout<<typeid(get).name()<<"\n";
	// for(int i =0;i<k;i++){
	// 	std::cout<<" next"<<i<<" "<<get[i]<<"\n";
	// 	search[i]=get[i];
	// 	std::cout<<" next search"<<i<<" "<<get[i]<<"\n";
	// }

	if(advanced ==2){
		for(int i=0;i< umap.GetN();i++){
			//std::cout<<"before pushback\n";
			backtrack.push_back(search);
			std::cout<<"after pushback\n";
			search = *umap[search];
			std::cout<<"after search \n";
			if(search == start){
				break;
			}
		}
		backtrack.push_back(search);
		int numofsamples = backtrack.size();
		std::cout<<" num of samples "<<numofsamples<<"\n";
	}


	// *plan = (double**) malloc(numofsamples*sizeof(double*));
	// for(int i=0;i<numofsamples;i++){
	// 	std::cout<<" next node \n";
	// 	(*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
	// 	for(int j=0;j<k;j++){
	// 		*plan[i][j] = search[j];
	// 		std::cout<<" Q"<<j<<" "<<search[j]<<"\n";
	// 	}
	// 	get = *umap[search];
	// 	if(search == start){
	// 		break;
	// 	}

	// }

	// *planlength = sizeof(plan[0]);
	// std::cout<<"planlength "<<planlength<<"\n";

	return backtrack;
	

}