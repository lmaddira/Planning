#include <stdio.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <map>
#include <math.h>
#include <limits>
#include <iostream>
#include <algorithm>
#include <queue>
#include <ctime>
#include "hashset.h"
#include "hashbase.h"
#include "hashtable.h"

class Object{
    
public:
    double g; 
    double h;
    double f;
    std::pair <int,int>  Node;

    Object(
    double g, 
    double h,
    double f,
    std::pair <int,int> Node){

    this->f = f;
    this->g = g;
    this->h = h; 
    this->Node = Node;
    }
    double get_f() const {return f;}
    double get_g() const {return g;}
    double get_h() const {return h;}
    std::pair <int,int> get_Node() const{return Node;} 
};
class myComparator { 
public: 
    int operator() (const Object& p1, const Object& p2) 
    { 
        return p1.get_f() > p2.get_f(); 
    } 
}; 
std::pair <int, int > Make_Node(int X,int Y){
    return std::make_pair(X,Y);
}



template <>
unsigned long long int HashBase<std::pair<int,int>>::HashCode(const std::pair<int,int> &key) const
{

    unsigned long long int hash = 0;
    hash = (key.first *31) + (key.second *23) ;

    return hash;

}

class Binary_search{
public:
    int robotposeX;
    int robotposeY;
    double* trajectory;
    int target_steps;
    int start_index;
    int End_index;
    int action_size;
    int curr_time;
    // Binary_search(){
    //     this->robotposeX = 0;
    //     this->robotposeY = 0;
    //     this->trajectory = NULL;
    //     this->target_steps = 0;
    //     this->start_index = 0;
    //     this->End_index = 0;
    //     this->action_size = 0;
    //     this->curr_time = 0;

    // }
    Binary_search(int robotposeX,
    int robotposeY,
    double* trajectory,
    int target_steps,
    int start_index,
    int End_index){
    this->robotposeX = robotposeX;
    this->robotposeY = robotposeY;
    this->trajectory = trajectory;
    this->target_steps = target_steps;
    this->start_index = start_index;
    this->End_index = End_index;
    this->action_size = action_size;
    this->curr_time = curr_time;
    }
    std::pair<int, int> Get_Mid_target_node(){
        int mid = round(this->start_index+this->End_index)/2;
        int mid_x = this->trajectory[mid];
        int mid_y = this->trajectory[mid+target_steps];
        return std::make_pair(mid_x,mid_y);
    }
    int Get_Mid_target_index(){
        int mid = round(this->start_index+this->End_index)/2;
        return mid;
    }


    bool If_Robotactions_morethan_targetactions(){
        if(this->action_size >= (this->Get_Mid_target_index()-curr_time)){
            return true;
        }
        return false;
    }

    std::pair<int,int>  Next_start_and_End_indices(){

        if(this->If_Robotactions_morethan_targetactions()){
            int new_start = this->Get_Mid_target_index();
            int new_end = this->End_index;
            return std::make_pair(new_start,new_end);
        }else{
            int new_start = this->start_index;
            int new_end = this->Get_Mid_target_index();
            return std::make_pair(new_start,new_end);
        }
    }
 
};

// class object2{
// public:
//     //int g_val;
//     std::pair <int, int> Node;
//     object2(std::pair <int, int> Node){
//         //this->g_val = g_val;
//         this->Node = Node;
//     }
//     int get_X() const{return this->Node.first;}
//     int get_Y() const{return this->Node.second;}
//     bool operator== (const object2& n1){
//         if (n1.get_X() == this->get_X() && n1.get_X() == this->get_Y()){
//             return true;
//         }
        
//         return false;
//     }
// };     
std::vector<std::pair<int,int>> do_Astar(
    double* map,
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
    int start_index,end_index,mid_index;
){

    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    int goalposeX,goalposeY;
    HashTable <std::pair<int,int>,double> CLOSED_table;
    std::vector<Object> OPEN;
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
    return action;
}
