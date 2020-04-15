#include "planner.h"
static int state_count = 0;
using namespace std;
inline int f_val(State &state,int weight) //updates and returns f value of state
{
    state.f = state.g + weight* state.h;
    return state.f;
}
void update_f(State &state,int weight) //updates  f value of state
{
    state.f = state.g + weight* state.h;
    return;
}
class myComparator { 
public: 
    //int operator() (const Object& p1, const Object& p2)
    bool operator() (const State& s1, const State& s2) const
    { 
        return s1.f > s2.f; 
    } 
};

bool if_included(vector<State> &CLOSED,State &state)
{
    for(auto s : CLOSED){
        auto lhs = s.get_state_conditions();
        auto rhs = state.get_state_conditions();
        if( lhs == rhs)
            return true; 
    }
    return false;
}
State state_in_closed(vector<State> &CLOSED,State &state)
{
    for(auto s : CLOSED){
        auto lhs = s.get_state_conditions();
        auto rhs = state.get_state_conditions();
        if(lhs == rhs)
            return s; 
    }
    cout<<"check state existance \n";
    throw;
}
void remove_state(vector<State> &CLOSED,State best)
{
    int i;
    for(i =0;i<CLOSED.size();i++)
    {
        if(CLOSED[i] == best)
        break;
    }
    if(i!=CLOSED.size())
        CLOSED.erase(CLOSED.begin()+i);
    else
        cout<<"check state existance \n";   
        
    return;
}
bool action_from_to(State &from_state,State &to_state,Env* env,GroundedAction &temp_action)
{
    auto successors = env->get_successors(from_state);
    cout<<"got successors \n";
    for(auto s : successors)
    {
        if(s == to_state)
        {
            cout<<"found the action_from _to \n";
            auto temp = s.get_prev_action();
            temp_action.update_gc(temp.get_name(),temp.get_arg_values()); 
            cout<<" action from to "<<temp_action<<endl;
            return true;
        } 
    }
    return false;
}


list<GroundedAction> planner(Env* env)
{
    // this is where you insert your planner
    auto IC = env->get_initial_conditions();
    State start_state(IC);// without any prev_action
    //unordered_set<State,State_hash,State_comparator> CLOSED;// map of state_id , state.g val
    vector<State> CLOSED; 
    vector<State> OPEN;
    OPEN.clear();
    CLOSED.clear();
    int weight = 1; 
    start_state.g = 0;
    start_state.h = env->heuristics(start_state.get_state_conditions());
    update_f(start_state,weight);
    OPEN.push_back(start_state);
    int i =0;
    auto FC = env->get_goal_conditions();
    State goal_state(FC);//intiate it as goal

    while(OPEN.size() !=0)
    {
        i++;
        std::make_heap(OPEN.begin(),OPEN.end(),myComparator());
        std::push_heap(OPEN.begin(),OPEN.end(),myComparator());// maintaing the order of heap
        State min_state = OPEN.front();
        cout<<"min state is ";
        min_state.print();
        auto min_g_val = min_state.g;
        std::pop_heap(OPEN.begin(),OPEN.end(),myComparator());
        OPEN.pop_back();
        cout<<"size of open "<<OPEN.size()<<endl;
        
        if(min_state.h == 0)
        {
            auto gaction = min_state.get_prev_action();
            auto gcond = min_state.get_state_conditions();
            goal_state.update_state_conditions(gcond);
            goal_state.update_prev_action(gaction);
            goal_state.update_prev_conditions(min_state.prev_conditions);
            break;
        }  // this means reached goal
        CLOSED.push_back(min_state);
        auto successors = env->get_successors(min_state); 
        std::cout<<"no of successors"<<successors.size()<<std::endl;
        for(auto succ_state: successors)
        {
            cout<<"now looking at ";
            succ_state.print();
            if(!if_included(CLOSED,succ_state))// if not in closed already
            {
                if(succ_state.g > min_state.g + 1)
                {
                    succ_state.g = min_state.g + 1; // each transision is of same cost 
                    auto temp = min_state.get_state_conditions();
                    succ_state.h = env->heuristics(succ_state.get_state_conditions());
                    auto f_value = f_val(succ_state,weight);
                    succ_state.update_prev_conditions(temp);
                }
                int count =0;
                //cout<<"open size "<<OPEN.size()<<endl;
                for(int i =0;i<OPEN.size();i++){
                    // cout<<"open checking ";
                    // OPEN[i].print();
                    // cout<<" with ";
                    // succ_state.print();
                    if(OPEN[i] == succ_state)// rhs if has more cond then still considered equal
                    {
                        if(OPEN[i].g > succ_state.g)
                        {
                            OPEN[i].g = succ_state.g;
                            OPEN[i].f = succ_state.f;
                        }
                        count++;
                        break;
                    }
                }
                if(count == 0)
                {
                    OPEN.push_back(succ_state);
                }
            }else{
                cout<<"in closed already \n";
                State Cstate = state_in_closed(CLOSED,succ_state);
                if(Cstate.g > min_state.g + 1)
                {
                    remove_state(CLOSED,Cstate);
                    succ_state.g = min_state.g + 1; // each transision is of same cost 
                    auto temp = min_state.get_state_conditions();
                    succ_state.h = env->heuristics(succ_state.get_state_conditions());
                    auto f_value = f_val(succ_state,weight);
                    succ_state.update_prev_conditions(temp);
                    CLOSED.push_back(succ_state);
                }

            }
            
        }
        
    }
    // cout<<" printing states in open \n";
    // for(auto state : OPEN){
    //     state.print();
    //     cout<< "f val of state "<< state.f<<" g val "<<state.g<<" h val "<< state.h <<endl;
    // }
    // cout<<" found goal state \n";
    // cout<<"printing CLOSED states \n";
    // for(auto state : CLOSED){
    //     state.print();
    //     cout<< "f val of state "<< state.f<<" g val "<<state.g<<" h val "<< state.h <<endl;
    // }
    // blocks world example
    cout<<"size of OPEN outside the open loop "<<OPEN.size()<<"\n";
    list<GroundedAction> actions;
    auto track = goal_state.get_state_conditions();
    auto best_ac = goal_state.get_prev_action();
    // int best_h_val;
    State best(track);
    best.update_prev_action(best_ac);
    cout<<"start_state ";
    //goal_state.print();
    i =0;
    cout<<"At start closed size "<<CLOSED.size()<<endl;

    bool bbacktrack = true;
    if(bbacktrack)
    {
        
        State from_state(goal_state.prev_conditions);
        State to_state(goal_state.state_conditions);
        auto goal_action = goal_state.get_prev_action();
        GroundedAction temp_action(goal_action.get_name(),goal_action.get_arg_values());
        int k=1;
        while(k>0)
        {
            k++;
            if(action_from_to(from_state,to_state,env,temp_action))
            {
                actions.push_front(temp_action);
                // cout<<"action found "<<temp_action<<endl;
                if(if_included(CLOSED,from_state)){
                    to_state = state_in_closed(CLOSED,from_state);
                    cout<<"to state ";
                    to_state.print();
                    if(to_state.state_conditions == start_state.state_conditions)
                    {
                        cout<<"reached start \n";
                        return actions;
                    } 
                    // if(to_state.prev_conditions == NULL)
                    // {
                    //     cout<<"something worng ... check\n";
                    // }
                    from_state.update_state_conditions(to_state.prev_conditions);
                    // OPEN.erase(OPEN.begin()+i);
                    cout<<"from state ";
                    from_state.print();
                    
                    
                }else{
                    cout<<"check that the from state is not included in closed \n";
                }
            }else{
                cout<<"something is wrong please check \n";
            }
        }
        
    }else{
        while(i<10)//(CLOSED.size()>0)
        {
            i++;
            //actions.push_back(best.get_prev_action());
            int check = INT8_MAX;
            cout<<" now best is ";
            best.print();
            if(best == start_state) break;
            auto successors = env->get_successors(best);
            std::cout<<"no of successors"<<successors.size()<<std::endl;
            for(auto state : successors)
            {
                state.print();
                auto temp_action  = state.get_prev_action(); 
                //cout<< temp_action;
                if(if_included(CLOSED,state)){
                    cout<< "found in closed ";
                    auto cstate = state_in_closed(CLOSED,state);
                    if(check > cstate.g)
                    {   
                        cout<<"I'm here \n";
                        best.h = cstate.h;
                        auto btemp_ac = action_from_to(state,best,env,temp_action); 
                        cout<< btemp_ac<<"\n";
                        if(btemp_ac)
                        {
                            auto temp_cond = cstate.get_state_conditions();
                            best.update_state_conditions(temp_cond);
                            best.update_prev_action(temp_action);
                            best.print();
                        }
                        check = cstate.g;
                        
                    }
                    if(check == cstate.g)
                    {
                        if (best.h > cstate.h)
                        {
                            best.h = cstate.h;
                            auto btemp_ac = action_from_to(state,best,env,temp_action); 
                            if(btemp_ac)
                            {
                                auto temp_cond = cstate.get_state_conditions();
                                best.update_state_conditions(temp_cond);
                                best.update_prev_action(temp_action);
                                best.print();
                            }
                        } 
                    }
                }else{
                    cout<<"didn't find in closed \n";
                }
            }
            cout<<"best action "<<best.get_prev_action();
            remove_state(CLOSED,best);
            cout<<" after erase closed size "<<CLOSED.size()<<endl;      
            actions.push_front(best.get_prev_action());   
        }
    }
    // for(auto states : CLOSED){
    //     //auto temp =  states.first;
    //     states.print();
    //     //cout<<"it's g val"<<states.second<<endl;;

    // }
    // actions.push_back(GroundedAction("MoveToTable", { "A", "B" }));
    // actions.push_back(GroundedAction("Move", { "C", "Table", "A" }));
    // actions.push_back(GroundedAction("Move", { "B", "Table", "C" }));

    return actions;
}

int main(int argc, char* argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char* filename = (char*)("example.txt");
    if (argc > 1)
        filename = argv[1];

    cout << "Environment: " << filename << endl << endl;
    Env* env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    for (GroundedAction gac : actions)
    {
        cout << gac << endl;
    }

    return 0;
}