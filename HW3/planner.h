#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <map>
#include <set>
#include <vector>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6
#define unordered_set_of_GC unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> 


class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;
class State;

using namespace std;

bool print_status = true;

class GroundedCondition
{
private:
    string predicate;
    list<string> arg_values;
    bool truth = true;

public:
    GroundedCondition(string predicate, list<string> arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth;  // fixed
        for (string l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition& gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth;  // fixed
        for (string l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const GroundedCondition& pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition& rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth()) // fixed
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        temp += this->predicate;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition& lhs, const GroundedCondition& rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition& gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

class Condition
{
private:
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(string pred, list<string> args, bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (string ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    list<string> get_args() const
    {
        return this->args;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const Condition& cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition& rhs) const // fixed
    {

        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (string l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ConditionComparator
{
    bool operator()(const Condition& lhs, const Condition& rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition& cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

class Action
{
private:
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(string name, list<string> args,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& preconditions,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& effects)
    {
        this->name = name;
        for (string l : args)
        {
            this->args.push_back(l);
        }
        for (Condition pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (Condition pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action& rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream& operator<<(ostream& os, const Action& ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (Condition precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (Condition effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->get_name();
        temp += "(";
        for (string l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action& lhs, const Action& rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action& ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};
class GroundedAction
{
private:
    string name;
    list<string> arg_values;
    // unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> gPreconditions;
    // unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> gEffects;

public:
    GroundedAction(string name, list<string> arg_values)
    {
        this->name = name;
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    void update_gc(string name, list<string> arg_values)
    {
        this->name = name;
        this->arg_values.clear();
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }

    }
    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool operator==(const GroundedAction& rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream& operator<<(ostream& os, const GroundedAction& gac)
    {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->name;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

class State 
{
public:
    unordered_set_of_GC state_conditions; // current state conditions
    int state_id;
    int g = INT8_MAX;
    int h;
    int f;
    unordered_set_of_GC prev_conditions;
    vector<GroundedAction> prev_action;// action resulting this state --update this for lowest g value 
    State(unordered_set_of_GC &state_conditions)
    {
        for (GroundedCondition cond : state_conditions)
        {
            this->state_conditions.insert(cond);
        }
       // this->state_id = state_id;
        return;
    } 
    int get_stateid()
    {
        return this->state_id;
    }
    unordered_set_of_GC get_state_conditions()
    {
        return this->state_conditions;
    }
    void update_prev_action(GroundedAction &gaction)
    {
        prev_action.clear();
        prev_action.push_back(gaction);
    }
    void update_prev_conditions(unordered_set_of_GC &prev_conditions)
    {
        for (GroundedCondition cond : prev_conditions)
        {
            this->prev_conditions.insert(cond);
        }
       // this->state_id = state_id;
        return;

    }
    void update_state_conditions(unordered_set_of_GC &state_conditions)
    {
        this->state_conditions.clear();
        for (GroundedCondition cond : state_conditions)
        {
            this->state_conditions.insert(cond);
        }

    }
    GroundedAction get_prev_action()
    {
        return prev_action[0];
    }
    bool operator==(const State& rhs) const
    {
        unordered_set_of_GC rhs_cond = rhs.state_conditions; 
        for (auto gcond : this->state_conditions)
        {
            // cout<<"checking"<<gcond<<endl;
            if(rhs_cond.find(gcond)==rhs_cond.end())
            {
                // cout<<" not found "<<endl;
                return false;
            }
                
        }
        return true;
    }
    string toString() const
    {
        string temp =" ";
        for(auto cond : this->state_conditions)
        {
            temp+=cond.toString();
        }
        return temp;
    }
    void print()
    {
        for(auto c : this->state_conditions)
        {
            cout<< c <<" ";
        }
        cout<<endl;
        return;
    }

};
struct State_hash
{
    std::size_t operator()(const State& s) const
    {

        return hash<string>{}(s.toString());

    }
};
struct State_comparator
{
    bool operator()(const State& lhs, const State& rhs) const
    {
        return lhs == rhs;
    }
};


class Env
{
private:
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;

public:
    void remove_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(string symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(list<string> symbols)
    {
        for (string l : symbols)
            this->symbols.insert(l);
    }
    void add_action(Action action)
    {
        this->actions.insert(action);
    }
    unordered_set_of_GC get_initial_conditions()
    {
        return this->initial_conditions;
    }
    unordered_set_of_GC get_goal_conditions()
    {
        return this->goal_conditions;
    }
    Action get_action(string name)
    {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }
    // recursive function to get unique combo
    void combinations(int n,vector<vector<string>> &combo_subet,int i,int index,vector<string> &data,vector<string> &sym)
    { 
        if(index==n)
        {
            combo_subet.push_back(data);
            // cout<<"combinations ";
            // for(auto k : data)
            // {
            //     cout<< k <<" ";
            // }
            // cout<<endl;
            return;
        }
        if(i>=sym.size()) return;

        //cout<<" index"<<index<< " i "<<i <<"\n";
        data[index] = sym[i];
        // current data update
        combinations(n,combo_subet,i+1,index+1,data,sym);
        // going for next data
        // cout<<"going for next \n";
        // cout<<" index "<<index<< " i "<<i <<"\n";
        combinations(n,combo_subet,i+1,index,data,sym);
    }

    // permutations of each unique combo
    void all_combinations(vector<string> &one_combo,int i,int n,vector<vector<string>> &combo)
    {
        int j;
        if( i == n) 
        {
            combo.push_back(one_combo);
            // for(auto k : one_combo)
            // {
            //     cout<< k<<" ";
            // }
            // cout<<endl;
        }
        else{
            for(j=i;j< n;j++)
            {
                swap(one_combo[i],one_combo[j]);
                all_combinations(one_combo,i+1,n,combo);
                // cout<<"going for next \n";
                // cout<<" i "<< i << " j "<< j;
                swap(one_combo[i],one_combo[j]);
            }
        }
            
    }

    bool valid_actions(State &state,unordered_map<string,string> mapping_args,Action &ac)
    {
        
        auto conditions =  state.get_state_conditions(); // grounded condition
        // if(ac.get_name() == "Fill_Water_at")
        // {
        //     cout<<"checking for valid action"<< ac.get_name();
        //     cout<<"state conditions ";
        //     state.print();
        // }
        for(Condition precon  : ac.get_preconditions())
        {
            string pred = precon.get_predicate();
            list<string> arg_val = precon.get_args();
            list<string> arg_check;
            //cout<<" condition name"<< pred ;
            while(arg_val.size()!=0)
            {
                auto temp = arg_val.front();
                if(mapping_args.find(temp) != mapping_args.end())
                {
                    // if(ac.get_name() == "Fill_Water_at")
                    //     cout<<" "<< temp<<" mapping "<< mapping_args.find(temp)->second <<"\n";
                    arg_check.push_back(mapping_args.find(temp)->second);
                }else
                {
                   arg_check.push_back(temp); 
                }               
                arg_val.pop_front();
            }
            //cout<<"checking for condition ";
            GroundedCondition precondition(pred,arg_check,true);
            //cout<< precondition<<"\n";
            // if(ac.get_name() == "Fill_Water_at")
            // {
            //     cout<<"checking for condition ";
            //     cout<< precondition<<"\n";
            // }
            if(conditions.find(precondition) == conditions.end())
                return false;   
        }
        return true;
    }
    vector<GroundedAction>  valid_set_actions(State &state,Action &ac)
    {
        vector<GroundedAction> action_vec; 
        int n = ac.get_args().size();// number of symbols involved
        //cout<<" n val "<<n<<endl;
        vector<vector<string>> combo_subet;
        vector<vector<string>> combo;
        vector<string> data(n);
        vector<string> sym;
        sym.reserve(this->symbols.size());
        for(auto s: symbols)
        {
            sym.push_back(s);
            //cout<<" "<<s<<" "<<sym.size();
        }
        combinations(n,combo_subet,0,0,data,sym);// all combinations updated for current args
        for (auto one_combo : combo_subet)
        {
            all_combinations(one_combo,0,one_combo.size(),combo);
        }
        //cout<<" all combinations size "<<combo.size()<<endl;
        unordered_map<string,string> mapping_args;
        for (int i=0; i<combo.size();i++)
        {
            list<string> args_ac = ac.get_args();
            mapping_args.clear();
            // we already made sure that the args arg_check nd args_ac are same size
            int j=0;
            for(auto it = args_ac.begin(); it != args_ac.end(); ++it)
            {
                mapping_args[*it] = combo[i][j];
                // cout<<" map before valid sets"<< *it << " "<<combo[i][j] << endl;
                j++;
            }
            list<string> arg_check;
            for(auto d : combo[i] )
            {
                arg_check.push_back(d); // creating list of args to check with action preconditions
            }
            // if valid action then store the action and args // so use grounded action 
            if(valid_actions(state,mapping_args,ac))
            {
                // cout<<"valid transition for the args with action name "<<ac.get_name()<<"( ";
                // for(auto arg : arg_check)
                // {
                //     cout<< arg <<",";
                // }
                // cout<<" )"<<endl;
                GroundedAction gaction(ac.get_name(),arg_check);
                action_vec.push_back(gaction);
            }
            else{
                // cout<<"not vaid\n";
                // if(ac.get_name() == "Fill_Water_at")
                // {
                //     cout<<"not valid action"<<ac.get_name()<<"( ";
                //     for(auto arg : arg_check)
                //     {
                //         cout<< arg <<",";
                //     }
                //     cout<<" )"<<endl;
                // } 
            }
        }
              
        return action_vec;

    }
    State next_state(State &state, GroundedAction &action)
    {
        string action_name = action.get_name();
        list<string> args = action.get_arg_values();
        // we already know that this action is valid for the state, so not rechecking again
        // just finding the next state by imposing the effects and making new state out of it
        vector<string> args_vec;
        for(auto arg : args) args_vec.push_back(arg);
        Action env_action = this->get_action(action_name);
        list<string> env_action_args = env_action.get_args();
        int i = 0;
        unordered_map<string, string> mapping_args;
        for(auto it = env_action_args.begin(); it != env_action_args.end(); ++it)
        {
            mapping_args[*it] = args_vec[i];
            //cout<<" map in next-state "<< *it << " "<<args_vec[i] << endl;
            i++;
        }
        // if(action_name == "MoveToTable")// for move to table Table need to be there in args
        // {
        //mapping_args["Table"] = "Table";       
        // }
        unordered_set_of_GC conditions = state.get_state_conditions();
        unordered_set<Condition, ConditionHasher, ConditionComparator> env_effects = env_action.get_effects();
        for(Condition cond : env_effects)
        {
            //cout<<"next cond "<<cond<<"\n";
            auto predicate_env = cond.get_predicate();
            list<string> cond_args = cond.get_args();
            list<string> mapped_args;
            
            while(cond_args.size()!=0)
            {
                if(mapping_args.find(cond_args.front()) != mapping_args.end())
                    mapped_args.push_back(mapping_args.find(cond_args.front())->second);
                else
                    mapped_args.push_back(cond_args.front());
                
                //cout<<" args "<< mapping_args.find(cond_args.front())->second<<" ";
                cond_args.pop_front();
            } 
            if (cond.get_truth() == false)
            {
                //cout<<" clearing conditions \n";
                conditions.erase(GroundedCondition(predicate_env,mapped_args));
                //cout<<"done \n";
            }
            else
            {
                //cout<<"adding conditions \n";
                // if(predicate_env == "Clear" && mapped_args.size() == 1 && mapped_args.front()=="Table")
                // {

                // }else{
                conditions.insert(GroundedCondition(predicate_env, mapped_args));
                // }
            }
        }
        // cout<<" for action "<<action<<"\n";
        // cout<<" next state conditions ";
        // for(auto g : conditions)
        // {
        //     cout<<g<<" ";
        // } 
        // cout<<"\n";
        //state_count++;
        State succ(conditions);
        succ.update_prev_action(action);
        return succ;    
    }
    vector<State> get_successors(State &state)
    {
        vector<State> Succ;
        for( Action ac : this->actions)
        {
            vector<GroundedAction>  gactions  = valid_set_actions(state,ac);
            // cout<<" got the g actions now getting succ \n";
            for( GroundedAction each_action : gactions)
            {   
                Succ.push_back(next_state(state,each_action));
            }
            if(ac.get_name() == "Extinguish_first_time")
                cout<<" for action name "<<ac.get_name()<<" no of incre succ states "<<Succ.size()<<endl;
        }
        return Succ;
    }

    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }
    int heuristics(unordered_set_of_GC state_conditions);

    friend ostream& operator<<(ostream& os, const Env& w)
    {
        os << "***** Environment *****" << endl << endl;
        os << "Symbols: ";
        for (string s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (GroundedCondition s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (GroundedCondition g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (Action g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }
};


list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}


Env* create_env(char* filename)
{
    ifstream input_file(filename);
    Env* env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line == "")
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str()));  // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}
 // for now consider goal condition is passed to it every time
 // figure out a way of doing it without giving goal 

int Env::heuristics(unordered_set_of_GC state_conditions)
{
    // goal conditions may not have all the required conditions and the size may be small
    int h = 0;
    for(auto cond : this->goal_conditions)
    {
        // if the goal condition is not true then increase the value
        if(state_conditions.find(cond)== state_conditions.end())
            h++;
    }
    return h;
}