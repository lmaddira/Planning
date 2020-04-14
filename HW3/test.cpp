#include <vector>
#include <string>
#include <iostream>
#include <unordered_set>
#include <list>

using namespace std;

void combinations(int n,std::vector<std::vector<string>> &combo,int i,int index,std::vector<string> &data,std::vector<string> &symbols)
{
    if(index==n)
    {
        combo.push_back(data);
        for( auto k : data )
        {
            cout<< k <<" ";
        }
        std::cout<<std::endl;
        return;
    }
    if(i>=symbols.size()) return;
    //cout<<"I'm here \n";
    data[index] = symbols[i];
    //cout<<" index "<<index << " i "<< i;
    combinations(n,combo,i+1,index+1,data,symbols);
    //cout<<"going for next \n";
    //cout<<" index "<<index << " i "<< i;
    combinations(n,combo,i+1,index,data,symbols);
}
void all_combinations(vector<string> &one_combo,int i,int n,vector<vector<string>> &all_combo)
{
    int j;
    if( i == n) 
    {
        all_combo.push_back(one_combo);
        for(auto k : one_combo)
        {
            cout<< k<<" ";
        }
        cout<<endl;
    }
    else{
        for(j=i;j< n;j++)
        {
            swap(one_combo[i],one_combo[j]);
            all_combinations(one_combo,i+1,n,all_combo);
            swap(one_combo[i],one_combo[j]);
        }
    }
        
}

int main()
{
    std::vector<string> symbols;
    symbols.push_back("A");
    symbols.push_back("B");
    symbols.push_back("C");
    symbols.push_back("Table");
    std::vector<std::vector<string>> combo;
    int n = 3;
    std::vector<string> data(n);
    combinations(n,combo,0,0,data,symbols);
    vector<vector<string>> all_combo;
    for(auto each_combo : combo)
    {
        all_combinations(each_combo,0,each_combo.size(),all_combo);
    }
    unordered_set<string> cond;
    cond.insert("A");
    cond.insert("B");
    cond.insert("C");
    cond.insert("Table");
    for(auto i : cond)
    {
        cout<<i<<" ";
    }
    cout<<endl;
    list<string> sym;
    sym.push_back("AA");
    sym.push_back("BB");
    sym.push_back("CC");
    sym.push_back("Tablee");
    for(auto i : sym)
    {
        cout<<i<<" ";
    }


}