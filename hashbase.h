#ifndef HASHBASE_IS_INCLUDED
#define HASHBASE_IS_INCLUDED
/* { */

#include <vector>

class HashCommon
{
public:
	typedef unsigned long long CodeType;

	class Handle
	{
	public:
		long long int row,column;
		Handle()
		{
			row=-1;
			column=-1;
		}
		bool IsNotNull(void) const
		{
			if(0<=row && 0<=column)
			{
				return true;
			}
			return false;
		}
	};

	enum 
	{
		MINIMUM_TABLE_SIZE=7
	};

protected:
	long long int nElem;

public:
	inline HashCommon();
	inline bool IsNotNull(Handle hd) const;
	inline long long int GetN(void) const;

protected:
	template <class TableType>
	Handle First(const TableType &table) const;
	template <class TableType>
	Handle Next(const TableType &table,Handle hd) const;
	template <class TableType>
	void CleanUp(TableType &table);
	template <class EntryType>
	void Resize(std::vector <std::vector <EntryType> > &table,long long int tableSize);
	template <class EntryType,class KeyType>
	void Delete(std::vector <std::vector <EntryType> > &table,const KeyType &key,CodeType hashCode);
	template <class TableType,class KeyType>
	bool IsIncluded(const TableType &table,const KeyType &key,const CodeType hashCode) const;
	template <class TableType>
	void AutoResize(TableType &table);
};

template <class KeyType>
class HashBase
{
public:
	// This function must be specialized for a key type.
	HashCommon::CodeType HashCode(const KeyType &key) const;
};

inline HashCommon::HashCommon()
{
	nElem=0;
}
inline bool HashCommon::IsNotNull(Handle hd) const
{
	if(0<=hd.row && 0<=hd.column)
	{
		return true;
	}
	return false;
}
inline long long int HashCommon::GetN(void) const
{
	return nElem;
}

template <class TableType>
HashCommon::Handle HashCommon::First(const TableType &table) const
{
    Handle hd;
    hd.row=0;
    hd.column=0;
    while(hd.row<table.size())
    {
        if(0<table[hd.row].size())
        {
            return hd;
        }
        ++hd.row;
    }
    hd.row=-1;
    hd.column=-1;
    return hd;
}
template <class TableType>
HashCommon::Handle HashCommon::Next(const TableType &table,Handle hd) const
{
	if(true!=IsNotNull(hd))
	{
		return First(table);
	}

	++hd.column;
	if(hd.column<table[hd.row].size())
	{
		return hd;
	}

	hd.column=0;
	++hd.row;
	while(hd.row<table.size())
	{
		if(0<table[hd.row].size())
		{
			return hd;
		}
		++hd.row;
	}

	hd.row=-1;
	hd.column=-1;
	return hd;
}
template <class TableType>
void HashCommon::CleanUp(TableType &table)
{
	table.resize(MINIMUM_TABLE_SIZE);
	for(auto &t : table)
	{
		t.clear();
	}
	nElem=0;
}
template <class EntryType>
void HashCommon::Resize(std::vector <std::vector <EntryType> > &table,long long int tableSize)
{
	std::vector <EntryType> buffer;
	for(auto &t : table)
	{
		for(auto &e : t)
		{
			buffer.push_back((EntryType &&)e);
		}
		t.clear();
	}
	table.resize(tableSize);
	for(auto &b : buffer)
	{
		auto r=b.hashCode%table.size();
		table[r].push_back((EntryType &&)b);
	}
}
template <class EntryType,class KeyType>
void HashCommon::Delete(std::vector <std::vector <EntryType> > &table,const KeyType &key,CodeType hashCode)
{
	auto idx=hashCode%table.size();
	for(auto &e : table[idx])
	{
		if(e.hashCode==hashCode && e.hashKey==key)
		{
			std::swap(e,table[idx].back());
			table[idx].pop_back();
			--nElem;
			AutoResize(table);
			break;
		}
	}
}
template <class TableType,class KeyType>
bool HashCommon::IsIncluded(const TableType &table,const KeyType &key,const CodeType hashCode) const
{
	auto idx=hashCode%table.size();
	for(auto e : table[idx])
	{
		if(e.hashCode==hashCode && e.hashKey==key)
		{
			return true;
		}
	}
	return false;
}
template <class TableType>
void HashCommon::AutoResize(TableType &table)
{
	//Fill this function as a part of the assignment.
	if(table.size()<= nElem/10){
		int new_size = 2*nElem/10;
		//printf(" table size reached %d and resized to %d  \n", table.size(),new_size);
		Resize(table,new_size);

	}
	//for(auto e : table){
	//	if(table[e].size() > 20){
	//		int new_size = table.size() + 10;
	//		//printf(" table size reached %d and resized to %d  \n", table.size(),new_size);
	//		Resize(table,new_size);
	//
	//	}
	//}	

}

/* } */
#endif
