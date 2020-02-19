#ifndef HASHSET_IS_INCLUDED
#define HASHSET_IS_INCLUDED
/* { */

#include <vector>
#include <stdio.h>

#include "hashbase.h"



////////////////////////////////////////////////////////////

template <class KeyType>
class HashSet : public HashBase <KeyType>,public HashCommon
{
private:
	class Entry
	{
	public:
		KeyType hashKey;
		CodeType hashCode;
	};

	std::vector <std::vector <Entry> > table;

public:
	using HashCommon::Handle;
	using HashCommon::CodeType;
	using HashBase<KeyType>::HashCode;

	HashSet();
	~HashSet();
	void CleanUp(void);

	void Add(const KeyType &key);
	bool IsIncluded(const KeyType &key) const;
	void SetSize(long long int tableSize);
	void Resize(long long int tableSize);
	void Delete(const KeyType &key);

	Handle First(void) const;
	Handle Next(Handle hd) const;
	const KeyType *operator[](Handle hd) const;
};

template <class KeyType>
HashSet<KeyType>::HashSet()
{
	table.resize(MINIMUM_TABLE_SIZE);
}
template <class KeyType>
HashSet<KeyType>::~HashSet()
{
}
template <class KeyType>
void HashSet<KeyType>::CleanUp(void)
{
	CleanUp(table);
}
template <class KeyType>
void HashSet<KeyType>::Add(const KeyType &key)
{
	auto hashCode=HashCode(key);
	auto idx=hashCode%table.size();
	for(auto e : table[idx])
	{
		if(e.hashCode==hashCode && e.hashKey==key)
		{
			return;
		}
	}
	Entry entry;
	entry.hashKey=key;
	entry.hashCode=hashCode;
	table[idx].push_back(entry);
	++nElem;
	AutoResize(table);
}
template <class KeyType>
bool HashSet<KeyType>::IsIncluded(const KeyType &key) const
{
	return HashCommon::IsIncluded(table,key,HashCode(key));
}
template <class KeyType>
void HashSet<KeyType>::Resize(long long int tableSize)
{
	HashCommon::Resize(table,tableSize);
}
template <class KeyType>
void HashSet<KeyType>::Delete(const KeyType &key)
{
	HashCommon::Delete(table,key,HashCode(key));
}

template <class KeyType>
HashCommon::Handle HashSet<KeyType>::First(void) const
{
	return HashCommon::First(table);
}

template <class KeyType>
HashCommon::Handle HashSet<KeyType>::Next(Handle hd) const
{
	return HashCommon::Next(table,hd);
}

template <class KeyType>
const KeyType *HashSet<KeyType>::operator[](Handle hd) const
{
	return &table[hd.row][hd.column].hashKey;
}

/* } */
#endif
