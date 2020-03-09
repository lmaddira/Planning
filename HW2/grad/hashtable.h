#ifndef HASHTABLE_IS_INCLUDED
#define HASHTABLE_IS_INCLUDED
/* { */

#include <vector>
#include <stdio.h>

#include "hashbase.h"

template <class KeyType,class ValueType>
class HashTable : public HashBase <KeyType>,public HashCommon
{
private:
	class Entry
	{
	public:
		KeyType hashKey;
		CodeType hashCode;
		ValueType value;
	};

	std::vector <std::vector <Entry> > table;

public:
	using HashCommon::Handle;
	using HashCommon::CodeType;
	using HashBase<KeyType>::HashCode;

	HashTable();
	~HashTable();
	void CleanUp(void);

	void Update(const KeyType &key,const ValueType &value);
	bool IsIncluded(const KeyType &key) const;
	void Resize(long long int tableSize);
	void Delete(const KeyType &key);

	ValueType *operator[](const KeyType key);
	const ValueType *operator[](const KeyType key) const;

	Handle First(void) const;
	Handle Next(Handle hd) const;
	const KeyType *operator[](Handle hd) const;
};

template <class KeyType,class ValueType>
HashTable<KeyType,ValueType>::HashTable()
{
	table.resize(MINIMUM_TABLE_SIZE);
}
template <class KeyType,class ValueType>
HashTable<KeyType,ValueType>::~HashTable()
{
}
template <class KeyType,class ValueType>
void HashTable<KeyType,ValueType>::CleanUp(void)
{
	HashCommon::CleanUp(table);
}
template <class KeyType,class ValueType>
void HashTable<KeyType,ValueType>::Update(const KeyType &key,const ValueType &value)
{
	auto hashCode=HashCode(key);
	auto idx=hashCode%table.size();
	for(auto &e : table[idx])
	{
		if(e.hashCode==hashCode && e.hashKey==key)
		{
			e.value=value;
			return;
		}
	}
	Entry entry;
	entry.hashKey=key;
	entry.hashCode=hashCode;
	entry.value=value;
	table[idx].push_back(entry);
	++nElem;
	AutoResize(table);
}
template <class KeyType,class ValueType>
bool HashTable<KeyType,ValueType>::IsIncluded(const KeyType &key) const
{
	return HashCommon::IsIncluded(table,key,HashCode(key));
}
template <class KeyType,class ValueType>
void HashTable<KeyType,ValueType>::Resize(long long int tableSize)
{
	HashCommon::Resize(table,tableSize);
}
template <class KeyType,class ValueType>
void HashTable<KeyType,ValueType>::Delete(const KeyType &key)
{
	HashCommon::Delete(table,key,HashCode(key));
}

template <class KeyType,class ValueType>
ValueType *HashTable<KeyType,ValueType>::operator[](const KeyType key)
{
	auto hashCode=HashCode(key);
	auto idx=hashCode%table.size();
	for(auto &e : table[idx])
	{
		if(e.hashCode==hashCode && e.hashKey==key)
		{
			return &e.value;
		}
	}
	return nullptr;
}

template <class KeyType,class ValueType>
const ValueType *HashTable<KeyType,ValueType>::operator[](const KeyType key) const
{
	auto hashCode=HashCode(key);
	auto idx=hashCode%table.size();
	for(auto &e : table[idx])
	{
		if(e.hashCode==hashCode && e.hashKey==key)
		{
			return &e.value;
		}
	}
	return nullptr;
}

template <class KeyType,class ValueType>
HashCommon::Handle HashTable<KeyType,ValueType>::Next(Handle hd) const
{
	return HashCommon::Next(table,hd);
}

template <class KeyType,class ValueType>
const KeyType *HashTable<KeyType,ValueType>::operator[](Handle hd) const
{
	return &table[hd.row][hd.column].hashKey;
}

template <class KeyType,class ValueType>
HashCommon::Handle HashTable<KeyType,ValueType>::First(void) const
{
	return HashCommon::First(table);
}

/* } */
#endif
