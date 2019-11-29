#ifndef HASH_TABLE_H
#define HASH_TABLE_H
#include <light_arena.h>

typedef struct {
	unsigned long long hash;
	char* key;
	char  data[0];
} HTE;

// Entry of index 0 is invalid
typedef struct {
	unsigned long long  entry_capacity;
	unsigned long long  entry_count;
	unsigned long long  data_size_bytes;
	HTE* entries;
	Light_Arena* arena;
} Hash_Table;

Hash_Table hash_table_create(unsigned long long capacity, unsigned long long data_size_bytes);
void*      hash_table_get(Hash_Table* table, const char* key);
void*      hash_table_insert(Hash_Table* table, const char* key, void* data);
void       hash_table_remove(Hash_Table* table, const char* key);
void       hash_table_free(Hash_Table* table);
HTE*       hash_table_get_raw(Hash_Table* table, unsigned long long index);
unsigned long long        hash_table_get_index(Hash_Table* table, const char* key);

#if defined(HASH_TABLE_IMPLEMENT)
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#define KEY_ARENA_SIZE (512 * 1024)

static unsigned long long 
fnv_1_hash_str(const char* s) {
	unsigned long long hash = 14695981039346656037llu;
	unsigned long long fnv_prime = 1099511628211llu;
	for (unsigned long long i = 0; s[i] != 0; ++i) {
		hash = hash * fnv_prime;
		hash = hash ^ s[i];
	}
	return hash;
}

HTE*
hash_table_get_raw(Hash_Table* table, unsigned long long index) {
	HTE* ptr = (HTE*)((char*)table->entries + index * (sizeof(HTE) + table->data_size_bytes));
	return ptr;
}

unsigned long long
hash_table_get_index(Hash_Table* table, const char* key) {
	unsigned long long hash = fnv_1_hash_str(key);
	unsigned long long index = hash % table->entry_capacity;

	HTE* entry = hash_table_get_raw(table, index);

	while (entry->hash != 0) {
		if (entry->hash == hash) {
			// hash collision
			if (strcmp(entry->key, key) == 0) {
				// entry is already in the hash table
				return index;
			}
		}
		++index;
		if (index == table->entry_capacity)
			index = 0;
		entry = hash_table_get_raw(table, index);
	}

	return 0;
}

void*
hash_table_get(Hash_Table* table, const char* key) {
	unsigned long long hash = fnv_1_hash_str(key);
	unsigned long long index = hash_table_get_index(table, key);
	HTE* ptr = (HTE*)((char*)table->entries + index * (sizeof(HTE) + table->data_size_bytes));
	if (ptr->hash != 0) {
		return ptr->data;
	}
	return 0;
}

Hash_Table 
hash_table_create(unsigned long long capacity, unsigned long long data_size_bytes) {
	Hash_Table result;
	result.entry_capacity = capacity;
	result.entry_count = 0;
	result.data_size_bytes = data_size_bytes;
	result.entries = (HTE*)calloc(capacity, sizeof(HTE) + data_size_bytes);
	result.entries[0].hash = (unsigned long long)0;		// reserved entry
	result.arena = arena_create(KEY_ARENA_SIZE);

	return result;
}

// insert
void*
hash_table_insert(Hash_Table* table, const char* _key, void* data) {
	char* key = (char*)arena_alloc(table->arena, sizeof(char) * (strlen(_key) + 1));
	strcpy(key, _key);

	if (table->entry_count == table->entry_capacity / 4) {
		// grow table perhaps
		return 0;
	}
	unsigned long long hash = fnv_1_hash_str(key);
	unsigned long long index = hash % table->entry_capacity;
	unsigned long long index_cpy = index;

	HTE* entry = hash_table_get_raw(table, index);

	while (entry->hash != 0) {
		if (entry->hash == hash) {
			// hash collision
			if (strcmp(entry->key, key) == 0) {
				// entry is already in the hash table
				return entry->data;
			}
		}

		++index;
		if (index == table->entry_capacity)
			index = 0;

		// if index == index_cpy, hash_table is full!
		assert(index != index_cpy);

		entry = hash_table_get_raw(table, index);
	}

	entry->hash = hash;
	entry->key = key;
	memcpy(&entry->data, data, table->data_size_bytes);
	table->entry_count++;
	return entry->data;
}

// remove
void 
hash_table_remove(Hash_Table* table, const char* key) {
	unsigned long long index = hash_table_get_index(table, key);
	HTE* entry = hash_table_get_raw(table, index);
	entry->hash = 0;
}

void
hash_table_free(Hash_Table* table) {
	arena_free(table->arena);
	free(table->entries);
}

#endif
#endif