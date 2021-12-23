#ifndef KV_STORAGE_H_
#define KV_STORAGE_H_

//
// Created by jsmar on 12/06/2021.
//

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <cassert>

//**
// A simple key value storage utility using a Radix tree
// by Juan S. Marquerie
// TODO: Cleanup and dealloc
// */

inline int string_similarity(char* str1,
                             unsigned int str1_len,
                             char* str2,
                             unsigned int str2_len) {
    int count = 0;
    unsigned int min_len = MIN(str1_len, str2_len);
    for(; min_len > count ; str1++, str2++) {
        if (*str1 == *str2) {
            count++;
        } else {
            break;
        }
    }
    return count;
}


union uKVStorage {
    char str[30];
    int integer;
    float floating_point;
};

struct sRadNode {
    char       key[32]          = "";
    int        key_len          = 0;
    uKVStorage result;
    bool       has_result     = false;

    sRadNode *children[256];
    bool is_full[256]         = { false };
};

inline void RN_init(sRadNode *node) {
    memset(node->is_full, false, 256);
    node->key_len        = 0;
    node->has_result     = false;
}

inline void RN_clean(sRadNode *node) {
    free(node->key);
    for(int i = 0; i < 256; i++) {
        if (node->is_full[i]) {
            RN_clean(node->children[i]);
            free(node->children[i]);
        }
    }
}

inline bool RadNode_is_leaf(sRadNode *node) {
    return node->has_result;
}
#include <iostream>
inline bool Rad_Node_get(sRadNode *node,
                        const char *key,
                        const int key_len,
                        uKVStorage *to_retrieve) {
    int index = (int) *key;

    // Early stop
    if (!node->is_full[index]) {
        return false;
    }

    sRadNode *it_node = node->children[index];

    // Traverse the tree until a leaf is found
    char *res_key = (char*) key;
    int res_key_len = key_len;
    while(true) {
        int similarity = string_similarity(it_node->key, it_node->key_len, res_key, res_key_len);
        std::cout << it_node->key << " "<<similarity<<"  " << res_key << std::endl;

        // Found the node with the equal key
        if (similarity == res_key_len) {
            break;
        }

        res_key += similarity;
        res_key_len -= similarity;

        // In there is no match for the keys, or the next iteration is empty, early exit
        if (similarity < it_node->key_len || !it_node->is_full[*res_key]) {
            return false;
        }

        it_node = it_node->children[*res_key];
    }

    memcpy(to_retrieve, &it_node->result, sizeof(uKVStorage));
    return true;
}

// TODO: Malloc on here might be screwing oerformance on the future <- bottleneck ??
inline void Rad_Node_add(sRadNode *node,
                         const char *key,
                         const int key_len,
                         uKVStorage *to_store) {
    // Comparar con el nodo actual
    // Si no esta vacio
    // Si la longitud es menos que el nodo -> Division
    // Si no esta vacio, iterar a nuevo nodo actual

    // Create the new node to insert
    sRadNode *new_node = (sRadNode*) malloc(sizeof(sRadNode));
    memcpy(&new_node->result, to_store, sizeof(uKVStorage));
    new_node->has_result = true;


    sRadNode* it_node = node;
    char it_key[32] = "";
    memcpy(it_key, key, key_len * sizeof(char));
    int it_key_len = key_len;

    while(it_node != NULL) {
        int curr_key = *it_key;

        if (!it_node->is_full[curr_key]) {
            // Add the new node on empty space;
            memcpy(new_node->key, it_key, sizeof(char) * it_key_len);
            new_node->key_len = it_key_len;

            it_node->children[curr_key] = new_node;
            it_node->is_full[curr_key] = true;
            break;
        }
        
        unsigned int similarity = string_similarity(it_key, it_key_len, it_node->key, it_node->key_len);


        if (similarity < it_key_len) {
            // create a branch
            sRadNode *old_root = (sRadNode*) malloc(sizeof(sRadNode));

            old_root->key_len = it_node->key_len - similarity;
            memcpy(old_root->key, it_node->key + similarity, old_root->key_len);
            memcpy(old_root->children, it_node->children, sizeof(sRadNode::children));
            memcpy(old_root->is_full, it_node->is_full, sizeof(sRadNode::is_full));
            memcpy(&old_root->result, &it_node->result, sizeof(uKVStorage));
            old_root->has_result = it_node->has_result;

            it_node->key_len = similarity;
            memcpy(it_node->key, it_key, similarity);
            memset(it_node->is_full, false, sizeof(sRadNode::is_full));
            memset(it_node->children, 0, sizeof(sRadNode::is_full)); // This isnt really needed
            it_node->has_result = false;

            it_node->is_full[*old_root->key] = true;
            it_node->children[*old_root->key] = old_root;

            memcpy(new_node->key, it_key + similarity, it_key_len - similarity);
            new_node->key_len = it_key_len;

            it_node->children[*new_node->key] = new_node;
            it_node->is_full[*(new_node->key)] = true;
            break;
        }

        // Crop the key and iterate to the next node
        // Copy cropped key
        memcpy(it_key, it_key + similarity, (sizeof(char) * 32) - similarity);
        it_key_len -= similarity;
        it_node = it_node->children[*it_key];
    }
}

struct sKVStorage {
    sRadNode  *root_node;
};

inline void KVS_init(sKVStorage *storage) {
    storage->root_node = (sRadNode*) malloc(sizeof(sRadNode));
    RN_init(storage->root_node);
}

inline void KVS_clean(sKVStorage *storage) {
    RN_clean(storage->root_node);
}

inline void KVS_discard(sKVStorage *storage) {

}

inline void KVS_add(sKVStorage *kv_storage, const char *key,
                    const int key_len,
                    const int to_store) {

    uKVStorage result{};
    result.integer = to_store;
    Rad_Node_add(kv_storage->root_node,
                 key,
                 key_len,
                 &result);
}

inline void KVS_add(sKVStorage *kv_storage, const char *key,
                    const int key_len,
                    const float to_store) {
    uKVStorage result{};
    result.floating_point = to_store;
    Rad_Node_add(kv_storage->root_node,
                 key,
                 key_len,
                 &result);
}

inline void KVS_add(sKVStorage *kv_storage, const char *key,
                    const int key_len,
                    const char to_store[30]) {
    uKVStorage result{};
    // This is kinda yuck yuck, but since both of them are only allowed to go out of 30 chars...
    // should be fine right..? (no, but will fix it later)
    strcpy(&result.str[0], &to_store[0]);
    Rad_Node_add(kv_storage->root_node,
                 key,
                 key_len,
                 &result);
}

inline int KVS_get_int(sKVStorage *kv_storage,
                   const char* key,
                   const int key_len) {
    uKVStorage result;
    bool success = Rad_Node_get(kv_storage->root_node,
                                key,
                                key_len,
                                &result);

    return (success) ? result.integer : -1;
}

inline float KVS_get_float(sKVStorage *kv_storage,
                     const char* key,
                     const int key_len) {
    uKVStorage result;
    bool success = Rad_Node_get(kv_storage->root_node,
                                key,
                                key_len,
                                &result);

    return (success) ? result.floating_point : -1;
}

#endif // KV_STORAGE_H_
