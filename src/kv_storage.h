#ifndef KV_STORAGE_H_
#define KV_STORAGE_H_

//
// Created by jsmar on 12/06/2021.
//

#include <string.h>
#include <stdlib.h>
#include <unistd.h>

//**
// A simple key value storage utility using a Radix tree
// by Juan S. Marquerie
// TODO: Cleanup and dealloc
// */

inline int string_similarity(char* str1,
                             char* str2) {
    int count = 0;
    for(; *str1 == *str2 && *str1 != '\0'; str1++, str2++) {
        count++;
    }
    return count;
}


union uKVStorage {
    char str[30];
    int integer;
    float floating_point;
};

struct sRadNode {
    char        *key          = NULL;
    int        key_len        = 0;
    uKVStorage result;
    bool       has_result     = false;

    sRadNode *children[256];
    bool is_full[256]         = { false };
};

inline void RN_init(sRadNode *node) {
    memset(node->is_full, false, 256);
    node->key          = NULL;
    node->key_len        = 0;
    node->has_result     = false;
}

inline void RN_clean(sRadNode *node) {

    //TODO: cleaning up..
    for(int i = 0; i < 256; i++) {
        if (node->is_full[i]) {

        }
    }
}

inline bool RadNode_is_leaf(sRadNode *node) {
    return node->has_result;
}

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
    while(!RadNode_is_leaf(it_node)) {
        int similarity = string_similarity(res_key, it_node->key);

        if (similarity < it_node->key_len) { // No match
            return false;
        }

        res_key += similarity;
        if (!it_node->is_full[(int) *res_key]) { // Also no match
            return false;
        }

        // Step down to the next level
        it_node = it_node->children[(int) *res_key];
    }

    memcpy(to_retrieve, &it_node->result, sizeof(uKVStorage));
    return true;
}

inline void Rad_Node_add(sRadNode *node,
                         const char *key,
                         const int key_len,
                         uKVStorage *to_store) {
    int index = (int) *key;

    if (!node->is_full[index]) {
        sRadNode *new_child = (sRadNode*) malloc(sizeof(sRadNode));
        RN_init(new_child);
        new_child->key_len = key_len;
        new_child->key = (char*) malloc(key_len + 1);
        strcpy(new_child->key, key);
        memcpy(&new_child->result, to_store, sizeof(uKVStorage));
        new_child->has_result = true;

        node->is_full[index] = true;
        node->children[index] = new_child;

        return;
    }

    sRadNode *it_node = node->children[index];
    char *res_key = (char*) key;
    int res_key_len = key_len;

    while(true) {
        int old_index = (int) *it_node->key;
        int similarity = string_similarity(res_key, it_node->key);

        // NOTE: needs a more case, when the key is bigger than the it_node->key
        if (similarity == it_node->key_len) {
            // Reduce the key and teh key len, and iterate to the next node
            res_key += similarity;
            res_key_len -= similarity;
            it_node = it_node->children[(int) *res_key];
        } else {
            // Since the similarity is not equal, it creates a bifurcation on the tree,
            // storing the result

            res_key += similarity;
            res_key_len -= similarity;

            // Create a new leaf with the old root
            sRadNode *new_leaf_1 = (sRadNode*) malloc(sizeof(sRadNode));
            memcpy(new_leaf_1, it_node, sizeof(sRadNode));
            new_leaf_1->key = (char*) malloc(strlen(it_node->key) - similarity + 1);
            strcpy(new_leaf_1->key, similarity + it_node->key);
            new_leaf_1->key_len = strlen(new_leaf_1->key);

            // Create a new leaf with the result
            sRadNode *new_leaf_2 = (sRadNode*) malloc(sizeof(sRadNode));
            RN_init(new_leaf_2);
            new_leaf_2->key = (char*) malloc(sizeof(sizeof(char) * res_key_len + 1));
            strcpy(new_leaf_2->key, res_key);
            new_leaf_2->key_len = res_key_len;
            memcpy(&new_leaf_2->result, to_store, sizeof(uKVStorage));
            new_leaf_2->has_result = true;

            // Modify and clean the root
            memset(it_node->is_full, false, 256);
            memset(it_node->children, NULL, 256);

            char* tmp = (char*) malloc(similarity + 1);
            memcpy(tmp, it_node->key, similarity);
            tmp[similarity] = '\0';
            free(it_node->key);
            it_node->key = tmp;
            it_node->key_len = similarity;
            it_node->has_result = false;

            it_node->children[(int) *new_leaf_1->key] = new_leaf_1;
            it_node->is_full[(int) *new_leaf_1->key] = true;
            it_node->children[(int) *new_leaf_2->key] = new_leaf_2;
            it_node->is_full[(int) *new_leaf_2->key] = true;
            break;
        }

    }
}

struct sKVStorage {
    sRadNode  *root_node;
};

inline void KVS_init(sKVStorage *storage) {
    storage->root_node = (sRadNode*) malloc(sizeof(sRadNode));
    RN_init(storage->root_node);
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
