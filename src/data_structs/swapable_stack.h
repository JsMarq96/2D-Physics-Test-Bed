#ifndef _SWAPABLE_STACK_H_
#define _SWAPABLE_STACK_H_

#include <cstring>
#include "../math.h"

#define STACKS_NUM 2

/**
 * Swapable stack
 * A data strcuture made for swappgin between two stacks
 * Example: you need to modify a array, but mainting a copy
 *          for iterating, and on top of that you need to iterate
 *          the produced array.
 * This data structure will clean the resulting code and use the least amount
 * of memmory allocations
 * By Juan S. Marquerie
 * */


struct sSwapableVector3Stacks {
  sVector3  *stack1 = NULL;
  sVector3   *stack2 = NULL;

  sVector3 *stacks_list[STACKS_NUM] = { NULL };

  size_t stacks_sizes[STACKS_NUM] = { 0 };

  int swap_index = 0;
  size_t stack_max_size = 0;

  // =================
  // LIFECYCLE FUNCTIONS
  // ================
  void init(const size_t  istack_size) {
    stack1 = (sVector3*) malloc(sizeof(sVector3) * istack_size);
    stack2 = (sVector3*) malloc(sizeof(sVector3) * istack_size);

    stack_max_size = istack_size;

    stacks_list[0] = stack1;
    stacks_list[1] = stack2;
  }

  void clean() {
    free(stack1);
    free(stack2);
  }

  void swap() {
    swap_index = (swap_index == 1) ? 0 : 1;
  }

  // ============
  // STACK FUNCTIONS
  // =========== 
  inline bool is_current_stack_full() const {
    return stacks_sizes[swap_index] == stack_max_size;
  }

  inline sVector3 get_element_from_current_stack(const int index) const {
    return stacks_list[swap_index][index];
  }

  inline void add_element_to_current_stack(const sVector3 &value) {
    stacks_list[swap_index][ stacks_sizes[swap_index] ] =  value;
    stacks_sizes[swap_index]++;
  }

  void clean_current_stack() {
    stacks_sizes[swap_index] = 0;
    memset(stacks_list[swap_index], 0, stack_max_size * sizeof(sVector3));
  }

  sVector3* get_current_stack() {
    return  stacks_list[swap_index];
  }

  inline int get_current_stacks_size() const {
    return stacks_sizes[swap_index];
  }

};

#endif
