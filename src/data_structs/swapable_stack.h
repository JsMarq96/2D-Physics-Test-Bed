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

struct sSwapableStack {
  char  *stack1 = NULL;
  char  *stack2 = NULL;

  char *stacks_list[STACKS_NUM] = { NULL };

  size_t stacks_sizes[STACKS_NUM] = { 0 };

  int swap_index = 0;
  size_t element_size = 0;
  size_t stack_max_size = 0;

  // =================
  // LIFECYCLE FUNCTIONS
  // ================
  void init(const size_t  ielement_size, 
            const size_t  istack_size) {
    stack1 = (char*) malloc(ielement_size * istack_size);
    stack2 = (char*) malloc(ielement_size * istack_size);

    element_size = ielement_size;
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

  inline void* get_element_from_current_stack(const int index) const {
    return (void*) &stacks_list[swap_index][index * element_size];
  }

  inline void add_element_to_current_stack(const void *value) {
    memcpy(&stacks_list[swap_index][ stacks_sizes[swap_index] * element_size ], value, element_size);
  }

  void clean_current_stack() {
    memset(stacks_list[swap_index], 0, stack_max_size * element_size);
  }

  void *get_current_stack() {
    return (void*) stacks_list[swap_index];
  }

};

#endif
