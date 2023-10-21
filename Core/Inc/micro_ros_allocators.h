#ifndef MICRO_ROS_ALLOCATORS_
#define MICRO_ROS_ALLOCATORS_

void * microros_allocate(size_t size, void * state);

void microros_deallocate(void * pointer, void * state);

void * microros_reallocate(void * pointer, size_t size, void * state);

void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

#endif //MICRO_ROS_ALLOCATORS
