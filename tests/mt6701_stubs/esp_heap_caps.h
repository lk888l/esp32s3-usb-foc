#pragma once
#include <cstddef>
#include <cstdint>
inline constexpr uint32_t MALLOC_CAP_DMA = 1U << 3, MALLOC_CAP_INTERNAL = 1U << 11;
void* heap_caps_aligned_alloc(size_t, size_t, uint32_t);
void heap_caps_free(void*);
