#include "VirtualMemory.h"
#include "PhysicalMemory.h"

// _DS_ stans for Daniel Stav and is used as a unique identifier in case of name collisions
#define DS_NO_RESULT (-1)
#define DS_FAIL (0)
#define DS_OK (1)
#define DS_PAGE (1)
#define DS_TABLE (0)
#define DS_MIN(a, b) ((a)<(b)?(a):(b))
#define DS_MAX(a, b) ((a)<(b)?(b):(a))
#define DS_BITS_PER_SECTION ((uint64_t)((VIRTUAL_ADDRESS_WIDTH - OFFSET_WIDTH)/ TABLES_DEPTH))
#define DS_MASK_OF_SIZE(size) ((uint64_t)((1<<size)-1))
#define DS_OFFSET_MASK ((uint64_t)(PAGE_SIZE-1))
#define DS_EMPTY_VALUE 0

int get_status(uint64_t depth) {
    return depth < TABLES_DEPTH ? DS_TABLE : DS_PAGE;
}

/**
 * Extracts the offset from an address
 * @param virtual_address a full virtual address
 * @return the offset in the address
 */
uint64_t get_offset(uint64_t virtual_address) {
    return virtual_address & DS_OFFSET_MASK;
}

/**
 * Extracts the table address from the virtual address (Removes the offset).
 * @param virtual_address - The address the caller wants to translate into
 * table address.
 * @return The part of the address without the offset
 */
uint64_t get_path_address(uint64_t virtual_address) {
    return virtual_address >> OFFSET_WIDTH;
}

//uint64_t extract_binary(uint64_t number, uint64_t startIndex, uint64_t endIndex) {
//    uint64_t len = endIndex - startIndex;
//    uint64_t mask = MASK_OF_SIZE(len);
//    return (number >> startIndex) & mask;
//}
//
//uint64_t reverse_binary(uint64_t number, uint64_t size) {
//    uint64_t reversed_number = 0;
//    for (uint64_t i = 0; i < size; ++i) {
//        uint64_t bit = number & 1;
//        reversed_number = (reversed_number << 1) | bit;
//        number >>= 1;
//    }
//    return reversed_number;
//}

/**
 *
 * @param table_address
 * @param index
 * @return
 */
uint64_t get_sub_table_address(uint64_t table_address, uint64_t index) {
//    uint64_t size = VIRTUAL_ADDRESS_WIDTH - OFFSET_WIDTH;
//    uint64_t reversed = reverse_binary(table_address, size);
//    return reverse_binary(extract_binary(reversed, index * BITS_PER_SECTION, (index + 1) * BITS_PER_SECTION),size);
    uint64_t shift_amount = ((VIRTUAL_ADDRESS_WIDTH - OFFSET_WIDTH) - (index + 1) * DS_BITS_PER_SECTION);
    uint64_t subsection_mask = DS_MASK_OF_SIZE(DS_BITS_PER_SECTION) << shift_amount;
    return (table_address & subsection_mask) >> shift_amount;
}


/**
 * Verifies that a given frame is empty.
 * @param frame_index - The index of the frame the caller needs to verify.
 * @return True if the frame is empty, o.w false.
 */
bool is_frame_empty(uint64_t frame_index) {
    word_t buffer;
    for (uint64_t i = 0; i < PAGE_SIZE; i++) {
        PMread(frame_index * PAGE_SIZE + i, &buffer);
        if (buffer != 0)
            return false;
    }
    return true;
}

void remove_all_references_of_table(word_t table_index_to_remove, word_t frame_index = 0, word_t depth = 0) {
    if (get_status(depth) != DS_TABLE) return;
    word_t tmp = 0;
    for (int offset = 0; offset < PAGE_SIZE; ++offset) {
        PMread(frame_index * PAGE_SIZE + offset, &tmp);
        if (tmp != DS_EMPTY_VALUE) {
            remove_all_references_of_table(table_index_to_remove, tmp, depth + 1);
            if (tmp == table_index_to_remove)
                PMwrite(frame_index * PAGE_SIZE + offset, 0);
        }
    }
}


/**
 * Clears an entire frame by writing 0 into all of its memory slots.
 * @param frame_index - The index of the frame the callers wants to clear.
 */
void clear_frame(uint64_t frame_index) {
    for (uint64_t i = 0; i < PAGE_SIZE; i++) {
        PMwrite(frame_index * PAGE_SIZE + i, 0);
    }
}


uint64_t calculate_cyclical_distance(uint64_t x, uint64_t y) {
    return DS_MIN(NUM_PAGES - (x - y), (x - y));
}

/**
 * will put the address of the target to evict in winner_address
 * @param target the address which we want to evict someone for it
 * @param winner_address a pointer to where the answer will be put
 *
 * ========================= FOR INTERNAL USE IN THE RECURSION ======================
 * @param current_value [INTERNAL] the currently partly built address
 * @param current_frame_index [INTERNAL] the frame which we are checking now
 * @param depth [INTERNAL] the current depth of the recursion for the stop criteria
 * @param winner_distance [INTERNAL] a stack pointer which will be (safely) shared between all sub-calls to compare results
 */
void find_who_to_evict(uint64_t target,
                       word_t *winner_address,
                       word_t *winner_frame,
                       word_t current_value = 0,
                       word_t current_frame_index = 0,
                       uint64_t depth = 0,
                       uint64_t *winner_distance = nullptr) {
    //if this is the first call we need to 'own' winner_distance, so we create here a
    // dummy variable to be used as the base for this pointer.
    //This is safe because this is happening only in the very first level of the recursion and
    //will be destroyed naturally only when we got back to the caller for this function
    uint64_t reserved = 0;
    if (winner_distance == nullptr) winner_distance = &reserved;

    //stop criteria
    if (depth == TABLES_DEPTH) {
        uint64_t distance = calculate_cyclical_distance(target, current_value);
        if (*winner_distance < distance) {
            *winner_distance = distance;
            *winner_address = current_value;
            *winner_frame = current_frame_index;
        }
        return;
    }

    //recursion
    word_t tmp = 0;
    for (int offset = 0; offset < PAGE_SIZE; ++offset) {
        PMread(current_frame_index * PAGE_SIZE + offset, &tmp);
        if (tmp != DS_EMPTY_VALUE) {
            word_t new_value = (current_value << DS_BITS_PER_SECTION) + offset;
            find_who_to_evict(target,
                              winner_address,
                              winner_frame,
                              new_value,
                              tmp,
                              depth + 1,
                              winner_distance);
        }
    }
}


word_t _get_priority_1(uint64_t table_address, word_t frame_index = 0, word_t depth = 0,
                       uint64_t *current_path_ptr = nullptr) {
    if (get_status(depth) == DS_TABLE) {
        uint64_t current_path = 0;
        if (current_path_ptr == nullptr)current_path_ptr = &current_path;
        uint64_t real_path_with_respect_to_depth =
                table_address >> DS_BITS_PER_SECTION * (TABLES_DEPTH - depth);
        word_t tmp = 0;
        bool has_children = false;
        for (int offset = 0; offset < PAGE_SIZE; ++offset) {
            PMread(frame_index * PAGE_SIZE + offset, &tmp);
            if (tmp != DS_EMPTY_VALUE) {
                has_children = true;
                *current_path_ptr <<= DS_BITS_PER_SECTION;
                *current_path_ptr += offset;
                word_t child_res = _get_priority_1(table_address, tmp, depth + 1, current_path_ptr);
                *current_path_ptr -= offset;
                *current_path_ptr >>= DS_BITS_PER_SECTION;
                if (child_res != DS_NO_RESULT) return child_res;
            }
        }
        if (*current_path_ptr == real_path_with_respect_to_depth) {
            return DS_NO_RESULT;
        }
        if (!has_children) return frame_index == 0 ? 1 : frame_index;
    }
    return DS_NO_RESULT;
}

word_t _get_priority_2(word_t frame_index = 0, word_t depth = 0, word_t *max_reference_so_far_ptr = nullptr) {
    word_t max_referenced_so_far = 0;
    if (max_reference_so_far_ptr == nullptr) max_reference_so_far_ptr = &max_referenced_so_far;
    *max_reference_so_far_ptr = DS_MAX(*max_reference_so_far_ptr, frame_index);
    if (get_status(depth) == DS_TABLE) {
        word_t tmp = 0;
        for (int offset = 0; offset < PAGE_SIZE; ++offset) {
            PMread(frame_index * PAGE_SIZE + offset, &tmp);
            if (tmp != DS_EMPTY_VALUE)
                _get_priority_2(tmp, depth + 1, max_reference_so_far_ptr);
        }
        if (max_reference_so_far_ptr == &max_referenced_so_far)
            return *max_reference_so_far_ptr < NUM_FRAMES - 1 ? *max_reference_so_far_ptr + 1 : DS_NO_RESULT;
    }
    return DS_NO_RESULT;
}

word_t _get_priority_3(uint64_t address, uint64_t current_frame_index) {
    //values should be initialized otherwise it will cause problems inside the function call
    word_t evict_page_address = -1, evict_frame_index = -1;
    find_who_to_evict(address, &evict_page_address, &evict_frame_index);
    PMevict(evict_frame_index, evict_page_address);
    remove_all_references_of_table(evict_frame_index);
    return evict_frame_index;
}

/***
 * return the index of the next empty frame or -1
 * @param current_frame_index index of the starting point frame
 * @return index or -1 if there are no empty frames
 */
word_t evict_and_get_empty_frame(int current_frame_index = 0, uint64_t depth = 0) {
    if (is_frame_empty(current_frame_index)) {
        remove_all_references_of_table(current_frame_index);
        return current_frame_index;
    }
    word_t res = DS_NO_RESULT;
    if (depth == TABLES_DEPTH) {
        res = current_frame_index + 1;
    } else {
        word_t sub_frame_index = DS_NO_RESULT;
        for (int i = 0; i < PAGE_SIZE; ++i) {
            PMread(current_frame_index * PAGE_SIZE + i, &sub_frame_index);
            if (sub_frame_index != 0) {
                int sub_res = evict_and_get_empty_frame(sub_frame_index, depth + 1);
                if (sub_res < 0) return DS_NO_RESULT;
                else return sub_res;
            }
        }
        res = current_frame_index + 1;
    }
    return res >= NUM_FRAMES ? DS_NO_RESULT : res;
}

word_t handle_page_fault(uint64_t address, uint64_t current_frame_index) {
    // empty table?
    word_t res_priority_1 = _get_priority_1(address);
    if (res_priority_1 != DS_NO_RESULT) {
        remove_all_references_of_table(res_priority_1);
        return res_priority_1;
    }


//    last referenced
    word_t res_priority_2 = _get_priority_2();
    if (res_priority_2 != DS_NO_RESULT)
        return res_priority_2;

    //evict a page
    return _get_priority_3(address, current_frame_index);
}

uint64_t get_correct_frame_index(uint64_t table_address, bool read = false) {
    uint64_t current_frame_index = 0;
    word_t new_frame_index = 0;
    for (int depth = 0; depth < TABLES_DEPTH; ++depth) {
        uint64_t current_sub_index = get_sub_table_address(table_address, depth);
        PMread(current_frame_index * PAGE_SIZE + current_sub_index, &new_frame_index);
        if (new_frame_index == DS_EMPTY_VALUE) {
            new_frame_index = handle_page_fault(table_address, current_frame_index);
            if (depth < TABLES_DEPTH - 1)
                clear_frame(new_frame_index);
            PMwrite(current_frame_index * PAGE_SIZE + current_sub_index, new_frame_index);
        }
        current_frame_index = new_frame_index;
    }
    PMrestore(current_frame_index, table_address);
    return current_frame_index;
}

/**
 *
 * @param virtual_address
 * @return
 */
uint64_t virtual_to_physical(uint64_t virtual_address, bool read = false) {
    uint64_t offset = get_offset(virtual_address);
    uint64_t table_address = get_path_address(virtual_address);
    return get_correct_frame_index(table_address, read) * PAGE_SIZE + offset;
}


void VMinitialize() {
    clear_frame(0);
}

int VMread(uint64_t virtual_address, word_t *value) {
    if (virtual_address >= VIRTUAL_MEMORY_SIZE || value == nullptr) return DS_FAIL;
    uint64_t physical_address = virtual_to_physical(virtual_address, true);
    PMread(physical_address, value);
    return DS_OK;
}

int VMwrite(uint64_t virtual_address, word_t value) {
    if (virtual_address >= VIRTUAL_MEMORY_SIZE) return DS_FAIL;
    uint64_t physical_address = virtual_to_physical(virtual_address);
    PMwrite(physical_address, value);
    return DS_OK;
}

