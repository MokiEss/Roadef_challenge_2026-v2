/*
 * Software Name : networktools
 * SPDX-FileCopyrightText: Copyright (c) Orange SA
 * SPDX-License-Identifier: MIT
 *
 * This software is distributed under the MIT licence,
 * see the "LICENSE" file for more details or https://opensource.org/license/MIT
 *
 * Authors: see CONTRIBUTORS.md
 * Software description: An efficient C++ library for modeling and solving network optimization problems
 */

/**
 * @file
 * @brief Various set implementations
 *
 * @author Morgan Chopin (morgan.chopin@orange.com)
 */

#ifndef _NT_SETS_H_
#define _NT_SETS_H_

#include "../@deps/sparsehash/dense_hash_set"

namespace nt {

  /**
   * @class NumericalSet
   * @headerfile sets.h
   * @brief Set for storing numerical values. It is a simple wrapper of the google::dense_hash_set
   * class.
   *
   * The `NumericalSet` class provides a set data structure for storing numerical values. It is a
   * wrapper around the `google::dense_hash_set` class from the `sparsehash-c11` library. For more
   * information about the `google::dense_hash_set` class, refer to the documentation at
   * https://github.com/sparsehash/sparsehash-c11/tree/master/docs.
   *
   * Example of usage:
   *
   * @code
   *   int main() {
   *     // Create a NumericalSet object to store integers
   *     NumericalSet<int> numSet;
   *
   *     // Insert values into the set
   *     numSet.insert(5);
   *     numSet.insert(10);
   *     numSet.insert(3);
   *     numSet.insert(7);
   *
   *     // Check if a value exists in the set
   *     bool containsValue = numSet.contains(10);
   *     if (containsValue) {
   *       std::cout << "The set contains the value 10." << std::endl;
   *     } else {
   *       std::cout << "The set does not contain the value 10." << std::endl;
   *     }
   *
   *     // Convert the set to an array
   *     TrivialDynamicArray<int> array = numSet.toArray();
   *
   *     // Print the values in the array
   *     std::cout << "Values in the array: ";
   *     for (int i = 0; i < array.size(); ++i) {
   *       std::cout << array[i] << " ";
   *     }
   *     std::cout << std::endl;
   *
   *     // Populate the set from an array
   *     TrivialDynamicArray<int> newArray = {2, 4, 6, 8};
   *     numSet.fromArray(newArray);
   *
   *     // Iterate over the set and print the values
   *     std::cout << "Values in the set after populating from the array: ";
   *     for (const auto& value : numSet) {
   *       std::cout << value << " ";
   *     }
   *     std::cout << std::endl;
   *
   *     return 0;
   *   }
   * @endcode
   *
   * @tparam TYPE The type of numerical values to be stored in the set.
   */
  template < class TYPE >
  struct NumericalSet: public google::dense_hash_set< TYPE > {
    using value_type = TYPE;

    static_assert(std::is_arithmetic< value_type >::value, "NumericalSet : Non-numerical type");

    using Parent = google::dense_hash_set< value_type >;
    using iterator = typename Parent::iterator;
    using const_iterator = typename Parent::const_iterator;

    /**
     * @brief Default constructor.
     *
     * Constructs a NumericalSet object.
     * Initializes the google::dense_hash_set with the maximum possible value of TYPE as the
     * empty key.
     */
    NumericalSet() {
      NumericalSet& m = *this;
      // google::dense_hash_set requires the definition of a value to identify
      // free spaces. Here, we use the maximum possible value of value_type.
      m.set_empty_key(std::numeric_limits< value_type >::max());
    }

    /**
     * @brief Check if the set contains a specific value.
     *
     * @param Value The value to check for existence in the set.
     * @return true if the value is found in the set, false otherwise.
     */
    bool contains(const value_type& v) const {
      const NumericalSet& m = *this;
      return m.find(v) != m.end();
    }

    /**
     * @brief Convert the set to a TrivialDynamicArray.
     *
     * @return TrivialDynamicArray<value_type> An array containing all the values in the set.
     */
    TrivialDynamicArray< value_type > toArray() const {
      const NumericalSet&               m = *this;
      TrivialDynamicArray< value_type > array(m.size());
      int                               i = 0;
      for (const value_type& v: m)
        array[i++] = v;
      return array;
    }

    /**
     * @brief Populate the set from an array.
     *
     * Clears the set and inserts all the values from the provided array into the set.
     *
     * @param array The array containing the values to be inserted into the set.
     */
    void fromArray(const ArrayView< value_type > array) {
      NumericalSet& m = *this;
      m.clear();
      for (int k = 0; k < array.size(); ++k)
        m.insert(array[k]);
    }
  };

}   // namespace nt

#endif
