//
//  simple_hash_table.h
//  PlayerTracking
//
//  Created by jimmy on 7/21/15.
//  Copyright (c) 2015 Disney Reasearch. All rights reserved.
//

#ifndef __PlayerTracking__simple_hash_table__
#define __PlayerTracking__simple_hash_table__

#include <vcl_vector.h>
#include <assert.h>

// simple hash table using frame number as index
template <class Type>
class SimpleHashTable
{
private:
    vcl_vector<Type> data_;
    vcl_vector<long int> hashtable_;
public:
    // data_fn: frame number in data, it is a property of the data
    SimpleHashTable(const vcl_vector<int> & data_fn, const vcl_vector<Type> & data)
    {
        assert(data_fn.size() == data.size());
        for (int i = 0; i<data_fn.size()-1; i++) {
            assert(data_fn[i] < data_fn[i+1]);
        }
        
        hashtable_.resize(data_fn.back() + 1, -1);
        for (int i = 0; i<data_fn.size(); i++) {
            hashtable_[data_fn[i]] = i;    // fn as index in the table
        }
        data_ = data;
    }
    
    bool find(unsigned long fn, Type & ret)
    {
        if (fn >= hashtable_.size() ||
            hashtable_[fn] == -1) {
            return false;
        }
        ret = data_[hashtable_[fn]];
        return true;
    }
};

#endif /* defined(__PlayerTracking__simple_hash_table__) */
