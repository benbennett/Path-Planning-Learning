#ifndef PRIORITY_DICT_HPP_
#define PRIORITY_DICT_HPP_
#include <vector> 
#include <iterator>
#include <list>
#include <cmath>
#include <queue>
#include <functional>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/functional/hash.hpp>
#include <iostream>
namespace  planning 
{
    template <typename Z, class T >
    class priority_dict
    {
    private:
        priority_queue< T , vector< T >,greater< T >  > queue_;
        std::map< Z , T  >  dict_; 
    public:
      T top()
      {
        T mr;
        bool cont = true;
        while (cont && !queue_.empty())
        {
            mr = queue_.top();
            if(dict_.find(mr.getState()->getPoint())!=dict_.end())
            {
              T rhs = dict_[mr.getState()->getPoint()];
              if(rhs!=mr)
              {
                queue_.pop();
              }
              else
              {
                cont=false;
              }
            }
            else
            {
                queue_.pop();
            }
        }
        //happens when we have pop to much 
        // and the queue should have really been empty.
        if(queue_.empty())
        {
            std::cerr<<"Pop from an empty queue."<<endl;
            throw 100;
        }
        return mr;
      }
      void pop()
      {
        T mr = top();
        if(!dict_.empty())
        {
            dict_.erase(mr.getState()->getPoint());
            //don't really need pop , but here for clarity.
            queue_.pop();
        }
      }
      bool empty()
      {
        return dict_.empty();
      }
      void push( T in )
      {
            dict_[in.getState()->getPoint()] = in;
            queue_.push(in);
      }
      void remove(Z in)
      {
          dict_.erase(in);
      }
      bool contains(Z in)
      {
        return (dict_.find(in)!=dict_.end());
      }

    };


}
#endif
