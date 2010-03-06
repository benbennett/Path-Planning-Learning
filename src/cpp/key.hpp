#ifndef KEY_HPP_ 
#define KEY_HPP_ 
#include <vector> 
#include <iterator>
#include <cmath>
#include <queue>
#include <functional>
#include <boost/unordered_set.hpp>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/functional/hash.hpp>
#include <iostream>
namespace  planning 
{

	template<typename Z, typename R> 
		class Key 
		{
			public:
				Key( shared_ptr < State < Z , R >  >  s,R  eps)
				{
					state_ = s;
					if( s->g()> s->rhs())
					{
						k1 = s->rhs()+eps*s->h();
						k2 = s->rhs();
					}
					else
					{
						k1 = s->g()+ s->h();
						k2 = s->g();
					}			
				}
				Key( Key<Z,R> const  & other)
				{
					state_ = other.state_;

					k1 = other.k1; 
					k2 = other.k2; 
				}

				Key( Key<Z,R>   &  other,R  eps)
				{
					state_ = other.state_;
					if( state_->g()> state_->rhs())
					{
						k1 = state_ ->rhs()+eps*(state_->h());
						k2 = state_ ->rhs();
					}
					else
					{
						k1 = state_->g()+ state_->h();
						k2 = state_->g();
					}			
				}

				Key( Key<Z,R>  const &  other,R  eps)
				{
					state_ = other.state_;

					if( state_->g()> state_->rhs())
					{
						k1 = state_ ->rhs()+eps*(state_->h());
						k2 = state_ ->rhs();
					}
					else
					{
						k1 = state_->g()+ state_->h();
						k2 = state_->g();
					}			
				}
				Key()
				{
					k1 = INF;
					k2 = INF;
				}
				Key(R k1_in, R k2_in)
				{
					k1=k1_in; 
					k2=k2_in;
				}
			public:
				R k1;
				R k2;
				shared_ptr < State < Z , R >  > state_;
			public:
				void setState( shared_ptr < State < Z , R >  > in)
				{
					state_ = in;
				}
				shared_ptr < State < Z , R >  >  getState()
				{
					return state_;
				}
				bool operator==(const Key &rhs) const
				{
					if(k1==rhs.k1 && k2==rhs.k2)
						return true;
					return false;
				}
				bool operator>=(const Key &rhs) const
				{
					if(*this>rhs || *this==rhs )
						return true;
					return false;
				}
				bool operator<=(const Key &rhs) const
				{
					if (*this<rhs || *this==rhs)
						return true;
					return false;
				}
				bool operator!=(const Key &rhs) const
				{
					if(*this == rhs)
						return false;
					return true;
				}
				bool operator<(const Key &rhs) const
				{
					if(k1< rhs.k1)
						return true;
					if(k1==rhs.k1 && k2<rhs.k2)
						return true;	
					return false;
				}
				bool operator>(const Key &rhs) const
				{
					if( *this== rhs)
						return false;
					return !(*this< rhs);	
				}
				Key &  operator= (const Key & rhs)
				{
					k1 = rhs.k1;
					k2 = rhs.k2;
					state_ = rhs.state_;
					return (*this);
				}
				friend std::ostream& operator << (std::ostream& os, const Key<Z,R>& in)
				{
					os<<in.k1 <<","<<in.k2 << " "<<*in.state_;
					return os;	
				}
		};

}
#endif
