
/*
Copyright (c) 2010, Benjamin Bennett 
All rights reserved.
(BSD License)
Redistribution and use in source and binary forms, with or without modification, 
permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, 
this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution.
* The names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef KEY_HPP_ 
#define KEY_HPP_ 
#include <vector> 
#include <iterator>
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
