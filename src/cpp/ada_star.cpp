#include "ada_star.hpp"
namespace planning{

		template <typename Z , typename R>state<Z,R>::state()
		{
		}
		/*   
		 *
		 */
		template <typename Z , typename R>void state<Z,R>::init(tuple pos, const tuple & goal )
		{
		}
		/*
		 * Going to be used in the hash function for map methods 
		 */
		template <typename Z , typename R>long state<Z,R>::hash()
		{
		}
		template <typename Z , typename R>state<Z,R> state<Z,R>::get_min_successor()
		{ 
			state<Z,R> mr;	
			return mr;
		}
		template <typename Z , typename R>void state<Z,R>::remove_successor(state const & s) 
		{
		}

		template <typename Z , typename R>void state<Z,R>::rhs()
		{
		}

		template <typename Z , typename R>void state<Z,R>::set_rhs(R)
		{
		}

		template <typename Z , typename R>void state<Z,R>::csprime_gsprime(state const & sprime )
		{
		}

		template <typename Z , typename R> R state<Z,R>::cost(state & sprime)
		{
		}

		template <typename Z , typename R>R state<Z,R>::g()
		{
		}

		template <typename Z , typename R>void state<Z,R>::set_g(R g )
		{
		}

		template <typename Z , typename R>R  state<Z,R>::h()
		{
		}

		template <typename Z , typename R>bool state<Z,R>::isGoal()
		{
		}

		template <typename Z , typename R>
			std::vector< state<Z,R> > StateTransSpace<Z,R>::StateTranFunc(state<Z,R> at, std::vector< Z [2] >)
		{

		}
		template <typename Z , typename R>StateTransSpace<Z,R>::StateTransSpace()
		{

		}

		template< typename Z, typename R> bool Key<Z,R>::operator==(const Key &rhs)
		{
			if(k1==rhs->k1 && k2==rhs->k2)
				return true;
			return false;
		}
		template< typename Z, typename R> bool Key<Z,R>::operator>=(const Key &rhs)
		{
			if(this>rhs || this==rhs)
				true;
			return false;
		}
		template< typename Z, typename R> bool Key<Z,R>::operator<=(const Key &rhs)
		{
			if (this < rhs || this=rhs)
				return true;
			return false;
		}
		template< typename Z, typename R> bool Key<Z,R>::operator!=(const Key &rhs)
		{
			if(this==rhs)
				return false;
			return true;
		}
		template< typename Z, typename R> bool Key<Z,R>::operator>(const Key &rhs)
		{	
			return !(this<rhs);
		}
		template< typename Z, typename R> bool Key<Z,R>::operator<(const Key &rhs)
		{
			if(k1< rhs->k1)
				return true;
			if(k1==rhs->k1 && k1<rhs->k2)
			   return true;	
			return false;
		}

}
