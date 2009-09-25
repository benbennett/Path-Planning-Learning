#include "ada_star.hpp"
namespace planning{

		template <typename Z , typename R>State<Z,R>::State()
		{
		}
		/*   
		 *
		 */
		template <typename Z , typename R>void State<Z,R>::init(tuple pos, const tuple & goal)
		{
		}
		/*
		 * Going to be used in the hash function for map methods 
		 */
		template <typename Z , typename R>long State<Z,R>::hash()
		{
		}
		template <typename Z , typename R>State<Z,R> State<Z,R>::getMinSuccessor()
		{ 
			State<Z,R> mr;	
			return mr;
		}
		template <typename Z , typename R>void State<Z,R>::removeSuccessor(State const & s) 
		{
		}

		template <typename Z , typename R>void State<Z,R>::rhs()
		{
		}

		template <typename Z , typename R>void State<Z,R>::setRhs(R)
		{
		}

		template <typename Z , typename R>void State<Z,R>::csprimeGsprime(State const & sprime)
		{
		}

		template <typename Z , typename R> R State<Z,R>::cost(State & sprime)
		{
		}

		template <typename Z , typename R>R State<Z,R>::g()
		{
		}

		template <typename Z , typename R>void State<Z,R>::setG(R g )
		{
		}

		template <typename Z , typename R>R  State<Z,R>::h()
		{
		}

		template <typename Z , typename R>bool State<Z,R>::isGoal()
		{
		}

		template <typename Z , typename R>
			std::vector< State<Z,R> > StateTransSpace<Z,R>::StateTranFunc(State<Z,R> at, std::vector< Z [2] >)
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
