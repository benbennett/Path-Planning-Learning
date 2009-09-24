/*
   Simple implementation of
   "Anytime Dynamic A*: An Anytime, Replanning Algorithm" by
   Maxim Likhachev , Dave Ferguson , Geoff Gordon , Anthony Stentz , and Sebastian Thrun

   Warning this may be incorrect, mainly put together to learn from there paper.
   See paper at: 
http://www.ri.cmu.edu/pub_files/pub4/likhachev_maxim_2005_1/likhachev_maxim_2005_1.pdf
*/

#include <vector> 
namespace  planning 
{
	template <typename Z , typename R>
		class state
		{
			public: 
				typedef std::vector<Z> tuple;	
			private:
				tuple point;
				/*tuple goal;
				R rhs_value;
				R gofs;
				*/
			public:
				state();
				/*   
				 *
				 */
				void init(tuple pos, const tuple & goal );
				
				/*
				 * Going to be used in the hash function for map methods 
				 */
				long hash();

				state get_min_successor();

				void remove_successor(state const & s); 

				void rhs();

				void set_rhs(R);

				void csprime_gsprime(state const & sprime );

				R cost(state & sprime);

				R g();

				void set_g(R g );

				R  h();

				bool isGoal();
				//TODO over load ostream operator and == operator

		};

}
