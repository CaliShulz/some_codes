#include <iostream>
#include <map>
#include <Eigen/Dense>
#include <string>
#include <vector>

using namespace std;

// Solve Traveling Salesman Problem using Dynamic Programming from matrix of intertravel costs

class CostHandler{

    public:

        Eigen::MatrixXi travel_costs;
        map<int,char> city_map;
        map<string,int> cost_map;

        int n_cities;

        CostHandler(Eigen::MatrixXi tc){

            travel_costs = tc;
            n_cities = tc.rows();

            // build map integer to char name of the city
            char letter_temp = 'A';
            for (int i=0 ; i < n_cities ; i++){

                city_map.insert( pair<int,char>(i,letter_temp));
                letter_temp++;
            }

            // Build cost map cost["BA"] = 4    B-->A travel
            // Convert travel matrix to pairwise distance cost

            for (int i=0 ; i < n_cities ; i++){
                for (int j=0 ; j < n_cities ; j++ ){
                    char letter_from = city_map[i];
                    char letter_to   = city_map[j];
                    int  cost_ij     = tc(i,j);
                    // cout << "The cost from " << letter_from << " to " << letter_to << " is  " << cost_ij << "\n";
                    string pair_temp;
                    pair_temp += letter_from;
                    pair_temp += letter_to;

                    cost_map.insert(pair<string,int>(pair_temp,cost_ij)  );

                }
            } 
        }

        int stage_cost(string fr , string to){
            string travel_plan ;

            travel_plan += fr.back();
            travel_plan += to[to.find(fr.back())+1];

            // cout << travel_plan << " ";
            return cost_map[travel_plan];
        }

        int final_cost(string fr){

            string travel_plan;

            travel_plan += fr.back();
            travel_plan += "A";       // We go back home

            return cost_map[travel_plan];

        }

};


class MotionPlanner{

  public:

    map<pair<int,string>,int> V;    // Value function

    MotionPlanner(){};

    void compute_V(map<int,vector<string>> stage2states,map<string,vector<string>> reachable_from,CostHandler ch){

      for(int i = 0 ; i < stage2states[ch.n_cities-1].size() ; i++){          
        V[make_pair(ch.n_cities-1,stage2states[ch.n_cities-1][i])] = ch.final_cost(stage2states[ch.n_cities-1][i]);
      }

      for(int tt = ch.n_cities-1 ; tt>0 ; tt--){

        for(int kk = 0 ; kk < stage2states[tt-1].size() ; kk++){
        
          int V_temp = ch.travel_costs.sum();     
          vector<string> reachable_from_kk = reachable_from[stage2states[tt-1][kk]];

          for(int i = 0 ; i < reachable_from_kk.size() ; i++){

            int J =  V[make_pair(tt,reachable_from_kk[i])];
            int Q  =  ch.stage_cost(stage2states[tt-1][kk],reachable_from_kk[i]) ;
            // cout << "Transition from " << stage2states[tt-1][kk] << " to " << reachable_from_kk[i] << " costs = " << J+Q << " \n";

            if (J+Q < V_temp){
                V_temp = J+Q;
            }
          }

          V[make_pair(tt-1, stage2states[tt-1][kk])] = V_temp; 

        }
      }

    }


    vector<string> play_dp(map<string,vector<string>> reachable_from,CostHandler ch){

      string          current_state = "A";
      vector<string>  path;

      path.push_back(current_state);

      for(int tt=0 ; tt<ch.n_cities-1 ; tt++){

        int Q_tmp = 10000;
        string temp_next_state;
        vector<string> reachable_from_kk = reachable_from[current_state];

        for(int i = 0 ; i < reachable_from_kk.size() ; i++){

          int Q = ch.stage_cost(current_state,reachable_from_kk[i]) + V[make_pair(tt+1,reachable_from_kk[i])];

          if (Q < Q_tmp){
            Q_tmp = Q;
            temp_next_state = reachable_from_kk[i];
          }


        }

        // Advance state
        current_state = temp_next_state;
        path.push_back(current_state);

      }

      return path;

      }



};


int main(int argc, char** argv){ 

  // Travel costs

  Eigen::MatrixXi tc(5,5);
  tc << 0 , 5 , 1 , 15  , 10 , 
        5 , 0 , 20,  4  , 10 ,
        1 , 20,  0,  3  , 10 ,
        15,  4,  3,  0  , 10 ,
        10, 10, 10,  10 ,  0 ;

  
  // TODO Generate random symmetric matrix with zero diagonal entries
  /*
  const int n_dim = 4;
  Eigen::MatrixXd m = (Eigen::MatrixXd::Random(n_dim,n_dim)+Eigen::MatrixXd::Ones(n_dim,n_dim))*5;  
  Eigen::MatrixXi tc = m.cast<int>();    
  */
  


  cout << "Travel cost matrix: \n" ;
  cout << tc << "\n";

  CostHandler ch(tc);            // Object that handle stage costs

  // Associate city number with letters

  /*
  std::cout << "city map contains :\n";
  for (std::map<int,char>::iterator it=ch.city_map.begin(); it!=ch.city_map.end(); ++it)
        std::cout << it->first << " => " << it->second << '\n';

    std::cout << "cost map contains :\n";
  for (std::map<string,int>::iterator it=ch.cost_map.begin(); it!=ch.cost_map.end(); ++it)
        std::cout << it->first << " => " << it->second << '\n';
  
  */

  // Generate map from stage to list of states

  map<int,vector<string>> stage2states;
 
  map<string,vector<string>> reachable_from;

  stage2states.insert( pair<int,vector<string>>(0,{"A"}));
  
  // cout << "Populate tree...  \n" ;
  for(int i = 0 ; i < ch.n_cities-1 ; i++){                                // cycle over stages

      //cout << " Stage : " << i << " States: \n";
      vector<string> stage_i_states = stage2states[i];
      //ps.vector(stage_i_states);
      vector<string> stage_ip1_states ;

      for (int j=0 ; j < stage_i_states.size() ; j++){                // cycle over stage i states

        vector<string> reached_from_j ;   // very likely not needed

          for (int k=0 ; k < ch.n_cities ; k++){
              
              // cout << stage_i_states[j] << " contains " << city_map[k] << "? "  << (stage_i_states[j].find(city_map[k]) == string::npos) << "\n" ;
              // cout << stage_i_states[j].find(city_map[k]) << "\n";
              if (stage_i_states[j].find(ch.city_map[k]) == string::npos){              // did not find city k in the currrent state

                string new_state = stage_i_states[j] + ch.city_map[k];       // append  new state to this one

                // cout << "New state to add: " << new_state << "\n";
                stage_ip1_states.push_back(new_state);
                reached_from_j.push_back(new_state);

              }
          }


        reachable_from[stage_i_states[j]] = reached_from_j; 

      }

    // ps.vector(stage_ip1_states);
    stage2states.insert( pair<int,vector<string>>(i+1,stage_ip1_states));

  }

  MotionPlanner mp;
  mp.compute_V(stage2states,reachable_from,ch);
  vector<string> path = mp.play_dp(reachable_from,ch);

  // Online play

  cout << "Optimal Cost is: " << mp.V[make_pair(0,"A")] << "\n" ;

  cout << "Optimal Path is: ";
  for (int i = 0; i<path.size() ; i++){
      cout << " " << path[i] ;
  }
  cout << " \n" ;



  return 0;
}


