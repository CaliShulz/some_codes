# some_codes
miscellaneous codes from various topics

dp2.cpp solves the traveling salesman problem using exact dynamic programming.

It takes as input the matrix of intertravel costs.

Example of output with   tc << 0 , 5 , 1 , 15  , 10 , 
                               5 , 0 , 20,  4  , 10 ,
                               1 , 20,  0,  3  , 10 ,
                               15,  4,  3,  0  , 10 ,
                               10, 10, 10,  10 ,  0 ;
                               
Output: 

Travel cost matrix: 
 0  5  1 15 10
 5  0 20  4 10
 1 20  0  3 10
15  4  3  0 10
10 10 10 10  0
Optimal Cost is: 28
Optimal Path is:  A AC ACD ACDB ACDBE 
