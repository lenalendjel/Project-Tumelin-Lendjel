include "globals.mzn";
include "data.dzn"; % Including the data file 
int: t = n + 1;

array[1..n, 1..m] of var bool: path;
array[1..m, 1..t, 1..t] of var bool: symmetry;

array[1..m] of var int: tour_distance;


var int: max_distance = max([tour_distance[j] | j in 1..m]);
var int: min_distance = min([tour_distance[j] | j in 1..m]);
var int: distance_balance = max_distance - min_distance;

constraint m <= n;

% each item is delivered by exactly one courier  
constraint forall(i in 1..n)(count([path[i,x] | x in 1..m],true) = 1);

% each courier makes at least one delivery
constraint forall(x in 1..m)(count([path[i,x]| i in 1..n], true) >= 1);

% each courier delivers no more items than their weight capacity
constraint forall(x in 1..m)(sum([SizeS[i] | i in 1..n where path[i,x]]) <= LoadSize[x]);

% sum of the distances traveled by the courier, allows to measure the performance of the journey for each courier
constraint forall(j in 1..m)(tour_distance[j] = sum([Distances[i, k] | i in 1..n, k in 1..n where path[i, j] /\ path[k, j]]));

% Symmetry constraint
constraint forall(i,j in 1..n where i < j /\ Distances[i,j] == Distances[j,i])(
  forall(k in 1..m)(not symmetry[k,j,i])
);

solve minimize distance_balance;

