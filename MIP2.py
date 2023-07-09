
#!pip3 install gurobipy

import numpy as np
import gurobipy as gp
from gurobipy import Model, GRB, quicksum
from os import scandir


filespath = 'Instances/'
data_list = []

with scandir(filespath) as file_list:
    for filename in file_list:
        with open(filespath + filename.name) as f:
            lines = f.readlines()

            num_couriers = int(lines[0].strip())
            num_items = int(lines[1].strip())
            courier_loads = [int(i) for i in lines[2].strip().split()]
            item_sizes = [int(i) for i in lines[3].strip().split()]
            distance_matrix = [[int(j) for j in i.strip().split()] for i in lines[4:]]

            data_list.append({
                "m": num_couriers,
                "n": num_items,
                "vertex_order": courier_loads,
                "s": item_sizes,
                "D": distance_matrix
            })
data_list = sorted(data_list, key=lambda d: d['m'] + d['n'])
data = data_list[11]
m = data['m']
n = data['n']
vertex_order = data['vertex_order']
s = data['s']
D = np.array(data['D'])

# Create Gurobi model
model = gp.Model()

# Variables
items = [i for i in range(1, n + 1)]
vertices = [0] + items
arcs = [(i, j) for i in vertices for j in vertices if i != j]

goes_to = model.addVars(arcs, vtype=GRB.BINARY)
vertex_order = model.addVars(vertices, lb=0, ub=n, vtype=GRB.INTEGER)
courier_assignment = {}
aux_vars = {}

for i in range(1, n + 1):
    courier_assignment[i] = model.addVars(range(m), vtype=GRB.BINARY)
    aux_vars[i] = model.addVars(range(m), vtype=GRB.INTEGER)

# Objective function
# Objective function
model.modelSense = gp.GRB.MINIMIZE



# Constraints
model.addConstrs(quicksum(goes_to[i, j] for j in vertices if j != i) == 1 for i in items)
model.addConstrs(quicksum(goes_to[i, j] for i in vertices if i != j) == 1 for j in items)
model.addConstr(quicksum(goes_to[0, j] for j in items) <= m)
model.addConstr(quicksum(goes_to[0, j] for j in items) == quicksum(goes_to[j, 0] for j in items))
# Sub-path elimination
model.addConstr(vertex_order[0] == 1)
model.addConstrs((vertex_order[i] + goes_to[i, j]) <= (vertex_order[j] + n * (1 - goes_to[i, j])) for i, j in arcs if j != 0)
# Constraints related to courier
for k in range(m):
    model.addConstrs((aux_vars[i][k] <= courier_assignment[i][k]) for i in items)
    model.addConstrs((aux_vars[i][k] <= goes_to[0, i]) for i in items)
    model.addConstrs((aux_vars[i][k] >= courier_assignment[i][k] + goes_to[0, i] - 1) for i in items)
    model.addConstr(quicksum(aux_vars[i][k] for i in items) <= 1)
    # If goes_to[i,j] = True, i,j share the same courier
    for i, j in arcs:
        if i != 0 and j != 0:
            model.addConstr(goes_to[i, j] + courier_assignment[i][k] - courier_assignment[j][k] <= 1)
            model.addConstr(goes_to[i, j] - courier_assignment[i][k] + courier_assignment[j][k] <= 1)
# Every item should be visited by exactly one courier
model.addConstrs(quicksum(courier_assignment[i][k] for k in range(m)) == 1 for i in items)
# The total load of items served by each courier cannot exceed the courier's capacity
model.addConstrs(
    quicksum(s[i - 1] * courier_assignment[i][k] for i in items) <= vertex_order[k] for k in range(m)
)

for k in range(1, m + 1):
    model.addConstr(quicksum(vertex_order[k] for k in range(1, m + 1)) == k)


for i in range(1, n + 1):
    for j in range(i + 1, n + 1):
        model.addConstr(quicksum(courier_assignment[i][k] + courier_assignment[j][k] for k in range(m)) <= 1)

# Solver settings
model.params.ScaleFlag = 1
model.params.MIPFocus = 1
model.Params.TimeLimit = 300

# Optimize model
model.optimize()

# Create objective expression and variable
objective_expr = quicksum(goes_to[i, j] * D[i][j] for i, j in arcs)
objective_var = model.addVar(lb=objective_expr.getValue(), ub=objective_expr.getValue())

# Additional constraint to enforce the objective variable bounds
model.addConstr(objective_var == objective_expr)

# Update the objective to use the auxiliary variable
model.setObjective(objective_var)


# Print solution
path = {}
try:
    for i in items:
        if goes_to[(0, i)].goes_to > 0.1:
            for k in range(m):
                if int(str(courier_assignment[i][k])[-6:-4]) > 0:
                    current_courier = k
            current_courier = current_courier + 1
            path[current_courier] = []
            aux = [0, i]
            while i != 0:
                j = i
                for k in vertices:
                    if j != k and goes_to[(j, k)].goes_to > 0.9:
                        aux.append(k)
                        i = k
            path[current_courier].append(aux)

    for key in path.keys():
        current_courier = str(path[key])
        current_courier = current_courier.replace("[", "")
        current_courier = current_courier.replace("]", "")
        current_courier = current_courier.replace(", ", " => ")
        path[key] = current_courier

    keys = [i for i in path.keys()]
    print("path planned:")

    for key in sorted(keys):
        print("Courier " + str(key))
        print(path[key])

    print("---------------------------")
    print("Total distance:", int(model.ObjVal))
except AttributeError:
    print("Total distance: N/A")
