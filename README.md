# lifehack2024


1: Demand Prediction 
- data_prediction() function called.
- Using a Linear Regression Model.
- Randomly split dataset 80:20 for training and testing data.
- Check goodness of fit of model from Explained Variance, Mean Squared Error and Root Mean Squared Error.


2: Managing Inventory 
- integrated_inventory() is called to check if a restock is required. Average stock and re order points are calculated and printed.
- If the average stock is less than or equal to average ROP, a store deficit will be calculated to determine quantity of restock required.
- The restock_qty is adjusted based on whether the store has a surplus or deficit.
- This will then call the solve_CVRP() function to optimise the route for how the restock will be delivered.


3: Capacitated Vehicle Routing Provlem (CVRP) was implemented through a few functions:

1. create_data_model(demand)
- Includes an adjacency matrix / distance matrix and an array for vehicle capacities
- Returns port (starting point) location

2. print_solution(data, manager, routing, solution) 

3. solve_CVRP(demand) 
- distance_callback(from_index, to_index) returns the distance between the two nodes
- demand_callback(from_index) returns the demand of the node (store that the vehicle is delivering to)