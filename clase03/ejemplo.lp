\ Ejemplo
minimize
   x
subject to
   x >= -20
 \ x >= 0	\ Por defector en gurobi, solución:
bounds
   x free   \ quita la no-negatividad que hay por omisión
