\ Modelo de cursos de verano
maximize
   100 x1 + 75 x2 + 30 x3
subject to
   100 x1 + 50 u1 + 150 x2 + 20 u2 + 75 x3 + 30 u3 <= 250
   \ Se deben de pasar a la forma estándar, todas las variables de un lado y las constantes de otro
   x1 - u1 <= 0 	\ x1 <= u1
   x2 - u2 <= 0		\ x2 <= u2
   x3 - u3 <= 0		\ x3 <= u3

   \ Se pueden poner las restricciones de los rangos de las variables aquí, pero es mas bonito en bounds:
   \ x1 >= 0 
   \ x1 <= 1
\ Por defecto, se asume que las variables son reales y no negativas, pero si se debe de poner el rango y qué variables son binarias:
bounds
   0 <= x1 <= 1
   0 <= x2 <= 1
   0 <= x3 <= 1
binaries
   u1 u2 u3
end \ es opcional

