
#include <iostream>
#include <string>
#include <math.h>
#include <stdio.h>
#include <gurobi_c++.h>

template<typename... T>
std::string formato(const char* s, const T&... v) {
   char bufer[32 + 1];
   sprintf(bufer, s, v...);
   return std::string(bufer);
}

struct punto {
   double x, y;
};

double distancia(punto p1, punto p2) {
   return hypot(p1.x - p2.x, p1.y - p2.y);
}

int main( ) try {
   // lectura de la entrada

   int n;
   std::cin >> n;     // suponiendo que es m�ltiplo de 3

   punto arr[n];
   for (int i = 0; i < n; ++i) {
      std::cin >> arr[i].x >> arr[i].y;
   }

   // construcci�n del modelo

   GRBEnv ambiente;
   GRBModel modelo(ambiente);

   GRBVar x[n][n];
   for (int i = 0; i < n; ++i) {
      for (int j = i + 1; j < n; ++j) {
         x[i][j] = modelo.addVar(0, 1, 0, GRB_BINARY, formato("x%d_%d", i, j));
         x[j][i] = x[i][j];
      }
   }

   GRBVar s[n];
   for (int i = 0; i < n; ++i) {
      s[i] = modelo.addVar(1, 2, 0, GRB_INTEGER, formato("s%d", i));
   }

   GRBLinExpr objetivo;
   for (int i = 0; i < n; ++i) {
      for (int j = i + 1; j < n; ++j) {
         objetivo += distancia(arr[i], arr[j]) * x[i][j];
      }
   }
   modelo.setObjective(objetivo, GRB_MINIMIZE);

   for (int i = 0; i < n; ++i) {
      GRBLinExpr ex;
      for (int j = 0; j < n; ++j) {
         if (i != j) {
            ex += x[i][j];
         }
      }
      modelo.addConstr(s[i] == ex);
   }

   for (int i = 0; i < n; ++i) {
      for (int j = i + 1; j < n; ++j) {
         modelo.addConstr(s[i] + s[j] >= 2 + x[i][j]);
         modelo.addConstr(s[i] + s[j] <= 4 - x[i][j]);
      }
   }

   modelo.write("3matching.lp");
   modelo.optimize( );
   if (modelo.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
      modelo.write("3matching.sol");
   } else if (modelo.get(GRB_IntAttr_Status) == GRB_UNBOUNDED) {
      std::cout << "Modelo no acotado\n";
   } else if (modelo.get(GRB_IntAttr_Status) == GRB_INFEASIBLE) {
      std::cout << "Modelo infactible\n";
   } else {
      std::cout << "Status no manejado\n";
   }
} catch (const GRBException& e) {
   std::cout << e.getMessage( ) << "\n";
}

// g++ -std=c++20 -O3 3matching.cpp -lgurobi_c++ -lgurobi100 -o 3matching
// ./3matching

