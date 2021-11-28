\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  5 x[A] + 7 x[B] + 8 x[C] + 4 x[D] + 5 x[E] + 6 x[F] + 3 x[G] + 4 x[H]
   + 9 x[I] + 2 x[J]
Subject To
 A_C: - y[A] + y[C] + x[A] >= 32
 C_J: - y[C] + y[J] + x[C] >= 36
 B_F: - y[B] + y[F] + x[B] >= 28
 F_J: - y[F] + y[J] + x[F] >= 54
 B_E: - y[B] + y[E] + x[B] >= 28
 E_H: - y[E] + y[H] + x[E] >= 32
 B_D: - y[B] + y[D] + x[B] >= 28
 D_G: - y[D] + y[G] + x[D] >= 16
 G_I: - y[G] + y[I] + x[G] >= 17
 E_I: - y[E] + y[I] + x[E] >= 32
 G_H: - y[G] + y[H] + x[G] >= 17
 J_FIN: - y[J] + y[FINAL] + x[J] >= 18
 H_FIN: - y[H] + y[FINAL] + x[H] >= 20
 I_FIN: - y[I] + y[FINAL] + x[I] >= 34
 FIN_MAX: y[FINAL] <= 92
Bounds
 y[A] <= 92
 y[B] <= 92
 y[C] <= 92
 y[D] <= 92
 y[E] <= 92
 y[F] <= 92
 y[G] <= 92
 y[H] <= 92
 y[I] <= 92
 y[J] <= 92
 y[FINAL] <= 92
 x[A] <= 4
 x[B] <= 3
 x[C] <= 5
 x[D] <= 3
 x[E] <= 5
 x[F] <= 7
 x[G] <= 2
 x[H] <= 3
 x[I] <= 4
 x[J] <= 2
Generals
 y[A] y[B] y[C] y[D] y[E] y[F] y[G] y[H] y[I] y[J] y[FINAL] x[A] x[B] x[C]
 x[D] x[E] x[F] x[G] x[H] x[I] x[J]
End
