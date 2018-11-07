\ Model qcp
\ LP format - for model browsing. Use MPS format to capture full model detail.
Maximize
  C0
Subject To
 c0: C0 + C1 + C2 = 1
 qc0: [ C0 ^2 + C1 ^2 - C2 ^2 ] <= 0
 qc1: [ C0 ^2 - C1 * C2 ] <= 0
Bounds
End
