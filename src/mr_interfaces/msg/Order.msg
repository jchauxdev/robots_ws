# Pedido logistico creado en la App web (mr_logistics).
string id                    # identificador unico (p. ej. "P-0007"); vacio => lo asigna el gestor
string product               # etiqueta que entiende la vision del cobot: "rojo", "aruco_7", "apple", ...
int32 quantity               # unidades a cargar (>= 1)
string pickup_station        # punto con nombre donde espera el cobot (por defecto "cobot")
string destination           # punto con nombre de la estacion de entrega ("estacion_A", ...)
builtin_interfaces/Time stamp
