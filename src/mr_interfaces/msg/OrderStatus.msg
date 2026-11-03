# Estado de un pedido publicado por el gestor de misiones (mission_manager).
string id
string state      # RECIBIDO | EN_COLA | HACIA_COBOT | ESPERANDO_CARGA | CARGANDO | HACIA_DESTINO | ENTREGADO | REGRESANDO | FALLIDO | CANCELADO
string detail
string product
int32 quantity
string destination
builtin_interfaces/Time stamp
