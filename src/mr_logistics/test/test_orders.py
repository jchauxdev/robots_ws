from mr_logistics.orders import OrderBook


def book():
    return OrderBook(['rojo', 'apple'], ['inicio', 'cobot', 'estacion_A'], 'cobot', 3)


def test_create_and_validate():
    b = book()
    r, msg = b.create('rojo', 2, 'estacion_A')
    assert r.id == 'P-0001' and r.state == 'RECIBIDO'
    assert b.create('pera', 1, 'estacion_A')[0] is None
    assert b.create('rojo', 0, 'estacion_A')[0] is None
    assert b.create('rojo', 4, 'estacion_A')[0] is None
    assert b.create('rojo', 1, 'cobot')[0] is None          # destino = estacion del cobot
    assert b.create('rojo', 1, 'luna')[0] is None
    assert b.create('apple', 1, 'inicio')[0].id == 'P-0002'


def test_states_and_pending():
    b = book()
    r, _ = b.create('rojo', 1, 'estacion_A')
    b.set_state(r.id, 'EN_COLA')
    assert [x.id for x in b.pending()] == [r.id]
    b.set_state(r.id, 'HACIA_COBOT')
    assert b.pending() == []
    b.set_state(r.id, 'COMPLETADO')
    b.set_state(r.id, 'HACIA_DESTINO')                     # un pedido terminado no se reabre
    assert b.get(r.id).state == 'COMPLETADO'
    assert [h[1] for h in b.get(r.id).history] == ['RECIBIDO', 'EN_COLA', 'HACIA_COBOT', 'COMPLETADO']
