"""Modelo de pedidos de la App logistica (sin dependencias de ROS; probado con pytest)."""
from __future__ import annotations

import itertools
import threading
import time
from dataclasses import asdict, dataclass, field

STATES = ['RECIBIDO', 'EN_COLA', 'HACIA_COBOT', 'ESPERANDO_CARGA', 'CARGANDO', 'HACIA_DESTINO',
          'ENTREGADO', 'REGRESANDO', 'COMPLETADO', 'FALLIDO', 'CANCELADO']
FINAL = {'COMPLETADO', 'FALLIDO', 'CANCELADO'}


@dataclass
class OrderRecord:
    id: str
    product: str
    quantity: int
    destination: str
    pickup_station: str = 'cobot'
    state: str = 'RECIBIDO'
    detail: str = ''
    created: float = field(default_factory=time.time)
    updated: float = field(default_factory=time.time)
    history: list = field(default_factory=list)

    def to_dict(self):
        return asdict(self)


class OrderBook:
    def __init__(self, products, stations, pickup='cobot', max_quantity=5):
        self.products = list(products)
        self.stations = set(stations)
        self.pickup = pickup
        self.max_quantity = max_quantity
        self._orders: dict[str, OrderRecord] = {}
        self._seq = itertools.count(1)
        self._lock = threading.Lock()

    def validate(self, product, quantity, destination):
        if product not in self.products:
            return f'Producto desconocido "{product}". Opciones: {", ".join(self.products)}'
        if not isinstance(quantity, int) or not 1 <= quantity <= self.max_quantity:
            return f'Cantidad invalida (1..{self.max_quantity})'
        if destination not in self.stations:
            return f'Destino desconocido "{destination}". Opciones: {", ".join(sorted(self.stations))}'
        if destination == self.pickup:
            return 'El destino no puede ser la estacion del cobot'
        return None

    def create(self, product, quantity, destination, order_id=''):
        err = self.validate(product, quantity, destination)
        if err:
            return None, err
        with self._lock:
            oid = order_id or f'P-{next(self._seq):04d}'
            if oid in self._orders:
                return None, f'Ya existe el pedido {oid}'
            rec = OrderRecord(oid, product, quantity, destination, self.pickup)
            rec.history.append((rec.created, 'RECIBIDO', ''))
            self._orders[oid] = rec
        return rec, f'Pedido {oid} recibido'

    def set_state(self, oid, state, detail=''):
        assert state in STATES, state
        with self._lock:
            rec = self._orders[oid]
            if rec.state in FINAL and state not in FINAL:
                return rec        # un pedido terminado no se reabre
            rec.state, rec.detail, rec.updated = state, detail, time.time()
            rec.history.append((rec.updated, state, detail))
            return rec

    def get(self, oid):
        return self._orders.get(oid)

    def all(self):
        with self._lock:
            return sorted(self._orders.values(), key=lambda r: r.created, reverse=True)

    def pending(self):
        with self._lock:
            return [r for r in sorted(self._orders.values(), key=lambda r: r.created)
                    if r.state in ('RECIBIDO', 'EN_COLA')]
