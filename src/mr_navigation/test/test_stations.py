import math

from mr_navigation import stations as st


def test_roundtrip(tmp_path):
    f = tmp_path / 'e.yaml'
    s = {'cobot': st.Station('cobot', 0.2, 5.15, math.radians(-90), 'Atraque'),
         'estacion_A': st.Station('estacion_A', 3.0, 1.0, 0.0)}
    st.save(f, 'map', s)
    frame, back = st.load(f)
    assert frame == 'map' and set(back) == {'cobot', 'estacion_A'}
    assert math.isclose(back['cobot'].yaw, math.radians(-90), abs_tol=1e-3)
    assert back['cobot'].description == 'Atraque'


def test_names():
    assert st.valid_name('estacion_A') and st.valid_name('zona-2')
    assert not st.valid_name('') and not st.valid_name('a b') and not st.valid_name('x' * 41)


def test_missing_file(tmp_path):
    assert st.load(tmp_path / 'no.yaml') == ('map', {})
